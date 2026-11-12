"""Read plates inside the ROI on RC522 requests and return results to the CAM."""
import argparse
import json
import sqlite3
import logging
import os
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

# Set before importing CharacterDetector/PaddleX. Models are already cached.
# This skips source availability checks, not model loading or camera requests.
os.environ.setdefault("PADDLE_PDX_DISABLE_MODEL_SOURCE_CHECK", "True")

import cv2
import numpy as np
import requests

from parking_store import ParkingStore
from camera.camera_client import CameraClient
from vision.character_detector import CharacterDetector
from vision.lp_detector import LPDetector


BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "models" / "LP_best.pt"
SAVED_DIR = BASE_DIR / "saved_images"
CROP_DIR = BASE_DIR / "crop_image"
DATABASE_PATH = BASE_DIR / "data" / "parking.sqlite3"
ACCESS_HTTP_TIMEOUT = (3, 3)
FRAME_COUNT = 5
COLLECT_TIMEOUT_SECONDS = 10.0
POLL_SECONDS = 0.5
HTTP_TIMEOUT = (10, 10)
LP_CONFIDENCE = 0.2
ROI_REL = (80 / 640, 100 / 480, 480 / 640, 280 / 480)
VIEW_WIDTH, VIEW_HEIGHT = 640, 480
PANEL_WIDTH = 240
WINDOW_WIDTH = VIEW_WIDTH + PANEL_WIDTH
WINDOW_HEIGHT = VIEW_HEIGHT
STREAM_FRESH_SECONDS = 2.0
VIETNAM_TZ = timezone(timedelta(hours=7))
LOGGER = logging.getLogger("parking_ocr")


def local_now():
    return datetime.now(VIETNAM_TZ)


class Stopped(Exception):
    pass


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "state": "LOADING_MODEL", "delivery": "--",
            "camera_id": None, "request_id": None, "frame_count": 0,
            "plate": "", "confidence": None, "raw_text": "", "reason": "",
            "recognized_at": None, "crop": None, "result_status": "--",
            "message": "Loading models; tap RC522 only after READY",
            "approval": None, "audit_state": "CONNECTING", "audit_message": "",
        }

    def update(self, **values):
        with self.lock:
            self.data.update(values)

    def snapshot(self):
        with self.lock:
            return dict(self.data)


def check_stop(stop):
    if stop.is_set():
        raise Stopped()


def save_image(path, image):
    if image is None or image.size == 0 or not cv2.imwrite(str(path), image):
        raise RuntimeError(f"Cannot save image: {path}")


def make_roi(frame):
    """Same relative ROI and (x1, y1, x2, y2) convention as test.py."""
    height, width = frame.shape[:2]
    rx, ry, rw, rh = ROI_REL
    if not (0 <= rx < 1 and 0 <= ry < 1 and rw > 0 and rh > 0):
        raise ValueError("Invalid ROI_REL")
    x1 = max(0, min(width - 1, round(width * rx)))
    y1 = max(0, min(height - 1, round(height * ry)))
    x2 = max(x1 + 1, min(width, x1 + round(width * rw)))
    y2 = max(y1 + 1, min(height, y1 + round(height * rh)))
    return x1, y1, x2, y2


def focus_score(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def collect_best_frame(camera, monitor, stop):
    # Score only the ROI; a sharp background outside it must not choose BEST.
    # Sequence/time establish new reception after the request, not exposure time.
    _, last_sequence, connection, _, _ = camera.snapshot()
    started_at = time.monotonic()
    deadline = started_at + COLLECT_TIMEOUT_SECONDS
    last_error = "No camera frames"
    count = 0
    best, best_roi, best_score, shape = None, None, -1.0, None
    while time.monotonic() < deadline:
        check_stop(stop)
        frame, sequence, current_connection, received_at, error = camera.snapshot()
        if error:
            last_error = error
        if current_connection != connection:
            connection = current_connection
            count = 0
            best, best_roi, best_score, shape = None, None, -1.0, None
            last_sequence = sequence
            monitor.update(frame_count=0)
            continue
        if error or frame is None or sequence <= last_sequence or received_at < started_at:
            stop.wait(0.01)
            continue
        last_sequence = sequence
        if frame.shape != shape:
            count = 0
            best, best_roi, best_score = None, None, -1.0
            shape = frame.shape
        roi = make_roi(frame)
        x1, y1, x2, y2 = roi
        score = focus_score(frame[y1:y2, x1:x2])
        count += 1
        if best is None or score > best_score:
            best, best_roi, best_score = frame.copy(), roi, score
        monitor.update(frame_count=count)
        if count == FRAME_COUNT:
            LOGGER.info("Collected %d new ROI frames; sharpness=%.2f", FRAME_COUNT, best_score)
            return best, best_roi, best_score
    raise RuntimeError(f"Camera collection timeout: {last_error}")


def process_request(camera, detector, ocr, camera_id, request_id, monitor, stop):
    started = time.monotonic()
    requested_at = local_now()
    prefix = f"{requested_at:%Y%m%d_%H%M%S_%f}_cam{camera_id}_req{request_id}"
    best_path = SAVED_DIR / f"{prefix}_best.jpg"
    crop_path = CROP_DIR / f"{prefix}_plate.jpg"
    detail_path = SAVED_DIR / f"{prefix}_result.json"
    packet = {"camera_id": camera_id, "request_id": request_id,
              "status": "ERROR", "plate": "", "confidence": 0}
    detail = {"requested_at": requested_at.isoformat(timespec="seconds"),
              "recognized_at": None, "best_frame": None, "crop_image": None}
    monitor.update(state="COLLECTING", camera_id=camera_id, request_id=request_id,
                   delivery="WAITING_RESULT", frame_count=0, plate="",
                   confidence=None, raw_text="", reason="", recognized_at=None, crop=None,
                   result_status="--",
                   message=f"RFID request received; collecting {FRAME_COUNT} new frames")
    try:
        best, roi, sharpness = collect_best_frame(camera, monitor, stop)
        check_stop(stop)
        monitor.update(state="SAVING", message="Saving the selected frame")
        save_image(best_path, best)
        detail["best_frame"] = str(best_path)

        x1, y1, x2, y2 = roi
        roi_xywh = (x1, y1, x2 - x1, y2 - y1)
        detail.update(roi_xywh=list(roi_xywh), best_roi_sharpness=sharpness)
        monitor.update(state="DETECTING_PLATE", message="Finding the plate inside the ROI")
        # LPDetector already returns global bbox coordinates when roi is supplied.
        # Its image input and padded plate crop are both constrained to this ROI.
        detections = detector.detect_candidates(best, roi=roi_xywh)
        check_stop(stop)
        detail["candidate_count"] = len(detections)
        if not detections:
            packet["status"] = "NO_PLATE"
            monitor.update(state="NO_PLATE", reason="NO_PLATE_DETECTED",
                           message="No plate detected; tap card to try again")
            return packet

        crop, bbox, lp_confidence = detections[0]
        save_image(crop_path, crop)
        detail.update(crop_image=str(crop_path), bbox_xywh=[int(v) for v in bbox],
                      lp_confidence=float(lp_confidence))
        monitor.update(state="READING_OCR", crop=crop,
                       message="Reading the saved ROI plate crop with PaddleOCR")
        disk_crop = cv2.imread(str(crop_path), cv2.IMREAD_COLOR)
        if disk_crop is None:
            raise RuntimeError("Cannot read saved plate crop")
        reading = ocr.recognize(disk_crop)
        check_stop(stop)
        detail["ocr"] = reading
        monitor.update(confidence=float(reading.get("confidence") or 0),
                       raw_text=reading.get("raw_text") or reading.get("text") or "")
        if reading.get("valid") and reading.get("text"):
            plate = reading["text"]
            if not (1 <= len(plate) <= 19 and
                    all("A" <= c <= "Z" or "0" <= c <= "9" for c in plate)):
                raise ValueError("OCR returned a plate unsuitable for UART")
            confidence = int(round(float(reading.get("confidence") or 0)))
            recognized_at = local_now().isoformat(timespec="seconds")
            detail["recognized_at"] = recognized_at
            packet.update(status="OK", plate=plate,
                          confidence=max(0, min(100, confidence)))
            monitor.update(state="RECOGNIZED", plate=plate, recognized_at=recognized_at,
                           message="OCR complete; sending result to CAM")
            LOGGER.info("[RECOGNIZED] plate=%s at=%s camera=%s request=%s",
                        plate, recognized_at, camera_id, request_id)
        else:
            packet["status"] = "OCR_FAILED"
            reason = reading.get("reason") or "LOW_CONFIDENCE_OR_INCOMPLETE"
            monitor.update(state="OCR_FAILED", reason=reason, message=f"OCR failed: {reason}")

        # This box belongs to the saved BEST frame, never the changing live view.
        annotated = best.copy()
        x, y, width, height = (int(v) for v in bbox)
        cv2.rectangle(annotated, (x1, y1), (x2 - 1, y2 - 1), (0, 220, 255), 2)
        cv2.rectangle(annotated, (x, y), (x + width, y + height), (0, 230, 0), 2)
        cv2.putText(annotated, packet["plate"] or packet["status"],
                    (x, max(22, y - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 230, 0), 2)
        annotated_path = SAVED_DIR / f"{prefix}_annotated.jpg"
        try:
            save_image(annotated_path, annotated)
            detail["annotated_image"] = str(annotated_path)
        except (RuntimeError, cv2.error) as error:
            detail["annotated_image_error"] = str(error)
            LOGGER.warning("Cannot save annotated image: %s", error)
        return packet
    except Stopped:
        raise
    except Exception as error:
        LOGGER.exception("OCR request=%s failed", request_id)
        detail["error"] = str(error)
        detail["recognized_at"] = None
        packet.update(status="ERROR", plate="", confidence=0)
        monitor.update(state="ERROR", plate="", recognized_at=None,
                       reason=str(error), message=str(error))
        return packet
    finally:
        detail.update(packet)
        detail["processing_ms"] = round((time.monotonic() - started) * 1000, 1)
        try:
            detail_path.write_text(json.dumps(detail, ensure_ascii=False, indent=2),
                                   encoding="utf-8")
        except (OSError, TypeError, ValueError):
            LOGGER.exception("Cannot save diagnostic JSON: %s", detail_path)


def read_request(http, base_url, expected_camera_id):
    with http.get(f"{base_url}/ocr-request", timeout=HTTP_TIMEOUT) as response:
        if response.status_code == 204:
            return None
        response.raise_for_status()
        data = response.json()
    if not isinstance(data, dict):
        raise ValueError("Invalid OCR request response")
    camera_id, request_id = data.get("camera_id"), data.get("request_id")
    if (type(camera_id) is not int or not 1 <= camera_id <= 255 or
            type(request_id) is not int or not 1 <= request_id <= 0xFFFFFFFF):
        raise ValueError("Invalid camera_id/request_id returned by CAM")
    if camera_id != expected_camera_id:
        raise ValueError(f"Wrong camera IP: expected {expected_camera_id}, got {camera_id}")
    return camera_id, request_id


def ocr_worker(camera, monitor, stop, camera_host, expected_camera_id):
    base_url = f"http://{camera_host}"
    cached_key = None
    cached_packet = None
    last_poll_error = None
    try:
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(f"LP model not found: {MODEL_PATH}")
        import torch
        # Leave CPU time for JPEG decoding and the GUI during YOLO inference.
        torch.set_num_threads(2)
        detector = LPDetector(model_path=str(MODEL_PATH), conf=LP_CONFIDENCE)
        check_stop(stop)
        ocr = CharacterDetector()
        check_stop(stop)
        monitor.update(state="WAITING_RFID", message="READY: tap RC522 card to start OCR")
        LOGGER.info("[READY] Waiting for RC522 requests via %s", base_url)
        with requests.Session() as http:
            while not stop.is_set():
                try:
                    key = read_request(http, base_url, expected_camera_id)
                    if last_poll_error is not None:
                        monitor.update(message="CAM connection restored")
                    last_poll_error = None
                except (requests.RequestException, ValueError) as error:
                    message = str(error)
                    monitor.update(message=f"CAM polling: {message}")
                    if message != last_poll_error:
                        LOGGER.warning("CAM request polling failed: %s", message)
                        last_poll_error = message
                    stop.wait(1)
                    continue
                if key is None:
                    if monitor.snapshot()["delivery"] == "RETRY":
                        monitor.update(state="WAITING_RFID", delivery="CHECK_MAIN",
                                       message="CAM has no pending request; check MAIN log")
                    stop.wait(POLL_SECONDS)
                    continue
                check_stop(stop)
                if key != cached_key:
                    LOGGER.info("[OCR START] camera=%s request=%s", *key)
                    cached_packet = process_request(
                        camera, detector, ocr, *key, monitor, stop)
                    cached_key = key
                    monitor.update(result_status=cached_packet["status"])
                    LOGGER.info("[OCR RESULT] %s", json.dumps(cached_packet))
                # A retry reuses this packet; OCR and recognized_at stay unchanged.
                check_stop(stop)
                monitor.update(delivery="SENDING")
                try:
                    packet = dict(cached_packet)
                    # Fresh clock anchor on each send; cached OCR content keeps its original time.
                    packet["server_time_ms"] = time.time_ns() // 1_000_000
                    with http.post(f"{base_url}/plate-result", json=packet,
                                   timeout=HTTP_TIMEOUT) as response:
                        if response.status_code == 409:
                            monitor.update(state="WAITING_RFID", delivery="STALE",
                                           message="Request expired or replaced; tap card again")
                            LOGGER.warning("[STALE] request=%s is no longer pending", key[1])
                        else:
                            response.raise_for_status()
                            monitor.update(state="WAITING_RFID", delivery="SENT_TO_CAM",
                                           message="CAM accepted forwarding; verify the MAIN log")
                            LOGGER.info("[DELIVERED TO CAM] request=%s", key[1])
                except requests.RequestException as error:
                    monitor.update(delivery="RETRY", message=f"Delivery retry: {error}")
                    LOGGER.warning("Delivery failed; keeping cached result: %s", error)
                stop.wait(POLL_SECONDS)
    except Stopped:
        return
    except Exception as error:
        monitor.update(state="ERROR", reason=str(error), message=str(error))
        LOGGER.exception("OCR worker stopped")


def approval_worker(camera_host, expected_camera_id, monitor, stop):
    """Commit actual MAIN grants, then acknowledge the event back to MAIN."""
    base_url = f"http://{camera_host}"
    store = None
    while not stop.is_set() and store is None:
        try:
            store = ParkingStore(DATABASE_PATH)
        except (OSError, sqlite3.Error, ValueError) as error:
            monitor.update(audit_state="DB_ERROR", audit_message=str(error))
            LOGGER.error("Cannot open parking database: %s", error)
            stop.wait(1)
    if store is None:
        return
    monitor.update(audit_state="CONNECTING", audit_message="")
    last_error = None
    next_refresh = 0.0
    try:
        with requests.Session() as http:
            while not stop.is_set():
                try:
                    if time.monotonic() >= next_refresh:
                        record = store.latest_for_camera(expected_camera_id)
                        monitor.update(approval=record)
                        next_refresh = time.monotonic() + 1.0
                    with http.get(f"{base_url}/access-event", timeout=ACCESS_HTTP_TIMEOUT) as response:
                        if response.status_code == 404:
                            raise ValueError("CAM firmware has no /access-event; update both firmwares")
                        response.raise_for_status()
                        event = None if response.status_code == 204 else response.json()
                    if event is not None:
                        # record_event returns after COMMIT. A retry is idempotent by event_id.
                        record = store.record_event(event, expected_camera_id)
                        monitor.update(approval=record, audit_state="SAVED", audit_message="")
                        LOGGER.info("[ACCESS SAVED] %s", json.dumps(record))
                        with http.post(f"{base_url}/access-event-ack",
                                       json={"event_id": event["event_id"]},
                                       timeout=ACCESS_HTTP_TIMEOUT) as response:
                            response.raise_for_status()
                        LOGGER.info("[ACCESS ACK] event=%s", event["event_id"])
                    elif last_error is not None:
                        monitor.update(audit_state="ONLINE", audit_message="")
                    if event is None and monitor.snapshot()["audit_state"] == "CONNECTING":
                        monitor.update(audit_state="ONLINE")
                    last_error = None
                except (requests.RequestException, sqlite3.Error, OSError, ValueError, TypeError) as error:
                    message = str(error)
                    monitor.update(audit_state="RETRY", audit_message=message)
                    if message != last_error:
                        LOGGER.warning("[ACCESS RETRY] %s", message)
                    last_error = message
                stop.wait(0.5)
    finally:
        store.close()


def format_event_time(value):
    if value is None:
        return "--"
    return datetime.fromtimestamp(value / 1000, VIETNAM_TZ).strftime("%d/%m %H:%M:%S")


def format_stay(record):
    if not record or record["entry_at_ms"] is None:
        return "--"
    if record["duration_seconds"] is not None:
        seconds = record["duration_seconds"]
        suffix = ""
    elif record["status"] == "ENTRY_APPROVED":
        seconds = max(0, time.time() - record["entry_at_ms"] / 1000)
        suffix = " RUN"
    else:
        return "--"
    whole = int(seconds)
    hours, remainder = divmod(whole, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d} ({whole / 3600:.3f}h){suffix}"


def put_text(image, message, x, y, scale=0.48, color=(240, 240, 240), thickness=1,
             width=None):
    text = str(message).encode("ascii", errors="replace").decode("ascii")
    width = image.shape[1] - x - 10 if width is None else width
    while text and cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0] > width:
        text = text[:-1]
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)


def fit_image(canvas, image, x, y, width, height):
    """Fit the entire image without stretching or clipping; return its transform."""
    if image is None:
        return None
    ih, iw = image.shape[:2]
    scale = min(width / iw, height / ih)
    new_width = min(width, max(1, round(iw * scale)))
    new_height = min(height, max(1, round(ih * scale)))
    resized = cv2.resize(image, (new_width, new_height),
                         interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    left = x + (width - new_width) // 2
    top = y + (height - new_height) // 2
    canvas[top:top + new_height, left:left + new_width] = resized
    return scale, left, top


def draw_screen(frame, data, scene, gate_name):
    # test.py's live view and two translucent bands, plus one small crop panel.
    image = np.full((WINDOW_HEIGHT, WINDOW_WIDTH, 3), (22, 22, 22), dtype=np.uint8)
    transform = fit_image(image, frame, 0, 0, VIEW_WIDTH, VIEW_HEIGHT)
    state = data["state"]
    color = ((80, 230, 100) if state in ("WAITING_RFID", "RECOGNIZED") else
             (80, 100, 255) if state in ("ERROR", "OCR_FAILED", "NO_PLATE") else (0, 220, 255))
    if transform is not None:
        scale, left, top = transform
        x1, y1, x2, y2 = make_roi(frame)
        start = (left + round(x1 * scale), top + round(y1 * scale))
        end = (left + round((x2 - 1) * scale), top + round((y2 - 1) * scale))
        cv2.rectangle(image, start, end, color, 2)
        put_text(image, "PROCESSING ROI", start[0] + 6, start[1] + 20,
                 0.50, color, width=max(1, end[0] - start[0] - 12))
    else:
        put_text(image, "Waiting for camera stream...", 120, 240, 0.65, width=450)
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (VIEW_WIDTH - 1, 91), (0, 0, 0), -1)
    cv2.rectangle(overlay, (0, VIEW_HEIGHT - 83), (VIEW_WIDTH - 1, VIEW_HEIGHT - 1),
                  (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.67, image, 0.33, 0, image)
    put_text(image, f"{gate_name} | {state}", 10, 27, 0.53, color, 2, 490)
    put_text(image, local_now().strftime("%H:%M:%S"), 528, 26, 0.55, width=103)
    put_text(image, f"FRAMES: {data['frame_count']}/{FRAME_COUNT}"
             f"   LAST RESULT: {data['result_status']}", 10, 53, 0.55, width=620)
    put_text(image, f"PLATE: {data['plate'] or '--'}", 10, 79, 0.65,
             (255, 255, 255), 2, 620)
    age = "--" if scene["age"] is None else f"{scene['age']:.1f}s"
    stream_color = (80, 230, 100) if scene["stream"] == "ONLINE" else (60, 120, 255)
    put_text(image, f"STREAM: {scene['stream']}   IMAGE AGE: {age}",
             10, 418, 0.48, stream_color, width=620)
    put_text(image, f"DELIVERY: {data['delivery']}   REQUEST: {data['request_id'] or '--'}",
             10, 444, 0.46, width=620)
    put_text(image, "q / ESC: Quit   OCR: RC522 only",
             10, 470, 0.44, (225, 225, 225), width=620)
    if transform is not None and scene["stream"] != "ONLINE":
        put_text(image, f"STALE IMAGE - {scene['stream']}", 115, 250, 0.65,
                 (40, 80, 255), 2, 500)

    panel_x = VIEW_WIDTH
    cv2.line(image, (panel_x, 0), (panel_x, WINDOW_HEIGHT - 1), (70, 70, 70), 1)
    x, width = panel_x + 10, PANEL_WIDTH - 20
    put_text(image, f"CAM {gate_name}", x, 25, 0.55, color, 2, width)
    cv2.rectangle(image, (x, 42), (x + width - 1, 179), (80, 80, 80), 1)
    if data["crop"] is not None:
        fit_image(image, data["crop"], x + 1, 43, width - 2, 135)
    else:
        put_text(image, "Waiting for RFID...", x + 7, 112, 0.44, width=width - 14)
    put_text(image, data["plate"] or "--", x, 205, 0.65, color, 2, width)
    confidence = "--" if data["confidence"] is None else f"{data['confidence']:.1f}/100"
    put_text(image, f"CONF: {confidence}", x, 229, 0.44, width=width)
    at = datetime.fromisoformat(data["recognized_at"]) if data["recognized_at"] else None
    put_text(image, "OCR: " + (at.strftime("%d/%m %H:%M:%S") if at else "--"),
             x, 252, 0.40, width=width)
    record = data["approval"]
    status = record["status"] if record else "WAITING_MAIN"
    put_text(image, "LAST ACCESS: " + status, x, 281, 0.40, width=width)
    put_text(image, "PLATE: " + (record["plate"] if record else "--"), x, 305, 0.46, width=width)
    put_text(image, "IN: " + format_event_time(record["entry_at_ms"] if record else None),
             x, 330, 0.43, width=width)
    put_text(image, "OUT: " + format_event_time(record["exit_at_ms"] if record else None),
             x, 355, 0.43, width=width)
    put_text(image, "STAY: " + format_stay(record), x, 380, 0.39, width=width)
    put_text(image, "UID: " + (record["uid"] if record else "--"), x, 405, 0.43, width=width)
    put_text(image, "LOG: " + data["audit_state"], x, 435, 0.42, width=width)
    put_text(image, data["audit_message"] or "APPROVED time | UTC+7", x, 459, 0.36, width=width)
    return image


def main(camera_host, expected_camera_id, gate_name):
    logging.basicConfig(level=logging.INFO,
                        format=f"%(asctime)s %(levelname)s [{gate_name}] %(message)s")
    SAVED_DIR.mkdir(parents=True, exist_ok=True)
    CROP_DIR.mkdir(parents=True, exist_ok=True)
    window_name = f"SMART PARKING - {gate_name} [{camera_host}]"
    camera = CameraClient(camera_host, timeout=(10, 10))
    monitor = MonitorState()
    stop = threading.Event()
    worker = None
    audit_worker = None
    camera.start()
    try:
        # All window/event calls stay here; model loading and OCR use the worker.
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        # A small initial window; all images are fitted in fixed-size panes.
        cv2.resizeWindow(window_name, 800, round(800 * WINDOW_HEIGHT / WINDOW_WIDTH))
        worker = threading.Thread(target=ocr_worker, args=(camera, monitor, stop, camera_host, expected_camera_id),
                                  name="parking-ocr-worker", daemon=True)
        audit_worker = threading.Thread(
            target=approval_worker, args=(camera_host, expected_camera_id, monitor, stop),
            name="parking-access-worker", daemon=True)
        audit_worker.start()
        worker.start()
        while True:
            frame, sequence, _, received_at, camera_error = camera.snapshot()
            now = time.monotonic()
            age = max(0.0, now - received_at) if frame is not None and received_at > 0 else None
            fresh = not camera_error and age is not None and age < STREAM_FRESH_SECONDS
            stream = ("ONLINE" if fresh else
                      "CONNECTING" if sequence == 0 else
                      "RECONNECTING" if camera_error else "STALLED")
            scene = {"stream": stream, "age": age}
            # Keep the last image on a gap, with its ORIGINAL age and a clear
            # stale label. It is not counted again or submitted to OCR.
            cv2.imshow(window_name, draw_screen(frame, monitor.snapshot(), scene, gate_name))
            key = cv2.waitKey(20) & 0xFF
            if key in (ord("q"), 27):
                break
            try:
                if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        cv2.destroyAllWindows()
        if worker is not None:
            worker.join(timeout=2)
        if audit_worker is not None:
            audit_worker.join(timeout=4)
        camera.close()
        LOGGER.info("Live monitor stopped")


def run_gate(gate_name, camera_id, environment_key):
    parser = argparse.ArgumentParser(description=f"Smart Parking {gate_name}")
    parser.add_argument("--host", default=os.environ.get(environment_key, "").strip()
                        or os.environ.get("PARKING_CAMERA_HOST", "").strip(),
                        help="Camera IP or host, without http://")
    args = parser.parse_args()
    if not args.host or any(c in args.host for c in "/ \t\r\n"):
        parser.error(f"Supply --host CAMERA_IP or set {environment_key}")
    main(args.host, camera_id, gate_name)


if __name__ == "__main__":
    raise SystemExit("Run app_entry.py --host ENTRY_IP or app_exit.py --host EXIT_IP")
