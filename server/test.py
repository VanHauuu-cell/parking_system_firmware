
import json
import time
import requests
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from config import CAMERA_HOST
from camera.camera_client import CameraClient
from vision.stream_processor import StreamProcessor
from vision.best_frame_selector import BestFrameSelector
from vision.capture_pipeline import CapturePipeline
from vision.lp_detector import LPDetector
from vision.character_detector import CharacterDetector


BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "models" / "LP_best.pt"
SAVED_DIR = BASE_DIR / "saved_images"
CROP_DIR = BASE_DIR / "crop_image"

FRAME_COUNT = 5
LP_CONFIDENCE = 0.25
WINDOW_NAME = "SMART PARKING - ESP32-CAM LIVE TEST"

# Vehicle-presence ROI only. LP detection uses the full best frame.
# For a 640x480 stream: x=80, y=100, w=480, h=280.
ROI_REL = (80 / 640, 100 / 480, 480 / 640, 280 / 480)


def make_roi(frame):
    h, w = frame.shape[:2]
    rx, ry, rw, rh = ROI_REL
    x = max(0, min(w - 1, round(w * rx)))
    y = max(0, min(h - 1, round(h * ry)))
    x2 = max(x + 1, min(w, x + round(w * rw)))
    y2 = max(y + 1, min(h, y + round(h * rh)))
    return (x, y, x2, y2)


def focus_score(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def save_image(path, image):
    if image is None or image.size == 0 or not cv2.imwrite(str(path), image):
        raise RuntimeError(f"Cannot write image: {path}")


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def draw_screen(frame, roi, state, count, locked, last_result, event=""):
    """Draw detection area, state, count, and last OCR result."""
    image = frame.copy()
    h, w = image.shape[:2]
    x1, y1, x2, y2 = roi

    colors = {
        "WAITING": (0, 210, 0),
        "COLLECTING": (0, 220, 255),
        "SELECTING": (0, 220, 255),
        "SAVING": (255, 210, 0),
        "DETECTING_PLATE": (255, 210, 0),
        "READING_OCR": (255, 210, 0),
        "RECOGNIZED": (0, 210, 0),
        "NO_PLATE": (0, 0, 255),
        "OCR_FAILED": (0, 0, 255),
        "ERROR": (0, 0, 255),
    }
    color = colors.get(state, (220, 220, 220))
    cv2.rectangle(image, (x1, y1), (x2 - 1, y2 - 1), color, 2)
    cv2.putText(image, "VEHICLE ROI", (x1 + 5, min(h - 10, y1 + 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (w, min(91, h)), (0, 0, 0), -1)
    cv2.rectangle(overlay, (0, max(0, h - 59)), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.67, image, 0.33, 0, image)

    plate = (last_result or {}).get("plate") or "--"
    cv2.putText(image, f"STATE: {state}", (10, 27),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(image, f"FRAMES: {count}/{FRAME_COUNT}   VEHICLE: {'YES' if locked else 'NO'}",
                (10, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(image, f"PLATE: {plate}", (10, 78),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(image, f"EVENT: {event or '-'}", (10, h - 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
    cv2.putText(image, "q: Quit    r: Re-arm    c: Capture now", (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return image


def process_best(best_frame, lp_detector, ocr, show_state):
    """Save BEST -> detect LP in full frame -> save CROP -> read CROP -> OCR."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    best_path = SAVED_DIR / f"{stamp}_best.jpg"
    crop_path = CROP_DIR / f"{stamp}_plate.jpg"
    result_path = SAVED_DIR / f"{stamp}_result.json"
    annotated_path = SAVED_DIR / f"{stamp}_annotated.jpg"

    result = {
        "status": "ERROR", "plate": "", "capture_id": stamp,
        "best_frame": str(best_path), "crop_image": None,
        "lp_confidence": None, "ocr_confidence": None, "bbox": None,
    }

    try:
        show_state("SAVING")
        save_image(best_path, best_frame)

        # Do not clip the plate using the vehicle-presence ROI.
        # LPDetector returns list[(crop, bbox_xywh, confidence)], sorted by score.
        show_state("DETECTING_PLATE")
        detections = lp_detector.detect_candidates(best_frame)
        print(f"[LP] Full-frame candidates: {len(detections)}")
        if not detections:
            result["status"] = "NO_PLATE"
            result["reason"] = "LP model found no plate in the full frame"
            return result

        crop, bbox_xywh, confidence = detections[0]
        x, y, width, height = [int(v) for v in bbox_xywh]
        # All output and drawing code uses (x1, y1, x2, y2).
        result["bbox"] = [x, y, x + width, y + height]
        result["lp_confidence"] = float(confidence)

        if not isinstance(crop, np.ndarray) or crop.size == 0:
            raise ValueError("LP detector returned an empty or invalid crop")
        if width <= 0 or height <= 0:
            raise ValueError("LP detector returned an invalid bbox")

        print(
            f"[LP] confidence={float(confidence):.3f} "
            f"bbox_xyxy={result['bbox']} crop_shape={crop.shape}"
        )

        # Save crop to the exact directory requested.
        save_image(crop_path, crop)
        result["crop_image"] = str(crop_path)

        show_state("READING_OCR")
        # OCR must read the saved crop, not the in-memory crop.
        crop_from_disk = cv2.imread(str(crop_path), cv2.IMREAD_COLOR)
        if crop_from_disk is None:
            raise RuntimeError(f"Cannot read crop image from disk: {crop_path}")
        ocr_result = ocr.recognize(crop_from_disk)
        result.update({
            "status": "RECOGNIZED" if ocr_result.get("valid") else "OCR_FAILED",
            "plate": ocr_result.get("text") or "",
            "raw_text": ocr_result.get("raw_text") or "",
            "ocr_confidence": ocr_result.get("confidence"),
            "preprocess": ocr_result.get("preprocess"),
            "psm": ocr_result.get("psm"),
        })

        if result["bbox"] is not None:
            px1, py1, px2, py2 = result["bbox"]
            annotated = best_frame.copy()
            cv2.rectangle(annotated, (px1, py1), (px2, py2), (0, 255, 0), 2)
            cv2.putText(annotated, result["plate"] or "OCR_FAILED",
                        (px1, max(25, py1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 255, 0), 2)
            save_image(annotated_path, annotated)
            result["annotated_image"] = str(annotated_path)

        return result

    except Exception as exc:
        result["status"] = "ERROR"
        result["reason"] = f"{type(exc).__name__}: {exc}"
        return result
    finally:
        save_json(result_path, result)

def send_plate_result(result):
    if result.get("status") != "RECOGNIZED":
        return

    payload = {
        "plate": result["plate"],
        "confidence": int(round(
            float(result.get("ocr_confidence") or 0)
        )),
    }

    url = f"http://{CAMERA_HOST}/plate-result"

    try:
        with requests.post(
            url,
            json=payload,
            timeout=(2, 3),
        ) as response:
            response.raise_for_status()

        print(
            f"[SEND OK] CAM accepted: "
            f"{payload['plate']} "
            f"confidence={payload['confidence']}"
        )

    except requests.RequestException as error:
        print(
            f"[SEND ERROR] plate={payload['plate']}: {error}"
        )

def main():
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(f"LP model not found: {MODEL_PATH}")

    SAVED_DIR.mkdir(parents=True, exist_ok=True)
    CROP_DIR.mkdir(parents=True, exist_ok=True)

    print(f"[CONFIG] CAMERA_HOST={CAMERA_HOST}")
    print(f"[MODEL]  {MODEL_PATH}")
    print(f"[OUTPUT] {SAVED_DIR} | {CROP_DIR}")

    camera = CameraClient(CAMERA_HOST)
    lp_detector = LPDetector(model_path=str(MODEL_PATH), conf=LP_CONFIDENCE)
    ocr = CharacterDetector()

    connected = camera.connect_stream()
    if connected is False:
        raise RuntimeError("ESP32-CAM stream connection failed")
    print("[CAMERA] Stream connected")

    # Read one real frame before creating an ROI, so it matches its resolution.
    first = None
    deadline = time.monotonic() + 10
    while first is None and time.monotonic() < deadline:
        first = camera.read_frame()
    if first is None:
        raise RuntimeError("Stream connected but no valid frame was received")

    roi = make_roi(first)
    x1, y1, x2, y2 = roi
    print(f"[ROI] x={x1} y={y1} w={x2-x1} h={y2-y1}")

    processor = StreamProcessor(
        roi_x=x1, roi_y=y1, roi_width=x2-x1, roi_height=y2-y1,
        threshold=25, min_changed_pixels=5000, persistence_frames=5
    )
    selector = BestFrameSelector(min_sharpness=0.0, max_candidates=FRAME_COUNT)
    pipeline = CapturePipeline(
        camera=camera, processor=processor, selector=selector,
        candidate_frames=FRAME_COUNT
    )

    state = "WAITING"
    frame_count = 0
    candidates = []
    latched = False
    collecting = False
    latest_result = None
    last_event = ""
    vehicle_locked = False
    current_frame = first

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = current_frame
            current_frame = camera.read_frame()
            if frame is None:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            vehicle_locked, _difference, event = processor.process_frame(frame)
            event_name = getattr(event, "name", str(event) if event else "").upper()
            if event_name and event_name != "NONE":
                last_event = event_name

            if event_name == "VEHICLE_ARRIVED" and not latched and not collecting:
                pipeline.reset()
                pipeline.start_collection()
                collecting = True
                frame_count = 0
                candidates.clear()
                state = "COLLECTING"
                latest_result = None
                print("[VEHICLE] ARRIVED -> collecting exactly 15 frames")

            if collecting:
                candidates.append(frame.copy())
                frame_count += 1
                pipeline.collect_frame(frame)

                if frame_count == FRAME_COUNT:
                    collecting = False
                    latched = True
                    state = "SELECTING"
                    cv2.imshow(WINDOW_NAME, draw_screen(
                        frame, roi, state, frame_count,
                        vehicle_locked, latest_result, last_event
                    ))
                    cv2.waitKey(1)

                    best_frame = pipeline.capture_best_frame()
                    if best_frame is None:
                        best_frame = max(candidates, key=focus_score)
                        print("[BEST] Selector returned None; Laplacian fallback used")
                    sharpness = focus_score(best_frame)
                    print(f"[BEST] selected among {FRAME_COUNT} frames; sharpness={sharpness:.2f}")
                    candidates.clear()

                    def show_state(new_state):
                        nonlocal state
                        state = new_state
                        cv2.imshow(WINDOW_NAME, draw_screen(
                            frame, roi, state, FRAME_COUNT,
                            vehicle_locked, latest_result, last_event
                        ))
                        cv2.waitKey(1)
                        print(f"[STATE] {new_state}")

                    latest_result = process_best(
                        best_frame, lp_detector, ocr, show_state
                    )

                    send_plate_result(latest_result)

                    state = latest_result["status"]
                    print("[RESULT]", json.dumps(
                        latest_result, ensure_ascii=False, indent=2
                    ))

            if event_name == "VEHICLE_LEFT" and latched:
                latched = False
                frame_count = 0
                pipeline.reset()
                processor.reset()
                state = "WAITING"
                print("[VEHICLE] LEFT -> re-armed")

            cv2.imshow(WINDOW_NAME, draw_screen(
                frame, roi, state, frame_count,
                vehicle_locked, latest_result, last_event
            ))
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            if key == ord("r"):
                processor.reset()
                pipeline.reset()
                latched = False
                collecting = False
                frame_count = 0
                candidates.clear()
                state = "WAITING"
                latest_result = None
                print("[MANUAL] Re-armed")
            if key == ord("c") and not collecting:
                pipeline.reset()
                pipeline.start_collection()
                latched = False
                collecting = True
                frame_count = 0
                candidates.clear()
                state = "COLLECTING"
                latest_result = None
                print("[MANUAL] Capture 15 frames")

    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(camera, "close"):
            camera.close()
        cv2.destroyAllWindows()
        print("[EXIT] Test finished")


if __name__ == "__main__":
    main()
