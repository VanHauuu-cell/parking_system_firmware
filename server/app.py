import logging
import os
import time

import cv2

from camera.camera_client import CameraClient
from config import CAMERA_HOST

import re
PLATE_RE = re.compile(r"^\d{2}[A-Z]{1,2}\d{4,6}$")

from vision.stream_processor import StreamProcessor
from vision.best_frame_selector import BestFrameSelector
from vision.capture_pipeline import CapturePipeline
from vision.plate_detector import PlateDetector
from vision.ocr_reader import PlateReader


SAVE_DIR = "saved_images"
CROP_DIR = "crop_image"

WINDOW_MAIN = "ESP32-CAM Parking System"
WINDOW_BEST = "Best Vehicle Frame"
WINDOW_CROP = "Cropped Plate"
WINDOW_DIFF = "Background Difference"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)

logger = logging.getLogger(__name__)


def save_image(directory, filename, image):
    """Lưu ảnh và kiểm tra kết quả."""

    if image is None or image.size == 0:
        logger.warning("Image is empty: %s", filename)
        return None

    os.makedirs(directory, exist_ok=True)

    path = os.path.join(directory, filename)

    if not cv2.imwrite(path, image):
        logger.error("Failed to save image: %s", path)
        return None

    logger.info("Saved image: %s", path)
    return path


def process_vehicle_image(best_frame, detector, reader, roi=None):
    timestamp = int(time.time() * 1000)
    original_path = save_image(
        SAVE_DIR, f"vehicle_{timestamp}_full.jpg", best_frame
    )

    candidates = detector.detect_candidates(best_frame, roi=roi)

    empty = {
        "original_path": original_path, "crop_path": None,
        "plate_image": None, "bbox": None, "plate_text": ""
    }
    if not candidates:
        logger.warning("No license plate candidate detected.")
        return empty

    chosen = None
    for crop, bbox, score in candidates:
        text = reader.read_text(crop)
        logger.info("Candidate bbox=%s OCR=%r", bbox, text)
        if PLATE_RE.match(text):          # khớp định dạng -> chốt luôn
            chosen = (crop, bbox, text)
            break
        if chosen is None:                # dự phòng: ứng viên điểm cao nhất
            chosen = (crop, bbox, text)

    crop, bbox, text = chosen
    crop_path = save_image(CROP_DIR, f"vehicle_{timestamp}_plate.jpg", crop)

    return {
        "original_path": original_path, "crop_path": crop_path,
        "plate_image": crop.copy(), "bbox": bbox, "plate_text": text
    }

def draw_system_status(
    frame,
    processor,
    vehicle_locked,
    collecting
):
    """Vẽ trạng thái hệ thống lên khung stream."""

    display = frame.copy()

    x = processor.roi_x
    y = processor.roi_y
    width = processor.roi_width
    height = processor.roi_height

    cv2.rectangle(
        display,
        (x, y),
        (x + width, y + height),
        (0, 255, 0),
        2
    )

    if processor.state == processor.STATE_IDLE:
        state_text = "IDLE"

    elif processor.state == processor.STATE_VERIFYING_IN:
        state_text = (
            f"VERIFYING IN "
            f"{processor.detect_count}/"
            f"{processor.persistence_frames}"
        )

    elif processor.state == processor.STATE_OCCUPIED:
        state_text = "OCCUPIED"

    elif processor.state == processor.STATE_VERIFYING_OUT:
        state_text = (
            f"VERIFYING OUT "
            f"{processor.leave_count}/"
            f"{processor.persistence_frames}"
        )

    else:
        state_text = "UNKNOWN"

    cv2.putText(
        display,
        f"State: {state_text}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    presence_text = (
        "VEHICLE PRESENT"
        if vehicle_locked
        else "NO VEHICLE"
    )

    cv2.putText(
        display,
        presence_text,
        (20, 75),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    if collecting:
        cv2.putText(
            display,
            "CAPTURING BEST FRAME",
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 100, 100),
            2
        )

    return display


def main():

    os.makedirs(SAVE_DIR, exist_ok=True)
    os.makedirs(CROP_DIR, exist_ok=True)

    camera = CameraClient(CAMERA_HOST)

    processor = StreamProcessor(
        roi_x=80,
        roi_y=100,
        roi_width=480,
        roi_height=280,
        threshold=25,
        min_changed_pixels=5000,
        persistence_frames=5
    )

    selector = BestFrameSelector(
        min_sharpness=100.0,
        max_candidates=20
    )

    pipeline = CapturePipeline(
        camera=camera,
        processor=processor,
        selector=selector,
        candidate_frames=20
    )

    detector = PlateDetector(
        min_area=4000
    )
    reader = PlateReader()

    current_plate_image = None
    current_plate_text = ""
    current_best_display = None

    roi = (processor.roi_x, processor.roi_y,
       processor.roi_width, processor.roi_height)
    result = process_vehicle_image(best_frame, detector, reader, roi=roi)

    try:
        camera.connect_stream()

        logger.info("ESP32-CAM stream connected")
        logger.info("Stream processor started")
        logger.info(
            "Place camera in an EMPTY scene "
            "to initialize background."
        )

        while True:

            frame = camera.read_frame()

            if frame is None:
                continue

            (
                vehicle_locked,
                difference,
                event
            ) = processor.process_frame(frame)

            # ----------------------------------------
            # 1. Xử lý sự kiện xe đến
            # ----------------------------------------
            if event == "VEHICLE_ARRIVED":

                logger.info("EVENT: VEHICLE_ARRIVED")

                # Xóa kết quả của lượt xe trước.
                current_plate_image = None
                current_plate_text = ""
                current_best_display = None

                pipeline.start_collection()

                logger.info(
                    "Started best-frame collection"
                )

            # ----------------------------------------
            # 2. Thu thập các frame ứng viên
            # ----------------------------------------
            if pipeline.state == pipeline.STATE_COLLECTING:

                is_done = pipeline.collect_frame(frame)

                if is_done:

                    best_frame = (
                        pipeline.capture_best_frame()
                    )

                    if best_frame is not None:

                        result = process_vehicle_image(
                            best_frame,
                            detector,
                            reader
                        )

                        current_plate_image = (
                            result["plate_image"]
                        )

                        current_plate_text = (
                            result["plate_text"]
                        )

                        # Hiển thị bbox trên ĐÚNG ảnh đã chọn.
                        current_best_display = (
                            best_frame.copy()
                        )

                        bbox = result["bbox"]

                        if bbox is not None:
                            x, y, w, h = bbox

                            cv2.rectangle(
                                current_best_display,
                                (x, y),
                                (x + w, y + h),
                                (0, 255, 0),
                                3
                            )

                            label = (
                                current_plate_text
                                if current_plate_text
                                else "PLATE CANDIDATE"
                            )

                            cv2.putText(
                                current_best_display,
                                label,
                                (x, max(y - 10, 25)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 255, 0),
                                2
                            )

            # ----------------------------------------
            # 3. Hiển thị kết quả OCR trên stream
            # ----------------------------------------
            display = draw_system_status(
                frame,
                processor,
                vehicle_locked,
                pipeline.state == pipeline.STATE_COLLECTING
            )

            if current_plate_text:
                cv2.putText(
                    display,
                    f"Plate: {current_plate_text}",
                    (20, 145),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            cv2.imshow(WINDOW_MAIN, display)

            if current_best_display is not None:
                cv2.imshow(
                    WINDOW_BEST,
                    current_best_display
                )

            if current_plate_image is not None:

                crop_display = cv2.resize(
                    current_plate_image,
                    None,
                    fx=2.0,
                    fy=2.0,
                    interpolation=cv2.INTER_CUBIC
                )

                cv2.imshow(
                    WINDOW_CROP,
                    crop_display
                )

            if difference is not None:
                cv2.imshow(
                    WINDOW_DIFF,
                    difference
                )

            # ----------------------------------------
            # 4. Bàn phím điều khiển
            # ----------------------------------------
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                logger.info("Stopping application")
                break

            elif key == ord("r"):

                processor.reset()
                pipeline.reset()

                current_plate_image = None
                current_plate_text = ""
                current_best_display = None

                for window in (
                    WINDOW_BEST,
                    WINDOW_CROP
                ):
                    try:
                        cv2.destroyWindow(window)
                    except cv2.error:
                        pass

                logger.info(
                    "Processor and pipeline reset. "
                    "Reinitialize background."
                )

            # ----------------------------------------
            # 5. Xử lý sự kiện xe rời đi
            # ----------------------------------------
            if event == "VEHICLE_LEFT":

                logger.info("EVENT: VEHICLE_LEFT")

                pipeline.reset()

                current_plate_image = None
                current_plate_text = ""
                current_best_display = None

                for window in (
                    WINDOW_BEST,
                    WINDOW_CROP
                ):
                    try:
                        cv2.destroyWindow(window)
                    except cv2.error:
                        pass

    except KeyboardInterrupt:
        logger.info("Application interrupted")

    except Exception:
        logger.exception("Parking vision application failed")

    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()