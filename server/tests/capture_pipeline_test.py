import cv2
import os
import time
import pytesseract

from camera.camera_client import CameraClient
from config import CAMERA_HOST

from vision.stream_processor import StreamProcessor
from vision.best_frame_selector import BestFrameSelector
from vision.capture_pipeline import CapturePipeline
from vision.plate_detector import PlateDetector
from vision.ocr_reader import PlateReader

def main():

    # -------------------------------------------------
    # Khởi tạo thư mục lưu ảnh
    # -------------------------------------------------
    SAVE_DIR = "saved"
    CROP_DIR = "crop_image"
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

    # Khởi tạo bộ chọn ảnh, pipeline, detector và reader
    selector = BestFrameSelector(min_sharpness=100.0, max_candidates=20)
    
    pipeline = CapturePipeline(
        camera=camera,
        processor=processor,
        selector=selector,
        candidate_frames=20
    )

    detector = PlateDetector()
    reader = PlateReader()    

    try:
        camera.connect_stream()

        print("ESP32-CAM stream connected")
        print("Stream processor started")
        print("Place the camera in an EMPTY scene to initialize the background.")

        while True:
            # 1. ĐỌC FRAME TỪ CAMERA VÀ CHẠY THUẬT TOÁN NHẬN DIỆN XE
            frame = camera.read_frame()
            if frame is None:
                continue

            (
                vehicle_locked,
                difference,
                event
            ) = processor.process_frame(frame)

            display_frame = frame.copy()

            # -------------------------------------------------
            # 2. Xử lý luồng Capture Pipeline, Plate Detection & OCR
            # -------------------------------------------------
            if event == "VEHICLE_ARRIVED":
                print("\nEVENT: VEHICLE_ARRIVED")
                pipeline.start_collection()
                print("Pipeline started collecting frames...")

            elif event == "VEHICLE_LEFT":
                print("\nEVENT: VEHICLE_LEFT")
                pipeline.reset()

            if pipeline.state == pipeline.STATE_COLLECTING:
                is_done = pipeline.collect_frame(frame)
                
                cv2.putText(display_frame, f"Capturing: {selector.candidate_count}/20",
                            (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 100, 100), 2)

                if is_done:
                    best_frame = pipeline.capture_best_frame()
                    if best_frame is not None:
                        timestamp = int(time.time())
                        
                        # Lưu ảnh toàn cảnh
                        os.makedirs("saved_images", exist_ok=True)
                        original_filename = os.path.join("saved_images", f"vehicle_{timestamp}_full.jpg")
                        cv2.imwrite(original_filename, best_frame)
                        
                        # Phát hiện và cắt biển số
                        print("Detecting license plate...")
                        cropped_plate, bbox = detector.detect(best_frame)
                        
                        if cropped_plate is not None:
                            os.makedirs("crop_image", exist_ok=True)
                            plate_filename = os.path.join("crop_image", f"vehicle_{timestamp}_plate.jpg")
                            cv2.imwrite(plate_filename, cropped_plate)
                            print(f"Saved cropped plate to: {plate_filename}")
                            
                            # Đọc chữ trên biển (OCR)
                            print("Reading license plate text...")
                            plate_number = reader.read_text(cropped_plate)
                            print(f">>> KẾT QUẢ BIỂN SỐ: {plate_number} <<<")
                            
                            # Vẽ khung xanh lá quanh biển số
                            x, y, w, h = bbox
                            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                            cv2.putText(display_frame, plate_number, (x, y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 3)
                        else:
                            print("Không tìm thấy khung biển số trong bức ảnh.")

            # -------------------------------------------------
            # 3. Vẽ ROI và Thông tin Trạng thái lên Màn hình
            # -------------------------------------------------
            cv2.rectangle(
                display_frame,
                (processor.roi_x, processor.roi_y),
                (processor.roi_x + processor.roi_width, processor.roi_y + processor.roi_height),
                (0, 255, 0), 2
            )

            if processor.state == processor.STATE_IDLE:
                state_text = "IDLE"
            elif processor.state == processor.STATE_VERIFYING_IN:
                state_text = f"VERIFYING IN {processor.detect_count}/{processor.persistence_frames}"
            elif processor.state == processor.STATE_OCCUPIED:
                state_text = "OCCUPIED"
            elif processor.state == processor.STATE_VERIFYING_OUT:
                state_text = f"VERIFYING OUT {processor.leave_count}/{processor.persistence_frames}"
            else:
                state_text = "UNKNOWN"

            cv2.putText(display_frame, f"State: {state_text}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            presence_text = "VEHICLE PRESENT" if vehicle_locked else "NO VEHICLE"
            cv2.putText(display_frame, presence_text, (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # -------------------------------------------------
            # 4. Hiển thị cửa sổ và Bắt phím điều khiển
            # -------------------------------------------------
            cv2.imshow("ESP32-CAM Parking System", display_frame)

            if difference is not None:
                cv2.imshow("Background Difference", difference)

            key = cv2.waitKey(1) & 0xFF
            
            if key == ord("q"):
                print("Stopping test...")
                break
            elif key == ord("r"):
                processor.reset()
                pipeline.reset()
                print("Processor and Pipeline reset. Background will be initialized again.")

    except Exception as error:
        print("System error:", error)

    finally:
        camera.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()