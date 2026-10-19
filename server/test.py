
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


BASE_DIR = Path(__file__).resolve().parent

MODEL_PATH = BASE_DIR / "models" / "Letters_detection.pt"
INPUT_DIR = BASE_DIR / "crop_image"
OUTPUT_DIR = BASE_DIR / "runs" / "letters_test"

CONF_THRESHOLD = 0.25
IMAGE_SIZE = 640


def sort_characters(detections):
    """
    Sắp xếp ký tự từ trái sang phải.
    Hỗ trợ biển số 1 dòng và 2 dòng.
    """
    if not detections:
        return ""

    heights = [
        item["y2"] - item["y1"]
        for item in detections
    ]

    median_height = float(np.median(heights))
    row_threshold = max(1.0, median_height * 0.65)

    # Sắp xếp theo tọa độ Y
    sorted_by_y = sorted(
        detections,
        key=lambda item: item["cy"]
    )

    rows = []

    for item in sorted_by_y:
        matched_row = None

        for row in rows:
            row_center = np.mean(
                [char["cy"] for char in row]
            )

            if abs(item["cy"] - row_center) < row_threshold:
                matched_row = row
                break

        if matched_row is not None:
            matched_row.append(item)
        else:
            rows.append([item])

    # Sắp xếp các hàng từ trên xuống
    rows.sort(
        key=lambda row: np.mean(
            [char["cy"] for char in row]
        )
    )

    result = []

    for row in rows:
        # Mỗi hàng: trái sang phải
        row.sort(key=lambda item: item["cx"])

        text = "".join(
            item["char"] for item in row
        )

        result.append(text)

    return "-".join(result)


def main():
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Model not found: {MODEL_PATH}"
        )

    if not INPUT_DIR.is_dir():
        raise FileNotFoundError(
            f"Input directory not found: {INPUT_DIR}"
        )

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(MODEL_PATH))

    extensions = {".jpg", ".jpeg", ".png", ".bmp"}

    image_paths = sorted(
        path for path in INPUT_DIR.iterdir()
        if path.is_file()
        and path.suffix.lower() in extensions
    )

    if not image_paths:
        print("No images found in crop_image/")
        return

    for image_path in image_paths:

        print(f"\nProcessing: {image_path.name}")

        results = model.predict(
            source=str(image_path),
            imgsz=IMAGE_SIZE,
            conf=CONF_THRESHOLD,
            device="cpu",
            verbose=False
        )

        result = results[0]
        detections = []

        if result.boxes is not None:

            for box in result.boxes:

                cls_id = int(box.cls.item())
                confidence = float(box.conf.item())

                x1, y1, x2, y2 = (
                    box.xyxy[0].cpu().numpy().tolist()
                )

                character = str(
                    model.names[cls_id]
                )

                detections.append({
                    "char": character,
                    "conf": confidence,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "cx": (x1 + x2) / 2,
                    "cy": (y1 + y2) / 2
                })

        plate_text = sort_characters(detections)

        print(f"Detected characters: {len(detections)}")
        print(f"Plate: {plate_text or 'NOT DETECTED'}")

        for item in detections:
            print(
                f"  {item['char']}: "
                f"{item['conf']:.2f}"
            )

        annotated = result.plot()

        cv2.putText(
            annotated,
            f"Plate: {plate_text}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        output_path = OUTPUT_DIR / image_path.name

        cv2.imwrite(
            str(output_path),
            annotated
        )

        print(f"Saved: {output_path}")

    print("\nLetter Detection Test Completed!")


if __name__ == "__main__":
    main()
