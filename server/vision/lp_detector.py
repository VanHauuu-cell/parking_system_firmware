from math import ceil, floor
from pathlib import Path

import numpy as np
from ultralytics import YOLO


class LPDetector:
    def __init__(
        self,
        model_path=None,
        conf=0.25,
        iou=0.45,
        image_size=640,
        padding=6,
        max_candidates=5,
        device="cpu",
    ):
        server_dir = Path(__file__).resolve().parents[1]

        if model_path is None:
            model_path = server_dir / "models" / "LP_best.pt"

        self.model_path = Path(model_path).resolve()

        if not self.model_path.is_file():
            raise FileNotFoundError(
                f"Khong tim thay model: {self.model_path}"
            )

        if not 0 < conf <= 1:
            raise ValueError("conf phai nam trong khoang (0, 1].")

        if not 0 < iou <= 1:
            raise ValueError("iou phai nam trong khoang (0, 1].")

        if padding < 0:
            raise ValueError("padding khong duoc am.")

        if max_candidates < 1 or image_size < 1:
            raise ValueError(
                "max_candidates va image_size phai lon hon 0."
            )

        self.conf = conf
        self.iou = iou
        self.image_size = image_size
        self.padding = padding
        self.max_candidates = max_candidates
        self.device = device

        # Chỉ tải một lần khi tạo đối tượng.
        self.model = YOLO(str(self.model_path))

        if self.model.task != "detect":
            raise ValueError(
                "Model LP phai thuoc tac vu object detection."
            )

    def detect_candidates(self, frame, roi=None):
        """
        frame:
            Ảnh BGR uint8 có dạng H x W x 3.

        roi:
            None hoặc (x, y, width, height).

        Trả về:
            list[(crop, bbox_xywh, confidence)]

        bbox_xywh tính trên ảnh frame ban đầu.
        Các kết quả được xếp theo confidence giảm dần.
        """
        if (
            not isinstance(frame, np.ndarray)
            or frame.size == 0
            or frame.ndim != 3
            or frame.shape[2] != 3
            or frame.dtype != np.uint8
        ):
            raise ValueError(
                "frame phai la anh BGR uint8 khong rong."
            )

        frame_height, frame_width = frame.shape[:2]

        offset_x = 0
        offset_y = 0
        work = frame

        if roi is not None:
            if len(roi) != 4:
                raise ValueError("ROI phai co 4 gia tri.")

            rx, ry, rw, rh = map(int, roi)

            if rw <= 0 or rh <= 0:
                raise ValueError(
                    "Chieu rong va cao ROI phai lon hon 0."
                )

            # Lấy phần giao giữa ROI và ảnh.
            left = max(0, rx)
            top = max(0, ry)
            right = min(frame_width, rx + rw)
            bottom = min(frame_height, ry + rh)

            if right <= left or bottom <= top:
                raise ValueError("ROI nam ngoai anh.")

            work = frame[top:bottom, left:right]
            offset_x = left
            offset_y = top

        prediction = self.model.predict(
            source=work,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.image_size,
            max_det=self.max_candidates,
            device=self.device,
            verbose=False,
        )[0]

        if prediction.boxes is None or len(prediction.boxes) == 0:
            return []

        boxes = prediction.boxes.xyxy.cpu().numpy()
        scores = prediction.boxes.conf.cpu().numpy()

        work_height, work_width = work.shape[:2]
        candidates = []

        for coordinates, score in zip(boxes, scores):
            x1, y1, x2, y2 = map(float, coordinates)

            if x2 <= x1 or y2 <= y1:
                continue

            # Mở rộng một ít quanh biển và giới hạn trong ảnh.
            left = max(0, floor(x1) - self.padding)
            top = max(0, floor(y1) - self.padding)
            right = min(
                work_width,
                ceil(x2) + self.padding,
            )
            bottom = min(
                work_height,
                ceil(y2) + self.padding,
            )

            if right <= left or bottom <= top:
                continue

            crop = work[top:bottom, left:right].copy()

            bbox = (
                left + offset_x,
                top + offset_y,
                right - left,
                bottom - top,
            )

            candidates.append(
                (crop, bbox, float(score))
            )

        candidates.sort(
            key=lambda item: item[2],
            reverse=True,
        )

        return candidates

    def detect(self, frame, roi=None):
        """
        Lấy ứng viên có confidence cao nhất.

        Trả về:
            (crop, bbox) hoặc (None, None).
        """
        candidates = self.detect_candidates(frame, roi=roi)

        if not candidates:
            return None, None

        crop, bbox, _ = candidates[0]
        return crop, bbox