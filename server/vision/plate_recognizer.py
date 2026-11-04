import re
from time import perf_counter

from vision.lp_detector import PlateDetector
from vision.character_detector import CharacterDetector


class PlateRecognizer:
    def __init__(
        self,
        lp_conf=0.25,
        character_conf=0.25,
        min_confidence=0.50,
        plate_pattern=r"[0-9]{2}[A-Z]{1,2}[0-9]{4,6}",
        device="cpu",
        plate_detector=None,
        character_detector=None,
    ):
        if not 0 <= min_confidence <= 1:
            raise ValueError(
                "min_confidence phai nam trong [0, 1]."
            )

        self.min_confidence = min_confidence
        self.pattern = re.compile(plate_pattern)

        # Khởi tạo một lần và tái sử dụng cho mọi ảnh.
        self.plate_detector = (
            plate_detector
            if plate_detector is not None
            else PlateDetector(
                conf=lp_conf,
                device=device,
            )
        )

        self.character_detector = (
            character_detector
            if character_detector is not None
            else CharacterDetector(
                conf=character_conf,
                device=device,
            )
        )

    def recognize(self, frame, roi=None):
        start = perf_counter()

        # Tạo kết quả mới cho mỗi ảnh.
        result = {
            "status": "NO_PLATE",
            "reason": "NO_DETECTION",
            "plate": "",
            "confidence": 0.0,
            "selected_index": None,
            "plates": [],
        }

        try:
            candidates = self.plate_detector.detect_candidates(
                frame,
                roi=roi,
            )

            for crop, bbox, lp_confidence in candidates:
                reading = self.character_detector.read(crop)

                text = reading["text"]

                confidence = min(
                    float(lp_confidence),
                    float(reading["confidence"]),
                )

                if not text:
                    status = "UNREADABLE"
                    reason = "NO_CHARACTERS"

                elif len(reading["lines"]) not in (1, 2):
                    status = "UNREADABLE"
                    reason = "ROW_COUNT"

                elif not self.pattern.fullmatch(text):
                    status = "UNREADABLE"
                    reason = "FORMAT_MISMATCH"

                elif confidence < self.min_confidence:
                    status = "UNREADABLE"
                    reason = "LOW_CONFIDENCE"

                else:
                    status = "OK"
                    reason = "READABLE"

                offset_x, offset_y, _, _ = bbox
                characters = []

                # Chuyển tọa độ ký tự từ ảnh crop về ảnh gốc.
                for item in reading["characters"]:
                    x1, y1, x2, y2 = item["bbox_xyxy"]

                    characters.append({
                        **item,
                        "bbox_frame_xyxy": [
                            offset_x + x1,
                            offset_y + y1,
                            offset_x + x2,
                            offset_y + y2,
                        ],
                    })

                result["plates"].append({
                    "status": status,
                    "reason": reason,
                    "text": text,
                    "lines": reading["lines"],
                    "bbox_xywh": [
                        int(value) for value in bbox
                    ],
                    "lp_confidence": float(lp_confidence),
                    "character_confidence": float(
                        reading["confidence"]
                    ),
                    "confidence": confidence,
                    "characters": characters,
                })

            plates = result["plates"]

            if len(plates) > 1:
                # Chưa có quy tắc chọn biển thuộc làn xe.
                result.update(
                    status="UNREADABLE",
                    reason="MULTIPLE_PLATES",
                )

            elif len(plates) == 1:
                candidate = plates[0]

                result.update(
                    status=candidate["status"],
                    reason=candidate["reason"],
                )

                if candidate["status"] == "OK":
                    result.update(
                        plate=candidate["text"],
                        confidence=candidate["confidence"],
                        selected_index=0,
                    )

        except Exception as error:
            result.update(
                status="ERROR",
                reason="PROCESSING_ERROR",
                plate="",
                confidence=0.0,
                selected_index=None,
                error=str(error),
            )

        result["processing_ms"] = round(
            (perf_counter() - start) * 1000,
            2,
        )

        return result