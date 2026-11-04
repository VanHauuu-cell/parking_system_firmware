"""PaddleOCR 3.3.1 adapter for the existing parking application.

Input: a BGR plate crop. Output: the same dictionary contract as before.
Model initialization happens once, in the application's OCR worker.
"""
import re

import cv2
import numpy as np
from paddleocr import PaddleOCR


class CharacterDetector:
    def __init__(self, min_confidence=75.0, min_chars=8, max_chars=9,
                 expected_rows=2):
        self.min_confidence = float(min_confidence)
        self.min_chars = int(min_chars)
        self.max_chars = int(max_chars)
        self.expected_rows = int(expected_rows)
        if not (0 <= self.min_confidence <= 100 and
                1 <= self.min_chars <= self.max_chars <= 19 and
                self.expected_rows in (1, 2)):
            raise ValueError("Invalid OCR acceptance settings")

        self.engine = PaddleOCR(
            text_detection_model_name="PP-OCRv5_mobile_det",
            text_recognition_model_name="en_PP-OCRv5_mobile_rec",
            use_doc_orientation_classify=False,
            use_doc_unwarping=False,
            use_textline_orientation=False,
            device="cpu",
            cpu_threads=2,
            enable_mkldnn=False,
            text_det_limit_side_len=640,
            text_det_limit_type="max",
            text_rec_score_thresh=0.0,
        )

    @staticmethod
    def _prepare(image):
        if (not isinstance(image, np.ndarray) or image.size == 0 or
                image.ndim != 3 or image.shape[2] != 3 or
                image.dtype != np.uint8):
            raise ValueError("OCR needs a nonempty BGR uint8 plate crop")
        # Preserve colour and strokes. PaddleOCR performs its own preprocessing.
        scale = max(1.0, min(3.0, 160.0 / image.shape[0]))
        if scale > 1.0:
            image = cv2.resize(image, None, fx=scale, fy=scale,
                               interpolation=cv2.INTER_CUBIC)
        return cv2.copyMakeBorder(image, 8, 8, 8, 8,
                                  cv2.BORDER_CONSTANT, value=(255, 255, 255))

    @staticmethod
    def _group_rows(segments):
        """Group boxes by height, then read each row from left to right."""
        if not segments:
            return []
        tolerance = max(4.0, 0.60 * float(np.median([
            part["height"] for part in segments
        ])))
        rows = []
        for part in sorted(segments, key=lambda item: item["center_y"]):
            nearest = min(rows, key=lambda row: abs(row["y"] - part["center_y"]),
                          default=None)
            if nearest is None or abs(nearest["y"] - part["center_y"]) > tolerance:
                rows.append({"y": part["center_y"], "parts": [part]})
            else:
                nearest["parts"].append(part)
                nearest["y"] = float(np.mean([
                    item["center_y"] for item in nearest["parts"]
                ]))
        rows.sort(key=lambda row: row["y"])
        return [sorted(row["parts"], key=lambda part: part["left"])
                for row in rows]

    def recognize(self, plate_image):
        image = self._prepare(plate_image)
        results = self.engine.predict(input=image)
        if len(results) != 1:
            raise RuntimeError("Expected one PaddleOCR result for one crop")
        result = results[0]
        texts = result["rec_texts"]
        scores = result["rec_scores"]
        polygons = result["rec_polys"]
        if not (len(texts) == len(scores) == len(polygons)):
            raise RuntimeError("Unexpected PaddleOCR result structure")

        segments = []
        for raw, raw_score, polygon in zip(texts, scores, polygons):
            raw = str(raw).strip()
            text = re.sub(r"[^A-Z0-9]", "", raw.upper())
            if not text:
                continue  # Standalone punctuation is not part of the UART plate.
            points = np.asarray(polygon, dtype=np.float64).reshape(-1, 2)
            score = float(raw_score)
            if (len(points) < 4 or not np.isfinite(points).all() or
                    not np.isfinite(score) or not 0.0 <= score <= 1.0):
                raise RuntimeError("Invalid PaddleOCR coordinates or score")
            segments.append({
                "raw_text": raw, "text": text,
                "confidence": round(score * 100.0, 2),
                "score": score * 100.0,
                "polygon": points.tolist(),
                "left": float(points[:, 0].min()),
                "center_y": float(points[:, 1].mean()),
                "height": max(1.0, float(np.ptp(points[:, 1]))),
            })

        rows = self._group_rows(segments)
        lines = ["".join(part["text"] for part in row) for row in rows]
        raw_lines = [" ".join(part["raw_text"] for part in row) for row in rows]
        line_scores = [min(part["score"] for part in row) for row in rows]
        text = "".join(lines)
        confidence = min(line_scores, default=0.0)

        # Keep low-confidence alphanumeric tokens: dropping them could turn a
        # partially recognized plate into a falsely accepted shorter plate.
        if not text:
            reason = "NO_TEXT"
        elif len(rows) != self.expected_rows:
            reason = f"EXPECTED_{self.expected_rows}_ROWS_FOUND_{len(rows)}"
        elif not self.min_chars <= len(text) <= self.max_chars:
            reason = "INVALID_LENGTH"
        elif confidence < self.min_confidence:
            reason = "LOW_CONFIDENCE"
        else:
            reason = "READABLE"
        valid = reason == "READABLE"
        return {
            "text": text if valid else "",
            "raw_text": " / ".join(raw_lines),
            "confidence": round(confidence, 2),
            "preprocess": "paddle_bgr", "psm": None,
            "valid": valid, "engine": "PaddleOCR",
            "reason": reason, "lines": lines,
            "line_scores": [round(value, 2) for value in line_scores],
            "segments": [{key: part[key] for key in
                          ("raw_text", "text", "confidence", "polygon")}
                         for row in rows for part in row],
        }

    def read_text(self, plate_image):
        return self.recognize(plate_image)["text"]

    def detect(self, plate_image):
        return self.recognize(plate_image)
