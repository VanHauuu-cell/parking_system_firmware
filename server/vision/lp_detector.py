import cv2
import numpy as np


class PlateDetector:

    def __init__(
        self,
        min_area=4000,
        max_area_ratio=0.80,       # so với diện tích ROI
        min_aspect_ratio=1.0,
        max_aspect_ratio=6.0,
        min_fill=0.55,             # contour_area / bbox_area
        min_ink=0.06,              # tỉ lệ pixel "chữ" tối thiểu
        max_ink=0.48,
        padding=4,
        max_candidates=5,
    ):
        self.min_area = min_area
        self.max_area_ratio = max_area_ratio
        self.min_aspect_ratio = min_aspect_ratio
        self.max_aspect_ratio = max_aspect_ratio
        self.min_fill = min_fill
        self.min_ink = min_ink
        self.max_ink = max_ink
        self.padding = padding
        self.max_candidates = max_candidates

    # ---------------------------------------------------------
    def _masks(self, gray):
        gray = cv2.bilateralFilter(gray, 9, 50, 50)
        gray = cv2.createCLAHE(2.0, (8, 8)).apply(gray)

        # A: cạnh -> đóng để cả viền biển + chữ thành một khối
        edges = cv2.Canny(gray, 50, 150)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (17, 9))
        edges_closed = cv2.morphologyEx(
            edges, cv2.MORPH_CLOSE, k, iterations=2
        )

        # B: Otsu cả hai cực tính (biển trắng và biển vàng/tối)
        _, otsu = cv2.threshold(
            gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        k2 = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        otsu_closed = cv2.morphologyEx(otsu, cv2.MORPH_CLOSE, k2)
        otsu_inv = cv2.morphologyEx(
            cv2.bitwise_not(otsu), cv2.MORPH_CLOSE, k2
        )

        return gray, (edges_closed, otsu_closed, otsu_inv)

    @staticmethod
    def _iou(a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ix = max(0, min(ax + aw, bx + bw) - max(ax, bx))
        iy = max(0, min(ay + ah, by + bh) - max(ay, by))
        inter = ix * iy
        union = aw * ah + bw * bh - inter
        return inter / float(union) if union else 0.0

    # ---------------------------------------------------------
    def detect_candidates(self, frame, roi=None):
        """
        Trả về list[(crop, bbox, score)] sắp xếp giảm dần theo điểm.
        bbox theo tọa độ KHUNG GỐC.
        """
        if frame is None or frame.size == 0:
            return []

        fh, fw = frame.shape[:2]
        ox = oy = 0
        work = frame

        if roi is not None:
            rx, ry, rw, rh = roi
            rx, ry = max(0, rx), max(0, ry)
            rw, rh = min(rw, fw - rx), min(rh, fh - ry)
            if rw <= 0 or rh <= 0:
                return []
            work = frame[ry:ry + rh, rx:rx + rw]
            ox, oy = rx, ry

        wh, ww = work.shape[:2]
        roi_area = ww * wh

        gray0 = (work if work.ndim == 2
                 else cv2.cvtColor(work, cv2.COLOR_BGR2GRAY))
        gray, masks = self._masks(gray0)

        cands = []
        for mask in masks:
            # EXTERNAL: bỏ contour con (mảnh chữ bên trong biển)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                if w <= 0 or h <= 0:
                    continue
                area = w * h
                ratio = w / float(h)

                if area < self.min_area or area > roi_area * self.max_area_ratio:
                    continue
                if not (self.min_aspect_ratio <= ratio <= self.max_aspect_ratio):
                    continue

                fill = cv2.contourArea(c) / float(area)
                if fill < self.min_fill:
                    continue

                # Biển có nền đồng nhất + chữ là thiểu số, bất kể màu
                patch = gray[y:y + h, x:x + w]
                _, b = cv2.threshold(
                    patch, 0, 255,
                    cv2.THRESH_BINARY + cv2.THRESH_OTSU
                )
                white = cv2.countNonZero(b) / float(area)
                ink = min(white, 1.0 - white)
                if not (self.min_ink <= ink <= self.max_ink):
                    continue

                # Hai cụm tỉ lệ: ~1.3 (2 dòng) và ~4.5 (1 dòng)
                ratio_score = max(
                    1.0 - abs(ratio - 1.35) / 1.0,
                    1.0 - abs(ratio - 4.5) / 2.0,
                    0.0,
                )

                score = (
                    0.40 * min(area / (roi_area * 0.25), 1.0)
                    + 0.25 * fill
                    + 0.20 * ratio_score
                    + 0.15 * min(ink / 0.25, 1.0)
                )
                cands.append((score, (x, y, w, h), ratio, fill, ink))

        if not cands:
            print("[PLATE] No candidate in ROI")
            return []

        cands.sort(key=lambda t: t[0], reverse=True)

        # Loại ứng viên trùng nhau
        kept = []
        for item in cands:
            if all(self._iou(item[1], k[1]) < 0.5 for k in kept):
                kept.append(item)
            if len(kept) >= self.max_candidates:
                break

        results = []
        for i, (s, (x, y, w, h), r, f, ink) in enumerate(kept):
            print(f"[PLATE] #{i+1}: bbox={(x+ox, y+oy, w, h)} "
                  f"ratio={r:.2f} fill={f:.2f} ink={ink:.2f} score={s:.3f}")

            left = max(0, x - self.padding)
            top = max(0, y - self.padding)
            right = min(ww, x + w + self.padding)
            bottom = min(wh, y + h + self.padding)

            crop = work[top:bottom, left:right].copy()
            if crop.size == 0:
                continue
            bbox = (left + ox, top + oy, right - left, bottom - top)
            results.append((crop, bbox, s))

        return results

    # Giữ tương thích với code cũ
    def detect(self, frame, roi=None):
        res = self.detect_candidates(frame, roi)
        if not res:
            return None, None
        return res[0][0], res[0][1]