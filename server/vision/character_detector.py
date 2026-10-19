import os
import re

import cv2
import pytesseract


class PlateReader:

    def __init__(
        self,
        tesseract_cmd_path=None,
        scale=2.5,
        use_erosion=False,
        psm= 6
    ):
        if tesseract_cmd_path:
            pytesseract.pytesseract.tesseract_cmd = (
                tesseract_cmd_path
            )
        elif os.name == "nt":
            # Đường dẫn mặc định thường gặp trên Windows.
            default_path = (
                r"C:\Program Files\Tesseract-OCR\tesseract.exe"
            )

            if os.path.isfile(default_path):
                pytesseract.pytesseract.tesseract_cmd = (
                    default_path
                )

        self.scale = scale
        self.use_erosion = use_erosion
        self.psm = psm

    def preprocess_image(self, plate_img):

        if (
            plate_img is None
            or plate_img.size == 0
        ):
            return None

        if len(plate_img.shape) == 2:
            gray = plate_img.copy()
        else:
            gray = cv2.cvtColor(
                plate_img,
                cv2.COLOR_BGR2GRAY
            )

        resized = cv2.resize(
            gray,
            None,
            fx=self.scale,
            fy=self.scale,
            interpolation=cv2.INTER_CUBIC
        )

        # Tăng tương phản cục bộ để hỗ trợ ảnh thiếu sáng.
        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )

        enhanced = clahe.apply(resized)

        blurred = cv2.GaussianBlur(
            enhanced,
            (3, 3),
            0
        )

        _, binary = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        if self.use_erosion:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (2, 2)
            )

            binary = cv2.erode(
                binary,
                kernel,
                iterations=1
            )

        return binary

    def read_text(self, plate_image):

        processed_img = self.preprocess_image(
            plate_image
        )

        if processed_img is None:
            return ""

        custom_config = (
            f"--oem 3 --psm {self.psm} "
            "-c tessedit_char_whitelist="
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
        )

        try:
            raw_text = pytesseract.image_to_string(
                processed_img,
                config=custom_config
            )

        except pytesseract.TesseractNotFoundError as error:
            raise RuntimeError(
                "Không tìm thấy Tesseract OCR. "
                "Hãy kiểm tra đường dẫn tesseract.exe."
            ) from error

        clean_text = re.sub(
            r"[^A-Z0-9]",
            "",
            raw_text.upper()
        )

        return clean_text