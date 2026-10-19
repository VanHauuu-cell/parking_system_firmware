import os
from datetime import datetime

from config import UPLOAD_DIR


def save_image(image_data):
    if not image_data:
        return None

    os.makedirs(UPLOAD_DIR, exist_ok=True)

    filename = (
        datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        + ".jpg"
    )

    filepath = os.path.join(
        UPLOAD_DIR,
        filename
    )

    with open(filepath, "wb") as file:
        file.write(image_data)

    return filepath