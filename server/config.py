import os

SERVER_HOST = "0.0.0.0"
SERVER_PORT = 5000

# Run one app.py process per camera, with a different environment value.
CAMERA_HOST = os.environ.get("PARKING_CAMERA_HOST", "").strip() 

UPLOAD_DIR = "received_images"

MAX_IMAGE_SIZE = 500 * 1024
