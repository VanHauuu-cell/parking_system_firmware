import cv2


from camera.camera_client import CameraClient
from config import CAMERA_HOST


def main():

    camera = CameraClient(
        CAMERA_HOST
    )

    try:

        camera.connect_stream()

        print(
            "ESP32-CAM stream connected"
        )

        while True:

            frame = camera.read_frame()

            cv2.imshow(
                "ESP32-CAM Stream",
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

    except Exception as error:

        print(
            "Stream error:",
            error
        )

    finally:

        camera.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
