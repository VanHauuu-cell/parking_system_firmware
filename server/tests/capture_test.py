import cv2

from camera.camera_client import CameraClient
from config import CAMERA_HOST


def main():

    camera = CameraClient(
        CAMERA_HOST
    )

    try:

        print(
            "Sending capture request..."
        )

        frame = camera.capture_frame()

        print(
            "Capture successful"
        )

        print(
            "Frame size:",
            frame.shape[1],
            "x",
            frame.shape[0]
        )

        cv2.imwrite(
            "capture_test.jpg",
            frame
        )

        print(
            "Saved: capture_test.jpg"
        )

        cv2.imshow(
            "Captured Frame",
            frame
        )

        cv2.waitKey(0)

        cv2.destroyAllWindows()

    except Exception as error:

        print(
            "Capture error:",
            error
        )

    finally:

        camera.close()


if __name__ == "__main__":
    main()

