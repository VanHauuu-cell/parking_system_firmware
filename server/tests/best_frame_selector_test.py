import cv2

from camera.camera_client import CameraClient
from config import CAMERA_HOST

from vision.best_frame_selector import (
    BestFrameSelector
)


def main():

    camera = CameraClient(
        CAMERA_HOST
    )

    selector = BestFrameSelector(
        min_sharpness=100.0,
        max_candidates=20
    )

    try:

        camera.connect_stream()

        print(
            "ESP32-CAM stream connected"
        )

        print(
            "Collecting candidate frames..."
        )

        while not selector.is_ready():

            frame = camera.read_frame()

            updated = selector.add_frame(
                frame
            )

            score = (
                selector.calculate_sharpness(
                    frame
                )
            )

            print(
                f"Candidate "
                f"{selector.candidate_count}/"
                f"{selector.max_candidates} | "
                f"Sharpness: {score:.2f}"
            )

            display_frame = frame.copy()

            cv2.putText(
                display_frame,
                (
                    f"Sharpness: "
                    f"{score:.2f}"
                ),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                display_frame,
                (
                    f"Best: "
                    f"{selector.best_score:.2f}"
                ),
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            cv2.imshow(
                "Candidate Frames",
                display_frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                return

        best_frame = (
            selector.get_best_frame()
        )

        if best_frame is None:

            print(
                "No suitable frame found"
            )

            return

        print()
        print(
            "Best frame selected"
        )

        print(
            f"Best sharpness: "
            f"{selector.get_best_score():.2f}"
        )

        cv2.imshow(
            "BEST FRAME",
            best_frame
        )

        cv2.imwrite(
            "best_frame_test.jpg",
            best_frame
        )

        print(
            "Saved: best_frame_test.jpg"
        )

        print(
            "Press any key to finish."
        )

        cv2.waitKey(0)

    except Exception as error:

        print(
            "Best frame selector error:",
            error
        )

    finally:

        camera.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()