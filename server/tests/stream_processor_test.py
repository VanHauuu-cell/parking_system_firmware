import cv2

from camera.camera_client import CameraClient
from config import CAMERA_HOST

from vision.stream_processor import StreamProcessor


def main():

    camera = CameraClient(
        CAMERA_HOST
    )

    processor = StreamProcessor(
        roi_x=80,
        roi_y=100,
        roi_width=480,
        roi_height=280,
        threshold=25,
        min_changed_pixels=5000,
        persistence_frames=5
    )

    try:

        camera.connect_stream()

        print(
            "ESP32-CAM stream connected"
        )

        print(
            "Stream processor started"
        )

        print(
            "Place the camera in an EMPTY scene "
            "to initialize the background."
        )

        while True:

            frame = camera.read_frame()

            (
                vehicle_locked,
                difference,
                event
            ) = processor.process_frame(
                frame
            )

            display_frame = frame.copy()

            # -------------------------------------------------
            # Draw ROI
            # -------------------------------------------------

            cv2.rectangle(
                display_frame,
                (
                    processor.roi_x,
                    processor.roi_y
                ),
                (
                    processor.roi_x +
                    processor.roi_width,
                    processor.roi_y +
                    processor.roi_height
                ),
                (0, 255, 0),
                2
            )

            # -------------------------------------------------
            # Display state
            # -------------------------------------------------

            if (
                processor.state ==
                processor.STATE_IDLE
            ):

                state_text = "IDLE"

            elif (
                processor.state ==
                processor.STATE_VERIFYING_IN
            ):

                state_text = (
                    "VERIFYING IN "
                    f"{processor.detect_count}/"
                    f"{processor.persistence_frames}"
                )

            elif (
                processor.state ==
                processor.STATE_OCCUPIED
            ):

                state_text = "OCCUPIED"

            elif (
                processor.state ==
                processor.STATE_VERIFYING_OUT
            ):

                state_text = (
                    "VERIFYING OUT "
                    f"{processor.leave_count}/"
                    f"{processor.persistence_frames}"
                )

            else:

                state_text = "UNKNOWN"

            cv2.putText(
                display_frame,
                f"State: {state_text}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            # -------------------------------------------------
            # Display presence
            # -------------------------------------------------

            if vehicle_locked:

                presence_text = "VEHICLE PRESENT"

            else:

                presence_text = "NO VEHICLE"

            cv2.putText(
                display_frame,
                presence_text,
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )

            # -------------------------------------------------
            # Print events
            # -------------------------------------------------

            if event is not None:

                print(
                    "EVENT:",
                    event
                )

            # -------------------------------------------------
            # Show camera frame
            # -------------------------------------------------

            cv2.imshow(
                "ESP32-CAM Stream Processor",
                display_frame
            )

            # -------------------------------------------------
            # Show background difference
            # -------------------------------------------------

            if difference is not None:

                cv2.imshow(
                    "Background Difference",
                    difference
                )

            # -------------------------------------------------
            # Keyboard control
            # -------------------------------------------------

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):

                print(
                    "Stopping test..."
                )

                break

            elif key == ord("r"):

                processor.reset()

                print(
                    "Processor reset"
                )

                print(
                    "Background will be "
                    "initialized again."
                )

    except Exception as error:

        print(
            "Stream processor error:",
            error
        )

    finally:

        camera.close()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()