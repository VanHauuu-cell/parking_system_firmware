import requests
import cv2
import numpy as np


class CameraClient:

    def __init__(
        self,
        camera_host,
        timeout=10
    ):
        self.camera_host = camera_host
        self.timeout = timeout

        self.stream_url = (
            f"http://{camera_host}/stream"
        )

        self.capture_url = (
            f"http://{camera_host}/capture"
        )

        self.stream = None

        # Persistent MJPEG buffer.
        # It must survive between read_frame() calls.
        self.stream_buffer = b""

    def connect_stream(self):

        self.stream = requests.get(
            self.stream_url,
            stream=True,
            timeout=self.timeout
        )

        self.stream.raise_for_status()

        content_type = self.stream.headers.get(
            "Content-Type",
            ""
        )

        if "multipart/x-mixed-replace" not in content_type:
            self.stream.close()
            self.stream = None

            raise RuntimeError(
                "Invalid stream content type"
            )

        self.stream_buffer = b""

        self.stream_iter = self.stream.iter_content(chunk_size=4096)

        return True

    def read_frame(self):

        if self.stream is None:
            raise RuntimeError(
                "Stream is not connected"
            )

        while True:

            # -------------------------------------------------
            # Search for JPEG start marker
            # -------------------------------------------------

            start = self.stream_buffer.find(
                b"\xff\xd8"
            )

            if start != -1:

                # Remove everything before JPEG start.
                if start > 0:
                    self.stream_buffer = (
                        self.stream_buffer[start:]
                    )

                # -------------------------------------------------
                # Search for JPEG end marker
                # -------------------------------------------------

                end = self.stream_buffer.find(
                    b"\xff\xd9",
                    2
                )

                if end != -1:

                    end += 2

                    jpeg_data = (
                        self.stream_buffer[:end]
                    )

                    self.stream_buffer = (
                        self.stream_buffer[end:]
                    )

                    frame = cv2.imdecode(
                        np.frombuffer(
                            jpeg_data,
                            dtype=np.uint8
                        ),
                        cv2.IMREAD_COLOR
                    )

                    if frame is None:
                        continue

                    return frame

            # -------------------------------------------------
            # Need more data from stream
            # -------------------------------------------------

            try:

                chunk = next(self.stream_iter)

            except StopIteration:

                raise RuntimeError(
                    "Stream ended"
                )

            if not chunk:
                continue

            self.stream_buffer += chunk

    def capture_frame(self):

        response = requests.post(
            self.capture_url,
            timeout=self.timeout
        )

        response.raise_for_status()

        content_type = response.headers.get(
            "Content-Type",
            ""
        )

        if "image/jpeg" not in content_type:
            raise RuntimeError(
                "Invalid capture response"
            )

        image_data = np.frombuffer(
            response.content,
            dtype=np.uint8
        )

        frame = cv2.imdecode(
            image_data,
            cv2.IMREAD_COLOR
        )

        if frame is None:
            raise RuntimeError(
                "Failed to decode captured JPEG"
            )

        return frame

    def close(self):

        if self.stream is not None:

            self.stream.close()

            self.stream = None

        self.stream_buffer = b""
