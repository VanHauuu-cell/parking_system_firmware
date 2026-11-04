import logging
import threading
import time

import cv2
import numpy as np
import requests


logger = logging.getLogger(__name__)


class CameraClient:
    MAX_BUFFER_BYTES = 2 * 1024 * 1024

    def __init__(self, camera_host, timeout=(3, 3)):
        self.camera_host = camera_host
        self.stream_url = f"http://{camera_host}/stream"
        self.capture_url = f"http://{camera_host}/capture"
        self.timeout = timeout

        self.stream = None
        self.stream_iter = None
        self.stream_buffer = b""

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

        self._frame = None
        self._sequence = 0
        self._connection = 0
        self._received_at = 0.0
        self._error = "Connecting"

    def _running(self):
        return (
            self._thread is not None
            and self._thread.is_alive()
        )

    # Dùng cho các bài test đọc stream đồng bộ trước đây.
    def connect_stream(self):
        if self._running():
            raise RuntimeError(
                "Use snapshot() while background mode is running"
            )

        self._stop.clear()
        self._connect_stream()
        return True

    def read_frame(self):
        if self._running():
            raise RuntimeError(
                "Use snapshot() while background mode is running"
            )

        return self._read_frame()

    # Dùng cho app.py: đọc stream bằng luồng riêng.
    def start(self):
        if self._running():
            return

        if self.stream is not None:
            raise RuntimeError(
                "Close the synchronous stream before start()"
            )

        self._stop.clear()

        with self._lock:
            self._frame = None
            self._error = "Connecting"

        self._thread = threading.Thread(
            target=self._worker,
            name="esp32-camera",
            daemon=True,
        )
        self._thread.start()

    def snapshot(self):
        """
        Trả về:
            frame, sequence, connection, received_at, error

        Không vẽ trực tiếp lên frame trả về.
        Dùng frame.copy() trước khi vẽ.
        """
        with self._lock:
            return (
                self._frame,
                self._sequence,
                self._connection,
                self._received_at,
                self._error,
            )

    def _connect_stream(self):
        self._close_stream()

        response = requests.get(
            self.stream_url,
            stream=True,
            timeout=self.timeout,
        )

        try:
            response.raise_for_status()

            content_type = response.headers.get(
                "Content-Type", ""
            ).lower()

            if "multipart/x-mixed-replace" not in content_type:
                raise RuntimeError(
                    f"Invalid stream type: {content_type}"
                )

        except Exception:
            response.close()
            raise

        self.stream = response
        self.stream_iter = response.iter_content(chunk_size=4096)
        self.stream_buffer = b""

        with self._lock:
            self._connection += 1

    def _read_frame(self):
        if self.stream is None:
            raise RuntimeError("Stream is not connected")

        deadline = time.monotonic() + 6.0

        while not self._stop.is_set():
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "No valid JPEG received for 6 seconds"
                )

            start = self.stream_buffer.find(b"\xff\xd8")

            if start < 0:
                # Giữ byte cuối vì marker có thể nằm giữa hai chunk.
                self.stream_buffer = self.stream_buffer[-1:]

            else:
                self.stream_buffer = self.stream_buffer[start:]
                end = self.stream_buffer.find(b"\xff\xd9", 2)

                if end >= 0:
                    jpeg = self.stream_buffer[:end + 2]
                    self.stream_buffer = self.stream_buffer[end + 2:]

                    frame = cv2.imdecode(
                        np.frombuffer(jpeg, dtype=np.uint8),
                        cv2.IMREAD_COLOR,
                    )

                    if frame is not None:
                        return frame

                    continue

            try:
                chunk = next(self.stream_iter)
            except StopIteration:
                raise RuntimeError("Stream ended") from None

            if chunk:
                self.stream_buffer += chunk

                if len(self.stream_buffer) > self.MAX_BUFFER_BYTES:
                    raise RuntimeError(
                        "JPEG buffer exceeded size limit"
                    )

        raise RuntimeError("Camera stopped")

    def _worker(self):
        while not self._stop.is_set():
            try:
                self._connect_stream()
                logger.info(
                    "Stream connected: %s",
                    self.stream_url,
                )

                while not self._stop.is_set():
                    frame = self._read_frame()

                    with self._lock:
                        self._frame = frame
                        self._sequence += 1
                        self._received_at = time.monotonic()
                        self._error = ""

            except Exception as error:
                with self._lock:
                    self._frame = None
                    self._error = str(error)

                if not self._stop.is_set():
                    logger.warning(
                        "Stream unavailable: %s",
                        error,
                    )

            finally:
                self._close_stream()

                with self._lock:
                    self._frame = None

            # Chờ trước khi kết nối lại; có thể dừng ngay khi close().
            self._stop.wait(2)

    def capture_frame(self):
        with requests.post(
            self.capture_url,
            timeout=self.timeout,
        ) as response:
            response.raise_for_status()

            content_type = response.headers.get(
                "Content-Type", ""
            ).lower()

            if "image/jpeg" not in content_type:
                raise RuntimeError("Invalid capture response")

            frame = cv2.imdecode(
                np.frombuffer(response.content, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )

            if frame is None:
                raise RuntimeError(
                    "Failed to decode captured JPEG"
                )

            return frame

    def _close_stream(self):
        if self.stream is not None:
            self.stream.close()

        self.stream = None
        self.stream_iter = None
        self.stream_buffer = b""

    def close(self):
        self._stop.set()

        if self._running():
            # Luồng đọc tự đóng kết nối HTTP của nó.
            self._thread.join(timeout=8)

            if self._running():
                logger.warning("Camera thread is still stopping")
        else:
            self._close_stream()

        with self._lock:
            self._frame = None