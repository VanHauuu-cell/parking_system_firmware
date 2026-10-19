class CapturePipeline:

    STATE_IDLE = 0
    STATE_COLLECTING = 1
    STATE_COMPLETED = 2

    def __init__(
        self,
        camera,
        processor,
        selector,
        candidate_frames=10
    ):
        self.camera = camera
        self.processor = processor
        self.selector = selector

        self.candidate_frames = candidate_frames

        self.state = self.STATE_IDLE

    def start_collection(self):

        self.selector.reset()

        self.state = self.STATE_COLLECTING

    def collect_frame(self, frame):

        if self.state != self.STATE_COLLECTING:
            return False

        # Thêm frame vào bộ chọn lọc độ nét
        self.selector.add_frame(frame)

        # Kiểm tra xem đã gom đủ số lượng frame yêu cầu chưa
        if self.selector.candidate_count >= self.candidate_frames:
            self.state = self.STATE_COMPLETED
            return True

        return False

    def capture_best_frame(self):

        if self.state != self.STATE_COMPLETED:
            return None

        # Lấy tấm ảnh có điểm độ nét cao nhất từ luồng stream
        best_stream_frame = self.selector.get_best_frame()

        if best_stream_frame is None:
            return None

        # TRẢ VỀ TRỰC TIẾP TẤM ẢNH NÉT NHẤT ĐÃ ĐƯỢC CHỌN LỌC
        # Không dùng time.sleep() hay self.camera.capture_frame() ở đây 
        # để tránh làm treo luồng HTTP của camera (tránh lỗi Stream ended)
        return best_stream_frame

    def reset(self):

        self.selector.reset()

        self.state = self.STATE_IDLE