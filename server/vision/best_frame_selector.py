import cv2


class BestFrameSelector:

    def __init__(
        self,
        min_sharpness=100.0,
        max_candidates= 10
    ):
        self.min_sharpness = min_sharpness
        self.max_candidates = max_candidates

        self.best_frame = None
        self.best_score = 0.0
        self.candidate_count = 0

    def calculate_sharpness(self, frame):

        if frame is None:
            return 0.0

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        laplacian = cv2.Laplacian(
            gray,
            cv2.CV_64F
        )

        return float(
            laplacian.var()
        )

    def add_frame(self, frame):
        if frame is None or self.candidate_count >= self.max_candidates:
            return False

        score = self.calculate_sharpness(frame)
        self.candidate_count += 1

        # Luôn cập nhật nếu điểm cao hơn điểm tốt nhất hiện tại (hoặc là frame đầu tiên)
        if self.best_frame is None or score > self.best_score:
            self.best_frame = frame.copy()
            self.best_score = score
            return True

        return False

    def get_best_frame(self):

        if self.best_frame is None:
            return None

        return self.best_frame.copy()

    def get_best_score(self):

        return self.best_score

    def is_ready(self):

        return (
            self.candidate_count >=
            self.max_candidates
        )

    def reset(self):

        self.best_frame = None
        self.best_score = 0.0
        self.candidate_count = 0