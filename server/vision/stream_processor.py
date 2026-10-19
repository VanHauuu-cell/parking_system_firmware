import cv2
import numpy as np

class StreamProcessor:

    STATE_IDLE = 0            # Không có xe
    STATE_VERIFYING_IN = 1    # Đang xác nhận xe vào
    STATE_OCCUPIED = 2        # Xe đang đậu / hiện diện
    STATE_VERIFYING_OUT = 3   # Đang xác nhận xe rời đi

    def __init__(
        self,
        roi_x=0,
        roi_y=0,
        roi_width=None,
        roi_height=None,
        threshold=25,
        min_changed_pixels=5000,
        persistence_frames=5,
        max_occupied_frames=3000 #30 fps * 100 giây = 3000 frames
    ):
        self.roi_x = roi_x
        self.roi_y = roi_y
        self.roi_width = roi_width
        self.roi_height = roi_height
        self.threshold = threshold
        self.min_changed_pixels = min_changed_pixels
        self.persistence_frames = persistence_frames

        # Thay previous_gray bằng background để lưu ảnh nền gốc
        # Sử dụng kiểu float để có thể cập nhật nền mượt mà (chống nhiễu sáng)
        self.background_gray = None 
        
        self.state = self.STATE_IDLE
        self.detect_count = 0
        self.leave_count = 0

        self.max_occupied_frames = max_occupied_frames
        self.occupied_frames_count = 0  # Đếm số khung hình xe đã chiếm dụng

    def set_roi(self, frame):
        height, width = frame.shape[:2]
        x, y = self.roi_x, self.roi_y
        roi_w = self.roi_width if self.roi_width is not None else width - x
        roi_h = self.roi_height if self.roi_height is not None else height - y
        return frame[y:y + roi_h, x:x + roi_w]

    def detect_presence(self, frame):
        roi = self.set_roi(frame)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        # Khởi tạo ảnh nền ở frame đầu tiên
        if self.background_gray is None:
            self.background_gray = gray.astype("float")
            return False, None

        # So sánh frame hiện tại với Ảnh Nền (Background) thay vì frame trước đó
        difference = cv2.absdiff(self.background_gray.astype("uint8"), gray)
        
        _, threshold_image = cv2.threshold(
            difference, self.threshold, 255, cv2.THRESH_BINARY
        )
        
        changed_pixels = cv2.countNonZero(threshold_image)
        is_occupied = changed_pixels >= self.min_changed_pixels

        # QUAN TRỌNG: Cập nhật background từ từ ĐỂ THÍCH NGHI VỚI ÁNH SÁNG
        # Chỉ cập nhật background khi CHẮC CHẮN không có xe (STATE_IDLE)
        # Hệ số 0.01 nghĩa là background học rất chậm, không bị ám ảnh xe lướt qua
        if not is_occupied and self.state == self.STATE_IDLE:
            cv2.accumulateWeighted(gray, self.background_gray, 0.01)

        return is_occupied, threshold_image

    def update_state(self, is_occupied):
        event = None

        if self.state == self.STATE_IDLE:
            if is_occupied:
                self.state = self.STATE_VERIFYING_IN
                self.detect_count = 1

        elif self.state == self.STATE_VERIFYING_IN:
            if is_occupied:
                self.detect_count += 1
                if self.detect_count >= self.persistence_frames:
                    self.state = self.STATE_OCCUPIED
                    self.occupied_frames_count = 0 # Bắt đầu đếm thời gian dừng
                    event = "VEHICLE_ARRIVED"
            else:
                self.state = self.STATE_IDLE
                self.detect_count = 0

        elif self.state == self.STATE_OCCUPIED:
            self.occupied_frames_count += 1
            
            # CƠ CHẾ CHỐNG KẸT (ANTI-GHOSTING)
            # Nếu vật thể nằm im quá lâu, coi như nó trở thành một phần của background
            if self.occupied_frames_count > self.max_occupied_frames:
                self.background_gray = None # Reset để frame tiếp theo lấy làm nền mới
                self.state = self.STATE_IDLE
                self.detect_count = 0
                self.leave_count = 0
                return "TIMEOUT_RESET" 

            if not is_occupied:
                self.state = self.STATE_VERIFYING_OUT
                self.leave_count = 1

        elif self.state == self.STATE_VERIFYING_OUT:
            if not is_occupied:
                self.leave_count += 1
                if self.leave_count >= self.persistence_frames:
                    self.state = self.STATE_IDLE
                    event = "VEHICLE_LEFT"
            else:
                # Nếu phát hiện lại có xe (xe chỉ nhích lên chút rồi dừng), quay lại OCCUPIED
                self.state = self.STATE_OCCUPIED
                self.leave_count = 0

        pass

        return event

    def process_frame(self, frame):
        if frame is None:
            return False, None, None

        is_occupied, difference = self.detect_presence(frame)
        event = self.update_state(is_occupied)
        
        vehicle_event_locked = (self.state == self.STATE_OCCUPIED)

        return vehicle_event_locked, difference, event

    def reset(self):
        self.background_gray = None
        self.state = self.STATE_IDLE
        self.detect_count = 0
        self.leave_count = 0