import cv2
import numpy as np


class Vision:
    def __init__(self, cam_index=0):
        """Initialize camera and build simple BGR ranges for presence-only detection."""
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.color_ranges = {
            "Red": [(np.array([0, 0, 180]), np.array([90, 90, 255]))],
            "Green": [(np.array([0, 180, 0]), np.array([90, 255, 90]))],
            "Pink": [(np.array([180, 0, 180]), np.array([255, 90, 255]))],
        }

        self.kernel = np.ones((3, 3), np.uint8)
        self.min_area = 10000

    def process_mask(self, mask):
        """Clean a binary mask and return its active pixel count."""
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        area = cv2.countNonZero(mask)
        return area, area >= self.min_area

    def process_frame(self, frame):
        """Detect whether each target color is present above the area threshold."""
        frame = cv2.flip(frame, 1)
        detections = {}

        for color, ranges in self.color_ranges.items():
            mask = np.bitwise_or.reduce([cv2.inRange(frame, lower, upper) for lower, upper in ranges])
            area, present = self.process_mask(mask)
            detections[color] = {"area": area, "present": present}

        y_position = 30
        for color in ("Red", "Green", "Pink"):
            info = detections[color]
            status = "YES" if info["present"] else "NO"
            label = f"{color}: {status} ({info['area']})"
            cv2.putText(frame, label, (10, y_position), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_position += 28

        return frame, detections
