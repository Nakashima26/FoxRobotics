import cv2
import numpy as np


class Vision:
    def __init__(self, cam_index=0):
        """Initialize the camera and HSV color ranges for the link-runs pipeline."""
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.color_ranges = {
            "Red": [
                (np.array([0, 150, 100]), np.array([10, 255, 255])),
                (np.array([170, 150, 100]), np.array([180, 255, 255])),
            ],
            "Green": [(np.array([40, 80, 50]), np.array([80, 255, 255]))],
            "Pink": [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
        }

        self.kernel = np.ones((3, 3), np.uint8)

    def process_color(self, frame, mask, color_name):
        """Find objects in a binary mask using link-runs when available."""
        if np.count_nonzero(mask) < 10000:
            return []

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        if np.count_nonzero(mask) == 0:
            return []

        if hasattr(cv2, "findContoursLinkRuns"):
            contours, _ = cv2.findContoursLinkRuns(mask.copy())
        else:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                objects.append((x, y, w, h))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return objects

    def process_frame(self, frame):
        """Compute masks in HSV and extract the largest objects per color."""
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = {
            color: np.bitwise_or.reduce([cv2.inRange(hsv, lower, upper) for lower, upper in ranges])
            for color, ranges in self.color_ranges.items()
        }

        positions = {color: self.process_color(frame, mask, color) for color, mask in masks.items()}
        return frame, positions
