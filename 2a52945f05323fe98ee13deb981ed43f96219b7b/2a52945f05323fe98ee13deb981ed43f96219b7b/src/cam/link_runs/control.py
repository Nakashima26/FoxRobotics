import time

import cv2

from vision import Vision


class CarController:
    cont = 0
    oldAction = ""
    oldColor = ""
    oldX = None

    def __init__(self, cam_index=0):
        """Initialize the controller and the link-runs vision pipeline."""
        self.vision = Vision(cam_index)
        self.state = "driving"
        self.blue_count = 0
        self.lap_count = 0
        self.prev_blue_detected = False

    def get_largest_object(self, objects):
        """Return the largest object and its x center."""
        if objects:
            largest = max(objects, key=lambda obj: obj[2] * obj[3])
            x, y, w, h = largest
            center_x = x + w // 2
            return largest, center_x
        return None, None

    def decide_action(self, positions, frame_width):
        """Decide the action based on detected red, green, and pink objects."""
        red_obj, red_x = self.get_largest_object(positions.get("Red", []))
        green_obj, green_x = self.get_largest_object(positions.get("Green", []))
        pink_obj, pink_x = self.get_largest_object(positions.get("Pink", []))

        if pink_obj:
            print(f"Estacionarse (Pink detected at X={pink_x})", flush=True)
            return "Estacionarse", "Pink", pink_x

        if red_obj:
            return "turn_right", "Red", red_x
        if green_obj:
            return "turn_left", "Green", green_x

        return "drive_straight", "None", None

    def control_motors(self, action, color, x_position):
        """Print the action and avoid repeating the same output."""
        if action == CarController.oldAction and color == CarController.oldColor:
            if color != "None" and x_position != CarController.oldX:
                print(f"X={x_position}")
                CarController.oldX = x_position
        else:
            if color != "None":
                print(f"Motors: {action} (Based on {color} at X={x_position})", flush=True)
            else:
                print(f"Motors: {action}", flush=True)

            CarController.cont = 0
            CarController.oldAction = action
            CarController.oldColor = color
            CarController.oldX = x_position
