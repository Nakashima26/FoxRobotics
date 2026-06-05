import time

import RPi.GPIO as GPIO
import serial

from vision import Vision


ser = serial.Serial("/dev/ttyS0", 115200, timeout=1)
time.sleep(2)
GPIO.setmode(GPIO.BCM)
output_pin = 17
GPIO.setup(output_pin, GPIO.OUT)


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
        """Simulate motor control and forward the x position to serial when useful."""
        if action == CarController.oldAction and color == CarController.oldColor:
            if color != "None" and x_position != CarController.oldX:
                print(f"X={x_position}")
                CarController.oldX = x_position
                ser.write(f"{x_position}\n".encode())
                respuesta = ser.readline().decode().strip()

                if respuesta:
                    print(f"respuesta: {respuesta}")
        else:
            if color != "None":
                print(f"Motors: {action} (Based on {color} at X={x_position})", flush=True)
                if color == "Green":
                    GPIO.output(output_pin, GPIO.HIGH)
                    ser.write(f"{x_position}\n".encode())
                    respuesta = ser.readline().decode().strip()

                    if respuesta:
                        print(f"respuesta: {respuesta}")
                else:
                    GPIO.output(output_pin, GPIO.LOW)
            else:
                print(f"Motors: {action}", flush=True)
                GPIO.output(output_pin, GPIO.LOW)
                x_position = 700
                ser.write(f"{x_position}\n".encode())
                respuesta = ser.readline().decode().strip()

                if respuesta:
                    print(f"respuesta: {respuesta}")

            CarController.cont = 0
            CarController.oldAction = action
            CarController.oldColor = color
            CarController.oldX = x_position
