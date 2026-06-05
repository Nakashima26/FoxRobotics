import time

import RPi.GPIO as GPIO
import serial


ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
time.sleep(2)
GPIO.setmode(GPIO.BCM)
output_pin = 17
GPIO.setup(output_pin, GPIO.OUT)


class CarController:
    cont = 0
    oldAction = ""
    oldColor = ""
    oldSignal = None

    def __init__(self, cam_index=0):
        """Initialize the controller with the presence-only vision pipeline."""
        from vision import Vision

        self.vision = Vision(cam_index)
        self.state = "driving"
        self.blue_count = 0
        self.lap_count = 0
        self.prev_blue_detected = False

    def decide_action(self, detections, frame_width):
        """Decide what to do based only on whether a color is present and how large it is."""
        red = detections.get("Red", {})
        green = detections.get("Green", {})
        pink = detections.get("Pink", {})

        red_present = red.get("present", False)
        green_present = green.get("present", False)
        pink_present = pink.get("present", False)
        red_area = red.get("area", 0)
        green_area = green.get("area", 0)
        pink_area = pink.get("area", 0)

        if pink_present:
            print("Estacionarse (Pink detected)", flush=True)
            return "Estacionarse", "Pink", pink_area

        if red_present and green_present:
            if red_area >= green_area:
                return "turn_right", "Red", red_area
            return "turn_left", "Green", green_area

        if red_present:
            return "turn_right", "Red", red_area
        if green_present:
            return "turn_left", "Green", green_area

        return "drive_straight", "None", 0

    def control_motors(self, action, color, signal_value):
        """Simulate motor control and forward a simple numeric signal to serial."""
        if action == CarController.oldAction and color == CarController.oldColor:
            if color != "None" and signal_value != CarController.oldSignal:
                print(f"Signal={signal_value}")
                CarController.oldSignal = signal_value
                ser.write(f"{signal_value}\n".encode())
                respuesta = ser.readline().decode().strip()

                if respuesta:
                    print(f"respuesta: {respuesta}")
        else:
            if color != "None":
                print(f"Motors: {action} (Based on {color} signal={signal_value})", flush=True)

                if color == "Green":
                    GPIO.output(output_pin, GPIO.HIGH)
                else:
                    GPIO.output(output_pin, GPIO.LOW)

                ser.write(f"{signal_value}\n".encode())
                respuesta = ser.readline().decode().strip()

                if respuesta:
                    print(f"respuesta: {respuesta}")
            else:
                print(f"Motors: {action}", flush=True)
                GPIO.output(output_pin, GPIO.LOW)
                ser.write(b"0\n")
                respuesta = ser.readline().decode().strip()

                if respuesta:
                    print(f"respuesta: {respuesta}")

            CarController.cont = 0
            CarController.oldAction = action
            CarController.oldColor = color
            CarController.oldSignal = signal_value
