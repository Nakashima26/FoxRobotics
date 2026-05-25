import time
import cv2
from vision import Vision
import RPi.GPIO as GPIO
import serial

# Configuración del puerto serial y pines
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
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
        """Inicializa el controlador del carro y la visión."""
        self.vision = Vision(cam_index)
        self.state = "driving"
        self.blue_count = 0
        self.lap_count = 0
        self.prev_blue_detected = False

    def get_largest_object(self, objects):
        """Devuelve el objeto más grande basado en el área."""
        if objects:
            largest = max(objects, key=lambda obj: obj[2] * obj[3])
            x, y, w, h = largest
            center_x = x + w // 2
            return largest, center_x
        return None, None 

    def decide_action(self, positions, frame_width):
        """Decide qué acción tomar basado en los colores detectados y devuelve la acción, el color y la posición X."""
        detected_colors = positions.keys()

        # Obtener los objetos más grandes de cada color
        red_obj, red_x = self.get_largest_object(positions.get("Red", []))
        green_obj, green_x = self.get_largest_object(positions.get("Green", []))
        pink_obj, pink_x = self.get_largest_object(positions.get("Pink", []))

        # Comportamientos especiales según el color detectado
        if pink_obj:
            print(f"Estacionarse (Pink detected at X={pink_x} after 3 laps)", flush=True)
            return "Estacionarse", "Pink", pink_x

        # Seleccionar la acción basada en los colores detectados
        if red_obj:
            return "turn_right", "Red", red_x
        elif green_obj:
            return "turn_left", "Green", green_x
        else:
            return "drive_straight", "None", None 

    def control_motors(self, action, color, x_position):
        """Simula el control de los motores y muestra qué color está determinando la acción solo una vez."""
        
        if action == CarController.oldAction and color == CarController.oldColor:
            if color != "None" and x_position != CarController.oldX: 
                print(f"X={x_position}")
                CarController.oldX = x_position 
                
                print(f"[DEBUG SERIAL] -> Enviando X={x_position}")
                ser.write(f"{x_position}\n".encode())
                
                print(f"[DEBUG SERIAL] <- Esperando respuesta...")
                respuesta = ser.readline().decode().strip()
                if respuesta:
                    print(f"[DEBUG SERIAL] respuesta: {respuesta}")
                else:
                    print(f"[DEBUG SERIAL] timeout: La ESP no respondio nada.")
                        
        else:
            if color != "None":
                print(f"Motors: {action} (Based on {color} at X={x_position})", flush=True)
                if color == "Green":
                    GPIO.output(output_pin, GPIO.HIGH)
                    
                    print(f"[DEBUG SERIAL GREEN] -> Enviando X={x_position}")
                    ser.write(f"{x_position}\n".encode())
                    
                    print(f"[DEBUG SERIAL GREEN] <- Esperando respuesta...")
                    respuesta = ser.readline().decode().strip()
                    if respuesta:
                        print(f"[DEBUG SERIAL GREEN] respuesta: {respuesta}")
                    else:
                        print(f"[DEBUG SERIAL GREEN] timeout: La ESP no respondio nada.")
                        
                else:
                    GPIO.output(output_pin, GPIO.LOW)
            else:
                print(f"Motors: {action}", flush=True)
                GPIO.output(output_pin, GPIO.LOW)
                x_position = 700
                
                print(f"[DEBUG SERIAL NONE] -> Enviando 700")
                ser.write(f"{x_position}\n".encode())
                
                print(f"[DEBUG SERIAL NONE] <- Esperando respuesta...")
                respuesta = ser.readline().decode().strip()
                if respuesta:
                    print(f"[DEBUG SERIAL NONE] respuesta: {respuesta}")
                else:
                    print(f"[DEBUG SERIAL NONE] timeout: La ESP no respondio nada.")
            
            CarController.cont = 0
            CarController.oldAction = action
            CarController.oldColor = color
            CarController.oldX = x_position
