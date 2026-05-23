import time
import cv2
from vision import Vision


class CarController:
    cont=0
    oldAction=""
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
        detected_colors = positions.keys()  # Solo procesar los colores detectados

        # Obtener los objetos más grandes de cada color
        red_obj, red_x = self.get_largest_object(positions.get("Red", []))
        green_obj, green_x = self.get_largest_object(positions.get("Green", []))
        #blue_obj, blue_x = self.get_largest_object(positions.get("Blue", []))
        #orange_obj, orange_x = self.get_largest_object(positions.get("Orange", []))
        pink_obj, pink_x = self.get_largest_object(positions.get("Pink", []))

        # Comportamientos especiales según el color detectado
        '''if blue_obj and not self.prev_blue_detected:
            self.blue_count += 1
            print(f"Iniciar vuelta (Blue detected at X={blue_x}) - Blue Count: {self.blue_count}", flush=True)

        self.prev_blue_detected = bool(blue_obj)

        if orange_obj:
            print(f"Terminar vuelta (Orange detected at X={orange_x})", flush=True)

        if self.blue_count >= 4:
            self.lap_count += 1
            self.blue_count = 0
            print(f"Vuelta completada - Total vueltas: {self.lap_count}", flush=True)
        '''
        if  pink_obj:
            print(f"Estacionarse (Pink detected at X={pink_x} after 3 laps)", flush=True)
            return "Estacionarse", "Pink", pink_x

        # Seleccionar la acción basada en los colores detectados
        if red_obj:
            return "turn_right", "Red", red_x
        elif green_obj:
            return "turn_left", "Green", green_x
        else:
            return "drive_straight", "None", None  # Ningún color relevante detectado

    def control_motors(self, action, color, x_position):
        """Simula el control de los motores y muestra qué color está determinando la acción solo una vez."""
        
        if action == CarController.oldAction and color == CarController.oldColor:
            if color != "None" and x_position != CarController.oldX:  # Solo imprime X si hay un color detectado
                print(f"X={x_position}")
                CarController.oldX = x_position 
        else:
            if color != "None" :
                print(f"Motors: {action} (Based on {color} at X={x_position})", flush=True)
                
            else:
                print(f"Motors: {action}", flush=True)

            CarController.cont = 0  # Restablece cont
            CarController.oldAction = action  # Guarda la nueva acción
            CarController.oldColor = color  # Guarda el nuevo color
            CarController.oldX = x_position 