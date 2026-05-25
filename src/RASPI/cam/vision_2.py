import cv2
import numpy as np

class Vision:
    def __init__(self, cam_index=0):
        """Inicializa la cámara y define los rangos de colores."""
        self.cap = cv2.VideoCapture(cam_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Reducir resolución para mejor rendimiento
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Definir los rangos de colores en HSV con NumPy
        self.color_ranges = {
            "Red": [(np.array([0, 150, 100]), np.array([10, 255, 255])),
                    (np.array([170, 150, 100]), np.array([180, 255, 255]))],
            "Green": [(np.array([40, 80, 50]), np.array([80, 255, 255]))],
            "Pink": [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
            "Blue": [(np.array([100, 150, 50]), np.array([130, 255, 255]))],
            "Orange": [(np.array([10, 150, 100]), np.array([25, 255, 255]))]
        }

        self.kernel = np.ones((3, 3), np.uint8)  # Kernel más pequeño para eficiencia

    def detect_rectangles(self, frame):
        """Detecta contornos rectangulares en la imagen."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rectangles = []
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if 0.5 < w/h < 2 and w > 10 and h > 20:  # Filtrar solo rectángulos
                rectangles.append((x, y, w, h))
        
        return rectangles

    def process_frame(self, frame):
        """Detecta rectángulos primero y luego identifica los colores."""
        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detectar rectángulos primero
        rectangles = self.detect_rectangles(frame)
        positions = {}

        for x, y, w, h in rectangles:
            roi = hsv[y:y+h, x:x+w]
            avg_color = np.mean(roi, axis=(0, 1))  # Obtener color promedio
            detected_color = None
            
            for color, ranges in self.color_ranges.items():
                for lower, upper in ranges:
                    if np.all(lower <= avg_color) and np.all(avg_color <= upper):
                        detected_color = color
                        break
                if detected_color:
                    break
            
            if detected_color:
                if detected_color not in positions:
                    positions[detected_color] = []
                positions[detected_color].append((x, y, w, h))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.putText(frame, detected_color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame, positions
