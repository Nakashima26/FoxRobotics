import cv2
import numpy as np
import math

# --- CONFIGURACIÓN ---
DISTANCIA_CALIBRACION_CM = 30.0  
ANCHO_CLICS_CM = 10.0            
ANCHO_OBJETO_CM = 6.9            

# Pipeline de GStreamer CORREGIDO (Full FoV sin zoom)
pipeline = "libcamerasrc ! video/x-raw, width=1640, height=1232, framerate=30/1 ! videoscale ! video/x-raw, width=640, height=480 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true sync=false"

puntos = []
frame_capturado = None
focal_length = None

color_ranges = {
    "Red": [(np.array([0, 150, 100]), np.array([10, 255, 255])),
            (np.array([170, 150, 100]), np.array([180, 255, 255]))],
    "Green": [(np.array([40, 80, 50]), np.array([80, 255, 255]))]
}
kernel = np.ones((3, 3), np.uint8)

def click_mouse(event, x, y, flags, param):
    global puntos, frame_capturado, focal_length
    if event == cv2.EVENT_LBUTTONDOWN:
        puntos.append((x, y))
        cv2.circle(frame_capturado, (x, y), 5, (0, 255, 0), -1)
        
        if len(puntos) == 2:
            cv2.line(frame_capturado, puntos[0], puntos[1], (255, 0, 0), 2)
            cv2.imshow("Calibracion", frame_capturado)
            
            p1, p2 = puntos
            pixeles = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            focal_length = (pixeles * DISTANCIA_CALIBRACION_CM) / ANCHO_CLICS_CM
            
            print(f"\nFocal Length calculado: {focal_length:.2f}")
            print("Presiona cualquier tecla en la ventana para continuar...")
        else:
            cv2.imshow("Calibracion", frame_capturado)

def run_system():
    global frame_capturado, focal_length
    
    # Abrir cámara con GStreamer
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    print(f"Pon tu regla a {DISTANCIA_CALIBRACION_CM}cm de la cámara y presiona 'c'.")
    while True:
        ret, frame = cap.read()
        if not ret: continue
        
        frame_mostrar = frame.copy()
        cv2.putText(frame_mostrar, "Alinea la regla y presiona 'C'", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Calibracion", frame_mostrar)
        
        if cv2.waitKey(1) & 0xFF == ord('c'):
            frame_capturado = frame.copy()
            break
            
    cv2.putText(frame_capturado, "Haz clic en 0cm y luego en 10cm", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Calibracion", frame_capturado)
    cv2.setMouseCallback("Calibracion", click_mouse)
    
    cv2.waitKey(0)
    cv2.destroyWindow("Calibracion")

    if focal_length is None:
        cap.release()
        return

    print("Deteccion iniciada. Presiona ESC para salir.")
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        for color_name, ranges in color_ranges.items():
            mask = np.bitwise_or.reduce([cv2.inRange(hsv, lower, upper) for lower, upper in ranges])
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 1000:
                    x, y, w, h = cv2.boundingRect(cnt)
                    
                    # Cálculo de distancia
                    distancia = (ANCHO_OBJETO_CM * focal_length) / w
                    
                    color_box = (0, 255, 0) if color_name == "Green" else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), color_box, 2)
                    
                    texto = f"{color_name}: {distancia:.1f} cm"
                    cv2.putText(frame, texto, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(frame, f"Focal: {focal_length:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.imshow("Deteccion y Distancia", frame)
        
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_system()
