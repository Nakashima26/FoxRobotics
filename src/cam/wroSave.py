import cv2
import subprocess
import time
import sys
from control import CarController

def start_ssr():
    ssr_command = [
       "simplescreenrecorder",
       "--start-hidden",
       "--output", "output.mp4",
       "--fps", "15",
       "--video-coding", "libx264"
    ]
    return subprocess.Popen(ssr_command)

def run_car(cam_index=0):
    # Inicia SimpleScreenRecorder en un proceso externo
    ssr_process = start_ssr()
    print("SimpleScreenRecorder iniciado.")

    car = CarController(cam_index)
    frame_skip = 2
    frame_count = 0

    try:
        while True:
            ret, frame = car.vision.cap.read()
            if not ret:
                break

            if frame_count % frame_skip == 0:
                processed_frame, positions = car.vision.process_frame(frame)
                action, color, x_pos = car.decide_action(positions, frame.shape[1])
                car.control_motors(action, color, x_pos)
                cv2.imshow("Procesado", processed_frame)
                
            frame_count += 1

            if cv2.waitKey(1) & 0xFF == 27:  # Si se presiona ESC
                print("ESC presionado, terminando...")
                break
    except Exception as e:
        print(f"Error: {e}")
    finally:
        car.vision.cap.release()
        cv2.destroyAllWindows()
        # Detén SimpleScreenRecorder
        ssr_process.terminate()
        ssr_process.wait()
        print("SimpleScreenRecorder detenido, video guardado.")

if __name__ == "__main__":
    run_car()
