import cv2
from control import CarController
import time
def run_car(cam_index=1):
    car = CarController(cam_index)
    time.sleep(2)
    frame_skip = 2  # Procesa cada 2 frames
    frame_count = 0

    while True:
        ret, frame = car.vision.cap.read()
        if not ret:
            print("DEBUG: No frame captured", flush=True)
            continue

        if frame_count % frame_skip == 0:  # Procesa solo cada 2 frames
            processed_frame, positions = car.vision.process_frame(frame)
            action, color, x_position = car.decide_action(positions, frame.shape[1])
            car.control_motors(action, color, x_position)
            cv2.imshow("Car Controller Vision", processed_frame)

        frame_count += 1

        if cv2.waitKey(1) & 0xFF == 27:
            break

    car.vision.cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_car()
