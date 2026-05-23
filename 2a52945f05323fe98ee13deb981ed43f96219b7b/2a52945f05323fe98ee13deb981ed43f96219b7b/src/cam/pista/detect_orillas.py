import cv2
import numpy as np


# Rango HSV para madera/beige/cafe claro.
# Ajustable segun la luz real de la pista.
WOOD_LOWER = np.array([5, 25, 40])
WOOD_UPPER = np.array([35, 180, 255])


def open_camera(cam_index: int):
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la camara.")
    return cap


def detect_turn_direction(frame, previous_left=None, previous_right=None, top_ratio=0.30):
    height, width = frame.shape[:2]
    top_limit = max(1, int(height * top_ratio))
    roi = frame[:top_limit, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, WOOD_LOWER, WOOD_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))

    mid = width // 2
    left_mask = mask[:, :mid]
    right_mask = mask[:, mid:]

    left_count = int(cv2.countNonZero(left_mask))
    right_count = int(cv2.countNonZero(right_mask))

    left_ratio = left_count / max(1, left_mask.size)
    right_ratio = right_count / max(1, right_mask.size)

    left_drop = previous_left is not None and left_ratio < previous_left * 0.85
    right_drop = previous_right is not None and right_ratio < previous_right * 0.85

    turn_text = None
    if left_drop and not right_drop:
        turn_text = "girar a la izquierda"
        print(turn_text, flush=True)
    elif right_drop and not left_drop:
        turn_text = "girar a la derecha"
        print(turn_text, flush=True)

    debug = {
        "mask": mask,
        "left_ratio": left_ratio,
        "right_ratio": right_ratio,
        "top_limit": top_limit,
        "turn_text": turn_text,
    }
    return debug


def draw_debug(frame, debug):
    height, width = frame.shape[:2]
    top_limit = debug["top_limit"]
    mid = width // 2

    cv2.rectangle(frame, (0, 0), (mid - 1, top_limit - 1), (0, 255, 0), 1)
    cv2.rectangle(frame, (mid, 0), (width - 1, top_limit - 1), (0, 255, 255), 1)

    overlay = frame.copy()
    colored_mask = cv2.cvtColor(debug["mask"], cv2.COLOR_GRAY2BGR)
    overlay[:top_limit, :] = cv2.addWeighted(frame[:top_limit, :], 0.65, colored_mask, 0.35, 0)
    frame[:top_limit, :] = overlay[:top_limit, :]

    cv2.putText(
        frame,
        f"L={debug['left_ratio']:.3f} R={debug['right_ratio']:.3f}",
        (10, top_limit + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    if debug["turn_text"]:
        cv2.putText(
            frame,
            debug["turn_text"],
            (10, top_limit + 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )


def run(cam_index=1):
    cap = open_camera(cam_index)
    previous_left = None
    previous_right = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No frame captured", flush=True)
            break

        debug = detect_turn_direction(frame, previous_left, previous_right)
        previous_left = debug["left_ratio"]
        previous_right = debug["right_ratio"]

        draw_debug(frame, debug)
        cv2.imshow("Pista", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
