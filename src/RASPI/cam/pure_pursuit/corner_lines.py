"""
Detección de líneas de esquina (naranja / azul) en la imagen BEV.

Recuperado del historial (commit 2ab26e2 ".ino updated + test turns",
2026-08-17), pero con un método de localización distinto: la versión
original tomaba el Y máximo de TODOS los pixeles del color en todo el
frame — un puñado de ruido disperso (que nunca forma una franja real)
contaba igual que la línea de verdad, y si el ruido caía más cerca del
robot que la línea real, ganaba el ruido.

Método actual: recorre el BEV fila por fila desde el robot hacia adelante
y se queda con la PRIMERA fila que tenga una corrida CONTIGUA de ese color
de al menos LINE_MIN_RUN_PX — exige que sea una franja real en esa fila
(no puntos sueltos), y como se evalúa fila por fila (no un bounding box de
todo el blob), una línea curva o en diagonal no se penaliza por su altura
total como sí le pasaba al enfoque de bounding box.

Convención BEV (ver bev.py): Y crece hacia ABAJO (hacia el robot). Un punto
con Y grande está CERCA del robot; Y chico está LEJOS (adelante).
"""

import cv2
import numpy as np

from . import config as C


def _line_mask(bev_hsv: np.ndarray, ranges) -> np.ndarray:
    masks = [cv2.inRange(bev_hsv, lo, hi) for lo, hi in ranges]
    return np.bitwise_or.reduce(masks) if len(masks) > 1 else masks[0]


def _longest_run(row: np.ndarray) -> int:
    """Ancho de la corrida contigua de pixeles>0 más larga en una fila 1D."""
    cols = np.where(row > 0)[0]
    if len(cols) == 0:
        return 0
    breaks = np.where(np.diff(cols) > 1)[0]
    runs = np.split(cols, breaks + 1)
    return max(len(r) for r in runs)


def _find_near_line_row(mask: np.ndarray, min_run_px: int) -> float | None:
    """
    Escanea desde la fila más cercana al robot (Y grande) hacia adelante
    (Y chico) y retorna la Y de la primera fila con una corrida contigua
    >= min_run_px. None si ninguna fila califica.
    """
    h = mask.shape[0]
    for y in range(h - 1, -1, -1):
        if _longest_run(mask[y, :]) >= min_run_px:
            return float(y)
    return None


def _fit_line_near(mask: np.ndarray, near_y: float, band_px: float,
                    min_points: int) -> tuple[float, float, float, float] | None:
    """
    Ajusta una recta (vx, vy, x0, y0) a los pixeles de la máscara dentro de
    una banda de +-band_px alrededor de near_y, cubriendo todo el ancho de
    la imagen — a diferencia de _find_near_line_row(), que solo mira una
    fila, esto junta pixeles de varias filas para poder estimar la
    PENDIENTE real de la línea (puede venir inclinada, no necesariamente
    horizontal).

    Retorna None si no hay suficientes pixeles en la banda (línea muy
    ocluida/fragmentada ahí) — en ese caso, quien llame debe usar un
    fallback horizontal en near_y en vez de una recta con pendiente.
    """
    h = mask.shape[0]
    y0b = max(0, int(near_y - band_px))
    y1b = min(h, int(near_y + band_px) + 1)
    band = mask[y0b:y1b, :]
    ys, xs = np.where(band > 0)
    if len(xs) < min_points:
        return None
    pts = np.column_stack([xs.astype(np.float32), (ys + y0b).astype(np.float32)])
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    return float(vx), float(vy), float(x0), float(y0)


def line_side_is_near(px: float, py: float,
                       line_params: tuple[float, float, float, float],
                       ref_x: float, ref_y: float) -> bool:
    """
    True si el punto (px, py) está del MISMO lado de la recta (vx,vy,x0,y0)
    que el punto de referencia (ref_x, ref_y) — normalmente el robot, cuyo
    lado por definición es "antes de la línea" (mi recta).

    Usa el signo del producto cruzado (dirección de la recta) x (vector al
    punto) — no importa la orientación de (vx,vy) que devuelva cv2.fitLine
    porque el signo se calibra contra la referencia, no contra un eje fijo.
    """
    vx, vy, x0, y0 = line_params
    cross_ref = vx * (ref_y - y0) - vy * (ref_x - x0)
    cross_pt  = vx * (py - y0) - vy * (px - x0)
    return (cross_ref >= 0) == (cross_pt >= 0)


def detect_lines(bev_bgr: np.ndarray) -> dict:
    """
    Retorna {'seen': bool, 'near_y': float|None} para la línea naranja.
    near_y = coordenada Y-BEV de la franja real más cercana al robot
    (no el pixel aislado más cercano — ver _find_near_line_row()).

    Azul se quitó de la ecuación (daba muchos falsos positivos/negativos y
    no era confiable) — por ahora solo se seguirá la línea naranja.

    Lectura CRUDA de un solo frame — brinca entre un segmento cercano
    parcialmente ocluido por un obstáculo (a veces cruza LINE_MIN_RUN_PX,
    a veces no, por un par de pixeles de diferencia frame a frame) y el
    siguiente segmento realmente visible más lejos. Para uso en el
    runtime real, usar OrangeLineTracker (abajo), que suaviza esto en
    el tiempo.
    """
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    mask = _line_mask(hsv, C.LINE_ORANGE_HSV)
    near_y = _find_near_line_row(mask, C.LINE_MIN_RUN_PX)
    return {"Orange": {"seen": near_y is not None, "near_y": near_y}}


class OrangeLineTracker:
    """
    Suaviza detect_lines() en el tiempo.

    La lectura cruda de un solo frame puede "brincar" entre dos segmentos
    reales distintos (uno cercano parcialmente ocluido por un obstáculo,
    otro más lejos y despejado) de un frame a otro — confirmado en pista:
    la Y reportada saltaba entre "muy pegada al obstáculo" y "bien
    separada" sin que el robot se moviera lo suficiente para justificarlo.

    En vez de confiar en la lectura de un solo frame, exige que una nueva
    lectura (mismo 'seen' y near_y dentro de una tolerancia) se repita
    PERSIST_FRAMES seguidos antes de aceptarla como el estado "real" — un
    brinco de un solo frame no alcanza a mover el valor reportado.
    """

    def __init__(self, persist_frames: int = 3, tolerance_px: float = 15.0):
        self.persist_frames = persist_frames
        self.tolerance_px = tolerance_px
        self.stable: dict = {"seen": False, "near_y": None, "line": None}
        self._candidate: dict | None = None
        self._candidate_count = 0

    def reset(self):
        self.stable = {"seen": False, "near_y": None, "line": None}
        self._candidate = None
        self._candidate_count = 0

    def _matches_candidate(self, raw: dict) -> bool:
        if self._candidate is None or raw["seen"] != self._candidate["seen"]:
            return False
        if not raw["seen"]:
            return True
        return abs(raw["near_y"] - self._candidate["near_y"]) <= self.tolerance_px

    def update(self, bev_bgr: np.ndarray) -> dict:
        raw = detect_lines(bev_bgr)["Orange"]

        if self._matches_candidate(raw):
            self._candidate_count += 1
        else:
            self._candidate = raw
            self._candidate_count = 1

        if self._candidate_count >= self.persist_frames:
            self.stable = dict(self._candidate)
            if self.stable["seen"]:
                # Ajusta la recta (con pendiente) SOLO una vez que near_y ya
                # es estable — así el ancla de la banda no "baila" también.
                hsv  = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
                mask = _line_mask(hsv, C.LINE_ORANGE_HSV)
                self.stable["line"] = _fit_line_near(
                    mask, self.stable["near_y"],
                    C.LINE_FIT_BAND_PX, C.LINE_FIT_MIN_POINTS,
                )
            else:
                self.stable["line"] = None

        return self.stable

    def classify(self, ox: float, oy: float, robot_x: float, robot_y: float) -> bool | None:
        """
        True si (ox, oy) está del mismo lado que el robot (antes de la
        línea, mi recta); False si está del otro lado (siguiente recta).
        None si no hay línea estable todavía (no se puede clasificar).

        Usa la recta con pendiente cuando se pudo ajustar (self.stable["line"]);
        si no hubo suficientes pixeles para ajustarla (línea muy ocluida/corta
        en este momento), cae de vuelta a comparar solo Y contra near_y —
        funciona igual de bien que antes cuando la línea SÍ es horizontal,
        y es mejor que no clasificar nada.
        """
        if not self.stable["seen"]:
            return None
        line = self.stable["line"]
        if line is not None:
            return line_side_is_near(ox, oy, line, robot_x, robot_y)
        return oy > self.stable["near_y"]


class TurnDirectionTracker:
    """
    Fija la dirección de giro (izquierda/derecha) inferida de la posición
    lateral de un obstáculo visto más allá de la línea naranja: si cae a la
    derecha del centro del robot, la pista gira a la derecha; si cae a la
    izquierda, gira a la izquierda.

    Igual que OrangeLineTracker, exige que la misma dirección salga
    PERSIST_FRAMES seguidos antes de fijarla — es una decisión demasiado
    importante (si sale mal, el carro gira contra la dirección real) como
    para fijarla de un solo dato ruidoso.

    Una vez fijada (self.direction is not None), NO vuelve a cambiar en
    toda la carrera — mismo criterio que direccionIzquierda/primerGiro en
    el ESP32 (PurePursuit.ino), pero determinado por visión en vez de
    ultrasónicos, y potencialmente antes de llegar físicamente a la
    primera esquina.
    """

    def __init__(self, persist_frames: int = 5):
        self.persist_frames = persist_frames
        self.direction: str | None = None   # "L" o "R"; None = aún no confirmada
        self._candidate: str | None = None
        self._candidate_count = 0

    def reset(self):
        self.direction = None
        self._candidate = None
        self._candidate_count = 0

    def update(self, bev_obstacles_beyond: list[tuple[float, float, str]],
               robot_x: float) -> str | None:
        if self.direction is not None:
            return self.direction   # ya fija, no se vuelve a evaluar

        if not bev_obstacles_beyond:
            self._candidate = None
            self._candidate_count = 0
            return None

        guess = "R" if bev_obstacles_beyond[0][0] > robot_x else "L"
        return self._commit_guess(guess)

    def update_ultrasonic(self, dL: float | None, dR: float | None) -> str | None:
        """Misma persistencia que update(), pero con apertura ultrasónica."""
        if self.direction is not None:
            return self.direction
        guess = infer_turn_dir_from_ultrasonics(dL, dR)
        if guess is None:
            self._candidate = None
            self._candidate_count = 0
            return None
        return self._commit_guess(guess)

    def _commit_guess(self, guess: str) -> str | None:
        if guess == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = guess
            self._candidate_count = 1

        if self._candidate_count >= self.persist_frames:
            self.direction = self._candidate

        return self.direction


def is_interior_pass(direction: str | None, color: str) -> bool:
    """
    True si pasar este obstáculo (por su color, regla WRO: Rojo->derecha,
    Verde->izquierda) coincide con el lado hacia el que va a girar la
    pista — en ese caso, el giro mismo ya resuelve el paso, no hace falta
    bloquear detectarEsquina() esperando a que Pure Pursuit lo esquive del
    todo. False si no coincide (exterior) o si la dirección aún no se ha
    confirmado (default seguro: tratar como bloqueante, comportamiento de
    siempre).
    """
    if direction is None:
        return False
    if direction == "R":
        return color == "Red"
    return color == "Green"


def infer_turn_dir_from_ultrasonics(
    dL: float | None,
    dR: float | None,
    open_cm: float = C.PRE_TURN_OPEN_CM,
    margin_cm: float = C.PRE_TURN_OPEN_MARGIN_CM,
) -> str | None:
    """
    Infiera hacia dónde va a girar la pista mirando qué ultrasonido "abre"
    primero (misma señal que detectarEsquina en el ESP32, pero sin debounce).

    Útil cuando la naranja aún no se ve: rojo+giro-derecha e
    verde+giro-izquierda comparten el mismo patrón de apertura.
    """
    if dL is None or dR is None:
        return None
    if dL >= open_cm and dR < (open_cm - margin_cm):
        return "L"
    if dR >= open_cm and dL < (open_cm - margin_cm):
        return "R"
    return None
