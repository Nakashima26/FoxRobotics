"""
Segmentación de obstáculos por esquina + trigger de giro basado en líneas.

Flujo:
  1. Un obstáculo detectado MÁS ALLÁ de la línea de esquina (la línea queda
     entre el robot y él) se guarda como "en cola" — NO se le pasa a
     detect_centerline() ni al ESP32 como prio/mem todavía.
  2. Cuando el obstáculo ACTUAL deja de estar activo Y el ESP32 confirma
     que volvió a SIGUIENDO (RECUPERANDO completado, heading realineado),
     el tracker se ARMA.
  3. Armado, observa naranja/azul según el color del obstáculo en cola y
     dispara (una sola vez) cuando esa línea se pierde o está cerca.
"""

from . import config as C


class SegmentTracker:
    def __init__(self):
        self.reset()

    def reset(self):
        self.queued_color: str | None = None
        self._armed = False
        self._fired = False
        self._prev_esp_state: str | None = None

    def split(self, bev_obstacles, orange_info, blue_info):
        line_ys = [i["near_y"] for i in (orange_info, blue_info) if i["seen"]]
        if not line_ys:
            return bev_obstacles, []

        line_y = max(line_ys)   # la línea vista más cercana al robot
        current, queued = [], []
        for ox, oy, color in bev_obstacles:
            (queued if oy < line_y else current).append((ox, oy, color))

        if queued and self.queued_color is None:
            self.queued_color = queued[0][2]

        return current, queued

    def update_arming(self, current_obstacle_active: bool, esp_state: str | None):
        if self.queued_color is None or self._armed:
            return
        realigned = (
            esp_state is None
            or (self._prev_esp_state == "R" and esp_state == "S")
        )
        if esp_state is not None:
            self._prev_esp_state = esp_state
        if not current_obstacle_active and realigned:
            self._armed = True

    def check_trigger(self, orange_info, blue_info) -> bool:
        if not self._armed or self._fired or self.queued_color is None:
            return False

        if self.queued_color == "Red":
            info, offset_px = orange_info, C.LINE_TRIGGER_OFFSET_RED_MM / C.MM_PER_PX
        elif self.queued_color == "Green":
            info, offset_px = blue_info, C.LINE_TRIGGER_OFFSET_GREEN_MM / C.MM_PER_PX
        else:
            return False

        lost = not info["seen"]
        near = (info["seen"] and info["near_y"] is not None
                and info["near_y"] >= (C.ROBOT_BEV_Y - C.LINE_PROXIMITY_PX + offset_px))

        if lost or near:
            self._fired = True
            return True
        return False