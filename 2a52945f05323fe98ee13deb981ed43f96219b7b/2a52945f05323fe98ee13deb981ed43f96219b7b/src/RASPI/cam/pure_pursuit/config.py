"""
Parámetros de configuración del pipeline Pure Pursuit — WRO Future Engineers.

Hardware:
  Cámara  : Raspberry Pi Camera v2  (FOV ~62°, resolución proceso 640×480)
  Motor   : N20 DC 50:1 + etapa LEGO 2:1  → ratio total 100:1
  Servo   : SG90  (centro = 80°, límite mecánico ±35° efectivo en rueda)
  Chassis : 210×140×80 mm  |  batalla ~100 mm (estimado)
  ESP32   : recibe protocolo V2 por UART Serial2 (RX=17, TX=16) @ 115200
  Pi GPIO : LED en 27, botón arranque en 17

Ajusta primero CALIB_REAL_MM para que coincida con tu montaje de cámara,
luego corre calibrate.py para generar bev_calib.npz.
"""

import numpy as np
from pathlib import Path

# ─── Rutas ────────────────────────────────────────────────────────────────────
CALIB_FILE = Path(__file__).parent / "bev_calib.npz"

# ─── BEV — imagen de salida ───────────────────────────────────────────────────
BEV_W      = 400          # píxeles de ancho
BEV_H      = 400          # píxeles de alto
MM_PER_PX  = 2.0          # escala: 1px = 2 mm  →  400px = 800 mm de cobertura

# Posición del robot en la imagen BEV (eje trasero, centro horizontal)
ROBOT_BEV_X = BEV_W // 2   # 200
ROBOT_BEV_Y = BEV_H - 20   # 380

# ─── Puntos de calibración en el suelo (mm, relativo a eje delantero del robot) ──
# x_mm: lateral  (+ = derecha del robot)
# y_mm: distancia hacia adelante desde el eje delantero
#
# Coloca marcadores físicos en el suelo en estas posiciones exactas y
# haz clic en ellos en el mismo orden dentro de calibrate.py:
#   A → izq-cerca    B → der-cerca    C → izq-lejos    D → der-lejos
CALIB_REAL_MM = np.float32([
    [-100.0, 200.0],   # A: 20 cm adelante, 10 cm izquierda
    [ 100.0, 200.0],   # B: 20 cm adelante, 10 cm derecha
    [-150.0, 380.0],   # C: 38 cm adelante, 15 cm izquierda
    [ 150.0, 380.0],   # D: 38 cm adelante, 15 cm derecha
])

# ─── Color del piso (HSV) — tapete WRO: beige / madera cálida ────────────────
FLOOR_LOWER = np.array([0,   0,  40])
FLOOR_UPPER = np.array([180, 60, 240])

# ─── Detección de centerline ──────────────────────────────────────────────────
CENTERLINE_ROW_STEP  = 15    # muestrear cada N filas
CENTERLINE_MIN_WIDTH = 20    # píxeles mínimos de espacio libre para aceptar fila
CENTERLINE_TOP_Y     = BEV_H // 3   # no subir más allá de 1/3 de la imagen

# ─── Manejo de obstáculos en BEV ─────────────────────────────────────────────
# Tamaño físico real de los obstáculos (latas de refresco WRO ≈ 65 mm diámetro)
OBS_REAL_DIAMETER_MM = 65.0
OBS_PHYSICAL_R_PX    = round(OBS_REAL_DIAMETER_MM / 2.0 / MM_PER_PX)  # ≈ 16 px
OBS_SAFETY_R_PX      = 19    # margen de seguridad adicional (px)
OBS_INFLATE_R        = OBS_PHYSICAL_R_PX + OBS_SAFETY_R_PX             # ≈ 35 px

OBS_BIAS_SHIFT = 28    # desplazamiento lateral para sesgo de color WRO (px)
#  Rojo  → el robot debe pasar por la DERECHA → se infla más a la izquierda
#  Verde → el robot debe pasar por la IZQUIERDA → se infla más a la derecha

# ─── Pure Pursuit ─────────────────────────────────────────────────────────────
LOOKAHEAD_PX   = 80.0    # distancia look-ahead en px BEV  (= 160 mm)
WHEELBASE_PX   = 50.0    # batalla del vehículo en px BEV   (= 100 mm)
MAX_STEER_DEG  = 35.0    # límite mecánico del servo en grados
MIN_PATH_PTS   = 4       # puntos mínimos de path para considerar PP válido

# ─── Protocolo serial ESP32 ───────────────────────────────────────────────────
# Cuando pp=1:  ESP32 usa ppSteerGain=35  →  obs=steer_deg/35
# Cuando pp=0:  ESP32 usa visionSteerGain=80  (comportamiento V1 actual)
PP_STEER_GAIN     = MAX_STEER_DEG   # 35.0

# ─── PID de fallback (igual que wro_runtime.py) ───────────────────────────────
RED_TARGET_PX    = 140
GREEN_TARGET_PX  = 500
PID_KP           = 0.012
PID_KD           = 0.004
CORR_LIMIT_PX    = 160.0

# ─── Cámara / captura ─────────────────────────────────────────────────────────
CAM_INDEX      = 1
SERIAL_PORT    = "/dev/ttyS0"
BAUDRATE       = 115200
PROCESS_EVERY  = 3       # procesar 1 de cada N frames capturados
WARMUP_FRAMES  = 40      # frames descartados para estabilizar exposición
