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
# IMPORTANTE — por qué 9 puntos y no 4:
#   Una homografía con exactamente 4 puntos es un ajuste EXACTO: no hay forma
#   de detectar ni corregir un clic impreciso durante la calibración, y la
#   transformación se vuelve muy poco confiable fuera del área que cubren esos
#   4 puntos (extrapolación agresiva).  Con 9 puntos en grilla 3×3,
#   cv2.findHomography(..., RANSAC) puede promediar/filtrar el ruido de los
#   clics, y el área cubierta coincide con el rango real que usa
#   centerline.py (desde muy cerca del robot hasta ~550mm adelante).
#
#   Coloca marcadores físicos en el suelo en estas posiciones exactas y haz
#   clic en ellos EN ESTE MISMO ORDEN dentro de calibrate.py.
#
#          izquierda        centro         derecha
#   lejos:   P7 (-220,550)  P8 (0,550)   P9 (220,550)
#   media:   P4 (-150,300)  P5 (0,300)   P6 (150,300)
#   cerca:   P1  (-80, 80)  P2 (0, 80)   P3  (80, 80)
CALIB_REAL_MM = np.float32([
    [ -80.0,  80.0],   # P1: cerca-izquierda
    [   0.0,  80.0],   # P2: cerca-centro
    [  80.0,  80.0],   # P3: cerca-derecha
    [-150.0, 300.0],   # P4: media-izquierda
    [   0.0, 300.0],   # P5: media-centro
    [ 150.0, 300.0],   # P6: media-derecha
    [-220.0, 550.0],   # P7: lejos-izquierda
    [   0.0, 550.0],   # P8: lejos-centro
    [ 220.0, 550.0],   # P9: lejos-derecha
])

# Etiquetas legibles para cada punto (mismo orden que CALIB_REAL_MM)
CALIB_POINT_LABELS = [
    "P1 cerca-izq", "P2 cerca-centro", "P3 cerca-der",
    "P4 media-izq", "P5 media-centro", "P6 media-der",
    "P7 lejos-izq", "P8 lejos-centro", "P9 lejos-der",
]

# Umbral de error medio de reproyección (px BEV) a partir del cual calibrate.py
# advierte que probablemente un clic quedó mal puesto.
CALIB_MAX_MEAN_ERR_PX = 4.0

WALL_MARGIN_PX = 22

# ─── Color del piso (HSV) — tapete WRO: beige / madera cálida ────────────────
FLOOR_LOWER = np.array([0, 0, 140])
FLOOR_UPPER = np.array([35, 80, 255])

FLOOR_LOWER_BLUE = np.array([95, 50, 10])
FLOOR_UPPER_BLUE = np.array([150, 255, 220])

FLOOR_LOWER_BLUE_WIDE = np.array([90, 15, 60])   # baja el mínimo de S y sube V
FLOOR_UPPER_BLUE_WIDE = np.array([150, 255, 230])

WALL_STRUCTURE_PX = 15   # <-- ajusta viendo tu imagen BEV real

FLOOR_LOWER_ORANGE = np.array([1,  70, 130])
FLOOR_UPPER_ORANGE = np.array([17, 160, 220])

# Lista usada por centerline.py — agrega más tuplas aquí si detectas más
# colores de piso que no quieres que bloqueen la ruta.
FLOOR_COLOR_RANGES = [
    (FLOOR_LOWER, FLOOR_UPPER),
    (FLOOR_LOWER_BLUE, FLOOR_UPPER_BLUE),
    (FLOOR_LOWER_ORANGE, FLOOR_UPPER_ORANGE),
]

# ─── Detección de centerline ──────────────────────────────────────────────────
CENTERLINE_ROW_STEP  = 15    # muestrear cada N filas
CENTERLINE_MIN_WIDTH = 20    # píxeles mínimos de espacio libre para aceptar fila
CENTERLINE_TOP_Y     = BEV_H // 3   # no subir más allá de 1/3 de la imagen
CENTERLINE_RAMP_PX   = 200   # horizonte de anticipo: empieza a abrir el path
                             # hacia el lado de paso a esta distancia Y de la lata
                             # (200 px ≈ 400 mm; debe ser > LOOKAHEAD_PX)
CENTERLINE_SMOOTH_WIN = 5    # ventana (impar) de media móvil sobre X post-muestreo

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
LOOKAHEAD_PX   = 100.0    # distancia look-ahead en px BEV  (= 160 mm)
WHEELBASE_PX   = 50.0    # batalla del vehículo en px BEV   (= 100 mm)
MAX_STEER_DEG  = 60.0    # límite mecánico del servo en grados
MIN_PATH_PTS   = 4       # puntos mínimos de path para considerar PP válido

# Lookahead variable
LOOKAHEAD_MIN_PX      = 45.0    # lookahead mínimo -> steer más agresivo posible
LOOKAHEAD_MAX_PX      = 100.0   # lookahead normal en pista libre
LOOKAHEAD_OBS_NEAR_PX = 120.0   # obstáculo a esta distancia (o menos) del robot -> lookahead mínimo
LOOKAHEAD_OBS_FAR_PX  = 260.0   # obstáculo a esta distancia (o más) -> ya no afecta el lookahead

CENTERLINE_URGENCY_RELAX = 3.0

# ─── Memoria de obstáculos (mapa rodante disperso) — obstacle_memory.py ───────
# El robot recuerda las latas vistas y las "arrastra" hacia sí mismo cuadro a
# cuadro usando avance asumido (velocidad) + giro del IMU (anguloGyro del ESP32),
# para no perder la inflación cuando la lata sale del campo de visión.
ROBOT_SPEED_MMS    = 350.0   # velocidad de marcha asumida (mm/s). Ajustar al carro real.
OBS_MEM_MATCH_PX   = 50.0    # radio para fusionar una detección nueva con una recordada (px BEV)
OBS_MEM_DECAY      = 0.12    # confianza perdida por frame sin re-ver el obstáculo (0..1)
OBS_MEM_MIN_CONF   = 0.4    # por debajo de esto el obstáculo recordado se descarta
OBS_MEM_REFRESH    = 1.0     # confianza al re-detectar (se satura en 1.0)
OBS_MEM_BEHIND_PAD = 10      # px: tirar el obstáculo cuando queda detrás del robot (bev_y > robot_y + pad)
OBS_MEM_MAX        = 12      # tope de obstáculos recordados (seguridad)
OBS_MEM_DEDUPE_PX  = 40.0 

# ─── Hint direccional (obstáculo lejano, fuera de rango BEV) ─────────────────
# Un objeto rojo/verde detectado en la imagen de cámara CRUDA (no en BEV) que
# todavía no proyecta dentro del rango calibrado.  Se usa SOLO para empezar a
# centrar el steer con anticipación.
FAR_HINT_ENABLED     = True
FAR_HINT_MIN_AREA_PX = 1200    # área mínima del bbox en cámara para confiar (ruido/falsos positivos)
FAR_HINT_MAX_STEER   = 12.0     # grados máx que puede aportar el hint (<< MAX_STEER_DEG)
FAR_HINT_KP          = 0.015   # ganancia proporcional: grados por px de offset
FAR_HINT_KD          = 0.004   # ganancia derivativa: amortigua saltos por ruido de detección
CAM_CENTER_X         = 320     # centro horizontal del frame de cámara (640/2)

# ─── Protocolo serial ESP32 ───────────────────────────────────────────────────
# Cuando pp=1:  ESP32 usa ppSteerGain=35  →  obs=steer_deg/35
# Cuando pp=0:  ESP32 usa visionSteerGain=80  (comportamiento V1 actual)
PP_STEER_GAIN     = MAX_STEER_DEG   # 35.0

# ─── PID de fallback (igual que wro_runtime.py) ───────────────────────────────
RED_TARGET_PX    = 140
GREEN_TARGET_PX  = 500
PID_KP           = 1.000
PID_KD           = 0.33
CORR_LIMIT_PX    = 160.0

# ─── Cámara / captura ─────────────────────────────────────────────────────────
CAM_INDEX      = 1
SERIAL_PORT    = "/dev/ttyS0"
BAUDRATE       = 115200
PROCESS_EVERY  = 3       # procesar 1 de cada N frames capturados
WARMUP_FRAMES  = 40      # frames descartados para estabilizar exposición
