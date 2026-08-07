 # Pure Pursuit — Instrucciones de uso

Todos los comandos se corren desde `src/RASPI/cam/`.

---

## 1. Calibrar la cámara (una sola vez)

Necesitas la cámara montada en su posición final en el robot, apuntando al piso.

Coloca 4 marcadores físicos (papel, cinta) en el suelo en estas posiciones relativas al eje delantero del robot:

| Punto | Lateral | Adelante |
|-------|---------|----------|
| A     | 10 cm izquierda | 20 cm |
| B     | 10 cm derecha   | 20 cm |
| C     | 15 cm izquierda | 38 cm |
| D     | 15 cm derecha   | 38 cm |

```bash
python -m pure_pursuit.calibrate
```

1. Se abre una ventana con la cámara en vivo
2. Presiona `C` para congelar el frame
3. Haz clic en los 4 marcadores en orden: A → B → C → D
4. Se abre una segunda ventana con la vista BEV en tiempo real — verifica que se vea bien
5. Presiona `S` para guardar → genera `pure_pursuit/bev_calib.npz`
6. Presiona `R` para rehacer si los puntos quedaron mal

> Si mueves o cambias el ángulo de la cámara, repite este paso.

---

## 2. Calibrar colores HSV (antes de cada competencia)

La iluminación del venue cambia los colores. Ajusta los rangos en `config.py`:

```python
# Piso blanco WRO
FLOOR_LOWER = np.array([0,   0,  160])
FLOOR_UPPER = np.array([180, 50, 255])
```

Si el blanco del piso se ve grisáceo o amarillento bajo las luces del venue, baja `FLOOR_LOWER[2]` hasta 140 o 120.

Los rangos de rojo y verde están en `vision.py` (el archivo de visión principal).

---

## 3. Probar solo la visión (sin robot, sin ESP32)

Útil para verificar que la BEV y la centerline se ven bien antes de conectar todo.

```bash
python -m pure_pursuit.test_vision
```

Con un video grabado en lugar de cámara en vivo:
```bash
python -m pure_pursuit.test_vision --video ruta/al/video.mp4
```

Lo que deberías ver:
- Ventana izquierda: cámara con los obstáculos detectados (rojo/verde marcados)
- Ventana derecha: BEV con la centerline en cyan, punto look-ahead en amarillo, robot en naranja
- Texto arriba: `pp=ON`, ángulo de dirección, cantidad de puntos del path

Presiona `S` en cualquier momento para guardar un screenshot.
Presiona `ESC` para salir.

---

## 4. Correr en el robot real

### Requisitos previos
- `bev_calib.npz` generado (paso 1)
- ESP32 con `PurePursuit.ino` ya flasheado
- Pi conectada al ESP32 por Serial2 (RX=17, TX=16)

### Comando

```bash
python -m pure_pursuit.runtime
```

### Secuencia de arranque automática
1. LED en GPIO27 enciende → Pi lista
2. Espera que presiones el botón en GPIO17
3. Calienta la cámara ~2 segundos
4. Manda `READY` al ESP32 → ESP32 responde y arranca motores
5. Loop: cámara → BEV → centerline → Pure Pursuit → serial al ESP32

### Flags útiles

```bash
# Sin ventana (en competencia sin monitor)
python -m pure_pursuit.runtime --no-window

# Grabar video de lo que ve la cámara
python -m pure_pursuit.runtime --no-window --record-orillas

# Puerto serial diferente (si usas adaptador USB)
python -m pure_pursuit.runtime --serial-port /dev/ttyUSB0
```

---

## Resumen de archivos

| Archivo | Para qué |
|---------|----------|
| `calibrate.py` | Calibrar la homografía BEV con la cámara real |
| `test_vision.py` | Probar visión + centerline sin robot ni serial |
| `runtime.py` | Runtime completo para competencia |
| `config.py` | Todos los parámetros ajustables (HSV, lookahead, gains) |
| `bev_calib.npz` | Archivo de calibración generado por calibrate.py |
