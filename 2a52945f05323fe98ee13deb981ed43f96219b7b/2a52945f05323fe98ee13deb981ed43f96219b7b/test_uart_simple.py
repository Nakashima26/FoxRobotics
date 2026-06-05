#!/usr/bin/env python3
"""
Test simple: manda un '1' por UART (GPIO 14=TX, 15=RX) a la ESP.
"""

import serial
import time

PORT = "/dev/ttyAMA0"   # cambia a /dev/ttyS0 si no existe ttyAMA0
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"[OK] Puerto abierto: {PORT} @ {BAUD}")
    time.sleep(0.5)

    while True:
        ser.write(b"1")
        print("[TX] Mandé: 1")
        time.sleep(1)

except serial.SerialException as e:
    print(f"[ERROR] No se pudo abrir {PORT}: {e}")
    print("        Prueba con /dev/ttyS0 o corre con sudo")
except KeyboardInterrupt:
    print("\n[EXIT] Interrumpido por usuario")
finally:
    try:
        ser.close()
    except:
        pass
