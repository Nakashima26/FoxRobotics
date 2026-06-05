#!/usr/bin/env python3
"""
Script de debug para verificar comunicación serial por GPIO 14 y 15 (UART0) en Raspberry Pi.
Verifica pines, abre puerto, y envía datos de prueba con logs detallados.
"""

import os
import sys
import time
import subprocess


def check_gpio_availability():
    """Verifica si los pines GPIO 14 y 15 están disponibles."""
    print("[DEBUG] === VERIFICANDO DISPONIBILIDAD DE PINES GPIO ===")

    try:
        result = subprocess.run(
            ["gpio", "readall"],
            capture_output=True,
            text=True,
            timeout=5
        )
        print("[DEBUG] gpio readall salida:")
        print(result.stdout)
        if "14" in result.stdout and "15" in result.stdout:
            print("[OK] Pines 14 y 15 encontrados en gpio readall")
        else:
            print("[WARN] Pines 14 y 15 NO encontrados en gpio readall")
    except FileNotFoundError:
        print("[WARN] gpio command no encontrado. Buscando /proc/device-tree...")
    except Exception as e:
        print(f"[WARN] Error al ejecutar gpio readall: {e}")

    # Verificar si la pi tiene /dev/ttyAMA0 o /dev/ttyS0
    print("\n[DEBUG] === VERIFICANDO PUERTOS SERIALES ===")
    for port in ["/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0"]:
        if os.path.exists(port):
            print(f"[OK] {port} EXISTE")
            # Intentar obtener permiso de lectura/escritura
            if os.access(port, os.R_OK):
                print(f"    - Permiso de lectura: OK")
            else:
                print(f"    - Permiso de lectura: DENEGADO")
            if os.access(port, os.W_OK):
                print(f"    - Permiso de escritura: OK")
            else:
                print(f"    - Permiso de escritura: DENEGADO")
        else:
            print(f"[WARN] {port} NO EXISTE")


def setup_serial_port(port: str, baudrate: int):
    """Configura el puerto serial usando termios."""
    print(f"\n[DEBUG] === CONFIGURANDO PUERTO SERIAL ===")
    print(f"[DEBUG] Puerto: {port}")
    print(f"[DEBUG] Baudrate: {baudrate}")

    try:
        import termios
        import fcntl

        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        print(f"[OK] Puerto abierto. File descriptor: {fd}")

        # Obtener atributos actuales
        attrs = termios.tcgetattr(fd)
        print(f"[DEBUG] Atributos originales obtenidos")

        # Configurar: 8 bits, 1 stop bit, sin paridad
        attrs[2] &= ~(termios.CSTOPB | termios.CSIZE)
        attrs[2] |= termios.CS8
        attrs[3] &= ~termios.ICANON  # Raw mode

        # Configurar timeouts
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 2

        # Configurar baudrate
        if baudrate == 115200:
            attrs[4] = attrs[5] = termios.B115200
            print(f"[DEBUG] Baudrate configurado a 115200")
        else:
            print(f"[WARN] Baudrate {baudrate} puede no estar soportado")
            attrs[4] = attrs[5] = termios.B115200

        # Aplicar configuración
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        print(f"[OK] Atributos aplicados correctamente")

        # Esperar a que se estabilice
        time.sleep(0.5)
        print(f"[DEBUG] Esperando estabilización...")

        return fd

    except Exception as e:
        print(f"[ERROR] Fallo al configurar puerto: {e}")
        return None


def send_test_data(fd: int, data: bytes):
    """Envía datos por el puerto serial."""
    if fd is None:
        print("[ERROR] File descriptor es None, no se puede enviar")
        return False

    try:
        bytes_sent = os.write(fd, data)
        print(f"[OK] {bytes_sent} bytes enviados")
        print(f"[DEBUG] Datos enviados (hex): {data.hex()}")
        print(f"[DEBUG] Datos enviados (str): {data.decode('utf-8', errors='replace')}")
        return True
    except Exception as e:
        print(f"[ERROR] Fallo al enviar datos: {e}")
        return False


def read_response(fd: int, timeout_s: float = 1.0):
    """Intenta leer respuesta del puerto serial."""
    if fd is None:
        print("[ERROR] File descriptor es None, no se puede leer")
        return None

    print(f"[DEBUG] Esperando respuesta por {timeout_s}s...")
    start = time.time()
    data = b""

    try:
        while time.time() - start < timeout_s:
            try:
                chunk = os.read(fd, 128)
                if chunk:
                    data += chunk
                    print(f"[DEBUG] Bytes recibidos: {len(chunk)}")
                    print(f"[DEBUG] Datos recibidos (hex): {chunk.hex()}")
                    print(f"[DEBUG] Datos recibidos (str): {chunk.decode('utf-8', errors='replace')}")
                else:
                    time.sleep(0.01)
            except BlockingIOError:
                time.sleep(0.01)

        if data:
            print(f"[OK] Total {len(data)} bytes recibidos")
            return data
        else:
            print(f"[WARN] No se recibió datos")
            return None

    except Exception as e:
        print(f"[ERROR] Error al leer: {e}")
        return None


def main():
    print("=" * 60)
    print("SERIAL DEBUG TOOL - Raspberry Pi GPIO 14/15 UART Test")
    print("=" * 60)

    PORT = "/dev/ttyAMA0"
    BAUDRATE = 115200

    # Verificar disponibilidad de pines
    check_gpio_availability()

    # Intentar abrir y configurar puerto
    fd = setup_serial_port(PORT, BAUDRATE)
    if fd is None:
        print("\n[FATAL] No se pudo abrir el puerto serial")
        sys.exit(1)

    try:
        # Prueba 1: Enviar datos simples
        print(f"\n[DEBUG] === PRUEBA 1: Envío de datos simples ===")
        test1_data = b"HOLA_PI\n"
        send_test_data(fd, test1_data)
        time.sleep(0.2)
        response1 = read_response(fd, timeout_s=0.5)

        # Prueba 2: Enviar números
        print(f"\n[DEBUG] === PRUEBA 2: Envío de números ===")
        test2_data = b"123,456,789\n"
        send_test_data(fd, test2_data)
        time.sleep(0.2)
        response2 = read_response(fd, timeout_s=0.5)

        # Prueba 3: Enviar formato V1 como el runtime original
        print(f"\n[DEBUG] === PRUEBA 3: Envío de mensaje V1 ===")
        test3_data = b"V1,obs=+0.123,turn=1,state=avoid_red,prio=1,mem=5\n"
        send_test_data(fd, test3_data)
        time.sleep(0.2)
        response3 = read_response(fd, timeout_s=0.5)

        # Prueba 4: Envío repetido para ver si la ESP responde en algún momento
        print(f"\n[DEBUG] === PRUEBA 4: Envíos repetidos (5 veces) ===")
        for i in range(5):
            print(f"\n[DEBUG] Envío #{i+1}")
            test_data = f"TEST_{i}\n".encode()
            send_test_data(fd, test_data)
            time.sleep(0.1)
            response = read_response(fd, timeout_s=0.3)
            if response:
                print(f"[OK] ¡Respuesta recibida en envío #{i+1}!")
                break

    finally:
        # Cerrar puerto
        print(f"\n[DEBUG] === CERRANDO PUERTO ===")
        try:
            os.close(fd)
            print(f"[OK] Puerto cerrado")
        except Exception as e:
            print(f"[WARN] Error al cerrar puerto: {e}")

    print("\n" + "=" * 60)
    print("Test completado. Si la ESP no responde:")
    print("1. Verifica que la ESP esté conectada correctamente")
    print("2. Verifica que TX (GPIO 14) → RX (ESP) y RX (GPIO 15) → TX (ESP)")
    print("3. Verifica que ambos dispositivos compartan GND")
    print("4. Prueba con minicom: minicom -D /dev/ttyAMA0 -b 115200")
    print("=" * 60)


if __name__ == "__main__":
    main()
