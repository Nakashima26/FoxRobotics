#!/usr/bin/env python3
"""
Diagnóstico completo de UART en Raspberry Pi.
"""

import os
import subprocess
import sys


def list_serial_devices():
    """Lista todos los dispositivos seriales disponibles."""
    print("[DEBUG] === BUSCANDO PUERTOS SERIALES ===\n")

    serial_devices = []
    for item in os.listdir("/dev"):
        if item.startswith("tty"):
            path = f"/dev/{item}"
            serial_devices.append(path)
            try:
                if os.access(path, os.R_OK):
                    read_perm = "✓"
                else:
                    read_perm = "✗"
                if os.access(path, os.W_OK):
                    write_perm = "✓"
                else:
                    write_perm = "✗"
                print(f"[OK] {path:20} | Lectura: {read_perm} | Escritura: {write_perm}")
            except Exception as e:
                print(f"[WARN] {path:20} | Error: {e}")

    if not serial_devices:
        print("[ERROR] ¡No se encontró ningún puerto serial!")
        return []

    return serial_devices


def check_uart_status():
    """Verifica el estado del UART en la configuración."""
    print("\n[DEBUG] === ESTADO ACTUAL DE UART ===\n")

    # Verificar si ttyAMA0 existe
    if os.path.exists("/dev/ttyAMA0"):
        print("[OK] /dev/ttyAMA0 EXISTE")
    else:
        print("[WARN] /dev/ttyAMA0 NO EXISTE - Probablemente deshabilitado en raspi-config")

    # Verificar alternativas
    if os.path.exists("/dev/ttyS0"):
        print("[OK] /dev/ttyS0 EXISTE - Esta es la alternativa principal")

    # Intentar leer device tree
    print("\n[DEBUG] Leyendo /proc/device-tree/aliases/serial0...")
    try:
        with open("/proc/device-tree/aliases/serial0", "r") as f:
            content = f.read().strip()
            print(f"[OK] serial0 apunta a: {content}")
    except FileNotFoundError:
        print("[WARN] /proc/device-tree/aliases/serial0 no encontrado")
    except Exception as e:
        print(f"[WARN] Error: {e}")

    # Verificar si el puerto está en uso
    print("\n[DEBUG] === COMPROBANDO SI EL PUERTO ESTÁ EN USO ===\n")
    for port in ["/dev/ttyAMA0", "/dev/ttyS0"]:
        try:
            result = subprocess.run(
                ["lsof", port],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.stdout:
                print(f"[WARN] {port} está en uso por:")
                print(result.stdout)
            else:
                print(f"[OK] {port} está LIBRE (si existe)")
        except FileNotFoundError:
            print("[DEBUG] 'lsof' no disponible, omitiendo...")
        except Exception as e:
            print(f"[DEBUG] Error al ejecutar lsof: {e}")


def check_gpio_pins():
    """Verifica el estado de los pines GPIO 14 y 15."""
    print("\n[DEBUG] === ESTADO DE PINES GPIO 14 Y 15 ===\n")

    gpio_paths = [
        "/sys/class/gpio/gpio14",
        "/sys/class/gpio/gpio15"
    ]

    for path in gpio_paths:
        gpio_num = path.split("/")[-1]
        if os.path.exists(path):
            print(f"[OK] {gpio_num} EXISTE")
            direction_file = f"{path}/direction"
            if os.path.exists(direction_file):
                with open(direction_file, "r") as f:
                    direction = f.read().strip()
                    print(f"    Dirección: {direction}")
        else:
            print(f"[INFO] {gpio_num} no exportado (normal si está en uso por UART)")


def show_solutions():
    """Muestra soluciones según lo encontrado."""
    print("\n" + "=" * 70)
    print("SOLUCIONES:")
    print("=" * 70)

    print("\n1. HABILITAR UART0 EN RASPI-CONFIG:")
    print("   $ sudo raspi-config")
    print("   → Interfacing Options → Serial")
    print("   → NO para shell por serial")
    print("   → SÍ para puerto serial hardware")
    print("   → Reiniciar: sudo reboot")

    print("\n2. ALTERNATIVA (si solo aparece /dev/ttyS0):")
    print("   Editar /boot/config.txt y agregar:")
    print("   [all]")
    print("   dtoverlay=uart0")
    print("   enable_uart=1")

    print("\n3. VERIFICAR CONEXIONES:")
    print("   Pi GPIO 14 (TX) → ESP RX")
    print("   Pi GPIO 15 (RX) → ESP TX")
    print("   Pi GND → ESP GND")

    print("\n4. PROBAR CON MINICOM:")
    print("   $ sudo apt-get install minicom")
    print("   $ minicom -D /dev/ttyS0 -b 115200")
    print("   (o ttyAMA0 si aparece)")

    print("\n5. SCRIPT PARA FORZAR UART (RIESGO):")
    print("   $ sudo nano /boot/cmdline.txt")
    print("   Remover 'console=serial0,115200' si existe")
    print("=" * 70)


def main():
    print("\n" + "=" * 70)
    print("DIAGNÓSTICO DE UART - RASPBERRY PI")
    print("=" * 70 + "\n")

    if os.geteuid() != 0:
        print("[WARN] Este script NO está corriendo como root.")
        print("       Algunos permisos pueden no ser detectados correctamente.")
        print("       Considera ejecutar: sudo python3 diagnose_uart.py\n")

    # Ejecutar diagnósticos
    serial_devices = list_serial_devices()
    check_uart_status()
    check_gpio_pins()
    show_solutions()

    print("\n[NEXT] Después de habilitar UART:")
    print("       python3 test_serial_debug.py\n")


if __name__ == "__main__":
    main()
