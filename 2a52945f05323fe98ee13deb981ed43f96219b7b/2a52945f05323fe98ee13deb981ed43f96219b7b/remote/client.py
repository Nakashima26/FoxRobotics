"""
Cliente remoto - corre en LA RASPBERRY PI.
Captura la pantalla y la manda al servidor. Recibe comandos de mouse/teclado.

Dependencias (instalar en la Pi):
    pip install websockets mss Pillow
    sudo apt install xdotool

Uso manual:
    REMOTE_SERVER=ws://192.168.1.XX:8765 python remote/client.py

O configurar REMOTE_SERVER en el archivo .service de systemd.
"""
import asyncio
import io
import json
import os
import subprocess
import sys
import websockets
import mss
from PIL import Image

SERVER_URL = os.environ.get("REMOTE_SERVER", "ws://192.168.1.100:8765")
FPS = 15
JPEG_QUALITY = 70
FRAME_SIZE = (1280, 720)

KEY_MAP = {
    "Enter": "Return", "Backspace": "BackSpace", "Escape": "Escape",
    "Tab": "Tab", "ArrowUp": "Up", "ArrowDown": "Down",
    "ArrowLeft": "Left", "ArrowRight": "Right", " ": "space",
    "Delete": "Delete", "Home": "Home", "End": "End",
    "PageUp": "Prior", "PageDown": "Next",
    "F1": "F1", "F2": "F2", "F3": "F3", "F4": "F4",
}


def execute_command(cmd: dict):
    t = cmd.get("type")
    if t == "mousemove":
        subprocess.run(
            ["xdotool", "mousemove", "--sync", str(cmd["x"]), str(cmd["y"])],
            check=False, capture_output=True,
        )
    elif t == "click":
        subprocess.run(
            ["xdotool", "mousemove", str(cmd["x"]), str(cmd["y"])],
            check=False, capture_output=True,
        )
        subprocess.run(
            ["xdotool", "click", str(cmd.get("button", 1))],
            check=False, capture_output=True,
        )
    elif t == "key":
        key = cmd.get("key", "")
        xkey = KEY_MAP.get(key, key) if len(key) > 1 else key
        subprocess.run(["xdotool", "key", xkey], check=False, capture_output=True)


async def stream_screen(ws):
    interval = 1.0 / FPS
    with mss.mss() as sct:
        monitor = sct.monitors[1]
        while True:
            shot = sct.grab(monitor)
            img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
            img = img.resize(FRAME_SIZE, Image.LANCZOS)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=JPEG_QUALITY)
            await ws.send(buf.getvalue())
            await asyncio.sleep(interval)


async def handle_commands(ws):
    async for msg in ws:
        if isinstance(msg, str):
            try:
                execute_command(json.loads(msg))
            except Exception as e:
                print(f"[client] Error ejecutando comando: {e}")


async def main():
    url = f"{SERVER_URL}/ws/pi"
    print(f"[client] Conectando a {url}")
    while True:
        try:
            async with websockets.connect(url, ping_interval=20) as ws:
                print("[client] Conectado")
                await asyncio.gather(stream_screen(ws), handle_commands(ws))
        except Exception as e:
            print(f"[client] Desconectado: {e}. Reintentando en 5s...")
            await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())
