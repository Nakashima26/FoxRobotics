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
import time
import traceback
import websockets
import mss
from PIL import Image

SERVER_URL = os.environ.get("REMOTE_SERVER", "ws://100.80.180.4:8765")
FPS = 15
JPEG_QUALITY = 70
FRAME_SIZE = (1280, 720)
CAM_FRAME_PATH = "/tmp/wro_cam_frame.jpg"
CAM_FRAME_MAX_AGE = 2.0

KEY_MAP = {
    "Enter": "Return", "Backspace": "BackSpace", "Escape": "Escape",
    "Tab": "Tab", "ArrowUp": "Up", "ArrowDown": "Down",
    "ArrowLeft": "Left", "ArrowRight": "Right", " ": "space",
    "Delete": "Delete", "Home": "Home", "End": "End",
    "PageUp": "Prior", "PageDown": "Next",
    "F1": "F1", "F2": "F2", "F3": "F3", "F4": "F4",
}

print(f"[client] DISPLAY={os.environ.get('DISPLAY', 'NO SETEADO')}")
print(f"[client] XAUTHORITY={os.environ.get('XAUTHORITY', 'NO SETEADO')}")
print(f"[client] USER={os.environ.get('USER', os.environ.get('LOGNAME', '?'))}")
print(f"[client] REMOTE_SERVER={SERVER_URL}")


screen_w, screen_h = 1024, 768


def scale(x, y):
    return round(x * screen_w / FRAME_SIZE[0]), round(y * screen_h / FRAME_SIZE[1])


def execute_command(cmd: dict):
    t = cmd.get("type")
    if t == "mousemove":
        sx, sy = scale(cmd["x"], cmd["y"])
        subprocess.run(
            ["xdotool", "mousemove", "--sync", str(sx), str(sy)],
            check=False, capture_output=True,
        )
    elif t == "click":
        sx, sy = scale(cmd["x"], cmd["y"])
        subprocess.run(
            ["xdotool", "mousemove", str(sx), str(sy)],
            check=False, capture_output=True,
        )
        subprocess.run(
            ["xdotool", "click", str(cmd.get("button", 1))],
            check=False, capture_output=True,
        )
    elif t == "key":
        key = cmd.get("key", "")
        shift = cmd.get("shift", False)
        ctrl = cmd.get("ctrl", False)
        alt = cmd.get("alt", False)
        xkey = KEY_MAP.get(key, key)
        
        # Construir modificadores para xdotool
        modifiers = []
        if ctrl:
            modifiers.append("ctrl")
        if alt:
            modifiers.append("alt")
        if shift:
            modifiers.append("shift")
        
        # Si es una tecla especial (en KEY_MAP o tiene nombre largo), usar 'key'
        if key in KEY_MAP or len(key) > 1:
            if modifiers:
                xdotool_cmd = modifiers + [xkey]
                subprocess.run(["xdotool", "key"] + xdotool_cmd, check=False, capture_output=True)
            else:
                subprocess.run(["xdotool", "key", xkey], check=False, capture_output=True)
        else:
            # Para caracteres normales, usar 'type' (maneja @, #, !, etc.)
            # Nota: type no soporta bien modificadores, pero rara vez se necesita
            subprocess.run(["xdotool", "type", key], check=False, capture_output=True)
    elif t == "clipboard":
        text = cmd.get("text", "")
        try:
            # Escribir en el portapapeles de la Pi usando xclip
            process = subprocess.Popen(
                ["xclip", "-selection", "clipboard"],
                stdin=subprocess.PIPE,
                check=False
            )
            process.communicate(input=text.encode('utf-8'))
            # Simular Ctrl+V para pegar
            subprocess.run(["xdotool", "key", "ctrl+v"], check=False, capture_output=True)
        except Exception as e:
            print(f"[client] Error al pegar desde portapapeles: {e}")


def get_cam_overlay():
    try:
        if time.time() - os.path.getmtime(CAM_FRAME_PATH) > CAM_FRAME_MAX_AGE:
            return None
        with open(CAM_FRAME_PATH, "rb") as f:
            data = f.read()
        cam_img = Image.open(io.BytesIO(data)).convert("RGB")
        return cam_img.resize((FRAME_SIZE[0] // 4, FRAME_SIZE[1] // 4), Image.LANCZOS)
    except Exception:
        return None


async def stream_screen(ws):
    interval = 1.0 / FPS
    print("[client] Abriendo mss...")
    try:
        with mss.MSS() as sct:
            print(f"[client] Monitores: {sct.monitors}")
            monitor = sct.monitors[0]
            print(f"[client] Monitor activo: {monitor}")
            global screen_w, screen_h
            screen_w = monitor["width"]
            screen_h = monitor["height"]
            print(f"[client] Resolución real: {screen_w}x{screen_h}")
            frame_count = 0
            while True:
                try:
                    shot = sct.grab(monitor)
                    img = Image.frombytes("RGB", shot.size, shot.bgra, "raw", "BGRX")
                    img = img.resize(FRAME_SIZE, Image.LANCZOS)
                    cam = get_cam_overlay()
                    if cam:
                        img.paste(cam, (FRAME_SIZE[0] - cam.width - 4, FRAME_SIZE[1] - cam.height - 4))
                    buf = io.BytesIO()
                    img.save(buf, format="JPEG", quality=JPEG_QUALITY)
                    await ws.send(buf.getvalue())
                    frame_count += 1
                    if frame_count % 30 == 0:
                        print(f"[client] {frame_count} frames enviados")
                except Exception as e:
                    print(f"[client] Error capturando frame: {e}")
                    traceback.print_exc()
                    raise
                await asyncio.sleep(interval)
    except Exception as e:
        print(f"[client] Error fatal en stream_screen: {e}")
        traceback.print_exc()
        raise


async def handle_commands(ws):
    async for msg in ws:
        if isinstance(msg, str):
            try:
                cmd = json.loads(msg)
                if cmd.get("type") != "mousemove":
                    print(f"[client] cmd recibido: {cmd}")
                execute_command(cmd)
            except Exception as e:
                print(f"[client] Error ejecutando comando: {e}")


async def main():
    url = f"{SERVER_URL}/ws/pi"
    print(f"[client] Conectando a {url}")
    while True:
        try:
            async with websockets.connect(url, ping_interval=20) as ws:
                print("[client] Conectado al servidor")
                await asyncio.gather(stream_screen(ws), handle_commands(ws))
        except Exception as e:
            print(f"[client] Desconectado: {e}. Reintentando en 5s...")
            traceback.print_exc()
            await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())
