"""
Servidor remoto - corre en TU COMPUTADORA.
Levanta un viewer en el browser para ver y controlar la Pi.

Uso:
    pip install fastapi uvicorn
    python remote/server.py

Luego abre: http://localhost:8765
"""
import asyncio
import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

app = FastAPI()

frame_data: bytes | None = None
pi_ws: WebSocket | None = None
viewer_connections: set[WebSocket] = set()

HTML = """<!DOCTYPE html>
<html>
<head>
  <title>FoxRobotics Remote</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { background: #111; display: flex; flex-direction: column; align-items: center; height: 100vh; }
    #bar { width: 100%; background: #222; color: #aaa; font-family: monospace; font-size: 13px;
           padding: 4px 12px; display: flex; gap: 16px; align-items: center; }
    #dot { width: 8px; height: 8px; border-radius: 50%; background: #555; }
    #dot.on { background: #4f4; }
    #screen { max-width: 100%; max-height: calc(100vh - 28px); object-fit: contain; cursor: crosshair; display: block; }
  </style>
</head>
<body>
  <div id="bar">
    <div id="dot"></div>
    <span id="status">Esperando Pi...</span>
  </div>
  <img id="screen" />
  <script>
    const img = document.getElementById('screen');
    const status = document.getElementById('status');
    const dot = document.getElementById('dot');
    const ws = new WebSocket(`ws://${location.host}/ws/viewer`);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => { status.textContent = 'Conectado'; dot.classList.add('on'); };
    ws.onclose = () => { status.textContent = 'Desconectado'; dot.classList.remove('on'); };

    ws.onmessage = (e) => {
      const blob = new Blob([e.data], { type: 'image/jpeg' });
      const url = URL.createObjectURL(blob);
      const old = img.src;
      img.src = url;
      if (old) URL.revokeObjectURL(old);
    };

    function imgCoords(e) {
      const r = img.getBoundingClientRect();
      return {
        x: Math.round((e.clientX - r.left) / r.width * (img.naturalWidth || 1280)),
        y: Math.round((e.clientY - r.top) / r.height * (img.naturalHeight || 720)),
      };
    }

    let lastMove = 0;
    img.addEventListener('mousemove', (e) => {
      const now = Date.now();
      if (now - lastMove < 50) return;
      lastMove = now;
      const {x, y} = imgCoords(e);
      if (ws.readyState === 1) ws.send(JSON.stringify({type: 'mousemove', x, y}));
    });

    img.addEventListener('mousedown', (e) => {
      e.preventDefault();
      const {x, y} = imgCoords(e);
      if (ws.readyState === 1) ws.send(JSON.stringify({type: 'click', x, y, button: e.button + 1}));
    });

    img.addEventListener('contextmenu', (e) => e.preventDefault());

    document.addEventListener('keydown', (e) => {
      if (document.activeElement === document.body || document.activeElement === img) {
        e.preventDefault();
        if (ws.readyState === 1) ws.send(JSON.stringify({type: 'key', key: e.key}));
      }
    });
  </script>
</body>
</html>"""


@app.get("/")
async def index():
    return HTMLResponse(HTML)


@app.websocket("/ws/pi")
async def pi_endpoint(ws: WebSocket):
    global pi_ws, frame_data
    await ws.accept()
    pi_ws = ws
    frames_received = 0
    print("[server] Pi conectada")
    try:
        while True:
            msg = await ws.receive()
            if "bytes" in msg:
                frame_data = msg["bytes"]
                frames_received += 1
                if frames_received % 30 == 0:
                    print(f"[server] {frames_received} frames recibidos, {len(frame_data)} bytes, viewers: {len(viewer_connections)}")
                dead = set()
                for viewer in viewer_connections:
                    try:
                        await viewer.send_bytes(frame_data)
                    except Exception:
                        dead.add(viewer)
                viewer_connections.difference_update(dead)
    except (WebSocketDisconnect, RuntimeError):
        print("[server] Pi desconectada")
        pi_ws = None


@app.websocket("/ws/viewer")
async def viewer_endpoint(ws: WebSocket):
    await ws.accept()
    viewer_connections.add(ws)
    if frame_data:
        await ws.send_bytes(frame_data)
    try:
        while True:
            msg = await ws.receive_text()
            try:
                cmd = json.loads(msg)
                if cmd.get("type") != "mousemove":
                    print(f"[server] cmd → Pi: {cmd}")
            except Exception:
                pass
            if pi_ws:
                await pi_ws.send_text(msg)
    except (WebSocketDisconnect, RuntimeError):
        viewer_connections.discard(ws)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    print(f"[server] Viewer en http://localhost:{args.port}")
    uvicorn.run(app, host="0.0.0.0", port=args.port)
