#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/pi/FoxRobotics"
SERVICE_SRC="$REPO_DIR/deploy/systemd/wro-runtime.service"
SERVICE_DST="/etc/systemd/system/wro-runtime.service"

if [[ ! -f "$SERVICE_SRC" ]]; then
  echo "No existe: $SERVICE_SRC"
  exit 1
fi

echo "Copiando servicio..."
sudo cp "$SERVICE_SRC" "$SERVICE_DST"

echo "Recargando systemd..."
sudo systemctl daemon-reload

echo "Habilitando servicio al arranque..."
sudo systemctl enable wro-runtime.service

echo "Iniciando servicio ahora..."
sudo systemctl restart wro-runtime.service

echo "Estado actual:"
sudo systemctl status wro-runtime.service --no-pager

echo "Logs en vivo (Ctrl+C para salir):"
echo "sudo journalctl -u wro-runtime.service -f"
