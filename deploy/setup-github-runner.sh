#!/bin/bash
# Setup script para GitHub Actions Self-Hosted Runner en Raspberry Pi
# Uso: bash setup-github-runner.sh <GITHUB_TOKEN>

set -e

if [ -z "$1" ]; then
    echo "❌ Error: Se requiere el token de GitHub"
    echo "Uso: bash setup-github-runner.sh <GITHUB_TOKEN>"
    echo ""
    echo "Para obtener el token:"
    echo "1. Ve a GitHub → Settings → Developer settings → Personal access tokens"
    echo "2. Crea uno nuevo con permisos 'repo' y 'workflow'"
    exit 1
fi

GITHUB_TOKEN="$1"
GITHUB_REPO="https://github.com/$(git config --get remote.origin.url | sed 's/.*github.com[:/]\([^/]*\/[^/]*\).*/\1/')"
RUNNER_DIR="/home/user/github-runner"
RUNNER_USER="user"

echo "🚀 Instalando GitHub Actions Self-Hosted Runner"
echo "📍 Repositorio: $GITHUB_REPO"
echo "👤 Usuario: $RUNNER_USER"
echo "📁 Directorio: $RUNNER_DIR"
echo ""

# 1. Crear directorio
echo "1️⃣ Creando directorio del runner..."
sudo mkdir -p "$RUNNER_DIR"
sudo chown "$RUNNER_USER:$RUNNER_USER" "$RUNNER_DIR"

# 2. Descargar runner
cd "$RUNNER_DIR"
echo "2️⃣ Descargando GitHub Actions Runner..."

# Detectar arquitectura
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    RUNNER_URL="https://github.com/actions/runner/releases/download/v2.121.0/actions-runner-linux-arm64-2.121.0.tar.gz"
elif [ "$ARCH" = "armv7l" ]; then
    RUNNER_URL="https://github.com/actions/runner/releases/download/v2.121.0/actions-runner-linux-arm-2.121.0.tar.gz"
else
    RUNNER_URL="https://github.com/actions/runner/releases/download/v2.121.0/actions-runner-linux-x64-2.121.0.tar.gz"
fi

echo "   Arquitectura detectada: $ARCH"
echo "   Descargando desde: $RUNNER_URL"

curl -s -L -o actions-runner.tar.gz "$RUNNER_URL"
tar xzf actions-runner.tar.gz
rm actions-runner.tar.gz

# 3. Instalar dependencias
echo "3️⃣ Instalando dependencias..."
sudo apt-get update -qq
sudo apt-get install -y -qq libssl-dev libffi-dev python3-dev >/dev/null 2>&1

# 4. Configurar runner
echo "4️⃣ Configurando runner..."
./config.sh --url "$GITHUB_REPO" --token "$GITHUB_TOKEN" --name "WRO-Pi-Runner" --work "_work" --unattended --replace

# 5. Instalar como servicio
echo "5️⃣ Instalando como servicio systemd..."
sudo ./svc.sh install "$RUNNER_USER"

# 6. Iniciar servicio
echo "6️⃣ Iniciando runner..."
sudo systemctl start actions.runner.*.service

# 7. Habilitar en boot
echo "7️⃣ Habilitando en arranque..."
sudo systemctl enable actions.runner.*.service

# 8. Verificar estado
echo ""
echo "8️⃣ Verificando estado..."
sleep 2
sudo systemctl status actions.runner.*.service --no-pager || true

echo ""
echo "✅ ¡Runner instalado correctamente!"
echo ""
echo "📋 Próximos pasos:"
echo "1. Ve a GitHub → Settings → Actions → Runners"
echo "2. Deberías ver 'WRO-Pi-Runner' en estado 'Idle'"
echo "3. Cambia 'runs-on: ubuntu-latest' a 'runs-on: self-hosted' en .github/workflows/deploy-pi.yml"
echo "4. Haz un push para probar"
echo ""
echo "Para ver logs:"
echo "  sudo journalctl -u actions.runner.*.service -f"
echo ""
echo "Para detener el runner:"
echo "  sudo systemctl stop actions.runner.*.service"
