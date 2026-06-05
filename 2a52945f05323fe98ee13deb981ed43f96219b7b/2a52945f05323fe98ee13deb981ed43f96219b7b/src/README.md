Control software
====

This directory contains the competition software used by the vehicle.

Primary runtime:
- [src/RASPI/cam/wro.py](RASPI/cam/wro.py)

## Run commands

### Raspberry Pi

Start the integrated runtime:

```bash
cd ~/FoxRobotics
source .venv/bin/activate
python src/RASPI/cam/wro_runtime.py --cam-index 0 --serial-port /dev/ttyAMA0 --baudrate 115200 --protocol signed --threaded-capture
```

Start it with track-edge recording:

```bash
cd ~/FoxRobotics
source .venv/bin/activate
python src/RASPI/cam/wro_runtime.py --cam-index 0 --serial-port /dev/ttyAMA0 --baudrate 115200 --protocol signed --threaded-capture --record-orillas --record-output ./videos_orillas
```

### ESP32

Flash the integrated firmware once, then it runs on boot:

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 src/ESP32/Controller_PI.ino
arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 src/ESP32/Controller_PI.ino
```

Replace `COM5` with your ESP32 port.

### Autostart on Raspberry Pi

```bash
cd ~/FoxRobotics
chmod +x scripts/install_autostart_pi.sh
./scripts/install_autostart_pi.sh
```

All artifacts required to resolve dependencies and build the project must be included in this directory as well.