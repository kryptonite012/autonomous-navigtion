# Installation

## 1. Development machine (simulation, any OS with Python 3.11+ and Node 20+)

```bash
git clone https://github.com/kryptonite012/autonomous-navigtion.git nexus
cd nexus
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env            # NEXUS_MODE=simulation by default
python3 -m pytest               # 115 tests, ~2 min; hardware tests are skipped
```

Run the backend and the dashboard dev server in two terminals:

```bash
python3 -m backend.main                       # API on :8000, simulated robot
```

```bash
cd frontend && npm ci && npm run dev          # dashboard on :5173 (proxies /api and /ws)
```

The admin password is printed in the log and stored in
`data/initial_admin_password` unless `NEXUS_ADMIN_PASSWORD` is set in `.env`.

For a single-port setup build the dashboard once (`npm run build`); the
backend then serves it at `http://localhost:8000/`.

Docker alternative:

```bash
docker compose up --build        # simulation mode on :8000
```

## 2. Raspberry Pi setup guide

### 2.1 Operating system

1. Flash **Raspberry Pi OS (64-bit, Bookworm)** with Raspberry Pi Imager.
   In the imager settings set hostname `nexus`, enable SSH, create a user
   and configure Wi-Fi.
2. Boot, then update:

```bash
sudo apt update && sudo apt full-upgrade -y && sudo reboot
```

3. Pi 5 power: in `/boot/firmware/config.txt` add `usb_max_current_enable=1`
   only if your 5 V supply really delivers 5 A.

### 2.2 Interfaces

```bash
sudo raspi-config nonint do_i2c 0        # I2C on
sudo raspi-config nonint do_spi 1        # SPI OFF: its pins are used by the side ultrasonics
```

Check the camera: `rpicam-hello --list-cameras`. Check I2C after wiring:
`i2cdetect -y 1`.

### 2.3 Install NEXUS

```bash
git clone https://github.com/kryptonite012/autonomous-navigtion.git ~/nexus
cd ~/nexus
sudo ./scripts/setup_pi.sh
```

The script installs apt packages (picamera2, lgpio, OpenCV), creates the
`nexus` service user in the `gpio,i2c,video,dialout` groups, copies the code
to `/opt/nexus`, creates a venv with system site packages, installs
`requirements-pi.txt`, builds the dashboard, generates `/opt/nexus/.env`
with a random secret key and `NEXUS_MODE=hardware`, and installs the systemd
units.

### 2.4 First hardware bring-up

1. **Wheels off the ground** (robot on a box).
2. `sudo -u nexus /opt/nexus/.venv/bin/python /opt/nexus/scripts/hardware_check.py`
   — every device you installed must show `ok`. Fix wiring before continuing.
3. Press and release the e-stop; the check prints its state.
4. `... hardware_check.py --motor-test` — each wheel should spin forward and
   its encoder count increase. If not, see [troubleshooting](troubleshooting.md).
5. `sudo systemctl start nexus && journalctl -u nexus -f`
6. Open `http://nexus.local:8000`, log in with the password from
   `sudo cat /opt/nexus/data/initial_admin_password`, change it under
   *Settings*.
7. Drive manually at low speed on the floor, test the obstacle stop with a
   box, then continue with [calibration](calibration.md).

### 2.5 Updating

```bash
cd ~/nexus && git pull && sudo ./scripts/setup_pi.sh && sudo systemctl restart nexus
```

`.env`, `data/` (database, maps, secrets) and `backups/` are preserved.

## 3. ROS 2 (optional)

ROS 2 Jazzy targets Ubuntu 24.04; there are no Raspberry Pi OS binaries.
Options on a Pi 5: install Ubuntu Server 24.04 instead of Raspberry Pi OS,
or run ROS in a `ros:jazzy` Docker container with device pass-through.
See [`ros2_ws/README.md`](../ros2_ws/README.md). Stop `nexus.service` while
the ROS base node runs.

## 4. Environment variables

| Variable | Default | Meaning |
|---|---|---|
| `NEXUS_MODE` | `simulation` | `simulation` or `hardware` |
| `NEXUS_CONFIG` | `config/nexus.yaml` | config file |
| `NEXUS_DATABASE_URL` | `sqlite:///data/nexus.db` | SQLAlchemy URL |
| `NEXUS_SECRET_KEY` | generated into `data/secret_key` | token signing key (≥ 32 chars) |
| `NEXUS_ADMIN_USERNAME` / `NEXUS_ADMIN_PASSWORD` | `admin` / random | first admin account |
| `NEXUS_ALLOWED_NETWORKS` | private ranges | CIDR allow-list for API clients |
| `NEXUS_SESSION_MINUTES` | 480 | token lifetime |
| `NEXUS_HOST` / `NEXUS_PORT` | `0.0.0.0` / `8000` | bind address |
| `NEXUS_LOG_LEVEL` | `INFO` | logging level |

The system needs no internet access at runtime; all assets are bundled.
