#!/usr/bin/env bash
# Raspberry Pi OS (64-bit, Bookworm) setup for PROJECT NEXUS.
# Run from the repository root:  sudo ./scripts/setup_pi.sh
set -euo pipefail

INSTALL_DIR=/opt/nexus
SRC_DIR=$(cd "$(dirname "$0")/.." && pwd)

if [[ $EUID -ne 0 ]]; then
  echo "run with sudo" >&2
  exit 1
fi

echo "==> system packages"
apt-get update
apt-get install -y python3-venv python3-pip python3-picamera2 python3-libcamera python3-lgpio \
  python3-opencv i2c-tools libatlas-base-dev nodejs npm git

echo "==> enable I2C and camera"
raspi-config nonint do_i2c 0 || true
raspi-config nonint do_camera 0 || true

echo "==> service user"
id nexus >/dev/null 2>&1 || useradd -r -m -d /var/lib/nexus -s /usr/sbin/nologin nexus
usermod -aG gpio,i2c,video,dialout nexus

echo "==> copy application to $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
rsync -a --delete --exclude node_modules --exclude .git --exclude data --exclude backups \
  --exclude .env "$SRC_DIR"/ "$INSTALL_DIR"/
mkdir -p "$INSTALL_DIR"/data "$INSTALL_DIR"/backups

echo "==> python environment"
# system site packages so picamera2/libcamera from apt are visible
python3 -m venv --system-site-packages "$INSTALL_DIR"/.venv
"$INSTALL_DIR"/.venv/bin/pip install --upgrade pip
"$INSTALL_DIR"/.venv/bin/pip install -r "$INSTALL_DIR"/requirements-pi.txt

echo "==> dashboard build"
(cd "$INSTALL_DIR"/frontend && npm ci --no-audit --no-fund && npm run build)

if [[ ! -f "$INSTALL_DIR"/.env ]]; then
  echo "==> creating .env"
  cp "$INSTALL_DIR"/.env.example "$INSTALL_DIR"/.env
  sed -i "s|^NEXUS_MODE=.*|NEXUS_MODE=hardware|" "$INSTALL_DIR"/.env
  sed -i "s|^NEXUS_SECRET_KEY=.*|NEXUS_SECRET_KEY=$(python3 -c 'import secrets;print(secrets.token_urlsafe(48))')|" "$INSTALL_DIR"/.env
  sed -i "s|^NEXUS_DATABASE_URL=.*|NEXUS_DATABASE_URL=sqlite:///$INSTALL_DIR/data/nexus.db|" "$INSTALL_DIR"/.env
  chmod 600 "$INSTALL_DIR"/.env
fi
chown -R nexus:nexus "$INSTALL_DIR"

echo "==> systemd"
cp "$INSTALL_DIR"/deployment/systemd/nexus*.service "$INSTALL_DIR"/deployment/systemd/nexus*.timer /etc/systemd/system/
systemctl daemon-reload
systemctl enable nexus.service nexus-backup.timer

cat <<MSG

Done. Before starting the service:
  1. Run the hardware check:   sudo -u nexus $INSTALL_DIR/.venv/bin/python $INSTALL_DIR/scripts/hardware_check.py
  2. Lift the robot so the wheels are off the ground for the first motor test.
  3. Start:                    sudo systemctl start nexus
  4. Admin password:           sudo cat $INSTALL_DIR/data/initial_admin_password
  5. Dashboard:                http://$(hostname -I | awk '{print $1}'):8000
MSG
