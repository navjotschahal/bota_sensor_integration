#!/usr/bin/env bash
# Manage the containerized Bota driver.
#   bota_container.sh build                 build the image (pulls latest bota-driver)
#   bota_container.sh start [config.json]   start detached (default bota_binary.json = USB)
#   bota_container.sh stop | restart | logs | status | shell
# Env overrides: BOTA_SERIAL_DEV, BOTA_DATA_PORT, BOTA_CONTROL_PORT, BOTA_TARE_ON_START, BOTA_READ_MODE
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_FILE="$PKG_DIR/docker/docker-compose.yml"

if ! docker info >/dev/null 2>&1; then
  echo "docker is not usable by $USER. One-time fix, then log out and in:" >&2
  echo "  sudo usermod -aG docker $USER" >&2
  exit 1
fi

compose() { docker compose -f "$COMPOSE_FILE" "$@"; }

cmd="${1:-status}"
case "$cmd" in
  build)
    compose build --pull ;;
  start)
    cfg="${2:-bota_binary.json}"
    [[ -f "$PKG_DIR/docker/config/$cfg" ]] || { echo "no such config: docker/config/$cfg" >&2; exit 1; }
    BOTA_CONFIG="/config/$cfg" compose up -d
    echo "started with /config/$cfg; follow with: $0 logs" ;;
  stop)
    compose down ;;
  restart)
    compose restart ;;
  logs)
    compose logs -f --tail=100 ;;
  status)
    compose ps
    PYTHONPATH="$PKG_DIR/src${PYTHONPATH:+:$PYTHONPATH}" python3 "$PKG_DIR/scripts/bota_stream_probe.py" --status --port "${BOTA_CONTROL_PORT:-50101}" || true ;;
  shell)
    compose exec bota_driver bash ;;
  *)
    echo "usage: $0 {build|start [config.json]|stop|restart|logs|status|shell}" >&2; exit 2 ;;
esac
