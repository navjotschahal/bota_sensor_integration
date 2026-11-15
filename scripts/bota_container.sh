#!/usr/bin/env bash
# One official-driver container per sensor, driven by config/sensors.yaml.
#   bota_container.sh build                    build the image (pulls latest bota-driver)
#   bota_container.sh start  <sensor|all>      start detached (compose project bota_<sensor>)
#   bota_container.sh stop   <sensor|all>
#   bota_container.sh restart <sensor|all>
#   bota_container.sh logs   <sensor>
#   bota_container.sh shell  <sensor>
#   bota_container.sh status                   registry view: present/absent, device, ports, container
#   bota_container.sh list                     same as status
# Extra env honoured per start: BOTA_TARE_ON_START, BOTA_READ_MODE, BOTA_DATA_HOST.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE_BASE="$PKG_DIR/docker/docker-compose.yml"
COMPOSE_SERIAL="$PKG_DIR/docker/docker-compose.serial.yml"
REG="${BOTA_SENSORS_YAML:-$PKG_DIR/config/sensors.yaml}"
SENSORS_PY=(/usr/bin/python3 "$PKG_DIR/scripts/bota_sensors.py" --registry "$REG")

die() { echo "bota_container: $*" >&2; exit 1; }

if ! docker info >/dev/null 2>&1; then
  die "docker is not usable by $USER. One-time fix, then log out and in:  sudo usermod -aG docker $USER"
fi

names() { "${SENSORS_PY[@]}" names; }

# Load the sensor's env (ports, config, device) into this shell.
load_env() {
  local name="$1"
  local env
  env="$("${SENSORS_PY[@]}" env "$name")" || die "unknown sensor '$name'. Registered: $(names | tr '\n' ' ')"
  eval "$env"
}

compose_for() {
  # usage: compose_for <sensor> <compose args...>; requires load_env first
  local name="$1"; shift
  local files=(-f "$COMPOSE_BASE")
  if [[ "${BOTA_HAS_SERIAL:-0}" == "1" ]]; then
    [[ -n "${BOTA_SERIAL_DEV:-}" ]] || die "sensor '$name' is not plugged in (no device matching its USB serial). Try: $0 status"
    [[ -e "$BOTA_SERIAL_DEV" ]] || die "device $BOTA_SERIAL_DEV for '$name' does not exist"
    files+=(-f "$COMPOSE_SERIAL")
  fi
  docker compose -p "bota_${name}" "${files[@]}" "$@"
}

each() {
  # usage: each <sensor|all> <fn>
  local target="$1" fn="$2"
  if [[ "$target" == "all" ]]; then
    local n; for n in $(names); do "$fn" "$n"; done
  else
    "$fn" "$target"
  fi
}

do_start() {
  local name="$1"
  load_env "$name"
  [[ -f "$PKG_DIR/docker/config/$(basename "$BOTA_CONFIG")" ]] || die "driver config $BOTA_CONFIG missing in docker/config/"
  compose_for "$name" up -d --remove-orphans
  echo "started $name: config $BOTA_CONFIG, device ${BOTA_SERIAL_DEV:-none}, data udp/$BOTA_DATA_PORT, control udp/$BOTA_CONTROL_PORT"
  echo "follow with: $0 logs $name"
}
do_stop() {
  local name="$1"
  load_env "$name" 2>/dev/null || true
  BOTA_HAS_SERIAL=0 docker compose -p "bota_${name}" -f "$COMPOSE_BASE" down --remove-orphans 2>/dev/null || true
  echo "stopped $name"
}
do_restart() { do_stop "$1"; do_start "$1"; }

cmd="${1:-status}"; target="${2:-}"
case "$cmd" in
  build)
    docker compose -f "$COMPOSE_BASE" build --pull ;;
  start)
    [[ -n "$target" ]] || die "usage: $0 start <sensor|all>   (sensors: $(names | tr '\n' ' '))"
    each "$target" do_start ;;
  stop)
    [[ -n "$target" ]] || die "usage: $0 stop <sensor|all>"
    each "$target" do_stop ;;
  restart)
    [[ -n "$target" ]] || die "usage: $0 restart <sensor|all>"
    each "$target" do_restart ;;
  logs)
    [[ -n "$target" ]] || die "usage: $0 logs <sensor>"
    docker logs -f --tail=100 "bota_${target}" ;;
  shell)
    [[ -n "$target" ]] || die "usage: $0 shell <sensor>"
    docker exec -it "bota_${target}" bash ;;
  status|list)
    "${SENSORS_PY[@]}" list
    echo
    for n in $(names); do
      if docker ps --format '{{.Names}}' | grep -qx "bota_${n}"; then
        port="$("${SENSORS_PY[@]}" get "$n" control_port)"
        printf "%-10s " "$n"
        PYTHONPATH="$PKG_DIR/src${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 "$PKG_DIR/scripts/bota_stream_probe.py" --status --port "$port" 2>&1 | cut -c1-150 || true
      fi
    done ;;
  *)
    echo "usage: $0 {build|start|stop|restart <sensor|all>|logs|shell <sensor>|status}" >&2; exit 2 ;;
esac
