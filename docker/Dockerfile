# Official Bota Gen A driver in its supported environment (glibc >= 2.34),
# forwarding frames over UDP to clients on the host. Build context is the
# bota_ft_sensor package root (see docker-compose.yml / bota_container.sh).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3 python3-venv python3-pip ca-certificates libcap2-bin \
 && rm -rf /var/lib/apt/lists/*

# Latest official driver from PyPI (manylinux_2_34 wheel, cp312 abi3).
RUN python3 -m venv /opt/venv \
 && /opt/venv/bin/pip install --upgrade pip \
 && /opt/venv/bin/pip install bota-driver
ENV PATH="/opt/venv/bin:${PATH}"

# Shared wire format + forwarder.
COPY src/bota_ft_sensor /opt/bota/bota_ft_sensor
COPY docker/bota_forwarder.py /opt/bota/bota_forwarder.py
ENV PYTHONPATH=/opt/bota

# Driver JSON configs are bind-mounted at /config at run time.
VOLUME ["/config"]
ENV BOTA_CONFIG=/config/bota_binary.json \
    BOTA_DATA_HOST=127.0.0.1 \
    BOTA_DATA_PORT=50100 \
    BOTA_CONTROL_PORT=50101 \
    BOTA_TARE_ON_START=1 \
    BOTA_READ_MODE=blocking

STOPSIGNAL SIGTERM
ENTRYPOINT ["python3", "/opt/bota/bota_forwarder.py"]
