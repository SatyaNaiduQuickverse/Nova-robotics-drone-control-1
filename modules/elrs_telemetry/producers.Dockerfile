# Image for the elrs-producers sidecar container.
#
# Runs 5 producer scripts (RC keeper + 4 FC-sourced telemetry producers)
# under a python supervisor. The supervisor restarts individual producers
# on crash; docker handles container-level failures via restart: unless-stopped.
#
# Build from modules/elrs_telemetry/:
#   docker compose build elrs-producers
#
# The producers POST to elrs-telemetry's /telemetry/raw on localhost:5003 and
# read drone-control's /telemetry/* on localhost:8080 — both reachable when
# the container runs with network_mode: host.
FROM python:3.13-slim

WORKDIR /app

# Producer dependencies — stdlib only (urllib, json, struct, math, time,
# signal, threading, subprocess). No pip installs needed.

# Copy producer scripts + their shared modules. Each producer treats /app
# as its working dir so relative imports of fc_client / producer_safety
# resolve correctly.
COPY tools/elrs_producer.py        ./
COPY tools/fc_battery_producer.py  ./
COPY tools/fc_gps_producer.py      ./
COPY tools/fc_attitude_producer.py ./
COPY tools/fc_flightmode_producer.py ./
COPY tools/fc_client.py            ./
COPY tools/producer_safety.py      ./
COPY tools/run_producers.py        ./

# Supervisor handles SIGTERM cleanly; docker stop will work.
CMD ["python3", "-u", "/app/run_producers.py"]
