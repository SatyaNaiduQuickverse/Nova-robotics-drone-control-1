# NovaROS — top-level orchestrator for the application stack.
#
# Each subsystem owns its own docker-compose.yml because they were
# developed at different times and have independent project namespaces.
# This Makefile iterates over them in dependency order for one-command
# deployment on a fresh Pi (or anywhere).
#
# Quickstart on a fresh Pi (after install.sh has set up Docker + boot config):
#
#   make verify     # preflight check (hardware presence + host config)
#   make build      # builds all 7 service images
#   make up         # production set: drone + cameras + web + elrs + bridge + mongodb
#   make ps         # show all container status (across 4 compose projects)
#
# Vision-detect (Hailo NPU YOLO) is opt-in only — the Pi throttles when
# vision runs alongside the rest. To bring it up explicitly:
#   make up-with-vision
#
# Full bring-down (stops all containers, preserves volumes/images):
#   make down

# Each compose file lives in its own project namespace.
ROOT_DIR    := $(shell pwd)
COMPOSE_ROOT_FILE   := $(ROOT_DIR)/docker-compose.yml
COMPOSE_ROS2_DIR    := $(ROOT_DIR)/ros2
COMPOSE_ELRS_DIR    := $(ROOT_DIR)/modules/elrs_telemetry
COMPOSE_BRIDGE_DIR  := $(ROOT_DIR)/modules/drone_bridge

# ros2/docker-compose.yml services we want in production:
#   drone   — drone-control container (FastAPI + MAVROS supervisor)
#   web     — web-control Flask UI
# Excluded: pi-cam (retired hardware — Pi Camera Module 3 no longer on the drone)
#           vision (Hailo NPU, opt-in only due to thermal throttling)
ROS2_PROD_SERVICES := drone web
ROS2_VISION_SERVICE := vision

.PHONY: help build up up-with-vision down restart ps logs verify clean prune

help:
	@echo "NovaROS deployment targets:"
	@echo ""
	@echo "  make verify          Hardware preflight (FC, ESP, cameras, hailo, boot config)"
	@echo "  make build           Build all 7 service images (safe; running containers unaffected)"
	@echo "  make up              Bring up production stack (excludes vision-detect)"
	@echo "  make up-with-vision  Bring up production + vision-detect (Pi may throttle)"
	@echo "  make down            Stop all containers (volumes preserved)"
	@echo "  make restart         Restart all production containers"
	@echo "  make ps              Show container status across all 4 compose projects"
	@echo "  make logs            Tail logs across all services (Ctrl-C to exit)"
	@echo "  make clean           Remove stopped containers"
	@echo "  make prune           Remove unused images + builder cache (frees disk)"
	@echo ""
	@echo "One-command fresh-Pi deploy:  make verify && make build && make up"

build: _check_disk
	@echo "[build] mongodb (image pull only)..."
	docker compose -f $(COMPOSE_ROOT_FILE) pull
	docker builder prune -f >/dev/null 2>&1 || true
	@echo ""
	@echo "[build] elrs_telemetry stack (daemon + producers, ~430 MB)..."
	cd $(COMPOSE_ELRS_DIR) && docker compose build
	docker builder prune -f >/dev/null 2>&1 || true
	@echo ""
	@echo "[build] drone_bridge (~330 MB)..."
	cd $(COMPOSE_BRIDGE_DIR) && docker compose build
	docker builder prune -f >/dev/null 2>&1 || true
	@echo ""
	@echo "[build] ros2 stack (drone-control 2.2 GB + pi-cam + vision 1.7 GB)..."
	@echo "[build] WARNING: this stack needs ~5 GB free during build"
	cd $(COMPOSE_ROS2_DIR) && docker compose build
	docker builder prune -f >/dev/null 2>&1 || true
	@echo ""
	@echo "[build] done"

# Internal: ensure at least 6 GB free before attempting a build.
# The ros2 stack alone needs ~5 GB peak; we want headroom.
_check_disk:
	@AVAIL=$$(df / --output=avail | tail -1); \
	AVAIL_GB=$$((AVAIL / 1024 / 1024)); \
	if [ $$AVAIL -lt 6291456 ]; then \
		echo "[build] ERROR: only $${AVAIL_GB} GB free on /; need at least 6 GB to build safely"; \
		echo "[build] run 'make prune' first to recover disk space"; \
		exit 1; \
	else \
		echo "[build] disk check: $${AVAIL_GB} GB free, proceeding"; \
	fi

up:
	@echo "[up] starting mongodb..."
	docker compose -f $(COMPOSE_ROOT_FILE) up -d
	@echo ""
	@echo "[up] starting ros2 stack (production services only — vision excluded)..."
	cd $(COMPOSE_ROS2_DIR) && docker compose up -d $(ROS2_PROD_SERVICES)
	@echo ""
	@echo "[up] starting elrs_telemetry + producers..."
	cd $(COMPOSE_ELRS_DIR) && docker compose up -d
	@echo ""
	@echo "[up] starting drone_bridge..."
	cd $(COMPOSE_BRIDGE_DIR) && docker compose up -d
	@echo ""
	@echo "[up] done. Run 'make ps' to verify, or 'make verify' for hardware preflight."

up-with-vision: up
	@echo ""
	@echo "[up-with-vision] additionally starting vision-detect (Hailo NPU)..."
	@echo "[up-with-vision] WARNING: Pi may thermal-throttle under combined load"
	cd $(COMPOSE_ROS2_DIR) && docker compose up -d $(ROS2_VISION_SERVICE)

down:
	@echo "[down] stopping drone_bridge..."
	-cd $(COMPOSE_BRIDGE_DIR) && docker compose down
	@echo "[down] stopping elrs_telemetry + producers..."
	-cd $(COMPOSE_ELRS_DIR) && docker compose down
	@echo "[down] stopping ros2 stack..."
	-cd $(COMPOSE_ROS2_DIR) && docker compose down
	@echo "[down] stopping mongodb..."
	-docker compose -f $(COMPOSE_ROOT_FILE) down
	@echo "[down] done"

restart:
	@echo "[restart] ros2 stack..."
	cd $(COMPOSE_ROS2_DIR) && docker compose restart $(ROS2_PROD_SERVICES)
	@echo "[restart] elrs..."
	cd $(COMPOSE_ELRS_DIR) && docker compose restart
	@echo "[restart] drone_bridge..."
	cd $(COMPOSE_BRIDGE_DIR) && docker compose restart

ps:
	@echo "=== all NovaROS containers ==="
	@docker ps --filter "name=novaros-mongodb" \
		--filter "name=drone-control" \
		--filter "name=pi-cam" \
		--filter "name=vision-detect" \
		--filter "name=web-control" \
		--filter "name=elrs-telemetry" \
		--filter "name=elrs-producers" \
		--filter "name=drone-bridge" \
		--format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'

logs:
	@echo "[logs] tailing across all 4 compose projects; Ctrl-C to stop"
	@docker logs -f --tail=20 \
		novaros-mongodb drone-control web-control \
		elrs-telemetry elrs-producers drone-bridge 2>&1 &
	@wait

verify:
	@bash scripts/verify_deployment.sh

clean:
	@echo "[clean] removing stopped containers..."
	docker container prune -f

prune:
	@echo "[prune] removing unused images + builder cache..."
	docker image prune -af
	docker builder prune -af
