#!/bin/bash
# Hardware + host-config preflight check for NovaROS production deployment.
#
# Called by `make verify`. Read-only — does not modify anything. Returns
# 0 if all production-critical checks pass, non-zero with a summary if any fail.
#
# Vision-detect deps (hailo) are checked but reported as warnings since
# vision is opt-in.

set -u

# deploy patch: resolve repo root from this script's own location instead of
# the original hardcoded /home/novaedge1 path (deploy user/home varies per Pi).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

PASS=0
FAIL=0
WARN=0

ok()    { echo "  [ OK ]  $1"; PASS=$((PASS + 1)); }
fail()  { echo "  [FAIL]  $1"; FAIL=$((FAIL + 1)); }
warn()  { echo "  [WARN]  $1"; WARN=$((WARN + 1)); }

echo "NovaROS deployment preflight"
echo "============================"

# -----------------------------------------------------------------------------
echo ""
echo "Disk space:"
AVAIL_KB=$(df / --output=avail | tail -1)
AVAIL_GB=$((AVAIL_KB / 1024 / 1024))
USED_PCT=$(df / --output=pcent | tail -1 | tr -dc '0-9')
if [ "$AVAIL_GB" -lt 2 ]; then
    fail "only ${AVAIL_GB} GB free on / (used ${USED_PCT}%)"
    fail "  builds and runtime image pulls will fail; run 'make prune' or free disk"
elif [ "$AVAIL_GB" -lt 6 ]; then
    warn "only ${AVAIL_GB} GB free on / (used ${USED_PCT}%)"
    warn "  not enough headroom for a fresh build of ros2 stack (~5 GB peak)"
    warn "  existing containers will keep running; build needs 'make prune' first"
else
    ok "${AVAIL_GB} GB free on / (used ${USED_PCT}%)"
fi

# -----------------------------------------------------------------------------
echo ""
echo "Docker:"
if command -v docker >/dev/null 2>&1; then
    ok "docker binary present ($(docker --version | head -1))"
else
    fail "docker not found — run install.sh"
fi

if docker compose version >/dev/null 2>&1; then
    ok "docker compose plugin present ($(docker compose version | head -1))"
else
    fail "docker compose plugin not found"
fi

# -----------------------------------------------------------------------------
echo ""
echo "Flight controller (CubeOrange / Pixhawk / MicoAir):"
FC_GLOB="/dev/serial/by-id/usb-CubePilot_CubeOrange+_*-if00 /dev/serial/by-id/usb-ArduPilot_Pixhawk1_*-if00 /dev/serial/by-id/usb-ArduPilot_MicoAir743v2_*-if00"
shopt -s nullglob
fc_matches=( $FC_GLOB )
shopt -u nullglob
if [ ${#fc_matches[@]} -gt 0 ]; then
    ok "FC enumerated at ${fc_matches[0]}"
else
    fail "no FC at /dev/serial/by-id/ (CubeOrange+ / Pixhawk1 / MicoAir743v2)"
    fail "  (check USB cable, check FC is in RUNTIME mode not bootloader)"
fi

# -----------------------------------------------------------------------------
echo ""
echo "ELRS ESP32 bridge:"
shopt -s nullglob
esp_matches=( /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00 )
shopt -u nullglob
if [ ${#esp_matches[@]} -gt 0 ]; then
    ok "ESP32 enumerated at ${esp_matches[0]}"
    # Warn if compose has a different specific path hardcoded
    if grep -q "ELRS_SERIAL_DEVICE=${esp_matches[0]}" "$REPO_ROOT/modules/elrs_telemetry/docker-compose.yml" 2>/dev/null; then
        ok "  compose ELRS_SERIAL_DEVICE matches enumerated path"
    else
        warn "  compose ELRS_SERIAL_DEVICE hardcoded to a different serial"
        warn "  → edit modules/elrs_telemetry/docker-compose.yml to point at ${esp_matches[0]}"
    fi
else
    fail "no ESP32 at /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00"
fi

# -----------------------------------------------------------------------------
echo ""
echo "USB cameras:"
shopt -s nullglob
cam_matches=( /dev/v4l/by-id/usb-*-video-index0 )
shopt -u nullglob
if [ ${#cam_matches[@]} -gt 0 ]; then
    for cam in "${cam_matches[@]}"; do
        ok "camera enumerated at $cam"
    done
else
    warn "no USB cameras enumerated (drone-control will start but /camera/* endpoints will be empty)"
fi

# -----------------------------------------------------------------------------
echo ""
echo "Pi 5 composite-out / VTX:"
if grep -qE '^dtoverlay=vc4-kms-v3d,composite' /boot/firmware/config.txt 2>/dev/null; then
    ok "composite overlay enabled in /boot/firmware/config.txt"
else
    warn "composite overlay NOT enabled — VTX path will not work"
    warn "  add: dtoverlay=vc4-kms-v3d,composite + enable_tvout=1 + sdtv_mode=2"
fi
if grep -qE 'video=Composite-1' /boot/firmware/cmdline.txt 2>/dev/null; then
    ok "Composite-1 mode set in /boot/firmware/cmdline.txt"
else
    warn "Composite-1 not set in cmdline.txt — VTX may run at wrong resolution"
fi
# Card index can be 1 or 2 depending on driver-load order after a reboot.
# Match any card?-Composite-1 — the connector identity, not its slot.
shopt -s nullglob
composite_matches=( /sys/class/drm/card*-Composite-1 )
shopt -u nullglob
if [ ${#composite_matches[@]} -gt 0 ]; then
    ok "composite DRM connector active (${composite_matches[0]##*/})"
else
    warn "composite DRM connector not present — composite-out won't work until reboot after boot-config change"
fi

# -----------------------------------------------------------------------------
echo ""
echo "Hailo NPU (for vision-detect, opt-in):"
if lsmod | grep -q '^hailo_pci'; then
    ok "hailo_pci kernel module loaded"
else
    warn "hailo_pci NOT loaded — vision-detect won't work, others unaffected"
fi
if [ -e /dev/hailo0 ]; then
    ok "/dev/hailo0 device present"
else
    warn "/dev/hailo0 missing — vision-detect won't work"
fi
if [ -e /usr/share/hailo-models/yolov8s_h8.hef ]; then
    ok "HEF model installed"
else
    warn "/usr/share/hailo-models/yolov8s_h8.hef missing — install hailo-all package or copy the HEF"
fi

# -----------------------------------------------------------------------------
echo ""
echo "============================"
echo "Summary: $PASS passed | $WARN warnings | $FAIL failures"
if [ $FAIL -gt 0 ]; then
    echo ""
    echo "Production deployment will fail until [FAIL] items are resolved."
    exit 1
fi
if [ $WARN -gt 0 ]; then
    echo ""
    echo "Production stack will run, but [WARN] items affect VTX or vision."
fi
exit 0
