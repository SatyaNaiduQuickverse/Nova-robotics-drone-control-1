"""BLE GATT server (stub).

Short-range Pi <-> Pi backup channel intended for mission planning,
calibration, and parameter exchange — NOT for streaming telemetry or
flying the drone. The ELRS RF link via service.py handles those.

This is a placeholder. When we wire it up:
  - python3-bless or bluez-peripheral on the drone (server)
  - bleak on the ground (client)
  - Service UUID:  6e400001-1234-4abc-8def-novadronebt00
  - CHAR_TELEM    6e400002-...   notify   drone -> ground   (status, low rate)
  - CHAR_CMD      6e400003-...   write    ground -> drone   (mission/cal ops)
  - CHAR_BULK     6e400004-...   read     ground reads on demand (snapshots)

Docker plumbing needed when we enable this:
  - Mount /var/run/dbus into the container
  - --net=host is already set
  - May need --cap-add NET_ADMIN, possibly --privileged
"""


def start(state, log):  # noqa: ARG001  (signature reserved)
    """Hook called by service.py if BLE is enabled. No-op until built."""
    log.info("ble: disabled (stub) — see ble_server.py to wire up")


if __name__ == "__main__":
    print("ble_server.py is a stub. See module docstring for the plan.")
