FROM python:3.13-slim

# bluez gives us hciconfig + the BlueZ daemon's CLI tools (only needed
# inside the container if we ever want to debug from a shell;  the
# actual BLE stack lives on the host kernel and is reached via DBus).
# libdbus + glib are bless's transitive runtime deps via dbus-fast on
# some platforms; harmless if dbus-fast doesn't link them.
RUN apt-get update && apt-get install -y --no-install-recommends \
        bluez \
        libglib2.0-0 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Pinned versions so reproducibility doesn't drift behind us.
# dbus-fast is the asyncio-native DBus library we use to talk to BlueZ
# directly (no bless wrapper — bless was removed because it silently
# de-registered the GATT app on disconnect). PyGObject is pulled in
# because dbus-fast uses GLib's signal types under the hood for some
# Variant marshalling paths.
RUN pip install --no-cache-dir \
        requests==2.32.3 \
        flask==3.0.3 \
        httpx==0.27.2 \
        websockets==13.1 \
        dbus-fast==2.24.4

WORKDIR /app
COPY . /app/drone_bridge

# Run as a package so the relative imports (from . import …) resolve.
CMD ["python3", "-u", "-m", "drone_bridge.main"]
