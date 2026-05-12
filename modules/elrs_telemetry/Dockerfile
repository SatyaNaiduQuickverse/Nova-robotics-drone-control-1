FROM python:3.13-slim

# pyserial talks to the ESP32 over USB-CDC. flask + flask_sock provide
# the REST + WebSocket surface. Nothing else is needed at runtime.
RUN pip install --no-cache-dir \
        pyserial==3.5 \
        flask==3.0.3 \
        flask-sock==0.7.0 \
        simple-websocket==1.0.0

WORKDIR /app
COPY crsf.py mux.py service.py ble_server.py ./

# Service binds 5003. network_mode: host in compose makes this the host port.
EXPOSE 5003

CMD ["python3", "-u", "service.py"]
