FROM arm64v8/ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    python3-colcon-common-extensions \
    python3-pip \
    geographiclib-tools \
    ffmpeg \
    v4l-utils \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh || true

RUN pip3 install --no-cache-dir fastapi uvicorn pyzbar pillow

WORKDIR /ros2_ws

COPY src/ src/

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select drone_control

RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "drone_control", "control_node"]
