#!/bin/bash
FCU_URL="${1:-serial:///dev/serial0:921600}"
sudo docker exec -it ros2-mavros bash -c "source /opt/ros/humble/setup.bash && ros2 launch mavros apm.launch fcu_url:=$FCU_URL"
