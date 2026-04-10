#!/bin/bash

#===============================================================================
# ROS 2 Humble + MAVROS Installation Script
# Target: Raspberry Pi 5 - Debian Trixie (ARM64)
#===============================================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo "==============================================================================="
echo "  ROS 2 Humble + MAVROS Installation"
echo "  Target: Raspberry Pi 5 - Debian Trixie (ARM64)"
echo "==============================================================================="
echo ""

#-------------------------------------------------------------------------------
# METHOD 1: Docker-based ROS 2 (Recommended for Debian)
#-------------------------------------------------------------------------------
log_info "Setting up ROS 2 Humble via Docker..."

# Create ROS 2 workspace directory
mkdir -p ~/ros2_ws/src
mkdir -p ~/novaros/ros2

# Create Dockerfile for ROS 2 Humble with MAVROS
cat > ~/novaros/ros2/Dockerfile << 'DOCKERFILE'
FROM ros:humble-ros-base

# Install MAVROS and dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets (required for MAVROS)
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# Set up entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
DOCKERFILE

# Create entrypoint script
cat > ~/novaros/ros2/ros_entrypoint.sh << 'ENTRYPOINT'
#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
ENTRYPOINT

# Create docker-compose for ROS 2
cat > ~/novaros/ros2/docker-compose.yml << 'COMPOSE'
version: '3.8'

services:
  ros2-mavros:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: ros2-mavros
    privileged: true
    network_mode: host
    volumes:
      - ~/ros2_ws:/ros2_ws
      - /dev:/dev
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
      - /dev/ttyACM0:/dev/ttyACM0
      - /dev/serial0:/dev/serial0
    restart: unless-stopped
    stdin_open: true
    tty: true
COMPOSE

log_success "Docker configuration created"

#-------------------------------------------------------------------------------
# Build ROS 2 Docker Image
#-------------------------------------------------------------------------------
log_info "Building ROS 2 Humble + MAVROS Docker image (this may take 10-20 minutes)..."
cd ~/novaros/ros2
docker compose build 2>&1 | tee ros2_build.log

log_success "Docker image built successfully"

#-------------------------------------------------------------------------------
# Create Helper Scripts
#-------------------------------------------------------------------------------
log_info "Creating helper scripts..."

# Script to start ROS 2 container
cat > ~/novaros/scripts/ros2-start.sh << 'SCRIPT'
#!/bin/bash
cd ~/novaros/ros2
docker compose up -d
echo "ROS 2 container started. Use 'ros2-shell' to access it."
SCRIPT
chmod +x ~/novaros/scripts/ros2-start.sh

# Script to get shell into ROS 2 container
cat > ~/novaros/scripts/ros2-shell.sh << 'SCRIPT'
#!/bin/bash
docker exec -it ros2-mavros bash
SCRIPT
chmod +x ~/novaros/scripts/ros2-shell.sh

# Script to stop ROS 2 container
cat > ~/novaros/scripts/ros2-stop.sh << 'SCRIPT'
#!/bin/bash
cd ~/novaros/ros2
docker compose down
echo "ROS 2 container stopped."
SCRIPT
chmod +x ~/novaros/scripts/ros2-stop.sh

# Script to run MAVROS with FCU connection
cat > ~/novaros/scripts/mavros-start.sh << 'SCRIPT'
#!/bin/bash
# Usage: mavros-start.sh [fcu_url]
# Default: serial:///dev/serial0:921600

FCU_URL="${1:-serial:///dev/serial0:921600}"

docker exec -it ros2-mavros bash -c "
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=$FCU_URL
"
SCRIPT
chmod +x ~/novaros/scripts/mavros-start.sh

# Create symlinks in /usr/local/bin
sudo ln -sf ~/novaros/scripts/ros2-start.sh /usr/local/bin/ros2-start
sudo ln -sf ~/novaros/scripts/ros2-shell.sh /usr/local/bin/ros2-shell
sudo ln -sf ~/novaros/scripts/ros2-stop.sh /usr/local/bin/ros2-stop
sudo ln -sf ~/novaros/scripts/mavros-start.sh /usr/local/bin/mavros-start

log_success "Helper scripts created"

#-------------------------------------------------------------------------------
# Create ROS 2 Native Installation Script (Alternative)
#-------------------------------------------------------------------------------
log_info "Creating native ROS 2 build script (for reference)..."

cat > ~/novaros/scripts/ros2-native-install.sh << 'NATIVESCRIPT'
#!/bin/bash
# ROS 2 Humble Native Build from Source
# Use this if you prefer native installation over Docker
# WARNING: This takes several hours to compile on Pi 5

set -e

echo "Installing ROS 2 Humble from source..."
echo "This will take 2-4 hours on Raspberry Pi 5"

# Install dependencies
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget

# Initialize rosdep
sudo rosdep init || true
rosdep update

# Create workspace
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble

# Get ROS 2 source
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos

# Install dependencies
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build (this takes a long time)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "ROS 2 Humble built. Source with: source ~/ros2_humble/install/setup.bash"
NATIVESCRIPT
chmod +x ~/novaros/scripts/ros2-native-install.sh

#-------------------------------------------------------------------------------
# Test Installation
#-------------------------------------------------------------------------------
log_info "Starting ROS 2 container for testing..."
cd ~/novaros/ros2
docker compose up -d

sleep 5

log_info "Testing ROS 2 installation..."
docker exec ros2-mavros bash -c "source /opt/ros/humble/setup.bash && ros2 --version" || {
    log_warn "ROS 2 test failed, container may still be starting"
}

log_info "Testing MAVROS installation..."
docker exec ros2-mavros bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep mavros" || {
    log_warn "MAVROS test failed"
}

#-------------------------------------------------------------------------------
# INSTALLATION COMPLETE
#-------------------------------------------------------------------------------
echo ""
echo "==============================================================================="
echo -e "${GREEN}  ROS 2 HUMBLE + MAVROS INSTALLATION COMPLETE!${NC}"
echo "==============================================================================="
echo ""
echo "Quick Commands:"
echo "  ros2-start      - Start ROS 2 container"
echo "  ros2-shell      - Get shell into ROS 2 container"
echo "  ros2-stop       - Stop ROS 2 container"
echo "  mavros-start    - Start MAVROS (connects to flight controller)"
echo ""
echo "Inside the container:"
echo "  ros2 topic list              - List all topics"
echo "  ros2 node list               - List all nodes"
echo "  ros2 launch mavros apm.launch fcu_url:=serial:///dev/serial0:921600"
echo ""
echo "Workspace: ~/ros2_ws"
echo "Docker files: ~/novaros/ros2/"
echo ""
echo "==============================================================================="
