#!/bin/bash

#===============================================================================
# NovaROS - Raspberry Pi 5 Drone Platform Setup Script
# For Advanced Police/Public Safety Drone Applications
# Target: Raspberry Pi 5 (8GB RAM) - Debian Trixie
#===============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

echo "==============================================================================="
echo "  NovaROS - Raspberry Pi 5 Drone Platform Installer"
echo "  Police/Public Safety Drone Application Stack"
echo "  Target: Debian Trixie (ARM64)"
echo "==============================================================================="
echo ""

#-------------------------------------------------------------------------------
# 1. SYSTEM UPDATE & BUILD ESSENTIALS
#-------------------------------------------------------------------------------
log_info "Updating system packages..."
apt update && apt upgrade -y

log_info "Installing build essentials and development tools..."
apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    curl \
    wget \
    unzip \
    vim \
    nano \
    htop \
    btop \
    tmux \
    screen \
    tree \
    jq \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release

log_success "Build essentials installed"

#-------------------------------------------------------------------------------
# 2. PYTHON 3 & PIP
#-------------------------------------------------------------------------------
log_info "Installing Python 3 and pip..."
apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    python3-wheel \
    python3-numpy \
    python3-scipy \
    python3-pandas \
    python3-matplotlib \
    python3-opencv \
    python3-pil \
    python3-flask \
    python3-requests \
    python3-serial \
    python3-yaml \
    python3-psutil \
    python3-gpiozero \
    python3-smbus2

log_success "Python 3 installed: $(python3 --version)"

#-------------------------------------------------------------------------------
# 3. PYTHON LIBRARIES FOR DRONE OPERATIONS (via pip)
#-------------------------------------------------------------------------------
log_info "Installing Python libraries for drone operations..."

# Create virtual environment for drone packages
python3 -m venv /opt/novaros-env --system-site-packages
source /opt/novaros-env/bin/activate

# MAVLink - Drone Communication Protocol
pip3 install \
    pymavlink \
    mavproxy \
    dronekit

# Additional libraries
pip3 install \
    geopy \
    pyproj \
    shapely \
    websockets \
    aiohttp \
    flask-cors \
    flask-socketio \
    fastapi \
    uvicorn \
    python-dotenv \
    loguru \
    click \
    tqdm \
    picamera2 \
    imageio

deactivate

# Create activation script
cat > /usr/local/bin/novaros-env << 'ENVSCRIPT'
#!/bin/bash
source /opt/novaros-env/bin/activate
ENVSCRIPT
chmod +x /usr/local/bin/novaros-env

log_success "Python libraries installed"

#-------------------------------------------------------------------------------
# 4. NETWORKING TOOLS
#-------------------------------------------------------------------------------
log_info "Installing networking tools..."
apt install -y \
    net-tools \
    iputils-ping \
    traceroute \
    nmap \
    tcpdump \
    wireshark-common \
    tshark \
    iptables \
    ufw \
    openssh-server \
    openssh-client \
    sshpass \
    netcat-openbsd \
    socat \
    dnsutils \
    whois \
    mtr \
    iperf3 \
    ethtool \
    wireless-tools \
    wpasupplicant \
    hostapd \
    dnsmasq \
    bridge-utils \
    iw

# Enable SSH
systemctl enable ssh
systemctl start ssh

log_success "Networking tools installed"

#-------------------------------------------------------------------------------
# 5. NODE.JS & NPM (for MERN Stack)
#-------------------------------------------------------------------------------
log_info "Installing Node.js LTS..."

# Install Node.js from Debian repos or NodeSource
curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
apt install -y nodejs || {
    log_warn "NodeSource install failed, using Debian packages..."
    apt install -y nodejs npm
}

# Install global npm packages
npm install -g \
    npm@latest \
    yarn \
    pm2 \
    nodemon

log_success "Node.js installed: $(node --version)"
log_success "NPM installed: $(npm --version)"

#-------------------------------------------------------------------------------
# 6. MONGODB (via Docker for ARM64 compatibility)
#-------------------------------------------------------------------------------
log_info "MongoDB will be installed via Docker for ARM64 compatibility..."
# MongoDB native ARM64 support is limited, Docker is more reliable

#-------------------------------------------------------------------------------
# 7. POSTGRESQL
#-------------------------------------------------------------------------------
log_info "Installing PostgreSQL..."
apt install -y \
    postgresql \
    postgresql-contrib \
    libpq-dev

# Enable PostgreSQL
systemctl enable postgresql
systemctl start postgresql

log_success "PostgreSQL installed: $(psql --version)"

#-------------------------------------------------------------------------------
# 8. DOCKER & DOCKER COMPOSE
#-------------------------------------------------------------------------------
log_info "Installing Docker..."

# Remove old versions
apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# Install Docker using convenience script
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
rm get-docker.sh

# Add users to docker group
# deploy patch: original hardcoded 'novaedge1'; use the real invoking user.
usermod -aG docker "${SUDO_USER:-$(logname 2>/dev/null)}" || true

# Install Docker Compose plugin
apt install -y docker-compose-plugin || {
    log_info "Installing docker-compose standalone..."
    curl -SL https://github.com/docker/compose/releases/latest/download/docker-compose-linux-aarch64 -o /usr/local/bin/docker-compose
    chmod +x /usr/local/bin/docker-compose
}

# Enable Docker service
systemctl enable docker
systemctl start docker

log_success "Docker installed: $(docker --version)"

# Pull MongoDB image for ARM64
log_info "Pulling MongoDB Docker image..."
docker pull mongo:7.0 &

#-------------------------------------------------------------------------------
# 9. VIDEO & STREAMING TOOLS
#-------------------------------------------------------------------------------
log_info "Installing video and streaming tools..."
apt install -y \
    ffmpeg \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-rtsp \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    v4l-utils \
    libv4l-dev \
    vlc

log_success "Video tools installed"

#-------------------------------------------------------------------------------
# 10. GPS & LOCATION SERVICES
#-------------------------------------------------------------------------------
log_info "Installing GPS tools..."
apt install -y \
    gpsd \
    gpsd-clients \
    gpsd-tools \
    chrony

log_success "GPS tools installed"

#-------------------------------------------------------------------------------
# 11. SERIAL & HARDWARE INTERFACES
#-------------------------------------------------------------------------------
log_info "Installing serial and hardware interface tools..."
apt install -y \
    minicom \
    picocom \
    i2c-tools \
    libi2c-dev \
    can-utils

# Enable I2C and SPI via raspi-config
raspi-config nonint do_i2c 0 2>/dev/null || true
raspi-config nonint do_spi 0 2>/dev/null || true
raspi-config nonint do_serial_hw 0 2>/dev/null || true

log_success "Hardware interface tools installed"

#-------------------------------------------------------------------------------
# 12. SECURITY TOOLS (For Police Applications)
#-------------------------------------------------------------------------------
log_info "Installing security tools..."
apt install -y \
    fail2ban \
    rkhunter \
    clamav \
    lynis \
    auditd

# Configure fail2ban
systemctl enable fail2ban
systemctl start fail2ban

log_success "Security tools installed"

#-------------------------------------------------------------------------------
# 13. SYSTEM MONITORING & PERFORMANCE
#-------------------------------------------------------------------------------
log_info "Installing system monitoring tools..."
apt install -y \
    iotop \
    nethogs \
    iftop \
    sysstat \
    lm-sensors \
    wavemon

log_success "Monitoring tools installed"

#-------------------------------------------------------------------------------
# 14. ADDITIONAL DEVELOPMENT LIBRARIES
#-------------------------------------------------------------------------------
log_info "Installing additional development libraries..."
apt install -y \
    libssl-dev \
    libffi-dev \
    libxml2-dev \
    libxslt1-dev \
    zlib1g-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libhdf5-dev \
    liblapack-dev \
    libopenblas-dev   # deploy patch: dropped libatlas-base-dev (removed in Trixie); openblas/lapack cover BLAS/LAPACK

log_success "Development libraries installed"

#-------------------------------------------------------------------------------
# 15. CONFIGURE SYSTEM SETTINGS
#-------------------------------------------------------------------------------
log_info "Configuring system settings..."

# Increase swap for 8GB RAM (set to 4GB swap)
if [ -f /etc/dphys-swapfile ]; then
    dphys-swapfile swapoff 2>/dev/null || true
    sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=4096/' /etc/dphys-swapfile
    dphys-swapfile setup 2>/dev/null || true
    dphys-swapfile swapon 2>/dev/null || true
fi

# Set GPU memory (256MB for video processing)
CONFIG_FILE="/boot/firmware/config.txt"
if [ -f "$CONFIG_FILE" ]; then
    if ! grep -q "gpu_mem=" "$CONFIG_FILE"; then
        echo "gpu_mem=256" >> "$CONFIG_FILE"
    fi
    if ! grep -q "dtparam=watchdog=on" "$CONFIG_FILE"; then
        echo "dtparam=watchdog=on" >> "$CONFIG_FILE"
    fi
fi

log_success "System settings configured"

#-------------------------------------------------------------------------------
# 16. CREATE PROJECT STRUCTURE
#-------------------------------------------------------------------------------
# deploy patch: legacy skeleton step neutralized.
# Original created /home/novaedge1/novaros and chowned it to user 'novaedge1',
# which does not exist on this Pi (real user is detected via $SUDO_USER) and
# aborted the whole script under 'set -e'. It also wrote a competing
# docker-compose.yml that is a strict subset (mongodb-only) of what the cloned
# repo already ships. The repo at ~/novaros provides the real project
# structure + root docker-compose.yml the Makefile drives, so nothing to do.
log_info "Project structure + MongoDB compose provided by the cloned repo (~/novaros)."
log_success "Project structure (repo) in place"

#-------------------------------------------------------------------------------
# INSTALLATION COMPLETE
#-------------------------------------------------------------------------------
echo ""
echo "==============================================================================="
echo -e "${GREEN}  INSTALLATION COMPLETE!${NC}"
echo "==============================================================================="
echo ""
echo "Installed Components:"
echo "  - Python 3 with scientific/drone libraries"
echo "  - MAVLink (pymavlink, mavproxy, dronekit) in /opt/novaros-env"
echo "  - Node.js 20.x LTS with npm/yarn/pm2"
echo "  - MongoDB 7.0 (via Docker)"
echo "  - PostgreSQL"
echo "  - Docker & Docker Compose"
echo "  - Networking tools (nmap, tcpdump, wireshark, etc.)"
echo "  - Video/Streaming (FFmpeg, GStreamer, VLC)"
echo "  - GPS tools (gpsd)"
echo "  - Security tools (fail2ban, clamav, etc.)"
echo "  - System monitoring tools (btop, htop, etc.)"
echo ""
echo "Quick Commands:"
echo "  - Activate drone env: source /opt/novaros-env/bin/activate"
echo "  - Or simply run: novaros-env"
echo "  - Start MongoDB: cd ~/novaros && docker compose up -d"
echo "  - Test MAVLink: mavproxy.py --help"
echo "  - Monitor system: btop"
echo ""
echo "Next Steps:"
echo "  1. Reboot the system: sudo reboot"
echo "  2. Start MongoDB: cd ~/novaros && docker compose up -d"
echo "  3. Configure your flight controller connection"
echo ""
echo "Note: ROS/MAVROS not installed (as requested)"
echo "==============================================================================="

log_warn "Please reboot the system to apply all changes: sudo reboot"
