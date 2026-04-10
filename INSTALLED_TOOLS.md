# NovaROS - Installed Tools Reference

## Target Platform
- **Device:** Raspberry Pi 5 (8GB RAM)
- **OS:** Debian GNU/Linux 13 (Trixie)
- **Application:** Advanced Police/Public Safety Drone Operations
- **Install Date:** 2026-02-01

---

## 1. System & Build Essentials
| Tool | Purpose |
|------|---------|
| build-essential | GCC, G++, make compilers |
| cmake | Build system generator |
| git | Version control |
| curl, wget | File download utilities |
| vim, nano | Text editors |
| htop, btop, tmux, screen | Process management & terminals |
| jq | JSON processor |

---

## 2. Python Stack
### Core
| Package | Version | Purpose |
|---------|---------|---------|
| python3 | 3.13.5 | Main interpreter |
| pip3 | Latest | Package manager |
| python3-venv | - | Virtual environments |

### System Packages (apt)
- numpy, scipy, pandas, matplotlib
- opencv, pillow
- flask, requests
- pyserial, gpiozero, smbus2

### MAVLink / Drone Communication (in /opt/novaros-env)
| Package | Version | Purpose |
|---------|---------|---------|
| pymavlink | 2.4.49 | MAVLink protocol library |
| mavproxy | 1.8.74 | MAVLink ground station |
| dronekit | 2.9.2 | High-level drone API |

### Additional Python Libraries (in venv)
- geopy, pyproj, shapely (GPS/Geospatial)
- websockets, aiohttp (Async networking)
- flask-cors, flask-socketio (Web)
- fastapi, uvicorn (Modern API)
- python-dotenv, loguru (Utilities)

---

## 3. Networking Tools
| Tool | Version | Purpose |
|------|---------|---------|
| nmap | 7.95 | Network scanner |
| tcpdump | 4.99.5 | Packet capture |
| tshark/wireshark | 4.4.7 | Protocol analyzer |
| iptables | 1.8.11 | Firewall rules |
| ufw | 0.36.2 | Uncomplicated firewall |
| openssh | 10.0p1 | SSH server/client |
| iperf3 | 3.18 | Bandwidth testing |
| hostapd | - | Access point daemon |
| dnsmasq | - | DHCP/DNS server |

---

## 4. MERN Stack
### Node.js
| Component | Version | Purpose |
|-----------|---------|---------|
| node | 20.20.0 | JavaScript runtime |
| npm | 11.8.0 | Package manager |
| yarn | Latest | Alt package manager |
| pm2 | Latest | Process manager |
| nodemon | Latest | Dev auto-restart |

### Databases
| Database | Version | Purpose |
|----------|---------|---------|
| MongoDB | 7.0 | Document database (Docker) |
| PostgreSQL | 17.7 | Relational database |

---

## 5. Docker
| Component | Version | Purpose |
|-----------|---------|---------|
| docker | 29.2.0 | Container runtime |
| docker-compose | 5.0.2 | Multi-container orchestration |

---

## 6. Video & Streaming
| Tool | Version | Purpose |
|------|---------|---------|
| ffmpeg | 7.1.3 | Video transcoding |
| gstreamer | 1.26.2 | Streaming pipelines |
| v4l-utils | - | Video4Linux utilities |
| vlc | - | Media player |

---

## 7. GPS & Location
| Tool | Purpose |
|------|---------|
| gpsd | GPS daemon |
| gpsd-clients | GPS client tools |
| chrony | NTP time sync |

---

## 8. Hardware Interfaces
| Tool | Purpose |
|------|---------|
| minicom | Serial terminal |
| picocom | Simple serial terminal |
| i2c-tools | I2C debugging |
| can-utils | CAN bus utilities |

---

## 9. Security Tools
| Tool | Purpose |
|------|---------|
| fail2ban | Intrusion prevention |
| rkhunter | Rootkit scanner |
| clamav | Antivirus |
| lynis | Security auditing |
| auditd | Audit daemon |

---

## 10. System Monitoring
| Tool | Purpose |
|------|---------|
| btop | Modern process viewer |
| htop | Process viewer |
| iotop | I/O monitor |
| nethogs | Network per-process |
| iftop | Network bandwidth |
| sysstat | System statistics |
| lm-sensors | Temperature sensors |
| wavemon | WiFi monitor |

---

## Project Structure
```
~/novaros/
├── src/
│   ├── mavlink/      # MAVLink communication
│   ├── vision/       # Computer vision
│   ├── networking/   # Network services
│   ├── api/          # REST/WebSocket APIs
│   └── database/     # DB models/queries
├── config/           # Configuration files
├── logs/             # Application logs
├── data/             # Data storage
├── scripts/          # Utility scripts
├── models/           # ML models
├── tests/            # Test files
├── docker-compose.yml
└── install.sh        # Reusable installer
```

---

## Quick Commands
```bash
# Activate drone environment
source /opt/novaros-env/bin/activate
# Or simply:
novaros-env

# Test MAVLink
mavproxy.py --help

# Start MongoDB
cd ~/novaros && docker compose up -d

# Monitor system
btop

# Check services
sudo systemctl status ssh postgresql docker fail2ban
```

---

## NOT Installed (As Requested)
- ROS (Robot Operating System)
- MAVROS (ROS-MAVLink bridge)

---

## Services Status
All services configured to start on boot:
- SSH (active)
- PostgreSQL (active)
- Docker (active)
- fail2ban (active)
