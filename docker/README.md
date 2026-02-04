# Docker Images

This directory contains Dockerfiles for different deployment scenarios:

## Dockerfile.dev
**For:** Mac/Windows development with GUI
**Features:**
- Full Ubuntu desktop via noVNC (access at http://localhost:6080)
- RViz for visualization
- Gazebo simulator (on Intel/AMD only)
- ZED2i camera support
- TurtleBot3 packages for Nav2 tutorials

**Usage:**
```bash
# Build and run
docker-compose up ros-dev

# Access via browser
open http://localhost:6080
```

## Dockerfile.jetson
**For:** Jetson Nano/Orin embedded deployment
**Features:**
- Headless (no GUI) for performance
- Optimized for ARM64
- ZED2i camera with Jetson-specific SDK
- Hardware access (UART, I2C, USB)
- Smaller image size

**Usage:**
```bash
# On Jetson, build and run
docker-compose up ros-jetson

# Enter container
docker exec -it gerbil-jetson bash
```
