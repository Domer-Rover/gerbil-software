# Docker Images

## Dockerfile.dev
Development environment with GUI support.

Features:
- Ubuntu desktop via noVNC (http://localhost:6080)
- RViz, Gazebo (Intel/AMD only)
- ZED2i camera support
- Navigation2 packages

Usage:
```bash
docker-compose up ros-dev
```

## Dockerfile.jetson
Headless environment for Jetson hardware.

Features:
- ARM64 optimized
- ZED2i with Jetson SDK
- Hardware access (UART, I2C, USB)

Usage:
```bash
docker-compose up ros-jetson
docker exec -it gerbil-jetson bash
```
