# Docker Images

## Simulation: see oracle-software
The Gazebo simulator lives in its own repo, **Domer-Rover/oracle-software**. Check it out next to
this one; its `oracle-sim` container mounts this repo's `src/` and runs Gerbil's real URDF, EKF
and Nav2 configs from it, so tuning stays here. What Gerbil provides for it: `use_sim`,
`use_sim_depth` and `use_controllers` launch args, `gerbil_description/urdf/gerbil.gazebo.xacro`
and `gerbil_description/config/gazebo_bridge*.yaml`.

## Dockerfile.dev
Development environment with GUI support. For simulation, use oracle-software instead.

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
