# Docker Images

## Dockerfile.sim (gerbil-sim): recommended for simulation
ROS 2 Humble and Gazebo Fortress in a browser desktop (noVNC). Builds natively on Apple-Silicon
Macs (arm64) and on x86, with no GPU needed: rendering uses Mesa llvmpipe.

```bash
docker compose build gerbil-sim
docker compose up -d gerbil-sim          # stop ros-dev first: both use port 6080
```

1. Open http://localhost:6080 (VNC password: `ubuntu`).
2. Open the **Gerbil Workspace** desktop icon (or any terminal) and run `gerbil-build`.
3. Run `source ~/ros2_ws/install/setup.bash`. New terminals do this automatically.

From the host, use `docker exec -it -u ubuntu gerbil-sim bash`. Run it as `ubuntu`, not root,
so GUI apps can reach the VNC display.

- `gerbil-build` runs `colcon build --symlink-install` as `ubuntu`, skipping the ZED, `gerbil_hw`,
  `roboclaw_serial` and `ldlidar_stl_ros2` packages (list: `docker/sim/colcon_defaults.yaml`).
  Extra arguments are passed to colcon, e.g. `gerbil-build --packages-select gerbil_gazebo`.
- `build/`, `install/` and `log/` live in named volumes. After rebuilding the image, reset them with
  `docker compose down -v`.
- Foxglove Studio can connect to `ws://localhost:8765` once foxglove_bridge runs.
- If the Gazebo GUI fails to create a GL context, try `MESA_GL_VERSION_OVERRIDE=4.5` or
  `ign gazebo --render-engine ogre`. It is not set by default, because it only changes the
  GL version Mesa reports, not what it can do.

See `docs/simulation.md` for running the simulator itself.

## Dockerfile.dev
Development environment with GUI support. For simulation, prefer gerbil-sim above.

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
