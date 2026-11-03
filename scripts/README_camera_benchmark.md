# ZED2i camera benchmark (Jetson Nano)

This benchmark compares ten ZED2i capture and publication profiles and records
the Jetson's **whole-system** CPU, GPU, and RAM usage while each profile runs.
It also measures the published image topic rate. Run it on the Jetson Nano,
not on a development laptop: the readings depend on the Nano's memory, cooling,
power mode, ROS graph, and competing workloads.

The script starts and stops the ZED camera once per profile. It does not edit
the checked-in ZED YAML files. Temporary ROS parameter overrides select each
profile. Other processes that are already running stay up, so their load is
included in the system telemetry.

## Before the test

1. Put the robot on a secure stand or otherwise make sure its wheels cannot
   move. Do not send navigation goals or run a motion test during this benchmark.
2. SSH to the Nano and source the same ROS environment used for deployment. For
   example:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/domerrover/gerbil-software/install/setup.bash
   cd ~/domerrover/gerbil-software
   ```

   Adjust the workspace path if the repository is checked out elsewhere.
3. Start the non-camera workload you want represented in the results, and let
   it settle. Keep the same nodes, Foxglove connection, detection subscribers,
   and other expected services running for every profile. The benchmark's
   `tegrastats` readings cover the whole Jetson, not just the camera process.
4. Make sure no other ZED camera node is running. The benchmark launches its
   own ZED node for each profile. A second camera node would conflict for the
   device and invalidate the run.

   **Launch limitation:** the current `gerbil_nav2.launch.py` starts its own
   ZED camera and has no `launch_zed:=false` argument. Do not launch it as-is
   alongside this benchmark. To measure Nav2 concurrently, the test setup must
   start Nav2 without its included camera (for example, by using a test launch
   that separates the ZED include). Record that setup with the results. The
   default run measures the ZED pipeline under whatever other processes are
   already running; it does not start the full autonav stack itself.

## Run

First, optionally list the ten planned profiles without touching the camera:

```bash
python3 scripts/benchmark_camera_configs.py --dry-run
```

Then run the standard test:

```bash
python3 scripts/benchmark_camera_configs.py
```

Each profile has a 20-second warm-up followed by a 20-second measurement. A
full run takes about 7 minutes plus camera startup and shutdown. Keep the Nano
on its normal power supply and in its normal deployment power mode. Do not
change CPU/GPU clocks, power mode, cooling, or the set of background processes
between profiles. If the robot is thermally throttling or RAM is nearly full,
stop the run and note that rather than forcing the remaining profiles through.

Options for a longer sample or a different image topic:

```bash
python3 scripts/benchmark_camera_configs.py --duration 60 --startup 30
python3 scripts/benchmark_camera_configs.py --topic /zed/zed_node/rgb/image_rect_color
```

The default topic is `/zed/zed_node/rgb/color/rect/image` (the SDK 5 topic).
Use the alternate topic only if that is the topic published by the installed
ZED wrapper version. The script expects `ros2`, `tegrastats`, and the Python
standard library to be available in the sourced environment.

## Results

The default output directory is `camera_benchmark/` in the current directory.
The timestamped CSV contains one row per profile, including:

- requested capture resolution/rate and published resolution/rate;
- measured ROS image topic rate;
- mean and approximate 95th-percentile CPU use per core, GPU use, and RAM use;
- paths to the raw `tegrastats` and ROS camera launch logs for that profile.

If a trial reports an empty image rate or the camera launch exits, inspect that
profile's `*-launch.log` before interpreting its load readings. The topic-rate
sample is time-bounded; if the selected topic does not appear during the sample,
the rate is left empty and the benchmark continues to the next profile.

Keep the CSV and raw logs together. Also report the Nano model/RAM size, power
mode, ambient/thermal conditions, ROS and ZED SDK versions, which other
processes were running, and whether Foxglove or object-detection consumers were
connected. Compare the system-load columns across profiles under the same
workload; do not pick a profile from resolution or FPS alone. A higher image
rate than requested can indicate the topic measurement or launch did not use
the expected settings; an empty rate means the selected topic was not observed.

The ten profiles are candidates, not a guarantee that every combination is
sustainable on the Nano. Treat high sustained GPU/CPU use, rising RAM use,
thermal throttling, dropped image rate, or unstable camera startup as a failed
profile. Choose a profile with headroom for the additional processes used
during real autonav, then validate it in a controlled on-robot run.
