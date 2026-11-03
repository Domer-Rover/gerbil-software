#!/usr/bin/env python3
"""Compare ten ZED2i camera profiles on a Jetson Nano.

Run on the Jetson from a sourced ROS 2 environment. This starts only the ZED
camera launch (no robot controllers); each profile is passed as a temporary
ROS parameter override, so repository camera configs are left unchanged.
"""

import argparse
import csv
import datetime as dt
import os
import re
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path


PROFILES = [
    # name, grab resolution, grab fps, publication resolution, downscale, pub fps
    ("vga15_native", "VGA", 15, "NATIVE", 1.0, 15),
    ("vga30_native", "VGA", 30, "NATIVE", 1.0, 30),
    ("vga60_native", "VGA", 60, "NATIVE", 1.0, 60),
    ("720p15_native", "HD720", 15, "NATIVE", 1.0, 15),
    ("720p30_native", "HD720", 30, "NATIVE", 1.0, 30),
    ("720p30_half_15", "HD720", 30, "CUSTOM", 2.0, 15),
    ("720p30_half_30", "HD720", 30, "CUSTOM", 2.0, 30),
    ("1080p15_native", "HD1080", 15, "NATIVE", 1.0, 15),
    ("1080p30_native", "HD1080", 30, "NATIVE", 1.0, 30),
    ("1080p30_half_15", "HD1080", 30, "CUSTOM", 2.0, 15),
]


def override_yaml(profile):
    _, resolution, grab_fps, pub_resolution, downscale, pub_fps = profile
    return (
        "/**:\n"
        "  ros__parameters:\n"
        "    general:\n"
        f"      grab_resolution: '{resolution}'\n"
        f"      grab_frame_rate: {grab_fps}\n"
        f"      pub_resolution: '{pub_resolution}'\n"
        f"      pub_downscale_factor: {downscale}\n"
        f"      pub_frame_rate: {float(pub_fps)}\n"
    )


def percentiles(values):
    if not values:
        return ""
    values.sort()
    return round(values[int((len(values) - 1) * 0.95)], 1)


def read_tegrastats(path):
    text = path.read_text(errors="replace") if path.exists() else ""
    ram, gpu, cpu = [], [], []
    for line in text.splitlines():
        m = re.search(r"RAM\s+(\d+)\s*/\s*(\d+)MB", line)
        if m:
            ram.append(100.0 * int(m.group(1)) / max(1, int(m.group(2))))
        m = re.search(r"GR3D_FREQ\s+(\d+)%", line)
        if m:
            gpu.append(float(m.group(1)))
        m = re.search(r"CPU\s*\[([^\]]+)\]", line)
        if m:
            cpu.extend(float(x) for x in re.findall(r"(\d+)%@", m.group(1)))
    return {
        "samples": len(ram),
        "ram_mean_pct": round(statistics.mean(ram), 1) if ram else "",
        "ram_p95_pct": percentiles(ram),
        "gpu_mean_pct": round(statistics.mean(gpu), 1) if gpu else "",
        "gpu_p95_pct": percentiles(gpu),
        "cpu_core_mean_pct": round(statistics.mean(cpu), 1) if cpu else "",
        "cpu_core_p95_pct": percentiles(cpu),
    }


def topic_rate(topic, seconds):
    """Sample ROS topic hz output; return mean rate when reported."""
    if shutil.which("ros2") is None:
        return ""
    proc = subprocess.Popen(
        ["ros2", "topic", "hz", topic, "--window", "20"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True,
    )
    output = []
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            line = proc.stdout.readline()
            if line:
                output.append(line)
            elif proc.poll() is not None:
                break
    finally:
        stop_process(proc)
    rates = []
    for line in output:
        m = re.search(r"average rate:\s*([\d.]+)", line)
        if m:
            rates.append(float(m.group(1)))
    return round(statistics.mean(rates), 2) if rates else ""


def stop_process(proc):
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=8)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=4)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=20,
                        help="steady-state measurement seconds per profile (default: 20)")
    parser.add_argument("--startup", type=int, default=20,
                        help="seconds to let the camera settle before measuring (default: 20)")
    parser.add_argument("--topic", default="/zed/zed_node/rgb/color/rect/image",
                        help="ROS image topic to measure (SDK 5 default)")
    parser.add_argument("--output", type=Path, default=Path("camera_benchmark"),
                        help="output directory for CSV and raw tegrastats logs")
    parser.add_argument("--camera-name", default="zed")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the ten profiles without starting hardware")
    args = parser.parse_args()

    if args.dry_run:
        print("name,resolution,grab_fps,pub_resolution,downscale,pub_fps")
        for p in PROFILES:
            print(",".join(map(str, p)))
        return 0
    if shutil.which("ros2") is None:
        parser.error("ros2 was not found; source /opt/ros/<distro>/setup.bash and the workspace install/setup.bash")
    if shutil.which("tegrastats") is None:
        parser.error("tegrastats was not found; run this script on the Jetson Nano")
    if args.duration < 10 or args.startup < 0:
        parser.error("--duration must be at least 10 seconds; --startup cannot be negative")

    args.output.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = args.output / f"camera-benchmark-{stamp}.csv"
    rows = []
    print(f"Results: {csv_path}")
    print("Each trial starts the ZED camera only; stop other camera/robot launches first.")

    with tempfile.TemporaryDirectory(prefix="zed-benchmark-") as tmp:
        for index, profile in enumerate(PROFILES, 1):
            name = profile[0]
            override = Path(tmp) / f"{name}.yaml"
            override.write_text(override_yaml(profile))
            stats_path = args.output / f"{stamp}-{name}-tegrastats.log"
            launch_cmd = [
                "ros2", "launch", "zed_wrapper", "zed_camera.launch.py",
                "camera_model:=zed2i", f"camera_name:={args.camera_name}",
                "publish_urdf:=false", "publish_tf:=false", "publish_map_tf:=false",
                f"ros_params_override_path:={override}",
            ]
            print(f"[{index}/10] {name}: starting camera")
            launch = subprocess.Popen(
                launch_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            stats = None
            try:
                time.sleep(args.startup)
                if launch.poll() is not None:
                    print(f"  launch exited early ({launch.returncode}); inspect ROS logs")
                stats = subprocess.Popen(
                    ["tegrastats", "--interval", "1000", "--logfile", str(stats_path)],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                measured_rate = topic_rate(args.topic, args.duration)
                time.sleep(1.2)  # let tegrastats flush its last sample
            except KeyboardInterrupt:
                print("\nInterrupted; stopping active processes.")
                stop_process(launch)
                if stats:
                    stop_process(stats)
                raise
            finally:
                stop_process(launch)
                if stats:
                    stop_process(stats)

            row = {
                "name": name,
                "grab_resolution": profile[1],
                "grab_fps": profile[2],
                "pub_resolution": profile[3],
                "downscale": profile[4],
                "pub_fps": profile[5],
                "measured_image_hz": measured_rate,
                **read_tegrastats(stats_path),
                "tegrastats_log": str(stats_path),
            }
            rows.append(row)
            print(f"  image={measured_rate or 'unavailable'} Hz; "
                  f"CPU(core mean/p95)={row['cpu_core_mean_pct']}/{row['cpu_core_p95_pct']}%; "
                  f"GPU(mean/p95)={row['gpu_mean_pct']}/{row['gpu_p95_pct']}%; "
                  f"RAM(mean/p95)={row['ram_mean_pct']}/{row['ram_p95_pct']}%")

    if rows:
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Done. Review {csv_path}; raw Jetson telemetry is in {args.output}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
