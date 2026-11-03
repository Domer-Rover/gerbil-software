# Camera benchmark results: camera-benchmark-20261103-172106.csv

Profiles measured: **10**

CPU columns show the mean and approximate 95th percentile across CPU core samples. GPU and RAM columns show mean and approximate 95th percentile for the whole Jetson.

| Profile | Capture | Grab FPS | Published | Target FPS | Measured Hz | CPU mean / p95 | GPU mean / p95 | RAM mean / p95 | Samples |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|
| vga15_native | VGA | 15 | NATIVE | 15 | — | 18.9% / 99.0% | 0.0% / 0.0% | 30.0% / 30.5% | 21 |
| vga30_native | VGA | 30 | NATIVE | 30 | — | 20.6% / 99.0% | 0.0% / 0.0% | 30.7% / 30.9% | 21 |
| vga60_native | VGA | 60 | NATIVE | 60 | — | 19.2% / 99.0% | 0.0% / 0.0% | 31.3% / 31.8% | 21 |
| 720p15_native | HD720 | 15 | NATIVE | 15 | — | 19.0% / 99.0% | 0.0% / 0.0% | 31.3% / 31.7% | 21 |
| 720p30_native | HD720 | 30 | NATIVE | 30 | — | 18.8% / 99.0% | 0.0% / 0.0% | 31.3% / 31.7% | 21 |
| 720p30_half_15 | HD720 | 30 | CUSTOM ÷2.0 | 15 | — | 28.2% / 99.0% | 0.0% / 0.0% | 31.6% / 31.9% | 21 |
| 720p30_half_30 | HD720 | 30 | CUSTOM ÷2.0 | 30 | — | 26.7% / 99.0% | 0.0% / 0.0% | 44.1% / 45.6% | 16 |
| 1080p15_native | HD1080 | 15 | NATIVE | 15 | — | 20.2% / 99.0% | 0.0% / 0.0% | 42.8% / 43.2% | 21 |
| 1080p30_native | HD1080 | 30 | NATIVE | 30 | — | 20.0% / 99.0% | 0.0% / 0.0% | 42.9% / 43.4% | 21 |
| 1080p30_half_15 | HD1080 | 30 | CUSTOM ÷2.0 | 15 | — | 20.0% / 99.0% | 0.0% / 0.0% | 42.2% / 43.0% | 21 |

## Review notes

No image rate was recorded for: `vga15_native`, `vga30_native`, `vga60_native`, `720p15_native`, `720p30_native`, `720p30_half_15`, `720p30_half_30`, `1080p15_native`, `1080p30_native`, `1080p30_half_15`. Check the configured image topic and each trial's launch log.

These are whole-device readings under the workload that was running during the test. Compare profiles only when the background workload and Nano power mode were consistent.

## Trial logs

- **vga15_native:** launch: `camera_benchmark/20261103-172106-vga15_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-vga15_native-tegrastats.log`
- **vga30_native:** launch: `camera_benchmark/20261103-172106-vga30_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-vga30_native-tegrastats.log`
- **vga60_native:** launch: `camera_benchmark/20261103-172106-vga60_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-vga60_native-tegrastats.log`
- **720p15_native:** launch: `camera_benchmark/20261103-172106-720p15_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-720p15_native-tegrastats.log`
- **720p30_native:** launch: `camera_benchmark/20261103-172106-720p30_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-720p30_native-tegrastats.log`
- **720p30_half_15:** launch: `camera_benchmark/20261103-172106-720p30_half_15-launch.log` · tegrastats: `camera_benchmark/20261103-172106-720p30_half_15-tegrastats.log`
- **720p30_half_30:** launch: `camera_benchmark/20261103-172106-720p30_half_30-launch.log` · tegrastats: `camera_benchmark/20261103-172106-720p30_half_30-tegrastats.log`
- **1080p15_native:** launch: `camera_benchmark/20261103-172106-1080p15_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-1080p15_native-tegrastats.log`
- **1080p30_native:** launch: `camera_benchmark/20261103-172106-1080p30_native-launch.log` · tegrastats: `camera_benchmark/20261103-172106-1080p30_native-tegrastats.log`
- **1080p30_half_15:** launch: `camera_benchmark/20261103-172106-1080p30_half_15-launch.log` · tegrastats: `camera_benchmark/20261103-172106-1080p30_half_15-tegrastats.log`
