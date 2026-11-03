# Camera benchmark results: camera-benchmark-20261103-180216.csv

Profiles measured: **10**

CPU columns show the mean and approximate 95th percentile across CPU core samples. GPU and RAM columns show mean and approximate 95th percentile for the whole Jetson.

| Profile | Capture | Grab FPS | Published | Target FPS | Measured Hz | CPU mean / p95 | GPU mean / p95 | RAM mean / p95 | Samples |
|---|---:|---:|---|---:|---:|---:|---:|---:|---:|
| vga15_native | VGA | 15 | NATIVE | 15 | 11.95 Hz | 21.4% / 50.0% | 7.2% / 25.0% | 61.3% / 70.4% | 21 |
| vga30_native | VGA | 30 | NATIVE | 30 | 25.28 Hz | 34.5% / 52.0% | 15.0% / 29.0% | 53.3% / 53.7% | 26 |
| vga60_native | VGA | 60 | NATIVE | 60 | 50.17 Hz | 43.3% / 55.0% | 26.9% / 36.0% | 54.0% / 54.5% | 21 |
| 720p15_native | HD720 | 15 | NATIVE | 15 | 48.77 Hz | 24.5% / 53.0% | 10.5% / 29.0% | 52.2% / 54.6% | 21 |
| 720p30_native | HD720 | 30 | NATIVE | 30 | 13.77 Hz | 49.8% / 66.0% | 16.6% / 25.0% | 54.0% / 54.3% | 23 |
| 720p30_half_15 | HD720 | 30 | CUSTOM ÷2.0 | 15 | 13.32 Hz | 46.5% / 61.0% | 8.8% / 26.0% | 53.8% / 54.0% | 29 |
| 720p30_half_30 | HD720 | 30 | CUSTOM ÷2.0 | 30 | — | 36.0% / 52.0% | 0.0% / 0.0% | 53.8% / 53.8% | 22 |
| 1080p15_native | HD1080 | 15 | NATIVE | 15 | — | 37.4% / 61.0% | 0.0% / 0.0% | 53.8% / 53.8% | 21 |
| 1080p30_native | HD1080 | 30 | NATIVE | 30 | — | 37.1% / 57.0% | 0.0% / 0.0% | 53.8% / 53.9% | 22 |
| 1080p30_half_15 | HD1080 | 30 | CUSTOM ÷2.0 | 15 | 10.72 Hz | 59.9% / 89.0% | 12.5% / 26.0% | 58.7% / 58.8% | 22 |

## Review notes

No image rate was recorded for: `720p30_half_30`, `1080p15_native`, `1080p30_native`. Check the configured image topic and each trial's launch log.

These are whole-device readings under the workload that was running during the test. Compare profiles only when the background workload and Nano power mode were consistent.

## Trial logs

- **vga15_native:** launch: `camera_benchmark/20261103-180216-vga15_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-vga15_native-tegrastats.log`
- **vga30_native:** launch: `camera_benchmark/20261103-180216-vga30_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-vga30_native-tegrastats.log`
- **vga60_native:** launch: `camera_benchmark/20261103-180216-vga60_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-vga60_native-tegrastats.log`
- **720p15_native:** launch: `camera_benchmark/20261103-180216-720p15_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-720p15_native-tegrastats.log`
- **720p30_native:** launch: `camera_benchmark/20261103-180216-720p30_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-720p30_native-tegrastats.log`
- **720p30_half_15:** launch: `camera_benchmark/20261103-180216-720p30_half_15-launch.log` · tegrastats: `camera_benchmark/20261103-180216-720p30_half_15-tegrastats.log`
- **720p30_half_30:** launch: `camera_benchmark/20261103-180216-720p30_half_30-launch.log` · tegrastats: `camera_benchmark/20261103-180216-720p30_half_30-tegrastats.log`
- **1080p15_native:** launch: `camera_benchmark/20261103-180216-1080p15_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-1080p15_native-tegrastats.log`
- **1080p30_native:** launch: `camera_benchmark/20261103-180216-1080p30_native-launch.log` · tegrastats: `camera_benchmark/20261103-180216-1080p30_native-tegrastats.log`
- **1080p30_half_15:** launch: `camera_benchmark/20261103-180216-1080p30_half_15-launch.log` · tegrastats: `camera_benchmark/20261103-180216-1080p30_half_15-tegrastats.log`
