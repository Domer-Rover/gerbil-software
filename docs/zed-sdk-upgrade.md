# Upgrading Gerbil to ZED SDK 5.2

Gerbil vendored `zed_wrapper` 4.2.1 while Capybara vendors 5.2.0. This branch
swaps Gerbil to 5.2.0 so both robots run one SDK. The wrapper version and the
installed SDK must match.

## 1. Check what Capybara has (source of truth)

On Capybara's Jetson:

```bash
cat /usr/local/zed/zed-config-version.cmake | head -5   # installed SDK version
ls /usr/local/zed/                                       # tools live here
```

Install the same version on Gerbil, not just "the latest".

## 2. Install the SDK on Gerbil's Jetson

Download the installer for your exact JetPack/L4T from
<https://www.stereolabs.com/developers/release> (the Jetson section), then:

```bash
chmod +x ZED_SDK_Tegra_*.run
./ZED_SDK_Tegra_*.run          # not sudo; it asks for sudo itself
```

Answer no to the Python API and samples unless you want them. It needs a few
GB free and pulls AI models on first run.

Verify:

```bash
/usr/local/zed/tools/ZED_Diagnostic
ls /dev/video*
```

## 3. Rebuild the workspace

```bash
cd ~/gerbil-software
git pull
rm -rf build install log        # the old wrapper's artifacts reference SDK 4
colcon build --symlink-install
source install/setup.bash
```

## 4. Check it came up

```bash
ros2 launch gerbil_bringup gerbil_foxglove.launch.py
ros2 topic list | grep zed_node | head
ros2 topic hz /zed/zed_node/odom
```

Topic names changed with the SDK: 5.x publishes
`/zed/zed_node/rgb/color/rect/image`, 4.x published
`/zed/zed_node/rgb/image_rect_color`. The Foxglove whitelist and
`scripts/aruco_detector.py` on this branch already use the 5.x names.

## What changed in the configs

| Setting | 4.2.1 | 5.2.0 |
|---|---|---|
| `pos_tracking_mode` | GEN_2 | **GEN_3** (the main reason to upgrade) |
| `grab_resolution` | HD720 | HD720 |
| `grab_frame_rate` | 30 | 30 |
| `area_memory` | false | false |
| `depth_mode` | NEURAL_LIGHT | NEURAL_LIGHT |
| `two_d_mode` | false | true (planar indoor robot) |

## If it goes wrong

The previous state is one branch away:

```bash
git checkout feat/nav2     # still on wrapper 4.2.1
rm -rf build install log && colcon build --symlink-install
```

The SDK install itself is not reverted by that; reinstall 4.2 from the
Stereolabs archive if needed.
