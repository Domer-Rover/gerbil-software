# Remote ROS2 Visualization

Visualize data from Jetson on your development machine.

## Network Diagram

```
Jetson (Headless)  ◄──WiFi/LAN──►  Dev Machine
• Robot nodes                       • RViz2
• Sensors                           • rqt tools
```

## Method 1: Automatic Discovery

### Prerequisites
- Both machines on same subnet
- No firewall blocking UDP multicast

### Jetson Setup
```bash
docker-compose up ros-jetson
docker exec -it gerbil-jetson bash

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
cd /home/ubuntu/ros2_ws
colcon build
source install/setup.bash

ros2 launch imu_package bno055_imu.launch.py
```

### Dev Machine Setup
```bash
docker-compose up ros-dev
# Access noVNC at http://localhost:6080

export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

ros2 topic list
rviz2
```

## Method 2: Explicit Network Configuration

### Jetson
```bash
hostname -I  # Note IP address

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Create `fastdds_profile.xml`:
```xml
<?xml version="1.0" encoding="UTF-8"?>
<profiles>
  <participant profile_name="custom_participant">
    <rtps>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator>
            <udpv4>
              <address>JETSON_IP</address>
            </udpv4>
          </locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator>
            <udpv4>
              <address>DEV_MACHINE_IP</address>
            </udpv4>
          </locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds_profile.xml
```

### Dev Machine
Mirror the same setup with IPs swapped.

## Method 3: WiFi Hotspot (University Networks)

Create WiFi hotspot on laptop, connect Jetson to hotspot. No network restrictions.

## Method 4: Direct Ethernet

Connect Jetson and laptop via Ethernet cable with static IPs:
- Jetson: 169.254.1.2/24
- Laptop: 169.254.1.1/24

## Method 5: ROS Bags (Offline)

Record on Jetson:
```bash
ros2 bag record -a
```

Replay on dev machine:
```bash
ros2 bag play <bag_file>
rviz2
```

## Troubleshooting

Check domain ID matches:
```bash
echo $ROS_DOMAIN_ID
```

Test connectivity:
```bash
ping <other-device-ip>
```

Test multicast:
```bash
# Jetson:
ros2 multicast send

# Dev:
ros2 multicast receive
```

Test ROS2:
```bash
# Jetson:
ros2 run demo_nodes_cpp talker

# Dev:
ros2 topic echo /chatter
```
