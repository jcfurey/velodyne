[![](https://github.com/ros-drivers/velodyne/workflows/Basic%20Build%20Workflow/badge.svg?branch=ros2)](https://github.com/ros-drivers/velodyne/actions)

Overview
========

Velodyne<sup>1</sup> is a collection of ROS<sup>2</sup> packages supporting `Velodyne high
definition 3D LIDARs`<sup>3</sup>.

This fork targets ROS 2 Humble and newer with C++17 or newer. CI covers
Humble, Jazzy, Kilted, and Lyrical. The packages provide standalone nodes and
composable components.

Build and test from the root of a ROS 2 workspace after sourcing its ROS environment:

```sh
colcon build --packages-up-to velodyne --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select velodyne_driver velodyne_pointcloud velodyne_laserscan velodyne_msgs
colcon test-result --verbose
```

Runtime configuration
---------------------

Subscriptions use sensor-data QoS, accepting both reliable and best-effort
publishers. Publishers retain reliable QoS with depth 10. All nodes support
ROS 2 QoS overrides for reliability, history, and depth. For example, a driver
parameter file can select best-effort packet publication:

```yaml
velodyne_driver_node:
  ros__parameters:
    qos_overrides:
      /velodyne_packets:
        publisher:
          reliability: best_effort
          depth: 5
```

Use the resolved topic name, including any namespace or remapping. Only the
driver's `enabled` and `time_offset` parameters can change at runtime; other
configuration parameters are read-only after startup.

Correctness and memory handling
-------------------------------

The driver handles end-of-file without spinning and allows prompt shutdown
while waiting for packets or a PCAP repeat delay. Fixed-packet scans adjust
for VLP-16, VLP-32C, and VLS-128 dual-return packet rates. PCAP playback checks
capture, IP, and UDP lengths and supports Ethernet VLAN tags and IPv4 options.

Point clouds retain the packed XYZIRT field layout (`point_step: 22`), with
byte-safe reads and writes. Missing organized-cloud points use NaN, and HDL-64
rows reserve space for all 64 rings. Calibration IDs, counts, and finite
corrections are validated before decoding. VLS-128 dual returns occupy separate
rows with the same firing times; 32-channel sensors interpolate azimuth using
per-channel firing offsets.

LaserScan conversion accepts packed fields, padded rows, and either byte order.
It keeps the nearest valid return in each angular bin and its corresponding
intensity, including at the azimuth wrap. `resolution` is in radians.

- <sup>1</sup>Velodyne: http://www.ros.org/wiki/velodyne
- <sup>2</sup>ROS: http://www.ros.org
- <sup>3</sup>`Velodyne high definition 3D LIDARs`: http://www.velodynelidar.com/lidar/lidar.aspx
