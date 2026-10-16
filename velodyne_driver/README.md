Velodyne ROS 2 driver
=====================

This is a ROS 2 driver for Velodyne devices.  It currently supports the 64E(S2, S2.1, S3), the 32E, the 32C, the VLP-16, and the VLS-128.  This driver is responsible for taking the data from the Velodyne and combining it into one message per revolution.  To turn that raw data into a pointcloud or laserscan, see the velodyne_pointcloud and velodyne_laserscan packages.

Published Topics
----------------
* `velodyne_packets` (`velodyne_msgs/VelodyneScan`) - The raw data from one full revolution of the sensor.

Parameters
----------
* `device_ip` (string) - The IP address that the Velodyne is on.  From the factory, this is 192.168.1.201.
* `gps_time` (bool) - Whether to use data capture time from a GPS, or from the local time.  Should only be set to True if a GPS is attached to the Velodyne.  False by default.
* `time_offset` (double) - An arbitrary "skew", in seconds, to add to the acquisition timestamp.  Defaults to 0.0.
* `enabled` (bool) - Whether the device should start-up enabled or not.  Defaults to True.
* `read_once` (bool) - Whether to only playback the data once (True) or continuously (False).  Only used in PCAP playback mode.  Defaults to False.
* `read_fast` (bool) - Whether to output the data as fast as possible, (True), or sleep the appropriate delay between packets (False).  Only used in PCAP playback mode.  Defaults to False.
* `repeat_delay` (double) - The time to wait between repeats in continuous playback mode.  Only used in PCAP playback mode.  Defaults to 0.0.
* `frame_id` (string) - The frame_id to use when constructing the header for the packet to be published.  Defaults to "velodyne".
* `model` (string) - The model number of the Velodyne attached.  This should be one of `64E`, `64E_S2`, `64E_S2.1`, `64E_S3`, `32E`, `32C`, `VLP16`, or `VLS128`.  Defaults to `64E`.
* `rpm` (double) - The RPM that the Velodyne is configured for.  Note that this is descriptive, not prescriptive, so this should be set to match the value configured through the Velodyne web interface.
* `pcap` (string) - The PCAP playback file to use to playback data from.  Only used in PCAP playback mode.  Defaults to the empty string.
* `cut_angle` (double) - The azimuth angle at which to declare a single rotation complete.  If this is less than 0, then a fixed number of packets (device-dependent) is used per rotation.  This mostly works, but can vary because of variations in the hardware.  If a positive number <= 2*Pi, a rotation will be declared "complete" when the azimuth reported by the device reaches that value.  Defaults to -1.0.
* `port` (int) - The port on which to receive data from the Velodyne.  From the factory, the Velodyne is configured to publish data on 2368.  Defaults to 2368.

* `timestamp_first_packet` (bool) - Use the first packet stamp for the scan header;
  otherwise use the last packet stamp. Defaults to False.

`enabled` and `time_offset` can change at runtime; all other driver configuration
is read-only after startup. Offsets are applied in seconds in local, GPS, and
PCAP modes. A PCAP stamp represents replay time, rather than capture time.

PCAP input requires Ethernet IPv4/UDP frames. VLAN tags and IPv4 options are
supported; truncated, fragmented, and incorrectly sized datagrams are skipped.
`read_once` stops the reader at EOF. Repeat delays and packet waits can be
interrupted when unloading a component. Repeat delay must be finite and between
0 and 86400 seconds. RPM must be finite and positive.

For fixed-packet scans, dual-return packet counts are twice the single-return
count for VLP-16 and VLP-32C, and three times for VLS-128. The cut-angle value
2*Pi is treated as zero. Negative cut angles disable cutting.

Publication defaults to reliable QoS with depth 10; reliability, history, and
depth can be overridden as described in the repository README.
