#!/bin/bash
# Runs inside the rplidar container (started by start.sh): keeps the lidar driver alive.
# The driver doesn't reconnect when the robot reboots (its TCP connection to the lidar dies,
# the node keeps running but publishes nothing). So: if no /scan arrives for 10 s, restart it.
# usage: lidar_watchdog.sh <robot ip>
# (ROS is already set up by the image's entrypoint)
while true; do
  echo "[watchdog] starting lidar driver ($1:44227)"
  # frame laser_fix: hangs on the re-stamped tree from odom_fix.py (the robot's own "laser" frame has broken stamps)
  ros2 launch sllidar_ros2 sllidar_s1_tcp_launch.py tcp_port:=44227 tcp_ip:="$1" frame_id:=laser_fix &
  pid=$!
  sleep 10  # time to connect
  # -k: ros2 topic echo sometimes ignores SIGTERM and hangs forever, which froze the watchdog
  while timeout -k 3 10 ros2 topic echo --once /scan --field header.stamp.sec >/dev/null 2>&1; do
    sleep 5
  done
  echo "[watchdog] no /scan for 10 s, restarting the lidar driver"
  kill -INT $pid 2>/dev/null
  for _ in 1 2 3 4 5; do kill -0 $pid 2>/dev/null || break; sleep 1; done
  pkill -9 -f sllidar_node 2>/dev/null
  wait $pid 2>/dev/null
done
