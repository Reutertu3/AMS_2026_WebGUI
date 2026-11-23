#!/bin/bash
# Stop everything start.sh started. The web control goes first: it stops the robot and a running script.
if [ -t 1 ]; then G=$'\e[32m' R=$'\e[31m' D=$'\e[2m' B=$'\e[1m' N=$'\e[0m'; else G='' R='' D='' B='' N=''; fi
stop() {
  printf '  %-36s' "$2"
  if docker ps -q -f name="^$1\$" | grep -q .; then
    if docker stop "$1" >/dev/null 2>&1; then echo "${G}✓ stopped${N}"; else echo "${R}✗ failed${N} ${D}(docker stop $1)${N}"; fi
  else
    echo "${D}– not running${N}"
  fi
}
echo "${B}EduRob${N} ${D}· stopping${N}"
stop edurob-webui "Web control (robot + script stop)"
stop edurob-slam "SLAM (unsaved map is lost)"
stop rplidar "Lidar driver"
stop micro-ros-agent "micro-ROS agent"
exit 0
