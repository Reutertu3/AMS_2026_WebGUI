#!/bin/bash
# Run a command in the ROS 2 kilted container (same domain/network as micro-ros-agent), with the ws venv active.
# usage: ./ros.sh [cmd...]            (default: interactive shell), e.g. ./ros.sh python3 scripts/square.py
#        ./ros.sh --turtle [cmd...]   start turtlesim and drive the turtle instead of the robot:
#                                     own ROS domain (robot can't move), cmd_vel/turtle_pose remapped to /turtle1/...
#        e.g. ./ros.sh --turtle python3 webui.py
#             ./ros.sh --turtle python3 scripts/square.py
DOMAIN=${DOMAIN:-12}  # ROS domain of the robot, other one: DOMAIN=13 ./ros.sh ...
INNER='source /opt/ros/kilted/setup.bash && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; } && { [ ! -f /ws/.venv/bin/activate ] || source /ws/.venv/bin/activate; } && exec "$@"'
EXTRA=()

if [ "$1" = "--turtle" ]; then
  shift
  DOMAIN=42
  # Show the turtlesim window on the host's X display
  EXTRA=(-e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
  [ -n "$XAUTHORITY" ] && EXTRA+=(-e XAUTHORITY=/tmp/.Xauthority -v "$XAUTHORITY":/tmp/.Xauthority:ro)
  # Window size: turtlesim is fixed at 500x500 px, Qt scales it by this factor (TURTLE_SCALE=1 for the original size)
  EXTRA+=(-e QT_SCALE_FACTOR="${TURTLE_SCALE:-2}")
  # Start turtlesim in the background (holonomic = it can strafe like the mecanum robot); append the remap to the command (a plain shell gets no remap)
  # Custom turtle picture: if ws/turtle.png exists, turtlesim gets a fake share folder where every
  # turtle image is that picture (scaled to fit 45x45 px, top of the picture = forward)
  INNER='source /opt/ros/kilted/setup.bash && { [ ! -f /ws/install/setup.bash ] || source /ws/install/setup.bash; } && { [ ! -f /ws/.venv/bin/activate ] || source /ws/.venv/bin/activate; }
    IMG_PREFIX=
    if [ -f /ws/turtle.png ]; then
      IMG_PREFIX=/tmp/turtle_img
      mkdir -p $IMG_PREFIX/share/turtlesim/images $IMG_PREFIX/share/ament_index/resource_index/packages
      touch $IMG_PREFIX/share/ament_index/resource_index/packages/turtlesim
      python3 -c "
import os, sys
from PIL import Image
im = Image.open(sys.argv[1]).convert(\"RGBA\")
im.thumbnail((45, 45))
canvas = Image.new(\"RGBA\", (45, 45), (0, 0, 0, 0))
canvas.paste(im, ((45 - im.width) // 2, (45 - im.height) // 2))
for name in os.listdir(sys.argv[2]):
    canvas.save(os.path.join(sys.argv[3], name))
" /ws/turtle.png /opt/ros/kilted/share/turtlesim/images $IMG_PREFIX/share/turtlesim/images
    fi
    AMENT_PREFIX_PATH=${IMG_PREFIX:+$IMG_PREFIX:}$AMENT_PREFIX_PATH /opt/ros/kilted/lib/turtlesim/turtlesim_node \
      --ros-args -p holonomic:=true >/dev/null 2>&1 &
    sleep 2
    if [ "$*" = bash ]; then
      echo "turtlesim running. Add  --ros-args -r cmd_vel:=/turtle1/cmd_vel -r turtle_pose:=/turtle1/pose  to your command."
      bash
    else
      "$@" --ros-args -r cmd_vel:=/turtle1/cmd_vel -r turtle_pose:=/turtle1/pose
    fi
    kill %1'
fi

# DETACH=<name> ./ros.sh ...  runs it in the background as container <name> (used by start.sh for the web server)
# IMAGE=<image> ./ros.sh ...   other image (start.sh: edurob-slam for the web server, it has the slam_toolbox services)
exec docker run --rm $(if [ -n "$DETACH" ]; then echo -d --name "$DETACH"; elif [ -t 0 ]; then echo -it; fi) $DOCKER_ARGS --net=host -u "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_DOMAIN_ID=$DOMAIN \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 -e PYTHONPATH=/ws "${EXTRA[@]}" \
  -v "$PWD":/ws -w /ws "${IMAGE:-osrf/ros:kilted-desktop}" \
  bash -c "$INNER" _ "${@:-bash}"
