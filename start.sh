#!/bin/bash
# Start everything for the real robot, in the background (safe to run twice):
#   micro-ros-agent  the robot connects to it (UDP 8888) -> cmd_vel, /tf odometry, /imu, gyro service
#   rplidar          lidar driver, reads the robot's lidar over TCP -> /scan (lidar_watchdog.sh)
#   edurob-webui     web control (webui.py), reachable from the WLAN
#   edurob-slam      only with --slam: slam_toolbox (mapping) + odom_fix.py, map shown in the web page
#                    (image built once from slam.Dockerfile)
# usage: ./start.sh            web page for everyone in the WLAN
#        ./start.sh --local    web page only on this laptop (http://localhost:PORT)
#        ./start.sh --slam     also build a map while driving (options can be combined)
#        DOMAIN=13 PORT=8081 ./start.sh
cd "$(dirname "$0")" || exit 1
ROBOT_IP=10.0.109.58
DOMAIN=${DOMAIN:-12}  # must match the robot settings page
PORT=${PORT:-8080}
HOST=0.0.0.0
SLAM=0
for arg in "$@"; do
  case "$arg" in
    --local) HOST=127.0.0.1 ;;
    --slam) SLAM=1 ;;
    *) echo "unknown option $arg (--local, --slam)"; exit 1 ;;
  esac
done

# ---------- output ----------
if [ -t 1 ]; then  # colours only in a terminal, not when piped into a file
  G=$'\e[32m' Y=$'\e[33m' R=$'\e[31m' B=$'\e[1m' D=$'\e[2m' C=$'\e[36m' N=$'\e[0m'
else
  G='' Y='' R='' B='' D='' C='' N=''
fi
step() { printf '  %-36s' "$1"; }              # label of a check, result follows on the same line
ok()   { echo "${G}✓ $1${N}"; }
note() { echo "${Y}● $1${N}"; }                # fine, but worth knowing (already running, restarted)
fail() { echo "${R}✗ $1${N}"; }
hint() { echo "      ${D}$1${N}"; }
section() { echo; echo "${B}$1${N}"; }

running() { docker ps -q -f name="^$1\$" | grep -q .; }
# docker stop, then wait until Docker has removed it (--rm), so the name is free for the new one
stop_container() {
  docker stop "$1" >/dev/null 2>&1
  for _ in $(seq 50); do docker ps -aq -f name="^$1\$" | grep -q . || return 0; sleep 0.1; done
}
# ROS (Fast DDS) fixes the laptop's network address when a program starts. If the laptop got a new IP
# since then (WLAN reconnect), the container still announces the old one and gets no data: restart it.
RESTARTED=""
restart_if_ip_changed() {
  RESTARTED=""
  if running "$1" && [ "$(docker inspect "$1" --format '{{index .Config.Labels "laptop_ip"}}')" != "$LAPTOP_IP" ]; then
    stop_container "$1"
    RESTARTED=" (laptop IP changed)"
  fi
}
# started / restarted / already running / failed, for a container start command
start_result() {
  if [ "$1" = 0 ]; then
    if [ -n "$RESTARTED" ]; then note "restarted$RESTARTED"; else ok "started"; fi
  else
    fail "could not start"; hint "$2"; exit 1
  fi
}

echo "${B}EduRob${N} ${D}· ROS domain $DOMAIN · robot $ROBOT_IP${N}"

# ---------- network ----------
section "Network"
step "Robot reachable (ping)"
if ping -c 3 -i 0.3 -W 2 $ROBOT_IP >/dev/null 2>&1; then
  ok "yes"
else
  fail "no answer"
  hint "Robot switched on? Laptop in the same WLAN as the robot?"
  hint "Nothing was started."
  exit 1
fi

LAPTOP_IP=$(ip -4 route get $ROBOT_IP 2>/dev/null | grep -oP 'src \K[\d.]+')
step "Laptop IP"
echo "${C}$LAPTOP_IP${N}"

# The robot connects to the agent at the IP on its settings page: has to be this laptop
step "Robot settings (Agent IP, domain)"
settings=$(curl -s -m 10 --retry 1 "http://$ROBOT_IP/configuration")  # the ESP32 answers slowly
agent_ip=$(grep -oP 'id="AgentIP".*?value="\K[^"]*' <<<"$settings")  # (its pattern attribute contains a >)
robot_domain=$(grep -oP 'id="DomainID".*?value="\K[^"]*' <<<"$settings")
if [ -z "$agent_ip" ]; then
  note "settings page didn't answer, not checked"
elif [ "$agent_ip" != "$LAPTOP_IP" ] || [ "$robot_domain" != "$DOMAIN" ]; then
  fail "Agent IP $agent_ip, domain $robot_domain"
  hint "Needs Agent IP $LAPTOP_IP and DomainID $DOMAIN: http://$ROBOT_IP/configuration, save, reboot the robot."
  hint "Nothing was started."
  exit 1
else
  ok "$agent_ip, domain $robot_domain"
fi

# ---------- services ----------
section "Services"
step "micro-ROS agent (UDP 8888)"
if running micro-ros-agent; then note "already running"; else
  docker run -d --rm --name micro-ros-agent --net=host -e ROS_DOMAIN_ID=$DOMAIN \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e MICROROS_DISABLE_SHM=1 \
    microros/micro-ros-agent:kilted udp4 --port 8888 -v4 >/dev/null
  start_result $? "docker logs micro-ros-agent"
  AGENT_NEW=1
fi

step "Lidar driver (TCP 44227)"
restart_if_ip_changed rplidar
if running rplidar && ! docker inspect rplidar --format '{{.Args}}' | grep -q lidar_watchdog; then
  stop_container rplidar  # old version without watchdog / frame laser_fix
  RESTARTED=" (new version)"
fi
if running rplidar; then note "already running"; else
  # with a watchdog: the driver doesn't reconnect by itself after a robot reboot
  docker run -d --rm --name rplidar --label laptop_ip=$LAPTOP_IP --net=host -e ROS_DOMAIN_ID=$DOMAIN \
    -v "$PWD/lidar_watchdog.sh":/lidar_watchdog.sh:ro \
    husarion/rplidar:jazzy-1.0.1-20250901 bash /lidar_watchdog.sh $ROBOT_IP >/dev/null
  start_result $? "docker logs rplidar"
fi

if [ $SLAM = 1 ]; then
  step "SLAM (slam_toolbox, mapping)"
  restart_if_ip_changed edurob-slam
  if running edurob-slam; then note "already running"; else
    if ! docker image inspect edurob-slam >/dev/null 2>&1; then
      printf '%s' "${D}building image once (~2 min)... ${N}"
      docker build -q -t edurob-slam -f slam.Dockerfile . >/dev/null 2>&1 \
        || { fail "image build failed"; hint "docker build -t edurob-slam -f slam.Dockerfile ."; exit 1; }
    fi
    docker run -d --rm --name edurob-slam --label laptop_ip=$LAPTOP_IP --net=host -e ROS_DOMAIN_ID=$DOMAIN \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
      -v "$PWD":/ws:ro edurob-slam bash -c \
      "source /opt/ros/kilted/setup.bash && (python3 /ws/odom_fix.py &) && \
       exec ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=/ws/slam_params.yaml" \
      >/dev/null
    start_result $? "docker logs edurob-slam"
  fi
fi

# The robot's message definitions (src/edurob_interfaces, needed for the gyro calibration) are built
# once into install/ (not in git). The web server and the scripts it starts load them when it starts.
step "Robot messages (edurob_interfaces)"
BUILT=0
if [ -f install/setup.bash ] && [ -f install/edurob_interfaces/share/edurob_interfaces/package.bash ]; then
  ok "built"
else
  printf '%s' "${D}building once (~1 min)... ${N}"
  if ./ros.sh colcon build --base-paths src --packages-select edurob_interfaces </dev/null >/dev/null 2>&1; then
    ok "built"
    BUILT=1
  else
    fail "build failed"
    hint "./ros.sh colcon build --base-paths src --packages-select edurob_interfaces"
    hint "Everything else works without it, only the gyro calibration doesn't."
  fi
fi

step "Web control (port $PORT)"
restart_if_ip_changed edurob-webui
if [ $BUILT = 1 ] && running edurob-webui; then
  stop_container edurob-webui  # started without the robot messages: restart so it loads them
  RESTARTED=" (to load the robot messages)"
fi
if running edurob-webui; then note "already running"; else
  # the slam image = the normal one + slam_toolbox, so the page can reset the map ("New map")
  image=osrf/ros:kilted-desktop
  docker image inspect edurob-slam >/dev/null 2>&1 && image=edurob-slam
  DETACH=edurob-webui IMAGE=$image DOCKER_ARGS="--label laptop_ip=$LAPTOP_IP" DOMAIN=$DOMAIN \
    ./ros.sh python3 webui.py --host $HOST --port $PORT >/dev/null
  start_result $? "docker logs edurob-webui"
fi

# ---------- robot data (asked from the web server) ----------
section "Robot data"
# the web server keeps the last scan forever: only scans from the last 2 s count
scan_fresh() { case "$state" in *'"scan_hz": 0.0'*|*'"scan": null'*) return 1 ;; *'"scan": {'*) return 0 ;; *) return 1 ;; esac; }
step "Waiting for odometry and lidar"
state=""
for _ in $(seq 30); do
  if ! running edurob-webui; then
    fail "web control stopped right away"
    docker logs edurob-webui 2>&1 | tail -5 | sed 's/^/      /'
    exit 1
  fi
  state=$(curl -s -m 1 "http://localhost:$PORT/state" 2>/dev/null)
  case "$state" in *'"source": "robot"'*) scan_fresh && break ;; esac
  printf '%s' "${D}.${N}"
  sleep 1
done
echo
step "Odometry"
case "$state" in
  *'"source": "robot"'*) ok "ok" ;;
  *) fail "none"
     if [ -n "$AGENT_NEW" ]; then hint "The agent was just started: reboot the robot so it connects (http://$ROBOT_IP/ → Reboot ESP32)."
     else hint "Reboot the robot (http://$ROBOT_IP/ → Reboot ESP32), then run ./start.sh again."; fi ;;
esac
step "Lidar"
if scan_fresh; then ok "ok"
else fail "none"; hint "Comes back by itself ~20 s after a robot reboot. Otherwise: lidar powered? docker logs rplidar"; fi
if [ $SLAM = 1 ]; then
  step "Map"
  case "$state" in
    *'"frame": "map"'*) ok "building" ;;
    *) note "not yet"; hint "Appears a few seconds after odometry and lidar arrive." ;;
  esac
fi

# ---------- how to use ----------
echo
if [ "$HOST" = 127.0.0.1 ]; then
  echo "${B}Web control${N}  ${C}http://localhost:$PORT${N}"
else
  echo "${B}Web control${N}  ${C}http://$LAPTOP_IP:$PORT${N}  ${D}(this laptop: http://localhost:$PORT)${N}"
  echo "             ${Y}Anyone in this WLAN can open it and drive.${N} ${D}Only this laptop: ./start.sh --local${N}"
fi
echo "${D}Logs         docker logs -f edurob-webui | edurob-slam | micro-ros-agent | rplidar${N}"
echo "${D}Stop all     ./stop.sh${N}"
