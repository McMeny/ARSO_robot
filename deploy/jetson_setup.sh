#!/usr/bin/env bash
# One-time (idempotent) setup of a freshly flashed Jetson Orin for eyesteer.
#   deploy/jetson_setup.sh            # install + build
#   deploy/jetson_setup.sh --dry-run  # print what would be done (used to test this script in a container)
# Detects Ubuntu -> ROS 2 distro (22.04 -> Humble, 24.04 -> Jazzy), installs ROS 2 + Nav2 + SLAM Toolbox
# + twist_mux + ros2_control + rosbridge + LiDAR driver + diffdrive_arduino, Python deps, PlatformIO,
# udev rules, certificates, systemd units (disabled), then builds the workspace.
set -euo pipefail

DRY=0
[[ "${1:-}" == "--dry-run" ]] && DRY=1
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run() { if [[ ${DRY} -eq 1 ]]; then echo "+ $*"; else echo "+ $*"; eval "$*"; fi; }
SUDO="sudo"
[[ $(id -u) -eq 0 ]] && SUDO=""

# ---------------------------------------------------------------- platform
# shellcheck source=/dev/null
. /etc/os-release
case "${VERSION_ID}" in
  22.04) DISTRO=humble ;;
  24.04) DISTRO=jazzy ;;
  *) echo "Unsupported Ubuntu ${VERSION_ID}: JetPack 6 (Orin) ships 22.04 -> Humble."; exit 1 ;;
esac
if [[ -f /etc/nv_tegra_release ]]; then
  echo "[setup] Jetson: $(head -1 /etc/nv_tegra_release)"
else
  echo "[setup] not a Jetson (fine for a laptop / container)"
fi
echo "[setup] Ubuntu ${VERSION_ID} -> ROS 2 ${DISTRO}"

# ---------------------------------------------------------------- ROS 2 apt repository
if [[ ! -f /etc/apt/sources.list.d/ros2.list && ! -f /etc/apt/sources.list.d/ros2.sources ]]; then
  run "${SUDO} apt-get update"
  run "${SUDO} apt-get install -y curl gnupg lsb-release software-properties-common"
  run "${SUDO} add-apt-repository -y universe"
  run "${SUDO} curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
  run "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main\" | ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null"
fi

PKGS=(
  "ros-${DISTRO}-ros-base" "ros-${DISTRO}-navigation2" "ros-${DISTRO}-nav2-bringup" "ros-${DISTRO}-slam-toolbox"
  "ros-${DISTRO}-twist-mux" "ros-${DISTRO}-ros2-control" "ros-${DISTRO}-ros2-controllers" "ros-${DISTRO}-rosbridge-suite"
  "ros-${DISTRO}-xacro" "ros-${DISTRO}-robot-state-publisher" "ros-${DISTRO}-joint-state-publisher" "ros-${DISTRO}-rmw-cyclonedds-cpp"
  "ros-${DISTRO}-teleop-twist-keyboard" "ros-${DISTRO}-tf2-tools" "ros-${DISTRO}-rviz2"
  libserial-dev python3-pip python3-colcon-common-extensions python3-rosdep python3-serial python3-yaml python3-scipy
  git openssl bluez shellcheck
)
run "${SUDO} apt-get update"
run "${SUDO} apt-get install -y ${PKGS[*]}"

# ---------------------------------------------------------------- source-built deps: diffdrive_arduino + LiDAR driver
DEPS_WS="${HOME}/eyesteer_deps_ws"
run "mkdir -p ${DEPS_WS}/src"
if [[ ! -d "${DEPS_WS}/src/sllidar_ros2" ]]; then
  run "git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git ${DEPS_WS}/src/sllidar_ros2"
fi
run "ROS_DISTRO=${DISTRO} bash ${ROOT}/deploy/build_deps_from_source.sh --diffdrive-only --ws ${DEPS_WS}"

# ---------------------------------------------------------------- Python deps
# Humble's scipy (1.8) needs numpy < 1.25; MuJoCo is only needed for on-device simulation.
PIP_PKGS="'numpy<1.25' pylsl muselsl pyserial platformio cbor2 'mujoco<3.12' pyyaml"
run "python3 -m pip install --user ${PIP_PKGS}"

# ---------------------------------------------------------------- environment
BASHRC="${HOME}/.bashrc"
add_line() { grep -qxF "$1" "${BASHRC}" 2>/dev/null || run "echo '$1' >> ${BASHRC}"; }
add_line "source ${ROOT}/deploy/env.sh"
add_line "export ROS_DOMAIN_ID=$(python3 -c "import yaml;print(yaml.safe_load(open('${ROOT}/config/hardware.yaml'))['compute']['ros_domain_id'])" 2>/dev/null || echo 42)"
add_line "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # if Wi-Fi discovery is flaky: rmw_zenoh_cpp (Jazzy) or a Cyclone peers list"

# ---------------------------------------------------------------- udev + groups
run "python3 ${ROOT}/deploy/udev/make_rules.py > /tmp/99-eyesteer.rules"
run "${SUDO} cp /tmp/99-eyesteer.rules /etc/udev/rules.d/99-eyesteer.rules"
run "${SUDO} udevadm control --reload-rules && ${SUDO} udevadm trigger"
run "${SUDO} usermod -aG dialout ${USER}"

# ---------------------------------------------------------------- certificates + systemd (installed, NOT enabled)
[[ -f "${ROOT}/deploy/certs/eyesteer.pem" ]] || run "bash ${ROOT}/deploy/certs/make_cert.sh $(hostname)"
for unit in eyesteer-web eyesteer-rosbridge eyesteer-robot; do
  run "sed -e 's#@ROOT@#${ROOT}#g' -e 's#@USER@#${USER}#g' ${ROOT}/deploy/systemd/${unit}.service | ${SUDO} tee /etc/systemd/system/${unit}.service >/dev/null"
done
run "${SUDO} systemctl daemon-reload"
echo "[setup] systemd units installed but disabled. Start for a demo with:  sudo systemctl start eyesteer-robot"

# ---------------------------------------------------------------- generate + build
run "cd ${ROOT} && python3 tools/compute_frequencies.py && python3 tools/generate_models.py && python3 tools/generate_world.py"
run "bash -c 'set +u; source /opt/ros/${DISTRO}/setup.bash; source ${DEPS_WS}/install/setup.bash; cd ${ROOT}/ros2_ws && colcon build --symlink-install'"
echo "[setup] done. Log out/in once (dialout group), then run: make preflight"
