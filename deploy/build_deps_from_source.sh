#!/usr/bin/env bash
# Build the ROS 2 dependencies that are not always available as binaries, WITHOUT sudo, into an
# underlay workspace (default ~/eyesteer_deps_ws) + libserial into ~/.local.
#
#   twist_mux        (ros-teleop/twist_mux)
#   rosbridge_suite  (RobotWebTools/rosbridge_suite)   + pip: tornado, pymongo (bson)
#   diffdrive_arduino (joshnewans/diffdrive_arduino, branch humble) + libserial (no apt package needed)
#
# jetson_setup.sh prefers apt (ros-$DISTRO-twist-mux, ros-$DISTRO-rosbridge-suite, libserial-dev) and
# only uses this script for diffdrive_arduino. On a dev machine without sudo (e.g. WSL) it builds all.
# Usage: deploy/build_deps_from_source.sh [--all|--diffdrive-only] [--ws DIR]
set -euo pipefail

MODE="all"
WS="${HOME}/eyesteer_deps_ws"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --all) MODE="all" ;;
    --diffdrive-only) MODE="diffdrive" ;;
    --ws) WS="$2"; shift ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
  shift
done

DISTRO="${ROS_DISTRO:-humble}"
set +u   # ROS setup scripts reference unset variables
# shellcheck disable=SC1090
source "/opt/ros/${DISTRO}/setup.bash"
set -u
PREFIX="${HOME}/.local"
mkdir -p "${WS}/src" "${PREFIX}"

clone() {  # url branch dir
  if [[ ! -d "${WS}/src/$3" ]]; then
    git clone --depth 1 -b "$2" "$1" "${WS}/src/$3"
  fi
}

# ---------------------------------------------------------------- libserial (header + lib in ~/.local)
if ! pkg-config --exists libserial && [[ ! -f "${PREFIX}/lib/libserial.so" ]]; then
  echo "[deps] building libserial -> ${PREFIX}"
  tmp="$(mktemp -d)"
  git clone --depth 1 https://github.com/crayzeewulf/libserial.git "${tmp}/libserial"
  cmake -S "${tmp}/libserial" -B "${tmp}/build" -DCMAKE_INSTALL_PREFIX="${PREFIX}" -DCMAKE_BUILD_TYPE=Release \
        -DLIBSERIAL_ENABLE_TESTING=OFF -DLIBSERIAL_BUILD_EXAMPLES=OFF -DLIBSERIAL_PYTHON_ENABLE=OFF \
        -DLIBSERIAL_BUILD_DOCS=OFF -DINSTALL_STATIC=OFF > /dev/null
  cmake --build "${tmp}/build" -j"$(nproc)" > /dev/null
  cmake --install "${tmp}/build" > /dev/null
  rm -rf "${tmp}"
fi

# ---------------------------------------------------------------- sources
clone https://github.com/joshnewans/diffdrive_arduino.git humble diffdrive_arduino
if [[ "${MODE}" == "all" ]]; then
  clone https://github.com/ros-teleop/twist_mux.git "${DISTRO}" twist_mux || clone https://github.com/ros-teleop/twist_mux.git rolling twist_mux
  clone https://github.com/RobotWebTools/rosbridge_suite.git "${DISTRO}" rosbridge_suite
  python3 -m pip install --user -q "tornado>=6" pymongo
fi

# diffdrive_arduino links libserial via pkg-config / find_library: point both at ~/.local
export PKG_CONFIG_PATH="${PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export CMAKE_PREFIX_PATH="${PREFIX}:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export CPATH="${PREFIX}/include:${CPATH:-}"
export LIBRARY_PATH="${PREFIX}/lib:${LIBRARY_PATH:-}"

cd "${WS}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  2>&1 | tail -15
echo "[deps] done. Source with:  source ${WS}/install/setup.bash && export LD_LIBRARY_PATH=${PREFIX}/lib:\$LD_LIBRARY_PATH"
