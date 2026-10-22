#!/usr/bin/env bash
# Source this to get the full eyesteer ROS environment:  source deploy/env.sh
# ROS 2 distro + optional source-built deps underlay (~/eyesteer_deps_ws) + this workspace.
_ES_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export EYESTEER_ROOT="${_ES_ROOT}"
_ES_DISTRO="${ROS_DISTRO:-}"
if [[ -z "${_ES_DISTRO}" ]]; then
  for d in jazzy humble; do [[ -f "/opt/ros/${d}/setup.bash" ]] && _ES_DISTRO="${d}" && break; done
fi
set +u
# shellcheck disable=SC1090
source "/opt/ros/${_ES_DISTRO}/setup.bash"
if [[ -f "${HOME}/eyesteer_deps_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/eyesteer_deps_ws/install/setup.bash"
fi
export LD_LIBRARY_PATH="${HOME}/.local/lib:${LD_LIBRARY_PATH:-}"
if [[ -f "${_ES_ROOT}/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_ES_ROOT}/ros2_ws/install/setup.bash"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONPATH="${_ES_ROOT}:${PYTHONPATH:-}"
