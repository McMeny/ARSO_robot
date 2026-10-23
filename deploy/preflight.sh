#!/usr/bin/env bash
# Run on the Jetson BEFORE the robot moves (Section 17 + 24.6). Prints a green/red summary.
#   deploy/preflight.sh                 # everything (prompts where a human is needed)
#   deploy/preflight.sh --skip-eeg      # robot only
#   deploy/preflight.sh --only mcu      # one check: ros mcu lidar tf web eeg alpha ssvep sim
#   deploy/preflight.sh --emulate       # rehearse against the MCU + Muse emulators (no hardware)
# Checks needing the running stack (tf, web) expect `ros2 launch eyesteer_bringup real.launch.py` in another terminal.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}" || exit 1
# shellcheck disable=SC1091
source deploy/env.sh >/dev/null 2>&1 || true

SKIP_EEG=0; ONLY=""; EMULATE=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-eeg) SKIP_EEG=1 ;;
    --only) ONLY="$2"; shift ;;
    --emulate) EMULATE=1 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done

GREEN=$'\e[32m'; RED=$'\e[31m'; YEL=$'\e[33m'; OFF=$'\e[0m'
declare -a RESULTS=()
record() { RESULTS+=("$1|$2|$3"); if [[ "$2" == PASS ]]; then echo "${GREEN}PASS${OFF} $1: $3"; elif [[ "$2" == SKIP ]]; then echo "${YEL}SKIP${OFF} $1: $3"; else echo "${RED}FAIL${OFF} $1: $3"; fi; }
want() { [[ -z "${ONLY}" || "${ONLY}" == "$1" ]]; }
hwval() { python3 -c "import yaml;d=yaml.safe_load(open('config/hardware.yaml'));print(eval('d$1'))"; }

MCU_DEV="$(hwval "['mcu']['device']")"
EMU_PID=""
if [[ ${EMULATE} -eq 1 ]]; then
  python3 firmware/emulator/mcu_emulator.py --robot field_robot --link /tmp/eyesteer_mcu_preflight >/tmp/mcu_emu.log 2>&1 &
  EMU_PID=$!
  MCU_DEV=/tmp/eyesteer_mcu_preflight
  sleep 2
fi

# 1. ROS 2 install + workspace build
if want ros; then
  if command -v ros2 >/dev/null && ros2 pkg prefix eyesteer_bringup >/dev/null 2>&1; then
    record "ros" PASS "ROS 2 ${ROS_DISTRO:-?} + workspace built"
  else
    record "ros" FAIL "ros2 or the eyesteer workspace missing: run deploy/jetson_setup.sh"
  fi
fi

# 2. MCU protocol, encoders (by hand), watchdog (wheels off the ground)
if want mcu; then
  if [[ -e "${MCU_DEV}" ]]; then
    out=$(python3 - "${MCU_DEV}" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 57600, timeout=1); time.sleep(2.0); s.reset_input_buffer()
def c(x): s.write((x + "\r").encode()); return s.readline().decode().strip()
v = c("v"); e = c("e"); st = c("s")
print(f"version='{v}' encoders='{e}' status='{st}'")
ok = v.startswith("V eyesteer") and len(e.split()) == 2 and st.startswith("S ")
# watchdog: command, stop sending, verify it trips within 200 ms
c("o 60 60"); time.sleep(0.45); st2 = c("s")
print(f"after 450 ms without commands: '{st2}'")
ok = ok and st2.split()[3] == "1"
print("OK" if ok else "BAD")
PY
)
    while IFS= read -r l; do echo "    ${l}"; done <<< "${out}"
    if echo "${out}" | tail -1 | grep -q OK; then record "mcu" PASS "protocol + watchdog (${MCU_DEV})"; else record "mcu" FAIL "MCU did not answer the eyesteer protocol / watchdog did not trip"; fi
    if [[ ${EMULATE} -eq 0 ]]; then
      echo "    Turn each wheel by hand now (5 s)..."
      python3 - "${MCU_DEV}" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 57600, timeout=1); time.sleep(2.0)
def c(x): s.write((x + "\r").encode()); return s.readline().decode().strip()
c("r"); time.sleep(5); print("    encoder counts after turning:", c("e"))
PY
      echo "    Press the e-stop button, then release it (10 s)..."
      out=$(python3 - "${MCU_DEV}" <<'PY'
import sys, time, serial
s = serial.Serial(sys.argv[1], 57600, timeout=1); time.sleep(2.0); s.reset_input_buffer()
def c(x): s.reset_input_buffer(); s.write((x + "\r").encode()); return s.readline().decode().strip()
c("x")
pressed = released_latched = False
t0 = time.time()
while time.time() - t0 < 10:
    f = c("s").split()                      # S <latched> <input> <watchdog> <moving>
    if len(f) == 5:
        pressed = pressed or f[2] == "1"
        if pressed and f[2] == "0":
            released_latched = f[1] == "1"   # latch must survive the release
            break
    time.sleep(0.05)
f = c("x"); cleared = c("s").split()[1:2] == ["0"]
print(f"pressed={pressed} latched_after_release={released_latched} cleared_by_reset={cleared}")
print("OK" if pressed and released_latched and cleared else "BAD")
PY
)
      while IFS= read -r l; do echo "    ${l}"; done <<< "${out}"
      if echo "${out}" | tail -1 | grep -q OK; then record "estop" PASS "button seen, latch held after release, cleared by reset"; else record "estop" FAIL "e-stop input not seen or latch not held (wiring? pins.<board>.estop in hardware.yaml)"; fi
    fi
  else
    record "mcu" FAIL "${MCU_DEV} not found (udev rule? cable? see DEPLOY.md)"
  fi
fi

# 3. LiDAR /scan ~10 Hz in laser_frame (stack running)
if want lidar; then
  hz=$(timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 average | awk '{print $3}')
  frame=$(timeout 5 ros2 topic echo --once /scan --field header.frame_id 2>/dev/null | head -1)
  if [[ -n "${hz}" ]] && python3 -c "import sys; sys.exit(0 if abs(float('${hz}') - 10) <= 2 else 1)" && [[ "${frame}" == "laser_frame" ]]; then
    record "lidar" PASS "/scan ${hz} Hz in ${frame}"
  else
    record "lidar" FAIL "/scan rate='${hz:-none}' frame='${frame:-none}' (is the stack running? /dev/lidar?)"
  fi
fi

# 4. TF tree
if want tf; then
  ok=1
  for pair in "map odom" "odom base_footprint" "base_footprint laser_frame"; do
    # shellcheck disable=SC2086
    timeout 5 ros2 run tf2_ros tf2_echo ${pair} 2>&1 | grep -q Translation || ok=0
  done
  if [[ ${ok} -eq 1 ]]; then record "tf" PASS "map -> odom -> base_footprint -> laser_frame"; else record "tf" FAIL "TF incomplete (localization / drivers running?)"; fi
fi

# 5. rosbridge + HTTPS web app
if want web; then
  if curl -sk --max-time 3 "https://localhost:$(hwval "['headset']['web_port']")/config/frequencies.json" | grep -q refresh_hz; then
    record "web" PASS "WebXR app served over HTTPS"
  else
    record "web" FAIL "web server not reachable on https://localhost:$(hwval "['headset']['web_port']")"
  fi
  if timeout 3 bash -c "</dev/tcp/127.0.0.1/9090" 2>/dev/null; then record "rosbridge" PASS "port 9090 open"; else record "rosbridge" FAIL "rosbridge not listening on 9090"; fi
fi

# 6. EEG (LSL) + real-hardware checks: stream/spectrum, eyes-closed alpha, SSVEP smoke test
if [[ ${SKIP_EEG} -eq 0 ]]; then
  EM=""; [[ ${EMULATE} -eq 1 ]] && EM="--emulate"
  for chk in stream alpha ssvep; do
    want eeg || want "${chk}" || continue
    if python3 tools/eeg_check.py "${chk}" ${EM} | tee "/tmp/eeg_${chk}.log" | sed 's/^/    /'; [[ ${PIPESTATUS[0]} -eq 0 ]]; then
      record "eeg_${chk}" PASS "$(tail -1 /tmp/eeg_${chk}.log)"
    else
      record "eeg_${chk}" FAIL "$(tail -1 /tmp/eeg_${chk}.log)"
    fi
  done
else
  record "eeg" SKIP "skipped (--skip-eeg)"
fi

# 7. the standalone scenario suite with THIS exact config
if want sim; then
  if python3 -m pytest -q tests/test_scenarios_quick.py >/tmp/preflight_sim.log 2>&1; then
    record "sim" PASS "standalone scenarios S1-S6 pass with this config ($(tail -1 /tmp/preflight_sim.log))"
  else
    record "sim" FAIL "standalone scenarios fail with this config: see /tmp/preflight_sim.log"
  fi
fi

[[ -n "${EMU_PID}" ]] && kill "${EMU_PID}" 2>/dev/null
echo
echo "================ PREFLIGHT SUMMARY ================"
FAILS=0
for r in "${RESULTS[@]}"; do
  IFS='|' read -r name res msg <<< "${r}"
  case "${res}" in PASS) c="${GREEN}" ;; SKIP) c="${YEL}" ;; *) c="${RED}"; FAILS=$((FAILS + 1)) ;; esac
  printf "%s%-14s %-5s%s %s\n" "${c}" "${name}" "${res}" "${OFF}" "${msg}"
done
echo "==================================================="
if [[ ${FAILS} -eq 0 ]]; then echo "${GREEN}ALL GREEN: ok to proceed (spotter present, e-stop in reach).${OFF}"; else echo "${RED}${FAILS} check(s) failed: do NOT enable motion.${OFF}"; fi
exit $(( FAILS > 0 ))
