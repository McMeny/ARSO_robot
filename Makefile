# eyesteer top-level targets. Python: $(PY); ROS commands need `source deploy/env.sh` (done for you here).
# On Windows use WSL / Git Bash with make; the ROS targets run inside the ROS environment.
SHELL := /bin/bash
PY ?= python3
ROS := source deploy/env.sh &&
JETSON ?= user@eyesteer.local
REMOTE_DIR ?= ~/eyesteer
world ?= supermarket
robot ?= walker
gaze ?= headset
eeg ?= synthetic
backend ?= mujoco

.PHONY: install generate test test-core test-firmware test-web test-ros test-integration render-all sim firmware flash \
        jetson-setup deploy preflight demo-sim demo-real decals certs shellcheck clean help

help:
	@grep -E '^[a-z-]+:.*?## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | column -t -s $$'\t'

install: ## Python deps for the core + standalone sim (+ PlatformIO, Playwright)
	$(PY) -m pip install -r requirements.txt platformio playwright shellcheck-py
	$(PY) -m playwright install chromium || true

generate: ## config -> frequency table, URDF/xacro/MJCF, controllers, Nav2 params, firmware header, worlds, maps, decals
	$(PY) tools/compute_frequencies.py
	$(PY) tools/generate_models.py
	$(PY) tools/generate_world.py
	$(PY) tools/make_decals.py

test: test-core test-firmware test-web ## everything that runs without ROS (ROS: make test-ros / test-integration)

test-core: ## pytest: models, frequencies, BCI, FSM, control, S1-S6 standalone
	$(PY) -m pytest -q tests --ignore=tests/integration

test-firmware: ## host unit tests of the firmware core (g++) + PlatformIO builds
	mkdir -p firmware/build && g++ -std=c++11 -Wall -Wextra -O1 -o firmware/build/test_core firmware/test/test_core.cpp firmware/lib/eyesteer_fw/eyesteer_fw.cpp && firmware/build/test_core

test-web: ## node unit tests of the headset app
	cd webxr && node --test tests/

test-ros: ## ROS launch/scenario tests in the sim stack (needs ROS 2 + built workspace)
	$(ROS) bash tests/ros/run_all.sh mujoco

test-integration: ## full-system golden path (HIL) x5 per robot + failure injection + WebXR (Playwright)
	$(PY) -m pytest -q tests/integration -x

render-all: ## render the six presentation videos + stills (Runtime C)
	for s in field_basic field_queue field_no_control field_overrides supermarket_walker supermarket_queue; do \
	  $(PY) standalone/app/run_scenario.py $$s & done; wait
	$(PY) tools/metrics_summary.py

sim: ## ROS sim: make sim world=supermarket robot=walker gaze=headset eeg=synthetic backend=mujoco
	$(ROS) ros2 launch eyesteer_bringup sim.launch.py world:=$(world) robot:=$(robot) eeg:=$(eeg) gaze:=$(gaze) backend:=$(backend) rviz:=true viewer:=true

demo-sim: ## gated bring-up of the sim (HIL) system; motion stays locked until all health gates are green
	$(ROS) $(PY) tools/demo.py sim --world $(world) --robot $(robot) --eeg $(eeg)

demo-real: ## gated bring-up on the Jetson (real robot)
	$(ROS) $(PY) tools/demo.py real --robot $(robot) --eeg lsl

firmware: ## build the firmware for every PlatformIO env
	cd firmware && $(PY) -m platformio run

flash: ## upload to the MCU on /dev/eyesteer_mcu (board + walker flag from config/hardware.yaml)
	$(PY) tools/flash.py

jetson-setup: ## run ON the Jetson once
	bash deploy/jetson_setup.sh

deploy: ## rsync to the Jetson, set up if needed, build: make deploy JETSON=user@host
	rsync -az --delete --exclude '.git' --exclude 'ros2_ws/build' --exclude 'ros2_ws/install' --exclude 'ros2_ws/log' \
	  --exclude 'firmware/.pio' --exclude 'outputs/videos' ./ $(JETSON):$(REMOTE_DIR)/
	ssh $(JETSON) 'cd $(REMOTE_DIR) && (command -v ros2 >/dev/null || bash deploy/jetson_setup.sh) && bash -c "source deploy/env.sh && cd ros2_ws && colcon build --symlink-install"'

preflight: ## real-hardware checks before the robot moves (run on the Jetson)
	bash deploy/preflight.sh

decals: ## calibration decal PDF
	$(PY) tools/make_decals.py

certs: ## self-signed TLS cert for the WebXR app + rosbridge
	bash deploy/certs/make_cert.sh

shellcheck: ## lint every shell script
	shellcheck deploy/*.sh deploy/certs/*.sh tests/ros/*.sh

clean: ## remove build products and generated outputs (keeps committed videos/maps)
	rm -rf ros2_ws/build ros2_ws/install ros2_ws/log firmware/.pio firmware/build .pytest_cache outputs/logs outputs/frames
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
