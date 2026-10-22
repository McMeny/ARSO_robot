"""M6: the WebXR app in desktop-preview mode against the running sim stack (Playwright, headless).

Loads the page from the real HTTPS web server, connects to the real rosbridge, renders the markers
from /markers, reflects /robot/status, gates on consent, and drives the simulated robot by aiming.
"""
import pytest

from harness import headset_client, ros_available, ros_cmd, sim_stack, wait_state

pytestmark = pytest.mark.skipif(not ros_available(), reason="ROS 2 workspace not available")


def test_preview_shows_markers_state_and_drives_robot():
    with sim_stack("field", "field_robot", "mujoco", require_headset=True):
        with headset_client() as (page, logs):
            st = wait_state(page, "s.connected && s.markers.length >= 6", 120)
            assert {m["label"] for m in st["markers"]} >= {"A", "B", "C", "D", "E", "CANCEL"}
            assert st["tableHash"] == "fc5c3bb54361"
            # nothing flickers / no gaze before consent; health keeps motion locked (headset gate)
            st = wait_state(page, "s.health && s.status", 90)
            assert not st["consent"]
            page.evaluate("window.eyesteer.consent()")
            # the headset gate turns green -> motion unlocks; robot state is live in the page
            st = wait_state(page, "s.health && s.health.motion_unlocked && s.status && s.status.display === 'WANDER'", 120)
            assert "SIMULATED" in page.inner_text("#eeg-badge")
            assert page.inner_text("#st-display") == "WANDER"
            # aim at marker C (synthetic gaze) -> SSVEP decoded -> ENGAGED -> C
            page.evaluate("window.eyesteer.aim(3)")
            st = wait_state(page, "s.status && s.status.state === 'ENGAGED' && s.status.goal_label === 'C'", 30)
            page.evaluate("window.eyesteer.aim(0)")
            assert page.inner_text("#st-display").startswith("ENGAGED")
            sels = page.evaluate("window.eyesteer.state().selections")
            assert sels and sels[0]["label"] == "C"
            page.screenshot(path="outputs/stills/webxr_preview_engaged.png")


def test_refresh_mismatch_blocks_run_and_reddens_gate():
    with sim_stack("field", "field_robot", "mujoco", require_headset=True, log_name="it_refresh.log"):
        with headset_client(emulate_refresh=72) as (page, logs):
            wait_state(page, "s.connected && s.markers.length >= 6", 120)
            page.evaluate("window.eyesteer.consent()")
            st = wait_state(page, "s.blocked && s.health && s.health.components.find(c => c.name === 'headset' && c.level === 2 "
                                  "&& c.message.includes('refresh'))", 60)
            assert not st["health"]["motion_unlocked"]
            assert page.is_visible("#warning")
