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


def test_desktop_demo_without_headset_or_muse():
    """demo:=true: no headset, no Muse. The browser's 2D map is the headset: keys / clicks = gaze, SIMULATED EEG
    through the real decoder, nothing flickers on screen, no consent needed, MR disabled."""
    with sim_stack("field", "field_robot", "mujoco", require_headset=True, extra="demo:=true", log_name="it_demo.log"):
        with headset_client() as (page, logs):
            st = wait_state(page, "s.connected && s.markers.length >= 6 && s.demo", 120)
            assert page.is_visible("#demo-banner") and not page.is_visible("#consent")
            assert page.is_disabled("#enter-xr")
            assert "SIMULATED" in page.inner_text("#demo-banner") and "SIMULATED" in page.inner_text("#eeg-badge")
            # require_headset is forced off: motion unlocks without a headset and without consent
            wait_state(page, "s.health && s.health.motion_unlocked && s.status && s.status.display === 'WANDER'", 120)
            assert not page.evaluate("window.eyesteer.state().consent")
            # no on-screen flicker: the marker's pixel stays constant over many frames
            u, v = page.evaluate("window.eyesteer.markerPx(3)")
            px = set()
            for _ in range(20):
                px.add(tuple(page.evaluate(f"Array.from(document.getElementById('preview').getContext('2d')"
                                           f".getImageData({int(u)}, {int(v)}, 1, 1).data)")))
                page.wait_for_timeout(37)
            assert len(px) == 1, f"marker pixel changed between frames (flicker): {px}"
            # click a marker = look at it
            box = page.locator("#preview").bounding_box()
            w = page.evaluate("document.getElementById('preview').width")
            ua, va = page.evaluate("window.eyesteer.markerPx(1)")
            page.mouse.click(box["x"] + ua * box["width"] / w, box["y"] + va * box["width"] / w)
            assert page.evaluate("window.eyesteer.state().aimed") == 1
            page.keyboard.press("Escape")
            assert page.evaluate("window.eyesteer.state().aimed") == 0
            # key 3 = look at C -> SIMULATED SSVEP -> decoder selection -> ENGAGED C; then the demo user looks away
            page.keyboard.press("3")
            wait_state(page, "s.status && s.status.state === 'ENGAGED' && s.status.goal_label === 'C'", 30)
            wait_state(page, "s.aimed === 0", 5)
            page.screenshot(path="outputs/stills/desktop_demo_engaged.png")
            # Space = e-stop
            page.keyboard.press(" ")
            wait_state(page, "s.status && s.status.state === 'ESTOP'", 10)
            assert not [l for l in logs if "PAGE ERROR" in l], logs
