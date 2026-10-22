"""M2: every FSM transition, queue dedupe/overflow, grace window, cancel, estop latch, link/localization loss."""
from eyesteer_core.fsm.mode_manager import ARRIVED, ENGAGED, ESTOP, HOLD, IDLE, LOST, PAUSED, ModeManager
from eyesteer_core.fsm.queue import GoalQueue

CANCEL = 6


def mm(idle="WANDER"):
    m = ModeManager(idle_mode=idle)
    m.set_markers([1, 2, 3, 4, 5, 6], [CANCEL])
    return m


def test_queue_dedupe_overflow():
    q = GoalQueue(3)
    assert q.push(1) == "queued" and q.push(1) == "duplicate"
    assert q.push(2) == "queued" and q.push(3) == "queued" and q.push(4) == "full"
    assert q.push(5, current=5) == "current"
    assert q.items() == [1, 2, 3] and q.pop() == 1


def test_idle_to_engaged_to_arrived_to_idle():
    m = mm()
    assert m.on_selection(3, 0.0) == [("send_goal", 3)] and m.state == ENGAGED
    m.on_nav_succeeded(10.0)
    assert m.state == ARRIVED
    assert m.tick(11.0) == []
    out = m.tick(12.01)
    assert out == [("idle", "WANDER")] and m.state == IDLE and m.goal is None


def test_queue_during_navigation_in_order():
    m = mm()
    m.on_selection(1, 0)
    m.on_selection(4, 1)
    m.on_selection(4, 2)   # duplicate
    m.on_selection(1, 3)   # current goal
    assert m.status().queue == [4]
    m.on_nav_succeeded(10)
    assert m.tick(11.0) == []
    assert m.tick(11.6) == [("send_goal", 4)] and m.goal == 4
    m.on_nav_succeeded(20)
    assert m.tick(22.1) == [("idle", "WANDER")]


def test_grace_window_accepts_engagement():
    m = mm()
    m.on_selection(2, 0)
    m.on_nav_succeeded(5)
    m.on_selection(5, 6.2)              # within the 2 s grace window
    assert m.tick(6.6) == [("send_goal", 5)]


def test_selection_after_grace_from_idle():
    m = mm()
    m.on_selection(2, 0)
    m.on_nav_succeeded(5)
    m.tick(7.1)
    assert m.state == IDLE
    assert m.on_selection(5, 8) == [("send_goal", 5)]


def test_cancel_clears_goal_and_queue():
    m = mm()
    m.on_selection(1, 0)
    m.on_selection(2, 1)
    out = m.on_selection(CANCEL, 2)
    assert ("cancel_goal",) in out and ("idle", "WANDER") in out
    assert m.state == IDLE and m.status().queue == [] and m.goal is None


def test_estop_latched_until_reset():
    m = mm()
    m.on_selection(1, 0)
    out = m.on_estop(1)
    assert m.state == ESTOP and ("stop",) in out
    assert m.on_selection(2, 2) == []
    m.on_estop(3, False)
    assert m.state == ESTOP
    m.reset(4)
    assert m.state == IDLE


def test_nav_failure_retries_then_idle():
    m = mm()
    m.on_selection(3, 0)
    for k in range(3):
        assert m.on_nav_failed(k + 1) == [("send_goal", 3)]
    out = m.on_nav_failed(5)
    assert m.state == IDLE and "failed" in m.reason and out == [("idle", "WANDER")]


def test_walker_idle_is_assist_and_pauses_on_link_loss():
    m = mm("ASSIST")
    m.on_selection(1, 0)
    assert m.on_link(1, "bci", False) == [("cancel_goal",)] and m.state == PAUSED
    m.on_selection(2, 1.5)
    assert m.status().queue == [2]
    assert m.on_link(2, "bci", True) == [("send_goal", 1)] and m.state == ENGAGED
    m.on_nav_succeeded(5)
    m.tick(6.6)
    assert m.goal == 2
    m.on_nav_succeeded(9)
    assert m.tick(11.1) == [("idle", "ASSIST")]


def test_field_headset_loss_finishes_goal_then_holds():
    m = mm()
    m.on_selection(1, 0)
    m.on_link(1, "headset", False)
    assert m.state == ENGAGED          # current goal continues
    m.on_nav_succeeded(5)
    assert m.tick(7.1) == [("idle", "HOLD")] and m.state == HOLD
    assert m.on_selection(2, 8) == []  # no new goals while disconnected
    assert m.on_link(9, "headset", True) == [("idle", "WANDER")]


def test_localization_lost_stops_and_blocks_goals():
    m = mm()
    m.on_selection(1, 0)
    out = m.on_localization(1, False)
    assert m.state == LOST and ("stop",) in out
    assert m.on_selection(2, 2) == []
    m.on_localization(3, True)
    assert m.state == IDLE
