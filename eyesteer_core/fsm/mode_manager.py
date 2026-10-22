"""Mode manager FSM (Sections 1.2, 11, D11, D12). ROS-free; the ROS node and the standalone sim wrap it.

States:
  IDLE      WANDER (field robot) or ASSIST (walker: moves only when pushed)
  ENGAGED   navigating to a selected marker; new selections are queued
  ARRIVED   at the goal: hold `hold_s`; a `grace_s` window still accepts engagement
  PAUSED    walker only: BCI signal or headset link lost mid-goal -> stop and hold, resume on recovery
  HOLD      stationary idle with a reason (e.g. headset link lost on the field robot)
  LOST      localization lost: stopped, no goals accepted until recovered
  ESTOP     latched until reset()

The FSM never outputs velocities (D7). It emits commands for its wrapper:
  ("send_goal", marker_id) ("cancel_goal",) ("idle", "WANDER"|"ASSIST"|"HOLD") ("stop",)
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .queue import GoalQueue

IDLE, ENGAGED, ARRIVED, PAUSED, HOLD, LOST, ESTOP = "IDLE", "ENGAGED", "ARRIVED", "PAUSED", "HOLD", "LOST", "ESTOP"


@dataclass
class FsmStatus:
    state: str
    idle_mode: str
    goal: object
    queue: list
    reason: str
    retries: int
    events: list = field(default_factory=list)

    @property
    def display(self) -> str:
        if self.state == IDLE:
            return self.idle_mode
        if self.state == ENGAGED:
            return f"ENGAGED -> {self.goal}"
        return self.state


class ModeManager:
    def __init__(self, idle_mode: str = "WANDER", hold_s: float = 1.5, grace_s: float = 2.0,
                 max_retries: int = 3, queue_len: int = 3, is_walker: bool | None = None):
        self.idle_mode = idle_mode
        self.is_walker = (idle_mode == "ASSIST") if is_walker is None else is_walker
        self.hold_s, self.grace_s, self.max_retries = hold_s, grace_s, max_retries
        self.queue = GoalQueue(queue_len)
        self.state = IDLE
        self.goal = None
        self.retries = 0
        self.reason = ""
        self.arrived_at = 0.0
        self.paused_reasons: set = set()
        self.link_lost = False
        self.log: list = []
        self.cancel_ids: set = set()
        self.valid_ids: set | None = None

    # ------------------------------------------------------------------ helpers
    def _emit(self, t, *cmd):
        self.log.append((round(t, 3), self.state, cmd))
        return [cmd]

    def _go_idle(self, t, reason="") -> list:
        self.goal, self.retries = None, 0
        self.reason = reason
        if self.link_lost and not self.is_walker:
            self.state = HOLD
            self.reason = reason or "headset link lost"
            return self._emit(t, "idle", "HOLD")
        self.state = IDLE
        return self._emit(t, "idle", self.idle_mode)

    def _start_goal(self, t, marker_id) -> list:
        self.state, self.goal, self.retries, self.reason = ENGAGED, marker_id, 0, ""
        return self._emit(t, "send_goal", marker_id)

    def set_markers(self, ids, cancel_ids):
        self.valid_ids = set(ids)
        self.cancel_ids = set(cancel_ids)

    # ------------------------------------------------------------------ events
    def on_selection(self, marker_id, t: float) -> list:
        if self.state in (ESTOP, LOST):
            return []
        if self.valid_ids is not None and marker_id not in self.valid_ids:
            return []
        if marker_id in self.cancel_ids:
            self.queue.clear()
            cmds = self._emit(t, "cancel_goal") if self.state in (ENGAGED, PAUSED) else []
            return cmds + self._go_idle(t, "cancelled by user")
        if self.state in (IDLE, HOLD):
            if self.state == HOLD and self.link_lost and not self.is_walker:
                return []  # no new goals while the headset link is down
            return self._start_goal(t, marker_id)
        if self.state in (ENGAGED, PAUSED):
            res = self.queue.push(marker_id, current=self.goal)
            self.log.append((round(t, 3), self.state, ("queue", marker_id, res)))
            return []
        if self.state == ARRIVED:
            if marker_id == self.goal and t - self.arrived_at < self.grace_s:
                return []
            res = self.queue.push(marker_id, current=None)
            self.log.append((round(t, 3), self.state, ("queue", marker_id, res)))
            return []
        return []

    def on_nav_succeeded(self, t: float) -> list:
        if self.state != ENGAGED:
            return []
        self.state, self.arrived_at = ARRIVED, t
        return self._emit(t, "arrived", self.goal)

    def on_nav_failed(self, t: float, reason: str = "planner failure") -> list:
        if self.state != ENGAGED:
            return []
        self.retries += 1
        if self.retries <= self.max_retries:
            self.log.append((round(t, 3), self.state, ("retry", self.goal, self.retries)))
            return [("send_goal", self.goal)]
        nxt = self.queue.pop()
        if nxt is not None:
            return self._start_goal(t, nxt)
        return self._go_idle(t, f"goal {self.goal} failed after {self.max_retries} retries: {reason}")

    def tick(self, t: float) -> list:
        if self.state == ARRIVED:
            dt = t - self.arrived_at
            if dt >= self.hold_s and self.queue:
                return self._start_goal(t, self.queue.pop())
            if dt >= max(self.hold_s, self.grace_s):
                return self._go_idle(t, f"arrived at {self.goal}")
        return []

    def on_estop(self, t: float, active: bool = True) -> list:
        if active and self.state != ESTOP:
            self.queue.clear()
            self.goal = None
            self.state, self.reason = ESTOP, "e-stop"
            return self._emit(t, "cancel_goal") + [("stop",)]
        return []

    def reset(self, t: float) -> list:
        """Clears a latched ESTOP (operator action)."""
        if self.state == ESTOP:
            self.paused_reasons.clear()
            return self._go_idle(t, "reset after e-stop")
        return []

    def on_localization(self, t: float, ok: bool) -> list:
        if not ok and self.state not in (LOST, ESTOP):
            self.queue.clear()
            had_goal = self.state in (ENGAGED, PAUSED)
            self.goal, self.state, self.reason = None, LOST, "localization lost"
            return (self._emit(t, "cancel_goal") if had_goal else []) + self._emit(t, "stop")
        if ok and self.state == LOST:
            return self._go_idle(t, "localization recovered")
        return []

    def on_link(self, t: float, kind: str, ok: bool) -> list:
        """kind: 'bci' (stream lost / BAD SIGNAL) or 'headset' (rosbridge disconnect).

        Field robot: the current goal continues; after a headset loss it idles in HOLD (no wander).
        Walker: stops and holds (PAUSED) until the link returns, then resumes the same goal.
        """
        if kind == "headset":
            self.link_lost = not ok
        if not ok:
            self.paused_reasons.add(kind)
        else:
            self.paused_reasons.discard(kind)
        if self.is_walker:
            if not ok and self.state == ENGAGED:
                self.state, self.reason = PAUSED, f"{kind} lost"
                return self._emit(t, "cancel_goal")
            if ok and self.state == PAUSED and not self.paused_reasons:
                self.state, self.reason = ENGAGED, ""
                return self._emit(t, "send_goal", self.goal)
            return []
        if ok and self.state == HOLD and not self.link_lost:
            return self._go_idle(t, "headset link restored")
        if not ok and kind == "headset" and self.state == IDLE:
            return self._go_idle(t, "headset link lost")
        return []

    def status(self) -> FsmStatus:
        return FsmStatus(self.state, self.idle_mode, self.goal, self.queue.items(), self.reason, self.retries)
