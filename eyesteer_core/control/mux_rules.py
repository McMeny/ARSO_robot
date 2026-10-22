"""Priority arbitration (D8), mirroring the twist_mux configuration generated from the same table.

Highest first: e-stop / dead-man locks -> zero; handle admittance (walker); teleop; navigation.
"""
from __future__ import annotations

from dataclasses import dataclass

# single source of truth for twist_mux (tools/generate_ros_params.py writes twist_mux.yaml from this)
TOPICS = [
    {"name": "admittance", "topic": "cmd_vel_admittance", "timeout": 0.3, "priority": 100},
    {"name": "teleop", "topic": "cmd_vel_teleop", "timeout": 0.5, "priority": 90},
    {"name": "navigation", "topic": "cmd_vel_nav", "timeout": 0.5, "priority": 10},
]
LOCKS = [
    {"name": "estop", "topic": "lock/estop", "timeout": 0.0, "priority": 255},
    {"name": "deadman", "topic": "lock/deadman", "timeout": 0.0, "priority": 254},
    {"name": "health", "topic": "lock/health", "timeout": 0.0, "priority": 253},
]


@dataclass
class Twist:
    v: float = 0.0
    w: float = 0.0

    def scaled(self, k: float) -> "Twist":
        return Twist(self.v * k, self.w * k)


class Mux:
    """Same semantics as twist_mux: a topic is active if its last message is younger than its timeout;
    the active topic with the highest priority wins; a lock with priority >= that topic blocks it (zero)."""

    def __init__(self, topics=TOPICS, locks=LOCKS):
        self.topics = {t["name"]: dict(t, last=-1e9, twist=Twist()) for t in topics}
        self.locks = {l["name"]: dict(l, active=False) for l in locks}

    def input(self, name: str, twist: Twist, t: float):
        tp = self.topics[name]
        tp["last"], tp["twist"] = t, twist

    def lock(self, name: str, active: bool):
        self.locks[name]["active"] = active

    def output(self, t: float) -> tuple[Twist, str]:
        lock_prio = max([l["priority"] for l in self.locks.values() if l["active"]], default=-1)
        best = None
        for tp in self.topics.values():
            if t - tp["last"] <= tp["timeout"] and (best is None or tp["priority"] > best["priority"]):
                best = tp
        if lock_prio >= 0:
            name = max((l for l in self.locks.values() if l["active"]), key=lambda l: l["priority"])["name"]
            if best is None or lock_prio >= best["priority"]:
                return Twist(), f"LOCK:{name.upper()}"
        if best is None:
            return Twist(), "NONE"
        return best["twist"], best["name"].upper()


class CmdVelWatchdog:
    """D9 (ROS side): zero the output if the last cmd_vel is older than `timeout`."""

    def __init__(self, timeout: float = 0.5):
        self.timeout = timeout
        self.last_t = -1e9
        self.last = Twist()

    def feed(self, twist: Twist, t: float):
        self.last, self.last_t = twist, t

    def output(self, t: float) -> tuple[Twist, bool]:
        stale = t - self.last_t > self.timeout
        return (Twist() if stale else self.last), stale


class DeadMan:
    """Walker grip: released for longer than `release_timeout` -> lock (zero)."""

    def __init__(self, release_timeout: float = 0.3):
        self.timeout = release_timeout
        self.released_since = None
        self.locked = True

    def update(self, gripped: bool, t: float) -> bool:
        if gripped:
            self.released_since = None
            self.locked = False
        else:
            if self.released_since is None:
                self.released_since = t
            self.locked = (t - self.released_since) >= self.timeout
        return self.locked


def twist_mux_yaml() -> dict:
    return {"twist_mux": {"ros__parameters": {
        "topics": {t["name"]: {"topic": t["topic"], "timeout": t["timeout"], "priority": t["priority"]} for t in TOPICS},
        "locks": {l["name"]: {"topic": l["topic"], "timeout": l["timeout"], "priority": l["priority"]} for l in LOCKS},
    }}}
