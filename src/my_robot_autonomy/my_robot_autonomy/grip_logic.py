"""Grip state from the two handle readings (no ROS here, so it can be unit tested).

  STOP   either hand below light_threshold for longer than release_grace_s
  WALK   both hands at or above light_threshold
  STEER  both hands at or above firm_threshold, held for steer_hold_s - shorter spikes
         (a startle squeeze) are ignored

Each threshold has hysteresis (+-hysteresis) so a reading hovering at a threshold doesn't
flicker between states. With one_handed, the designated hand alone gives all three states.
"""
STOP, WALK, STEER = 0, 1, 2
NAMES = {STOP: 'STOP', WALK: 'WALK', STEER: 'STEER'}


class Hand:
    """Held / squeezed flags for one hand, with hysteresis."""

    def __init__(self):
        self.held = False
        self.firm = False

    def update(self, value, light, firm, h):
        self.held = value >= light + h if not self.held else value >= light - h
        self.firm = value >= firm + h if not self.firm else value >= firm - h


class GripLogic:
    def __init__(self, light_threshold=0.2, firm_threshold=0.6, hysteresis=0.05,
                 release_grace_s=0.2, steer_hold_s=0.5, one_handed=False, hand='right'):
        self.light = light_threshold
        self.firm = firm_threshold
        self.h = hysteresis
        self.grace = release_grace_s
        self.hold = steer_hold_s
        self.one_handed = one_handed
        self.hand = hand
        self.left, self.right = Hand(), Hand()
        self.state = STOP
        self.released_since = None
        self.firm_since = None
        self.need_release = False

    def calibrate(self, relaxed, firm):
        """Thresholds from a recorded relaxed hold and firm squeeze (normalised 0..1).
        light: half way from nothing to the relaxed hold; firm: half way from relaxed to firm."""
        if firm - relaxed < 4 * self.h:
            raise ValueError(f'firm squeeze ({firm:.2f}) is too close to the relaxed hold ({relaxed:.2f})')
        self.light = max(relaxed / 2, 2 * self.h)
        self.firm = (relaxed + firm) / 2

    def update(self, left, right, t):
        """New readings at time t (s). Returns the grip state."""
        self.left.update(left, self.light, self.firm, self.h)
        self.right.update(right, self.light, self.firm, self.h)
        hands = [self.left if self.hand == 'left' else self.right] if self.one_handed else [self.left, self.right]
        held = all(x.held for x in hands)
        firm = all(x.firm for x in hands)
        if self.need_release:                 # after a safety stop: let go, then grip again
            if any(not x.held for x in hands):
                self.need_release = False
            self.state = STOP
            return self.state

        if not held:
            self.firm_since = None
            if self.released_since is None:
                self.released_since = t
            if t - self.released_since > self.grace:   # a brief slip doesn't stop the walker
                self.state = STOP
            return self.state
        self.released_since = None
        if firm:
            if self.firm_since is None:
                self.firm_since = t
            self.state = STEER if t - self.firm_since >= self.hold else WALK
        else:
            self.firm_since = None
            self.state = WALK
        return self.state

    def force_stop(self, latch=False):
        """An outside stop. With latch (room edge, user too far from the handles) it stays
        STOP until the hands are released and gripped again."""
        self.state = STOP
        self.need_release = self.need_release or latch
        self.released_since = None
        self.firm_since = None
