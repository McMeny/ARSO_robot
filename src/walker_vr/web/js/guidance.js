// What the participant should do next, shown in the headset from the moment VR starts. Pure functions of the
// session state, so every case can be tested without a headset (test/test_guidance.mjs).
// Text is two lines ("a\nb"), or '' when nothing needs saying.

const GRIP_HELD = 0.2;        // grip.yaml light_threshold
const STUCK_S = 0.6;          // hands on but still STOP for this long: the stop is latched (grip_logic.py)
const STILL_S = 2.0;          // stopped for this long after the walkthrough: say how to carry on
const FINISH_S = 6.0;         // how long the closing message stays up
const TURN_RAD = 0.5;         // how far the walker must turn to finish the turning step

const STEPS = {
  stationary: [
    'Step 1 of 7: Hold on\nPress and hold both grip buttons',
    'Step 2 of 7: Walker ready\nPush the left stick forward',
    'Step 3 of 7: Stop\nLet go of the left stick',
    'Step 4 of 7: Turn\nRight stick, and turn your body',
    'Step 5 of 7: Head turn\nLook aside, push, squeeze harder',
    'Step 6 of 7: Back up\nPull the left stick back',
    'Step 7 of 7: Full stop\nLet go of both grip buttons',
  ],
  room_scale: [
    'Step 1 of 7: Hold on\nPress and hold both grip buttons',
    'Step 2 of 7: Walker ready\nWalk forward',
    'Step 3 of 7: Stop\nStand still',
    'Step 4 of 7: Turn\nLead with one hand',
    'Step 5 of 7: Head turn\nLook aside, walk, squeeze harder',
    'Step 6 of 7: Back up\nStep backwards',
    'Step 7 of 7: Full stop\nLet go of both grip buttons',
  ],
};
const REGRIP = 7, FINISH = 8, FINISHED = 9;

// The first-time walkthrough. Each step moves on only when the participant has really done it.
export class Lesson {
  constructor() {
    this.step = 0;
    this.since = null;       // when the current step's condition became true
    this.yaw0 = null;        // heading when the turning step began
    this.finishedAt = 0;
  }

  next() { this.step += 1; this.since = null; }

  sustained(ok, seconds, t) {
    if (!ok) { this.since = null; return false; }
    if (this.since === null) this.since = t;
    return t - this.since >= seconds;
  }

  // held: both hands on and the walker accepts them; moving; speed: the push, m/s (+ forward); yaw: walker heading;
  // headSteering: the head is turning the walker
  update({ held, moving, speed, yaw, headSteering, t }) {
    switch (this.step) {
      case 0: if (held) this.next(); break;
      case 1: if (this.sustained(moving && speed > 0.02, 1.5, t)) this.next(); break;
      case 2: if (this.sustained(!moving && Math.abs(speed) <= 0.02, 1.0, t)) { this.next(); this.yaw0 = yaw; } break;
      case 3: if (Math.abs(Math.atan2(Math.sin(yaw - this.yaw0), Math.cos(yaw - this.yaw0))) > TURN_RAD) this.next(); break;
      case 4: if (this.sustained(headSteering, 1.0, t)) this.next(); break;
      case 5: if (this.sustained(moving && speed < -0.02, 1.0, t)) this.next(); break;
      case 6: if (!held) this.next(); break;
      case REGRIP: if (held) { this.next(); this.finishedAt = t; } break;
      case FINISH: if (t - this.finishedAt > FINISH_S) this.next(); break;
      default: break;
    }
  }

  text(mode) {
    if (this.step === REGRIP || this.step >= FINISHED) return '';
    if (this.step === FINISH) return 'Well done: you can drive\nExplore. Let go to stop';
    return STEPS[mode === 'stationary' ? 'stationary' : 'room_scale'][this.step];
  }
}

// s: connected, recentred, gripState ('STOP' | 'WALK' | 'STEER', undefined until the walker reports),
//    enabled (operator's Start walking), left, right (grip 0..1), mode ('room_scale' | 'stationary'),
//    safetyReason, busy (a hint or notice is already showing), active (moving, or being pushed),
//    step, lesson (the Lesson's step and text), stillFor, stuckFor (seconds)
export function guidance(s) {
  const started = s.step > 0;
  if (!s.connected) return 'Connection lost\nPlease wait';
  if (!s.recentred || !s.gripState) return 'Getting ready\nPlease wait';
  if (s.safetyReason) return 'Let go of both grip buttons\nThen hold A or X for 1 second';
  if (!s.enabled) return started ? 'Walker paused\nPlease wait' : 'Walker getting ready\nPlease wait';

  const goal = started ? 'to start moving again' : 'to start';
  const left = s.left >= GRIP_HELD, right = s.right >= GRIP_HELD;
  if (!left && !right) return started ? `Press and hold both grip buttons\n${goal}` : s.lesson;
  if (!left || !right) return `Also hold the ${left ? 'right' : 'left'} grip button\n${goal}`;
  if (s.gripState === 'STOP') return s.stuckFor > STUCK_S ? 'Let go of both grip buttons\nThen press them again' : '';

  if (s.busy) return '';
  if (s.lesson) return s.lesson;
  if (s.active || s.stillFor <= STILL_S) return '';
  return `${s.mode === 'stationary' ? 'Push the left stick forward' : 'Walk forward'}\nto start moving again`;
}
