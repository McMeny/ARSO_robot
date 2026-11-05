// What the participant should do next, shown in the headset from the moment VR starts. Pure functions of the
// session state, so every case can be tested without a headset (test/test_guidance.mjs).
// Text is two lines ("a\nb"), or '' when nothing needs saying.

const GRIP_HELD = 0.2;        // grip.yaml light_threshold, until the hand check sets this person's own
const STUCK_S = 0.6;          // hands on but still STOP for this long: the stop is latched (grip_logic.py)
const STILL_S = 2.0;          // stopped for this long after the walkthrough: say how to carry on
const FINISH_S = 6.0;         // how long the closing message stays up
const TURN_RAD = 0.5;         // how far the walker must turn to finish the turning step
const TOUCH = 0.1;            // hand check: a reading this high is a hand on the grip button
const CHECK_HOLD_S = 2.0;     // each hold is timed this long (grip_state_node averages the last second)
const MIN_SQUEEZE = 0.2;      // a firm squeeze must beat the gentle hold by this (GripLogic.calibrate)
const TEST_MARGIN = 0.05;     // grip.yaml hysteresis
const TEST_HOLD_S = 0.5;      // grip.yaml steer_hold_s
const NUDGE_S = 6.0;          // no firm squeeze after this long: ask for a harder one
const REPLY_S = 3.0;          // no answer from the walker after this long: start again
const DONE_S = 3.0;           // how long the closing message stays up

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

// The hand check, before anything else: a gentle hold, a firm squeeze, then a test of the result. It sets this
// person's grip thresholds (grip_state_node records the readings, so real handle sensors work the same way).
export class GripCheck {
  constructor() {
    this.stage = 'gentle';
    this.stageAt = null;
    this.since = null;       // when the current hold began
    this.t = 0;
    this.left = this.right = 0;
    this.gentle = 0;
    this.firm = 0;
    this.retry = false;
  }

  get active() { return this.stage !== 'finished'; }

  skip() { this.stage = 'finished'; }

  go(stage, t) { this.stage = stage; this.stageAt = t; this.since = null; }

  again(t) { this.go('gentle', t); this.retry = true; }

  held(ok, seconds, t) {
    if (!ok) { this.since = null; return false; }
    if (this.since === null) this.since = t;
    return t - this.since >= seconds;
  }

  // Returns the command for grip_state_node to record the last second of readings, or null.
  update({ left, right, t }) {
    Object.assign(this, { left, right, t });
    if (this.stageAt === null) this.stageAt = t;
    const v = Math.min(left, right);
    switch (this.stage) {
      case 'gentle':
        if (this.held(v >= TOUCH, CHECK_HOLD_S, t)) {
          this.gentle = v;
          this.go('firm', t);
          return 'calibrate_relaxed';
        }
        break;
      case 'firm':
        if (this.held(v >= this.gentle + MIN_SQUEEZE, CHECK_HOLD_S, t)) {
          this.go('saving', t);
          return 'calibrate_firm';
        }
        break;
      case 'saving': if (t - this.stageAt > REPLY_S) this.again(t); break;
      case 'relax': if (v <= this.firm - TEST_MARGIN) this.go('squeeze', t); break;
      case 'squeeze': if (this.held(v >= this.firm + TEST_MARGIN, TEST_HOLD_S, t)) this.go('done', t); break;
      case 'done': if (t - this.stageAt > DONE_S) this.go('finished', t); break;
      default: break;
    }
    return null;
  }

  // The walker's answer: { kind: 'relaxed' | 'firm', ok, firm (the new threshold) }
  reply({ kind, ok, firm }, t) {
    if (!ok) this.again(t);
    else if (kind === 'firm' && this.stage === 'saving') { this.firm = firm; this.go('relax', t); }
  }

  text() {
    const left = this.left >= TOUCH, right = this.right >= TOUCH;
    const count = () => Math.max(1, Math.ceil(CHECK_HOLD_S - (this.t - this.since)));
    switch (this.stage) {
      case 'gentle': {
        const title = this.retry ? "Let's try that again" : 'Hand check 1 of 3: Gentle hold';
        if (this.since !== null) return `${title}\nKeep holding gently: ${count()}`;
        if (left !== right) return `${title}\nAlso hold the ${left ? 'right' : 'left'} grip button`;
        return `${title}\nHold both grip buttons gently`;
      }
      case 'firm': {
        const title = 'Hand check 2 of 3: Firm squeeze';
        if (this.since !== null) return `${title}\nKeep squeezing: ${count()}`;
        const need = this.gentle + MIN_SQUEEZE;
        if ((this.left >= need) !== (this.right >= need)) {
          return `${title}\nSqueeze your ${this.left >= need ? 'right' : 'left'} hand harder`;
        }
        return `${title}\n${this.t - this.stageAt > NUDGE_S ? 'Squeeze harder than that' : 'Squeeze both buttons hard'}`;
      }
      case 'saving': return 'Saving your grip\nPlease wait';
      case 'relax': return 'Hand check 3 of 3: Test it\nRelax back to a gentle hold';
      case 'squeeze': return 'Hand check 3 of 3: Test it\nNow squeeze hard again';
      case 'done': return 'Hand check done\nYour grip is set';
      default: return '';
    }
  }
}

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
//    check (the GripCheck's text), light (this person's grip threshold),
//    step, lesson (the Lesson's step and text), stillFor, stuckFor (seconds)
export function guidance(s) {
  const started = s.step > 0;
  if (!s.connected) return 'Connection lost\nPlease wait';
  if (!s.recentred || !s.gripState) return 'Getting ready\nPlease wait';
  if (s.safetyReason) return 'Let go of both grip buttons\nThen hold A or X for 1 second';
  if (s.check) return s.check;
  if (!s.enabled) return started ? 'Walker paused\nPlease wait' : 'Walker getting ready\nPlease wait';

  const goal = started ? 'to start moving again' : 'to start';
  const held = s.light ?? GRIP_HELD;
  const left = s.left >= held, right = s.right >= held;
  if (!left && !right) return started ? `Press and hold both grip buttons\n${goal}` : s.lesson;
  if (!left || !right) return `Also hold the ${left ? 'right' : 'left'} grip button\n${goal}`;
  if (s.gripState === 'STOP') return s.stuckFor > STUCK_S ? 'Let go of both grip buttons\nThen press them again' : '';

  if (s.busy) return '';
  if (s.lesson) return s.lesson;
  if (s.active || s.stillFor <= STILL_S) return '';
  return `${s.mode === 'stationary' ? 'Push the left stick forward' : 'Walk forward'}\nto start moving again`;
}
