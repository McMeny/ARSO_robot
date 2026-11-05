// What the participant should do next, shown in the headset from the moment VR starts. A pure
// function of the session state, so every case can be tested without a headset (test/test_guidance.mjs).
// Returns two lines ("a\nb"), or '' when nothing needs saying.

const GRIP_HELD = 0.2;        // grip.yaml light_threshold
const STUCK_S = 0.6;          // hands on but still STOP for this long: the stop is latched (grip_logic.py)
const STILL_S = 2.0;          // stopped for this long after moving: say how to carry on
const TIP_S = 8.0;            // how long the turning tip stays up after the first move

// s: connected, recentred, gripState ('STOP' | 'WALK' | 'STEER', undefined until the walker reports),
//    enabled (operator's Start walking), left, right (grip 0..1), mode ('room_scale' | 'stationary'),
//    safetyReason, busy (a hint or notice is already showing), active (moving, or being pushed),
//    everMoved, movedFor, stillFor, stuckFor (seconds)
export function guidance(s) {
  if (!s.connected) return 'Connection lost\nPlease wait';
  if (!s.recentred || !s.gripState) return 'Getting ready\nPlease wait';
  if (s.safetyReason) return 'Let go of both grip buttons\nThen hold A or X for 1 second';
  if (!s.enabled) return s.everMoved ? 'Walker paused\nPlease wait' : 'Walker getting ready\nPlease wait';

  const goal = s.everMoved ? 'to start moving again' : 'to start';
  const left = s.left >= GRIP_HELD, right = s.right >= GRIP_HELD;
  if (!left && !right) return `Press and hold both grip buttons\n${goal}`;
  if (!left || !right) return `Also hold the ${left ? 'right' : 'left'} grip button\n${goal}`;
  if (s.gripState === 'STOP') return s.stuckFor > STUCK_S ? 'Let go of both grip buttons\nThen press them again' : '';

  if (s.busy) return '';
  const stationary = s.mode === 'stationary';
  if (s.active) {
    if (!s.everMoved || s.movedFor > TIP_S) return '';
    return stationary ? 'Right stick turns\nTurn your body with it' : 'Lead with one hand to turn\nHold back to slow down';
  }
  const go = stationary ? 'Push the left stick forward' : 'Walk forward';
  if (!s.everMoved) return `Walker ready: start moving\n${go}`;
  return s.stillFor > STILL_S ? `${go}\nto start moving again` : '';
}
