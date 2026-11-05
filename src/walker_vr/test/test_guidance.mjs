// node --test src/walker_vr/test/test_guidance.mjs
import assert from 'node:assert/strict';
import { test } from 'node:test';
import { guidance, GripCheck, Lesson } from '../web/js/guidance.js';

const ready = { connected: true, recentred: true, gripState: 'WALK', enabled: true, left: 1, right: 1, mode: 'stationary',
                safetyReason: '', busy: false, active: false, step: 1, lesson: 'A lesson\nstep', stillFor: 0, stuckFor: 0 };
const say = (over) => guidance({ ...ready, ...over });
const after = { step: 9, lesson: '' };

test('start-up: waits for the link, the recentre, the walker and the operator', () => {
  assert.match(say({ connected: false }), /Connection lost/);
  assert.match(say({ recentred: false }), /Getting ready/);
  assert.match(say({ gripState: undefined }), /Getting ready/);
  assert.match(say({ step: 0, enabled: false, gripState: 'STOP' }), /Walker getting ready/);
});

test('the first step shows with hands off; one hand is told about the other', () => {
  assert.equal(say({ step: 0, lesson: 'First', gripState: 'STOP', left: 0, right: 0 }), 'First');
  assert.match(say({ step: 0, gripState: 'STOP', left: 1, right: 0 }), /Also hold the right grip button/);
  assert.match(say({ step: 0, gripState: 'STOP', left: 0, right: 1 }), /Also hold the left grip button/);
});

test('a lesson step shows while walking, and never talks over a notice', () => {
  assert.equal(say({}), 'A lesson\nstep');
  assert.equal(say({ active: true }), 'A lesson\nstep');
  assert.equal(say({ gripState: 'STEER' }), 'A lesson\nstep');
  assert.equal(say({ busy: true }), '');
});

test('hands off at any later point: start moving again', () => {
  assert.equal(say({ gripState: 'STOP', left: 0, right: 0 }), 'Press and hold both grip buttons\nto start moving again');
  assert.match(say({ gripState: 'STOP', left: 1, right: 0 }), /to start moving again/);
});

test('after the walkthrough: quiet while driving, then how to carry on', () => {
  assert.equal(say({ ...after, active: true }), '');
  assert.equal(say({ ...after, stillFor: 1 }), '');
  assert.equal(say({ ...after, stillFor: 3 }), 'Push the left stick forward\nto start moving again');
  assert.equal(say({ ...after, stillFor: 3, mode: 'room_scale' }), 'Walk forward\nto start moving again');
});

test('stops: operator, safety and the latched grip', () => {
  assert.match(say({ enabled: false }), /Walker paused/);
  assert.match(say({ safetyReason: 'Room edge reached', gripState: 'STOP' }), /hold A or X/);
  assert.equal(say({ gripState: 'STOP', stuckFor: 0.1 }), '');
  assert.equal(say({ gripState: 'STOP', stuckFor: 1 }), 'Let go of both grip buttons\nThen press them again');
});

test('the walkthrough moves on only when each thing is really done', () => {
  const l = new Lesson();
  const still = { held: true, moving: false, speed: 0, yaw: 0 };
  const go = (over, t) => l.update({ ...still, ...over, t });

  assert.match(l.text('stationary'), /Step 1 of 7/);
  go({ held: false }, 0);
  assert.equal(l.step, 0);
  go({}, 1);
  assert.match(l.text('stationary'), /Step 2 of 7: Walker ready\nPush the left stick forward/);
  assert.match(l.text('room_scale'), /Walk forward/);

  go({ moving: true, speed: 0.3 }, 2);
  go({ moving: true, speed: 0.3 }, 3);          // a short push isn't enough
  assert.equal(l.step, 1);
  go({ moving: true, speed: 0.3 }, 3.6);
  assert.match(l.text('stationary'), /Step 3 of 7/);

  go({ moving: true, speed: 0.3 }, 4);          // still moving: not stopped yet
  go({}, 5);
  go({}, 6.1);
  assert.match(l.text('stationary'), /Step 4 of 7/);

  go({ yaw: 0.2 }, 7);
  assert.equal(l.step, 3);
  go({ yaw: 0.6 }, 8);
  assert.match(l.text('stationary'), /Step 5 of 7: Head turn/);

  go({ moving: true, speed: 0.3 }, 8.5);        // pushing without the head doesn't count
  go({ moving: true, speed: 0.3, headSteering: true }, 8.6);
  go({ moving: true, speed: 0.3, headSteering: true }, 8.9);
  assert.equal(l.step, 4);
  go({ moving: true, speed: 0.3, headSteering: true }, 9.7);
  assert.match(l.text('stationary'), /Step 6 of 7: Back up/);

  go({ moving: true, speed: 0.3 }, 9.8);        // forward doesn't count as backing up
  go({ moving: true, speed: -0.2 }, 10);
  go({ moving: true, speed: -0.2 }, 11.1);
  assert.match(l.text('stationary'), /Step 7 of 7/);

  go({}, 12);
  assert.equal(l.step, 6);
  go({ held: false }, 13);
  assert.equal(l.text('stationary'), '');       // guidance() asks for the grips back
  go({}, 14);
  assert.match(l.text('stationary'), /Well done/);
  go({}, 21);
  assert.equal(l.text('stationary'), '');
});

test('the turn counts either way, and across the +-pi seam', () => {
  const l = new Lesson();
  l.step = 3; l.yaw0 = 3.1;
  l.update({ held: true, moving: false, speed: 0, yaw: -3.1, t: 0 });
  assert.equal(l.step, 3);                      // only 0.08 rad apart
  l.update({ held: true, moving: false, speed: 0, yaw: -2.6, t: 1 });
  assert.equal(l.step, 4);
});

function runCheck(c, level, from, to) {
  const out = [];
  for (let t = from; t <= to + 1e-9; t += 0.1) {
    const name = c.update({ left: level, right: level, t });
    if (name) out.push(name);
  }
  return out;
}

test('the hand check: instructions, countdowns, and the two recordings', () => {
  const c = new GripCheck();
  assert.ok(c.active);
  c.update({ left: 0, right: 0, t: 0 });
  assert.equal(c.text(), 'Hand check 1 of 3: Gentle hold\nHold both grip buttons gently');
  c.update({ left: 0.4, right: 0, t: 0.1 });
  assert.match(c.text(), /Also hold the right grip button/);
  c.update({ left: 0, right: 0.4, t: 0.2 });
  assert.match(c.text(), /Also hold the left grip button/);

  assert.deepEqual(runCheck(c, 0.4, 1, 1.5), []);
  assert.match(c.text(), /Keep holding gently: 2/);
  assert.deepEqual(runCheck(c, 0.4, 1.6, 3.2), ['calibrate_relaxed']);
  assert.equal(c.text(), 'Hand check 2 of 3: Firm squeeze\nSqueeze both buttons hard');

  c.update({ left: 0.9, right: 0.4, t: 3.3 });
  assert.match(c.text(), /Squeeze your right hand harder/);
  c.update({ left: 0.4, right: 0.9, t: 3.4 });
  assert.match(c.text(), /Squeeze your left hand harder/);
  c.update({ left: 0.45, right: 0.45, t: 10 });
  assert.match(c.text(), /Squeeze harder than that/);

  assert.deepEqual(runCheck(c, 0.9, 11, 13.2), ['calibrate_firm']);
  assert.match(c.text(), /Saving your grip/);
});

test('the hand check tests the result, then finishes', () => {
  const c = new GripCheck();
  runCheck(c, 0.4, 0, 2.2);
  runCheck(c, 0.9, 3, 5.2);
  c.reply({ kind: 'relaxed', ok: true }, 5.3);
  assert.equal(c.stage, 'saving');
  c.reply({ kind: 'firm', ok: true, firm: 0.65 }, 5.4);
  assert.match(c.text(), /Relax back to a gentle hold/);
  runCheck(c, 0.9, 5.5, 6);
  assert.match(c.text(), /Relax back/);          // still squeezing
  runCheck(c, 0.4, 6.1, 6.3);
  assert.match(c.text(), /Now squeeze hard again/);
  runCheck(c, 0.9, 6.4, 6.6);
  assert.match(c.text(), /squeeze hard again/);  // a short squeeze isn't enough
  runCheck(c, 0.9, 6.7, 7.1);
  assert.match(c.text(), /Hand check done/);
  assert.ok(c.active);
  runCheck(c, 0.9, 7.2, 11);
  assert.ok(!c.active);
  assert.equal(c.text(), '');
});

test('the hand check never leaves them stuck', () => {
  const refused = new GripCheck();
  runCheck(refused, 0.4, 0, 2.2);
  runCheck(refused, 0.9, 3, 5.2);
  refused.reply({ kind: 'firm', ok: false }, 5.3);
  assert.equal(refused.text(), "Let's try that again\nHold both grip buttons gently");

  const silent = new GripCheck();
  runCheck(silent, 0.4, 0, 2.2);
  runCheck(silent, 0.9, 3, 5.2);
  runCheck(silent, 0.9, 5.3, 9);
  assert.equal(silent.stage, 'gentle');
  assert.ok(silent.retry);

  const skipped = new GripCheck();
  skipped.skip();
  assert.ok(!skipped.active);
});

test('the hand check comes before everything but the link, the recentre and safety', () => {
  assert.equal(say({ step: 0, check: 'Check', enabled: false, left: 0, right: 0, gripState: 'STOP' }), 'Check');
  assert.equal(say({ step: 0, check: 'Check' }), 'Check');
  assert.match(say({ check: 'Check', connected: false }), /Connection lost/);
  assert.match(say({ check: 'Check', recentred: false }), /Getting ready/);
  assert.match(say({ check: 'Check', safetyReason: 'Room edge reached' }), /hold A or X/);
});

test("the person's own grip threshold decides what counts as held", () => {
  assert.equal(say({ left: 0.15, right: 0.15, light: 0.1 }), 'A lesson\nstep');
  assert.match(say({ left: 0.15, right: 0.15, light: 0.3, gripState: 'STOP' }), /Press and hold both/);
});
