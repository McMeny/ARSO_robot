// node --test src/walker_vr/test/test_guidance.mjs
import assert from 'node:assert/strict';
import { test } from 'node:test';
import { guidance } from '../web/js/guidance.js';

const ready = { connected: true, recentred: true, gripState: 'WALK', enabled: true, left: 1, right: 1, mode: 'stationary',
                safetyReason: '', busy: false, active: false, everMoved: false, movedFor: 0, stillFor: 0, stuckFor: 0 };
const say = (over) => guidance({ ...ready, ...over });
const moved = { everMoved: true, movedFor: 20 };

test('start-up: waits for the link, the recentre, the walker and the operator', () => {
  assert.match(say({ connected: false }), /Connection lost/);
  assert.match(say({ recentred: false }), /Getting ready/);
  assert.match(say({ gripState: undefined }), /Getting ready/);
  assert.match(say({ enabled: false, gripState: 'STOP' }), /Walker getting ready/);
});

test('asks for the grips, then says the walker is ready', () => {
  assert.equal(say({ gripState: 'STOP', left: 0, right: 0 }), 'Press and hold both grip buttons\nto start');
  assert.match(say({ gripState: 'STOP', left: 1, right: 0 }), /Also hold the right grip button/);
  assert.match(say({ gripState: 'STOP', left: 0, right: 1 }), /Also hold the left grip button/);
  assert.equal(say({}), 'Walker ready: start moving\nPush the left stick forward');
  assert.equal(say({ mode: 'room_scale' }), 'Walker ready: start moving\nWalk forward');
});

test('after moving: says how to get going again', () => {
  assert.equal(say({ ...moved, gripState: 'STOP', left: 0, right: 0 }), 'Press and hold both grip buttons\nto start moving again');
  assert.match(say({ ...moved, gripState: 'STOP', left: 1, right: 0 }), /to start moving again/);
  assert.equal(say({ ...moved, stillFor: 1 }), '');
  assert.equal(say({ ...moved, stillFor: 3 }), 'Push the left stick forward\nto start moving again');
  assert.equal(say({ ...moved, stillFor: 3, mode: 'room_scale' }), 'Walk forward\nto start moving again');
});

test('quiet while moving, after a short turning tip', () => {
  assert.match(say({ active: true, everMoved: true, movedFor: 2 }), /Right stick turns/);
  assert.match(say({ active: true, everMoved: true, movedFor: 2, mode: 'room_scale' }), /Lead with one hand/);
  assert.equal(say({ active: true, ...moved }), '');
});

test('stops: operator, safety and the latched grip', () => {
  assert.match(say({ ...moved, enabled: false }), /Walker paused/);
  assert.match(say({ safetyReason: 'Room edge reached', gripState: 'STOP' }), /hold A or X/);
  assert.equal(say({ gripState: 'STOP', stuckFor: 0.1 }), '');
  assert.equal(say({ gripState: 'STOP', stuckFor: 1 }), 'Let go of both grip buttons\nThen press them again');
});

test('never talks over the blocked notice or a hint, and STEER counts as held', () => {
  assert.equal(say({ busy: true }), '');
  assert.match(say({ gripState: 'STEER' }), /Walker ready/);
});
