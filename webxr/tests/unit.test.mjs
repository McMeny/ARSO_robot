// Node unit tests (node --test webxr/tests): flicker scheduler, calibration math, serialization.
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";

import { solve, headsetToMap, mapToHeadset } from "../js/calib.js";
import { FlickerScheduler, isOn, sequence } from "../js/flicker.js";
import { markerArrayMsg, markerToMsg, msgToMarker, nextSlot, xrTimingMsg } from "../js/messages.js";
import { FrameTiming } from "../js/timing.js";

const table = JSON.parse(readFileSync(new URL("../../config/generated/frequencies.json", import.meta.url)));

test("square flicker sequences are exact for every n of the 90 Hz table", () => {
  for (const s of table.slots) {
    const n = s.frames_per_cycle;
    const seq = sequence(n, n * 6);
    // periodic with period n
    for (let k = n; k < seq.length; k++) assert.equal(seq[k], seq[k - n], `n=${n} k=${k}`);
    // exactly one on->off and one off->on transition per cycle
    let rises = 0;
    for (let k = 1; k < seq.length; k++) if (seq[k] === 1 && seq[k - 1] === 0) rises++;
    assert.equal(rises, 5, `n=${n}`);
    // duty cycle within one frame of 50 %
    const on = seq.slice(0, n).reduce((a, b) => a + b, 0);
    assert.ok(Math.abs(on - n / 2) <= 0.5 + 1e-9, `n=${n} on=${on}`);
  }
});

test("explicit sequences", () => {
  assert.deepEqual(sequence(8, 16), [1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0]);   // 11.25 Hz at 90 Hz
  assert.deepEqual(sequence(12, 12), [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0]);             // 7.5 Hz
  assert.deepEqual(sequence(7, 14), [1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0]);        // 12.857 Hz (odd n: 4 on / 3 off)
  assert.equal(isOn(0, 13), true);
});

test("produced frequency equals the table frequency at the configured refresh", () => {
  for (const s of table.slots) assert.ok(Math.abs(table.refresh_hz / s.frames_per_cycle - s.frequency_exact) < 1e-9);
  const fs = new FlickerScheduler(table);
  assert.equal(fs.mismatch(90), false);
  assert.equal(fs.mismatch(72), true);
  assert.equal(fs.framesFor("CANCEL"), 7);
});

test("calibration recovers a known transform within 1 mm", () => {
  const truth = { yaw: 0.7, tx: 2.1, ty: -1.3, floorHeight: 0.02 };
  const robotPose = [1.5, 0.8, -0.4];
  const decalF = [0.5, 0.0], decalR = [0.05, 0.0];
  const plate = 0.22;
  const dmap = (d) => {
    const c = Math.cos(robotPose[2]), s = Math.sin(robotPose[2]);
    return [robotPose[0] + c * d[0] - s * d[1], robotPose[1] + s * d[0] + c * d[1], plate];
  };
  const hF = mapToHeadset(dmap(decalF), truth);
  const hR = mapToHeadset(dmap(decalR), truth);
  const cal = solve(hF, hR, robotPose, decalF, decalR, plate);
  assert.ok(Math.abs(cal.yaw - truth.yaw) < 1e-6);
  assert.ok(Math.abs(cal.tx - truth.tx) < 1e-3 && Math.abs(cal.ty - truth.ty) < 1e-3);
  assert.ok(Math.abs(cal.floorHeight - truth.floorHeight) < 1e-3);
  assert.ok(cal.residual < 1e-3);
  const p = [3.3, -2.2, 1.4];
  const back = headsetToMap(mapToHeadset(p, cal), cal);
  for (let i = 0; i < 3; i++) assert.ok(Math.abs(back[i] - p[i]) < 1e-6);
});

test("calibration residual flags a bad touch (> 5 cm)", () => {
  const cal = solve([0, 1.2, -1], [0, 1.2, -1.6], [0, 0, 0], [0.5, 0], [0.05, 0], 0.22);  // 0.6 m apart vs 0.45 m
  assert.ok(cal.residual > 0.05);
});

test("marker <-> message round trip", () => {
  const m = { id: 3, label: "Milk", slot: 3, frequency: 8.18, x: 1, y: 2, z: 1.5, facing: 0.3 };
  const msg = markerToMsg(m);
  assert.equal(msg.slot, "3");
  assert.deepEqual(msg.position, { x: 1, y: 2, z: 1.5 });
  const back = msgToMarker(msg);
  assert.equal(back.label, "Milk");
  assert.equal(back.cancel, false);
  const arr = markerArrayMsg([m, { id: 6, label: "CANCEL", slot: "CANCEL", x: 0, y: 0, z: 1 }], "headset", table.hash);
  assert.equal(arr.header.frame_id, "headset");
  assert.equal(arr.markers[1].cancel, true);
  assert.equal(nextSlot([m], table), "1");
  assert.equal(xrTimingMsg({ refreshHz: 90, tableHash: "x", consent: true, mode: "run" }).refresh_hz, 90);
});

test("frame timing detects refresh rate and drops", () => {
  const ft = new FrameTiming();
  let t = 0;
  for (let k = 0; k < 300; k++) {
    t += k === 150 ? 1000 / 90 * 3 : 1000 / 90;
    ft.push(t);
  }
  assert.equal(ft.snappedHz(), 90);
  assert.ok(ft.stats().droppedPct > 0 && ft.stats().droppedPct < 1);
});
