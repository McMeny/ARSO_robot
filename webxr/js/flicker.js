// Frame-locked SSVEP flicker (Section 14.3). Driven by the FRAME COUNT, never wall time.
// A marker at f = R / n flickers with period n display frames:
//   square:   on  <=>  floor(frameIndex / (n / 2)) is even      (exact for even n; 7/6 split for n = 13)
//   sinusoid: luminance = 0.5 + 0.5 * cos(2 * pi * frameIndex / n)
export function isOn(frameIndex, n) {
  return Math.floor(frameIndex / (n / 2)) % 2 === 0;
}

export function luminance(frameIndex, n, mode = "square", lo = 0.0, hi = 1.0) {
  const v = mode === "sinusoid" ? 0.5 + 0.5 * Math.cos((2 * Math.PI * frameIndex) / n) : isOn(frameIndex, n) ? 1 : 0;
  return lo + (hi - lo) * v;
}

// Full on/off sequence over `frames` frames (used by the unit tests and the SSVEP smoke test).
export function sequence(n, frames, mode = "square") {
  const out = [];
  for (let k = 0; k < frames; k++) out.push(mode === "square" ? (isOn(k, n) ? 1 : 0) : luminance(k, n, mode));
  return out;
}

// Frequency actually produced on a display running at `actualHz` by a marker built for n frames.
export function producedFrequency(n, actualHz) {
  return actualHz / n;
}

export class FlickerScheduler {
  constructor(table) {
    this.table = table;                 // /config/frequencies.json (same file the decoder uses)
    this.frame = 0;
    this.bySlot = new Map(table.slots.map((s) => [String(s.slot), s]));
  }
  framesFor(slot) {
    const s = this.bySlot.get(String(slot));
    return s ? s.frames_per_cycle : null;
  }
  tick() {
    this.frame += 1;
    return this.frame;
  }
  level(slot, mode = "square") {
    const n = this.framesFor(slot);
    return n ? luminance(this.frame, n, mode) : 0;
  }
  // refresh mismatch -> the produced frequencies differ from the decoder's -> block run mode
  mismatch(actualHz, tol = 1.0) {
    return Math.abs(actualHz - this.table.refresh_hz) > tol;
  }
}
