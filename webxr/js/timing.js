// Display-refresh measurement and frame-timing statistics (Section 14.3).
export class FrameTiming {
  // emulateHz (TEST ONLY, ?emulate_refresh=90): the headless headset client reports a fixed display rate.
  constructor(windowSize = 240, emulateHz = 0) {
    this.emulateHz = emulateHz;
    this.windowSize = windowSize;
    this.dts = [];
    this.last = null;
    this.frames = 0;
    this.dropped = 0;
  }
  // t in milliseconds (requestAnimationFrame / XRFrame time)
  push(t) {
    if (this.last !== null) {
      const dt = t - this.last;
      if (dt > 0 && dt < 1000) {
        this.dts.push(dt);
        if (this.dts.length > this.windowSize) this.dts.shift();
        const nominal = this.nominalMs();
        if (nominal && dt > 1.5 * nominal) this.dropped += Math.round(dt / nominal) - 1;
      }
    }
    this.last = t;
    this.frames += 1;
  }
  median() {
    if (!this.dts.length) return 0;
    const s = [...this.dts].sort((a, b) => a - b);
    return s[Math.floor(s.length / 2)];
  }
  nominalMs() {
    return this.dts.length >= 10 ? this.median() : 0;
  }
  refreshHz() {
    const m = this.median();
    return m ? 1000 / m : 0;
  }
  // snap to a known display rate when close (Quest: 72 / 90 / 120; desktop: 60 / 144 ...)
  snappedHz() {
    if (this.emulateHz) return this.emulateHz;
    const hz = this.refreshHz();
    const known = [60, 72, 75, 90, 100, 120, 144, 165];
    let best = hz;
    for (const k of known) if (Math.abs(k - hz) < 2.5) best = k;
    return best;
  }
  stats() {
    const n = this.dts.length || 1;
    const mean = this.dts.reduce((a, b) => a + b, 0) / n;
    const sd = Math.sqrt(this.dts.reduce((a, b) => a + (b - mean) ** 2, 0) / n);
    return {
      refreshHz: this.snappedHz(),
      frameMsMean: mean,
      frameMsStd: sd,
      droppedPct: this.emulateHz ? 0 : (this.frames ? (100 * this.dropped) / (this.frames + this.dropped) : 0),
    };
  }
}
