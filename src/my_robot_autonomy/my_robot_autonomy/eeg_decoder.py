"""Reads the Muse over LSL and decides, per tagged hazard, whether its flicker frequency
is present in the EEG at Oz.

LSL stream (from BlueMuse / muselsl on Windows, or the muse_emulator)  ->  every `step` s,
on the last `window` s of Oz:
  for each hazard with a frequency on /hazards:
    score = response at its frequency / noise floor beside it   (ssvep.score)
    present now  if score >= threshold  AND  score >= ref_ratio x the median score of
                 frequencies NOT being shown (so a general rise in power doesn't count)
    present      if present now for windows_required windows in a row
  -> /probe_results (walker_interfaces/ProbeResult), signal_ok = false if the signal is bad

/eeg_status (JSON): stream state, Oz quality, sample rate, threshold, SSVEP check result.

SSVEP check (/operator_command "ssvep_check"): shows a test patch (/test_stimulus) at each
check frequency - off 4 s, on 8 s - and passes if the score while on is clearly above
the score while off. It then sets the threshold between the two.
"""
import collections
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_robot_autonomy import ssvep
from walker_interfaces.msg import HazardArray, ProbeResult

try:
    import pylsl
except ImportError:  # reported in /eeg_status
    pylsl = None


class EegDecoder(Node):
    def __init__(self):
        super().__init__('eeg_decoder')
        p = self.declare_parameter
        self.stream_type = p('stream_type', 'EEG').value
        self.stream_name = p('stream_name', '').value          # '' = any EEG stream
        self.oz_channel = p('oz_channel', 4).value              # Muse: TP9 AF7 AF8 TP10 AUX
        self.window = p('window', 2.0).value
        self.step = p('step', 0.5).value
        self.threshold = p('threshold', 3.5).value
        self.ref_ratio = p('ref_ratio', 1.5).value
        self.windows_required = p('windows_required', 3).value
        self.line_hz = p('line_frequency', 60.0).value
        self.table = list(p('frequencies', [7.5, 11.25, 8.18, 12.86, 6.92, 9.0, 6.43, 10.0]).value)
        self.check_freqs = list(p('check_frequencies', [7.5, 12.86]).value)

        self.buffer = collections.deque(maxlen=256 * 12)
        self.times = collections.deque(maxlen=256 * 12)
        self.lock = threading.Lock()
        self.stream_label = 'NO_STREAM'
        self.fs = 256.0
        self.channel_info = ''
        self.hazards = []
        self.streak = {}            # (hazard id, freq) -> windows in a row with a response
        self.last_quality = ('NO_STREAM', 0.0)
        self.check = {'state': 'not run'}
        self.check_plan = []        # remaining steps of a running SSVEP check
        self.check_scores = {}
        self.step_started = None

        self.result_pub = self.create_publisher(ProbeResult, 'probe_results', 50)
        self.status_pub = self.create_publisher(String, 'eeg_status', 10)
        self.stim_pub = self.create_publisher(String, 'test_stimulus', 10)
        self.create_subscription(HazardArray, 'hazards', lambda m: setattr(self, 'hazards', m.hazards), 10)
        self.create_subscription(String, 'operator_command', self.on_command, 10)
        self.create_timer(self.step, self.analyse)
        self.create_timer(1.0, self.publish_status)
        self.create_timer(0.25, self.run_check)

        if pylsl is None:
            self.get_logger().error('pylsl is not installed - run: pip install pylsl')
        else:
            threading.Thread(target=self.reader, daemon=True).start()

    # ---------- LSL ----------

    def reader(self):
        while rclpy.ok():
            self.get_logger().info(f'Looking for an LSL stream of type {self.stream_type}...')
            streams = pylsl.resolve_byprop('type', self.stream_type, timeout=5.0)
            if self.stream_name:
                streams = [s for s in streams if s.name() == self.stream_name]
            if not streams:
                continue
            info = streams[0]
            inlet = pylsl.StreamInlet(info, max_buflen=30,
                                      processing_flags=pylsl.proc_clocksync | pylsl.proc_dejitter)
            labels = self.channel_labels(inlet)
            channel = self.oz_channel
            for i, label in enumerate(labels):
                if 'aux' in label.lower() or label.lower() == 'oz':
                    channel = i
            self.fs = info.nominal_srate() or 256.0
            self.channel_info = (f'{info.name()}: {info.channel_count()} channels {labels or ""}, '
                                 f'Oz = channel {channel}, {self.fs:.0f} Hz nominal')
            self.get_logger().info('Connected to ' + self.channel_info)
            if info.channel_count() <= channel:
                self.get_logger().error(f'The stream has only {info.channel_count()} channels - no AUX/Oz '
                                        f'channel. Turn on the AUX channel in the streaming tool.')
            last_data = time.time()
            while rclpy.ok():
                samples, stamps = inlet.pull_chunk(timeout=0.5)
                if samples:
                    last_data = time.time()
                    with self.lock:
                        for s, t in zip(samples, stamps):
                            if len(s) > channel:
                                self.buffer.append(s[channel])
                                self.times.append(t)
                elif time.time() - last_data > 3.0:
                    self.get_logger().warn('EEG stream stopped - looking for it again')
                    with self.lock:
                        self.buffer.clear()
                        self.times.clear()
                    break

    @staticmethod
    def channel_labels(inlet):
        labels = []
        try:
            ch = inlet.info().desc().child('channels').child('channel')
            while not ch.empty():
                labels.append(ch.child_value('label'))
                ch = ch.next_sibling()
        except Exception:
            pass
        return labels

    def latest(self, seconds):
        with self.lock:
            if len(self.buffer) < 10:
                return None, None
            x = np.array(self.buffer)
            t = np.array(self.times)
        if pylsl is not None and pylsl.local_clock() - t[-1] > 2.0:
            return None, None  # stale
        n = int(seconds * self.fs)
        if len(x) < n:
            return None, None
        return x[-n:], t[-n:]

    def measured_rate(self):
        x, t = self.latest(3.0)
        if t is None or t[-1] - t[0] <= 0:
            return 0.0
        return (len(t) - 1) / (t[-1] - t[0])

    # ---------- decoding ----------

    def window_scores(self, freqs, exclude):
        raw, _ = self.latest(self.window)
        if raw is None:
            self.last_quality = ('NO_STREAM', 0.0)
            return None
        self.last_quality = ssvep.quality(raw, self.fs)
        if self.last_quality[0] != 'OK':
            return None
        x = ssvep.preprocess(raw, self.fs, self.line_hz)
        return {f: ssvep.score(x, self.fs, f, exclude=[e for e in exclude if abs(e - f) > 0.01]) for f in freqs}

    def analyse(self):
        tagged = [(h.id, h.frequency) for h in self.hazards if h.frequency > 0]
        if self.check_plan:
            return  # the SSVEP check owns the EEG while it runs
        shown = [f for _, f in tagged]
        refs = [f for f in self.table if all(abs(f - s) > 0.3 and abs(f - 2 * s) > 0.3 and abs(2 * f - s) > 0.3
                                             for s in shown)]
        scores = self.window_scores(shown + refs, exclude=shown)
        stamp = self.get_clock().now().to_msg()
        ref_level = float(np.median([scores[f] for f in refs])) if scores and refs else 1.0
        for hid, f in tagged:
            msg = ProbeResult(stamp=stamp, hazard_id=hid, frequency=f)
            if scores is None:
                msg.signal_ok, msg.present, msg.score = False, False, 0.0
                self.streak[(hid, f)] = 0
            else:
                s = scores[f]
                now = s >= self.threshold and s >= self.ref_ratio * ref_level
                self.streak[(hid, f)] = self.streak.get((hid, f), 0) + 1 if now else 0
                msg.signal_ok, msg.score = True, float(s)
                msg.present = self.streak[(hid, f)] >= self.windows_required
            self.result_pub.publish(msg)

    # ---------- SSVEP check ----------

    def on_command(self, msg):
        if msg.data == 'ssvep_check' and not self.check_plan:
            self.get_logger().info('SSVEP check started')
            self.check_plan = []
            for f in self.check_freqs:
                self.check_plan += [('off', f, 4.0), ('on', f, 8.0)]
            self.check_scores = {f: {'on': [], 'off': []} for f in self.check_freqs}
            self.check = {'state': 'running'}
            self.step_started = None

    def run_check(self):
        if not self.check_plan:
            return
        phase, f, duration = self.check_plan[0]
        now = time.time()
        if self.step_started is None:
            self.step_started = now
            self.stim_pub.publish(String(data=json.dumps({'freq': f, 'on': phase == 'on'})))
            self.check = {'state': 'running', 'step': f'{phase} {f:.2f} Hz'}
        elapsed = now - self.step_started
        if elapsed > self.window + 0.5:  # wait for a full window inside this step
            scores = self.window_scores([f], exclude=[])
            if scores:
                self.check_scores[f][phase].append(scores[f])
        if elapsed >= duration:
            self.check_plan.pop(0)
            self.step_started = None
            if not self.check_plan:
                self.finish_check()

    def finish_check(self):
        self.stim_pub.publish(String(data=json.dumps({'freq': 0, 'on': False})))
        details, ons, offs = [], [], []
        for f, s in self.check_scores.items():
            on = float(np.median(s['on'])) if s['on'] else 0.0
            off = float(np.median(s['off'])) if s['off'] else 0.0
            ok = on >= 2.5 and on >= 2.0 * max(off, 0.5)
            details.append({'freq': f, 'on': round(on, 2), 'off': round(off, 2), 'pass': ok})
            ons.append(on)
            offs.append(off)
        passed = all(d['pass'] for d in details) and bool(details)
        if passed:
            # threshold between the weakest response and the strongest no-response
            # never below 3.0 - lower lets noise through on long probes
            self.threshold = float(np.clip(np.sqrt(min(ons) * max(max(offs), 1.0)), 3.0, 6.0))
        self.check = {'state': 'pass' if passed else 'fail', 'details': details,
                      'threshold': round(self.threshold, 2),
                      'hint': '' if passed else 'Check the Oz electrode: gel, contact, and that it is on AUX'}
        self.get_logger().info(f'SSVEP check {"PASSED" if passed else "FAILED"}: {details} '
                               f'-> threshold {self.threshold:.2f}')

    # ---------- status ----------

    def publish_status(self):
        rate = self.measured_rate()
        label, sd = self.last_quality
        if pylsl is None:
            stream = 'NO_PYLSL'
        elif rate == 0.0:
            stream = 'NO_STREAM'
        elif label != 'OK':
            stream = 'POOR'
        else:
            stream = 'OK'
        status = {'stream': stream, 'quality': label, 'oz_sd_uv': round(sd, 1), 'sample_rate': round(rate, 1),
                  'threshold': round(self.threshold, 2), 'source': self.channel_info, 'check': self.check}
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = EegDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
