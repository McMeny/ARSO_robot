"""EEG setup checks, run before a session. Each prints PASS or FAIL and what to fix.

  ros2 run my_robot_autonomy eeg_check stream   the Muse stream arrives: ~256 Hz, an AUX (Oz) channel, a live signal
  ros2 run my_robot_autonomy eeg_check alpha    eyes open 10 s, then closed 10 s: alpha (8-12 Hz) at Oz should rise
                                                with the eyes closed - that means the Oz electrode has good contact
"""
import sys
import time

import numpy as np

from my_robot_autonomy import ssvep

try:
    import pylsl
except ImportError:
    pylsl = None


def find_stream():
    print('Looking for an LSL EEG stream (10 s)...')
    streams = pylsl.resolve_byprop('type', 'EEG', timeout=10.0)
    if not streams:
        print('FAIL  No EEG stream found.\n'
              '      - Is BlueMuse (or muselsl) streaming on Windows?\n'
              '      - Is WSL mirrored networking on (README step A3)?\n'
              '      - Are the LSL firewall ports open (README step A4)?')
        return None
    return streams[0]


def channel_labels(inlet):
    labels = []
    ch = inlet.info().desc().child('channels').child('channel')
    while not ch.empty():
        labels.append(ch.child_value('label'))
        ch = ch.next_sibling()
    return labels


def oz_index(labels, count):
    for i, label in enumerate(labels):
        if 'aux' in label.lower() or label.lower() == 'oz':
            return i
    return 4 if count > 4 else None


def record(inlet, seconds, channel):
    data, stamps = [], []
    end = time.time() + seconds
    while time.time() < end:
        samples, ts = inlet.pull_chunk(timeout=0.5)
        for s, t in zip(samples, ts):
            data.append(s[channel])
            stamps.append(t)
    return np.array(data), np.array(stamps)


def check_stream():
    info = find_stream()
    if info is None:
        return False
    inlet = pylsl.StreamInlet(info, processing_flags=pylsl.proc_clocksync | pylsl.proc_dejitter)
    labels = channel_labels(inlet)
    print(f'Found "{info.name()}": {info.channel_count()} channels {labels}, nominal {info.nominal_srate():.0f} Hz')
    oz = oz_index(labels, info.channel_count())
    if oz is None:
        print('FAIL  No AUX channel. Turn on the AUX channel in BlueMuse / muselsl, and check the Oz '
              'electrode is plugged into the Muse AUX input.')
        return False
    x, t = record(inlet, 5.0, oz)
    if len(x) < 100:
        print('FAIL  The stream exists but almost no data arrived. Restart the stream on Windows.')
        return False
    rate = (len(t) - 1) / (t[-1] - t[0])
    label, sd = ssvep.quality(x, rate)
    ok = abs(rate - 256) < 10 and label == 'OK'
    print(f'Oz (channel {oz}): {rate:.1f} Hz measured, signal {label}, sd {sd:.1f} uV')
    if abs(rate - 256) >= 10:
        print('FAIL  Sample rate should be about 256 Hz.')
    elif label != 'OK':
        print({'FLAT': 'FAIL  Oz is flat - the electrode is not connected or has no contact.',
               'SATURATED': 'FAIL  Oz is saturated - re-seat the electrode and add gel.',
               'NOISY': 'FAIL  Oz is very noisy - ask the participant to keep still; check the contact.',
               'LINE_NOISE': 'FAIL  Mains hum on Oz - check the electrode wire and the Muse fit.'}[label])
    else:
        print('PASS  The Muse stream is good.')
    return ok


def check_alpha():
    info = find_stream()
    if info is None:
        return False
    inlet = pylsl.StreamInlet(info, processing_flags=pylsl.proc_clocksync | pylsl.proc_dejitter)
    labels = channel_labels(inlet)
    oz = oz_index(labels, info.channel_count())
    if oz is None:
        print('FAIL  No AUX (Oz) channel - run "eeg_check stream" first.')
        return False
    fs = info.nominal_srate() or 256.0
    input('Ask the participant to keep their EYES OPEN and relaxed, then press Enter...')
    print('Recording, eyes open (10 s)...')
    x_open, _ = record(inlet, 10.0, oz)
    input('Now ask them to CLOSE their eyes, then press Enter...')
    print('Recording, eyes closed (10 s)...')
    x_closed, _ = record(inlet, 10.0, oz)
    if len(x_open) < fs * 5 or len(x_closed) < fs * 5:
        print('FAIL  Not enough data arrived.')
        return False
    a_open = ssvep.band_power(x_open - np.mean(x_open), fs, 8, 12)
    a_closed = ssvep.band_power(x_closed - np.mean(x_closed), fs, 8, 12)
    ratio = a_closed / a_open if a_open > 0 else 0.0
    print(f'Alpha at Oz: eyes closed / eyes open = {ratio:.2f}')
    if ratio >= 1.5:
        print('PASS  Alpha rises with the eyes closed - the Oz electrode has good contact.')
        return True
    print('FAIL  No clear alpha rise. Add gel, part the hair under the cup, press it on firmly, and try again.\n'
          '      (A few people have weak alpha. If the SSVEP check in the headset passes, that is fine too.)')
    return False


def main():
    if pylsl is None:
        print('FAIL  pylsl is not installed. Run: pip install pylsl')
        sys.exit(1)
    what = sys.argv[1] if len(sys.argv) > 1 else ''
    checks = {'stream': check_stream, 'alpha': check_alpha}
    if what not in checks:
        print(__doc__)
        sys.exit(2)
    sys.exit(0 if checks[what]() else 1)


if __name__ == '__main__':
    main()
