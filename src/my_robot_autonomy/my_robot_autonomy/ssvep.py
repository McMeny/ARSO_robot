"""SSVEP signal processing for one EEG channel (Oz). Used by eeg_decoder and eeg_check.

score(f) = response at f (and its 2nd harmonic) / the noise floor right beside it.
1 = noise level. A tagged hazard's frequency is also compared against frequencies that
are NOT being shown, so a general rise in EEG power doesn't look like a response.
"""
import numpy as np
from scipy.signal import butter, iirnotch, sosfiltfilt, filtfilt

NEIGHBOUR_OFFSETS = (-1.25, -1.0, -0.75, 0.75, 1.0, 1.25)  # Hz either side, for the noise floor


def preprocess(x, fs, line_hz=60.0):
    """Remove the slow drift, keep 5-40 Hz, and notch out mains hum."""
    x = np.asarray(x, dtype=float)
    x = x - np.mean(x)
    sos = butter(4, [5.0, 40.0], btype='bandpass', fs=fs, output='sos')
    x = sosfiltfilt(sos, x)
    if line_hz < fs / 2:
        b, a = iirnotch(line_hz, 30.0, fs)
        x = filtfilt(b, a, x)
    return x


def power_at(x, fs, f):
    """Power of x at exactly f Hz (Hann-windowed projection - f doesn't need to sit on an FFT bin)."""
    n = len(x)
    t = np.arange(n) / fs
    w = np.hanning(n)
    z = np.sum(x * w * np.exp(-2j * np.pi * f * t))
    return float(np.abs(z) ** 2) / float(np.sum(w ** 2))


def score(x, fs, f, exclude=(), harmonics=2):
    """Response at f and its harmonics, relative to the noise floor beside each one."""
    signal, noise = 0.0, 0.0
    for h in range(1, harmonics + 1):
        fh = f * h
        if fh >= fs / 2 - 2:
            break
        signal += power_at(x, fs, fh)
        refs = [fh + o for o in NEIGHBOUR_OFFSETS
                if all(abs(fh + o - e * k) > 0.3 for e in exclude for k in (1, 2))]
        noise += np.mean([power_at(x, fs, r) for r in refs]) if refs else 0.0
    return signal / noise if noise > 0 else 0.0


def quality(raw, fs):
    """Is the Oz signal usable?  Returns (label, standard deviation in uV)."""
    raw = np.asarray(raw, dtype=float)
    sd = float(np.std(raw))
    if sd < 0.5:
        return 'FLAT', sd            # electrode off or not connected
    if np.max(np.abs(raw - np.mean(raw))) > 1000:
        return 'SATURATED', sd
    if sd > 150:
        return 'NOISY', sd           # movement, loose contact
    spectrum = np.abs(np.fft.rfft((raw - np.mean(raw)) * np.hanning(len(raw)))) ** 2
    freqs = np.fft.rfftfreq(len(raw), 1 / fs)
    band = spectrum[(freqs > 5) & (freqs < 40)].mean()
    line = spectrum[(freqs > 58) & (freqs < 62)].mean() if fs > 124 else 0.0
    if band > 0 and line / band > 50:
        return 'LINE_NOISE', sd      # mains hum swamping the signal - check the reference/ground
    return 'OK', sd


def band_power(x, fs, lo, hi):
    spectrum = np.abs(np.fft.rfft(np.asarray(x) * np.hanning(len(x)))) ** 2
    freqs = np.fft.rfftfreq(len(x), 1 / fs)
    return float(spectrum[(freqs >= lo) & (freqs <= hi)].mean())
