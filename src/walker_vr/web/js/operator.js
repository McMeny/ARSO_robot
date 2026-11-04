// The operator page: start/stop walking (the VR build's stand-in for the grip), flicker on/off,
// recentre the participant, run the SSVEP check, and watch the EEG and every hazard.
import { connect } from './common.js';

const $ = (id) => document.getElementById(id);
const pill = (text, kind) => `<span class="pill ${kind}">${text}</span>`;
const rows = (pairs) => pairs.map(([k, v]) => `<span class="k">${k}</span><span>${v}</span>`).join('');

let enabled = false, flicker = true, walker = {}, eeg = {}, headset = null, headsetTime = 0;

const link = connect({
  walker_enabled: (m) => { enabled = m.value; render(); },
  flicker_enabled: (m) => { flicker = m.value; render(); },
  walker_status: (m) => { walker = m; render(); },
  eeg_status: (m) => { eeg = m; render(); },
  headset_status: (m) => { headset = m; headsetTime = Date.now(); render(); },
  hazards: (m) => renderHazards(m.hazards),
}, (ok) => {
  $('conn').className = `pill ${ok ? 'ok' : 'bad'}`;
  $('conn').textContent = ok ? 'connected' : 'disconnected';
});

$('start').onclick = () => link.send({ type: 'walker_enabled', value: true });
$('stop').onclick = () => link.send({ type: 'walker_enabled', value: false });
$('flicker-off').onclick = () => link.send({ type: 'flicker_enabled', value: false });
$('flicker-on').onclick = () => link.send({ type: 'flicker_enabled', value: true });
$('recentre').onclick = () => link.send({ type: 'command', name: 'recentre' });
$('check').onclick = () => link.send({ type: 'command', name: 'ssvep_check' });

function render() {
  const state = walker.state || '–';
  const kind = { NORMAL: 'ok', PROBING: 'ok', CAREFUL: 'warn', STOPPED: 'bad' }[state] || 'warn';
  $('walker').innerHTML = rows([
    ['Walking', enabled ? pill('yes', 'ok') : pill('stopped', 'bad')],
    ['State', pill(state, kind)],
    ['Speed cap', walker.speed_cap_percent !== undefined ? `${walker.speed_cap_percent}%` : '–'],
    ['Extra clearance', walker.extra_clearance ? `+${walker.extra_clearance} m` : 'none'],
    ['Steering', walker.machine_authority !== undefined
      ? `head ${Math.round(100 * (1 - walker.machine_authority))}% · machine ${Math.round(100 * walker.machine_authority)}%` : '–'],
  ]);

  const live = headset && Date.now() - headsetTime < 5000;
  const fpsKind = live && headset.mode === 'vr' && Math.abs(headset.fps - 90) > 5 ? 'warn' : 'ok';
  $('headset').innerHTML = rows([
    ['Flicker', flicker ? pill('allowed', 'ok') : pill('OFF', 'bad')],
    ['Headset', live ? pill(headset.mode === 'vr' ? 'in VR' : 'desktop view', 'ok') : pill('not connected', 'warn')],
    ['Refresh rate', live ? `${pill(`${headset.fps} fps`, fpsKind)} ${fpsKind === 'warn' ? '(flicker is tuned for 90)' : ''}` : '–'],
  ]);

  const streamKind = { OK: 'ok', POOR: 'warn' }[eeg.stream] || 'bad';
  $('eeg').innerHTML = rows([
    ['Stream', pill(eeg.stream || 'waiting', streamKind)],
    ['Oz signal', eeg.quality ? `${eeg.quality}, ${eeg.oz_sd_uv} µV` : '–'],
    ['Sample rate', eeg.sample_rate ? `${eeg.sample_rate} Hz` : '–'],
    ['Threshold', eeg.threshold ?? '–'],
    ['Source', eeg.source || '–'],
  ]);
  const c = eeg.check || {};
  if (c.state === 'running') $('check-result').textContent = `SSVEP check running… ${c.step || ''}`;
  else if (c.state === 'pass' || c.state === 'fail') {
    const d = (c.details || []).map((x) => `${x.freq} Hz: on ${x.on} / off ${x.off} ${x.pass ? '✓' : '✗'}`).join(' · ');
    $('check-result').innerHTML = `${pill(c.state.toUpperCase(), c.state === 'pass' ? 'ok' : 'bad')} ${d} · threshold ${c.threshold}` +
      (c.hint ? `<br>${c.hint}` : '');
  } else $('check-result').textContent = 'Not run yet.';
}

function renderHazards(hazards) {
  if (!hazards.length) return;
  const kind = { waiting: '', probing: 'ok', registered: 'ok', missed: 'bad', passed: '' };
  const list = [...hazards].sort((a, b) => (b.relevant - a.relevant) || (a.path_distance - b.path_distance));
  $('hazards').innerHTML = list.map((h) => `<tr>
    <td>${h.id}</td><td>${h.name}</td>
    <td>${pill(h.state, kind[h.state] || 'warn')}${h.relevant ? '' : ' <span class="muted">(not on path)</span>'}</td>
    <td>${h.path_distance < 50 ? h.path_distance.toFixed(1) + ' m' : '–'}</td>
    <td>${h.lateral < 50 ? h.lateral.toFixed(2) + ' m' : '–'}</td>
    <td>${h.frequency > 0 ? h.frequency.toFixed(2) + ' Hz' : '–'}</td>
    <td>${h.confidence ? h.confidence.toFixed(1) : '–'}</td></tr>`).join('');
}
render();
