// ===========================================================================
// app.js — talks DIRECTLY to the bridge node over plain HTTP (no rosbridge).
//   * telemetry  : EventSource('/events')  (Server-Sent Events, 10 Hz push)
//   * commands   : fetch POST '/cmd' with an ESP32 JSON command object
// Same page is served by the bridge node, so requests are same-origin.
// ===========================================================================
(() => {
  const $ = (id) => document.getElementById(id);
  const log = (m) => { $('log').textContent = m; };

  // ---- command sender ----
  function send(obj) {
    return fetch('/cmd', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(obj)
    }).catch((e) => log('send error: ' + e));
  }
  // throttle drive POSTs to ~25 Hz; node keepalive (20 Hz) covers the gaps
  let lastDriveTs = 0, pendingDrive = null;
  function drive(t, s) {              // t,s normalised -1..1
    const T = Math.round(t * 255), S = Math.round(s * 255);
    const now = performance.now();
    if (now - lastDriveTs < 40) { pendingDrive = [T, S]; return; }
    lastDriveTs = now; pendingDrive = null;
    send({ cmd: 'drive', t: T, s: S });
  }
  setInterval(() => {                 // flush any coalesced drive
    if (pendingDrive) { const [T, S] = pendingDrive; pendingDrive = null;
      lastDriveTs = performance.now(); send({ cmd: 'drive', t: T, s: S }); }
  }, 45);

  // ---- telemetry stream (SSE) ----
  function connect() {
    const es = new EventSource('/events');
    es.onopen = () => { $('conn').textContent = 'UI: connected'; $('conn').className = 'pill ok'; };
    es.onerror = () => {
      $('conn').textContent = 'UI: reconnecting…'; $('conn').className = 'pill bad';
      // EventSource auto-reconnects; nothing else to do
    };
    es.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }
      $('roll').textContent  = (+m.roll).toFixed(1);
      $('pitch').textContent = (+m.pitch).toFixed(1);
      $('batt').textContent  = (+m.battery).toFixed(2);
      $('busy').textContent  = m.busy ? 'YES' : 'no';
      $('busy').style.color  = m.busy ? 'var(--warn)' : 'var(--ink)';
      $('pwmL').textContent  = m.pwm_left;
      $('pwmR').textContent  = m.pwm_right;
      $('age').textContent   = (m.age_ms != null && m.age_ms >= 0)
        ? (m.age_ms / 1000).toFixed(1) + 's' : '–';
      const sp = $('serial');
      sp.textContent = 'serial: ' + (m.serial ? 'open' : 'down');
      sp.className = 'pill ' + (m.serial ? 'ok' : 'bad');
      const h = $('horizon').querySelector('.hline');
      h.style.transform =
        `translateY(${Math.max(-50, Math.min(50, m.pitch)) * 0.6}px) rotate(${-m.roll}deg)`;
    };
  }
  connect();

  // ---- joystick ----
  const pad = $('pad'), stick = $('stick'), R = 90;
  let dragging = false;
  const center = () => { stick.style.left = 'calc(50% - 35px)'; stick.style.top = 'calc(50% - 35px)'; };
  const place = (dx, dy) => { stick.style.left = `calc(50% - 35px + ${dx}px)`; stick.style.top = `calc(50% - 35px + ${dy}px)`; };
  function fromEvent(e) {
    const r = pad.getBoundingClientRect();
    const p = e.touches ? e.touches[0] : e;
    let dx = p.clientX - (r.left + r.width / 2);
    let dy = p.clientY - (r.top + r.height / 2);
    const mag = Math.hypot(dx, dy);
    if (mag > R) { dx = dx / mag * R; dy = dy / mag * R; }
    place(dx, dy);
    drive(-dy / R, dx / R);
  }
  const start = (e) => { dragging = true; fromEvent(e); e.preventDefault(); };
  const move  = (e) => { if (dragging) { fromEvent(e); e.preventDefault(); } };
  const end   = () => { if (dragging) { dragging = false; center(); drive(0, 0); } };
  pad.addEventListener('mousedown', start);
  window.addEventListener('mousemove', move);
  window.addEventListener('mouseup', end);
  pad.addEventListener('touchstart', start, { passive: false });
  pad.addEventListener('touchmove', move, { passive: false });
  pad.addEventListener('touchend', end);

  // ---- buttons ----
  $('estop').onclick  = () => { send({ cmd: 'stop' }); log('stop'); };
  $('cancel').onclick = () => { send({ cmd: 'cancelAuto' }); log('cancel'); };
  $('saveBtn').onclick = () => { send({ cmd: 'save' }); log('saved to flash'); };
  $('turnBtn').onclick = () => { send({ cmd: 'turnAngle', degrees: +$('turnDeg').value, speed: 160, msPerDegree: 8.0 }); log('turn'); };
  $('moveBtn').onclick = () => { send({ cmd: 'moveDistance', meters: +$('moveM').value, speed: 160, msPerMeter: 1200.0 }); log('move'); };
  $('gotoBtn').onclick = () => { send({ cmd: 'gotoCoord', x: +$('gx').value, y: +$('gy').value, speed: 160, msPerMeter: 1200.0, msPerDegree: 8.0 }); log('goto'); };
  $('testBtn').onclick = () => { send({ cmd: 'testMotor', m: +$('tMotor').value, pwm: +$('tPwm').value, ms: +$('tMs').value }); log('motor test'); };

  // ---- tuning sliders (debounced) ----
  const bind = (slider, out, cmd) => {
    const el = $(slider), o = $(out); let timer = null;
    el.addEventListener('input', () => {
      o.textContent = el.value;
      clearTimeout(timer);
      timer = setTimeout(() => send({ cmd, v: +el.value }), 120);
    });
  };
  bind('sMax', 'vMax', 'speed');
  bind('sTrim', 'vTrim', 'trim');
  bind('sMin', 'vMin', 'minpwm');
  bind('sSlew', 'vSlew', 'slew');
  bind('sDb', 'vDb', 'deadband');

  // ---- keyboard ----
  const keys = {};
  const kdrive = () => {
    const t = (keys.w || keys.ArrowUp ? 1 : 0) - (keys.s || keys.ArrowDown ? 1 : 0);
    const s = (keys.d || keys.ArrowRight ? 1 : 0) - (keys.a || keys.ArrowLeft ? 1 : 0);
    drive(t * 0.8, s * 0.8);
  };
  window.addEventListener('keydown', (e) => {
    if (e.key === ' ') { send({ cmd: 'stop' }); return; }
    keys[e.key] = true; kdrive();
  });
  window.addEventListener('keyup', (e) => { keys[e.key] = false; kdrive(); });

  center();
})();
