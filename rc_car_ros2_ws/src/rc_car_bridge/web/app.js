// ===========================================================================
// app.js — talks to the bridge node over plain HTTP (no rosbridge).
//   telemetry : EventSource('/events')  (SSE, includes IMU + PID + config)
//   commands  : fetch POST '/cmd'  (ESP32 JSON command objects)
// ===========================================================================
(() => {
  const $ = (id) => document.getElementById(id);
  const log = (m) => { $('log').textContent = m; };
  const MOTORS = [[0, 'LF'], [1, 'LR'], [2, 'RF'], [3, 'RR']];

  function send(obj) {
    return fetch('/cmd', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(obj)
    }).catch((e) => log('send error: ' + e));
  }

  // ---- drive (throttled POSTs; node keepalive fills gaps) ----
  let lastDriveTs = 0, pendingDrive = null;
  function drive(t, s) {
    const T = Math.round(t * 255), S = Math.round(s * 255);
    const now = performance.now();
    if (now - lastDriveTs < 40) { pendingDrive = [T, S]; return; }
    lastDriveTs = now; pendingDrive = null;
    send({ cmd: 'drive', t: T, s: S });
  }
  setInterval(() => {
    if (pendingDrive) { const [T, S] = pendingDrive; pendingDrive = null;
      lastDriveTs = performance.now(); send({ cmd: 'drive', t: T, s: S }); }
  }, 45);

  // ---- build per-motor rows ----
  const motorsEl = $('motors');
  MOTORS.forEach(([idx, name]) => {
    const row = document.createElement('div');
    row.className = 'mrow';
    row.innerHTML =
      `<span class="mname">${name}</span>
       <label class="inv"><input type="checkbox" id="inv${idx}">inv</label>
       <input type="range" id="mt${idx}" min="0" max="150" value="100">
       <span class="mtv" id="mtv${idx}">100%</span>
       <button id="mtest${idx}">Test</button>`;
    motorsEl.appendChild(row);
    let timer = null;
    $(`mt${idx}`).addEventListener('input', (e) => {
      $(`mtv${idx}`).textContent = e.target.value + '%';
      clearTimeout(timer);
      timer = setTimeout(() => send({ cmd: 'motorTrim', m: idx, v: +e.target.value }), 120);
    });
    $(`inv${idx}`).addEventListener('change', (e) =>
      send({ cmd: 'invert', m: idx, v: e.target.checked }));
    $(`mtest${idx}`).addEventListener('click', () =>
      { send({ cmd: 'testMotor', m: idx, pwm: 160, ms: 1500 }); log(`test ${name}`); });
  });

  // ---- config -> populate controls (only when it changes; skip the control
  //      the user is actively editing so we don't fight their drag) ----
  let lastCfg = '';
  const setIf = (el, val) => { if (el && document.activeElement !== el) el.value = val; };
  function applyConfig(c) {
    if (!c) return;
    const key = JSON.stringify(c);
    if (key === lastCfg) return;
    lastCfg = key;
    const map = [['sMax', 'maxSpeed', 'vMax'], ['sTrim', 'trim', 'vTrim'],
                 ['sMin', 'minPwm', 'vMin'], ['sSlew', 'slew', 'vSlew'],
                 ['sDb', 'deadband', 'vDb']];
    map.forEach(([s, k, o]) => {
      if (c[k] != null) { setIf($(s), c[k]); $(o).textContent = c[k]; }
    });
    if (c.kp != null) setIf($('kp'), c.kp);
    if (c.ki != null) setIf($('ki'), c.ki);
    if (c.kd != null) setIf($('kd'), c.kd);
    if (Array.isArray(c.inv)) c.inv.forEach((v, i) => { const e = $(`inv${i}`); if (e) e.checked = !!v; });
    if (Array.isArray(c.mt)) c.mt.forEach((v, i) => {
      setIf($(`mt${i}`), v); const o = $(`mtv${i}`); if (o) o.textContent = v + '%'; });
  }

  // ---- SSE telemetry ----
  const cube = $('cube');
  function connect() {
    const es = new EventSource('/events');
    es.onopen = () => { $('conn').textContent = 'UI: connected'; $('conn').className = 'pill ok'; };
    es.onerror = () => { $('conn').textContent = 'UI: reconnecting…'; $('conn').className = 'pill bad'; };
    es.onmessage = (ev) => {
      let m; try { m = JSON.parse(ev.data); } catch { return; }

      // orientation cube (signs may need flipping to match your IMU mounting)
      cube.style.transform =
        `rotateZ(${m.roll}deg) rotateX(${m.pitch}deg) rotateY(${-m.yaw}deg)`;
      $('roll').textContent  = (+m.roll).toFixed(1);
      $('pitch').textContent = (+m.pitch).toFixed(1);
      $('yaw').textContent   = (+m.yaw).toFixed(1);
      $('gx').textContent = (+m.gx).toFixed(2); $('gy').textContent = (+m.gy).toFixed(2);
      $('gz').textContent = (+m.gz).toFixed(2);
      $('ax').textContent = (+m.ax).toFixed(2); $('ay').textContent = (+m.ay).toFixed(2);
      $('az').textContent = (+m.az).toFixed(2);

      $('batt').textContent = (+m.battery).toFixed(2);
      $('busy').textContent = m.busy ? 'YES' : 'no';
      $('busy').style.color = m.busy ? 'var(--warn)' : 'var(--ink)';
      $('pwmL').textContent = m.pwm_left;  $('pwmR').textContent = m.pwm_right;
      $('age').textContent = (m.age_ms != null && m.age_ms >= 0)
        ? (m.age_ms / 1000).toFixed(1) + 's' : '–';

      // PID
      $('hhState').textContent = m.hh ? 'ON' : 'off';
      $('hhState').style.color = m.hh ? 'var(--acc)' : 'var(--ink)';
      $('pTgt').textContent = (+m.tgt).toFixed(1);
      $('pErr').textContent = (+m.err).toFixed(1);
      $('pOut').textContent = Math.round(+m.out);

      // status pills
      const sp = $('serial');
      sp.textContent = 'serial: ' + (m.serial ? 'open' : 'down');
      sp.className = 'pill ' + (m.serial ? 'ok' : 'bad');
      const ip = $('imuPill');
      ip.textContent = 'IMU: ' + (m.imu ? 'ok' : 'none');
      ip.className = 'pill ' + (m.imu ? 'ok' : 'bad');
      $('imuNote').style.display = m.imu ? 'none' : 'block';

      if (m.config) applyConfig(m.config);
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
    place(dx, dy); drive(-dy / R, dx / R);
  }
  const start = (e) => { dragging = true; fromEvent(e); e.preventDefault(); };
  const move  = (e) => { if (dragging) { fromEvent(e); e.preventDefault(); } };
  const end   = () => { if (dragging) { dragging = false; center(); drive(0, 0); } };
  pad.addEventListener('mousedown', start); window.addEventListener('mousemove', move);
  window.addEventListener('mouseup', end);
  pad.addEventListener('touchstart', start, { passive: false });
  pad.addEventListener('touchmove', move, { passive: false });
  pad.addEventListener('touchend', end);

  // ---- buttons ----
  $('estop').onclick  = () => { send({ cmd: 'stop' }); log('stop'); };
  $('cancel').onclick = () => { send({ cmd: 'cancelAuto' }); log('cancel'); };
  $('saveBtn').onclick = () => { send({ cmd: 'save' }); log('saved to flash'); };
  $('zeroYaw').onclick = () => { send({ cmd: 'zeroYaw' }); log('yaw zeroed'); };
  $('turnBtn').onclick = () => { send({ cmd: 'turnAngle', degrees: +$('turnDeg').value, speed: 160, msPerDegree: 8.0 }); log('turn'); };
  $('moveBtn').onclick = () => { send({ cmd: 'moveDistance', meters: +$('moveM').value, speed: 160, msPerMeter: 1200.0 }); log('move'); };
  $('gotoBtn').onclick = () => { send({ cmd: 'gotoCoord', x: +$('gx').value, y: +$('gy').value, speed: 160, msPerMeter: 1200.0, msPerDegree: 8.0 }); log('goto'); };

  // ---- PID ----
  $('pidSet').onclick = () => { send({ cmd: 'pid', kp: +$('kp').value, ki: +$('ki').value, kd: +$('kd').value }); log('PID gains applied'); };
  $('hhUseCur').onclick = () => { $('hhTarget').value = $('yaw').textContent; };
  $('hhOn').onclick = () => { send({ cmd: 'headingHold', enable: true, target: +$('hhTarget').value, throttle: +$('hhThrottle').value }); log('heading-hold ON'); };
  $('hhOff').onclick = () => { send({ cmd: 'headingHold', enable: false }); log('heading-hold OFF'); };

  // ---- tuning sliders ----
  const bind = (slider, out, cmd) => {
    const el = $(slider), o = $(out); let timer = null;
    el.addEventListener('input', () => {
      o.textContent = el.value;
      clearTimeout(timer);
      timer = setTimeout(() => send({ cmd, v: +el.value }), 120);
    });
  };
  bind('sMax', 'vMax', 'speed'); bind('sTrim', 'vTrim', 'trim');
  bind('sMin', 'vMin', 'minpwm'); bind('sSlew', 'vSlew', 'slew'); bind('sDb', 'vDb', 'deadband');

  // ---- keyboard ----
  const keys = {};
  const kdrive = () => {
    const t = (keys.w || keys.ArrowUp ? 1 : 0) - (keys.s || keys.ArrowDown ? 1 : 0);
    const s = (keys.d || keys.ArrowRight ? 1 : 0) - (keys.a || keys.ArrowLeft ? 1 : 0);
    drive(t * 0.8, s * 0.8);
  };
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT') return;
    if (e.key === ' ') { send({ cmd: 'stop' }); return; }
    keys[e.key] = true; kdrive();
  });
  window.addEventListener('keyup', (e) => { keys[e.key] = false; kdrive(); });

  // request config on load (node also asks on boot)
  send({ cmd: 'getConfig' });
  center();
})();
