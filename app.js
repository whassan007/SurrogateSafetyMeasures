'use strict';
/* ═══════════════════════════════════════════════════════════
   SSAM — Surrogate Safety Assessment Module
   app.js — Physics engine + rendering + telemetry
═══════════════════════════════════════════════════════════ */

(function () {

  /* ── CANVAS SETUP ───────────────────────────────────────── */
  const canvas = document.getElementById('sim-canvas');
  const ctx    = canvas.getContext('2d');

  function resizeCanvas() {
    const area = canvas.parentElement;
    canvas.width  = area.clientWidth;
    canvas.height = area.clientHeight;
    if (!sim.isPlaying) drawFrame();
  }
  window.addEventListener('resize', resizeCanvas);

  /* ── CONFIG ─────────────────────────────────────────────── */
  const SCALE    = 6;    // px per metre
  const VEH_L    = 4.5;  // vehicle length (m)
  const VEH_W    = 2.2;  // vehicle width (m)
  const ZONE_R   = 6;    // conflict zone half-size (m)

  // Colour palette
  const C = {
    bg:       '#080f1c',
    grid:     'rgba(20,50,90,0.5)',
    road:     '#101e35',
    roadEdge: 'rgba(30,70,130,0.4)',
    axis:     'rgba(30,70,130,0.7)',
    zone:     'rgba(255,140,0,0.06)',
    zoneEdge: 'rgba(255,140,0,0.20)',
    egoBody:  '#00d4ff',
    egoGlow:  'rgba(0,212,255,0.35)',
    tgtBody:  '#cc2020',
    tgtGlow:  'rgba(204,32,32,0.35)',
    label:    'rgba(180,220,255,0.55)',
    crosshair:'rgba(0,180,255,0.15)',
  };

  /* ── SIMULATION STATE ───────────────────────────────────── */
  const sim = {
    // ── config (read from sliders) ──
    vEgoInit    : 15,
    vTgtInit    : 12,
    brakeRate   : 8,
    hardBrakeTgt: 6,
    ttcThreshold: 1.5,

    // ── vehicle state ──
    ego : { x:0, y:0, vx:0, vy:0, ax:0 },
    tgt : { x:0, y:0, vx:0, vy:0, ay:0 },

    // ── time & control ──
    t          : 0,
    isPlaying  : false,
    animId     : null,
    lastTs     : 0,
    simSpeed   : 1.0,

    // ── metrics ──
    curTTC     : Infinity,
    minTTC     : Infinity,
    maxSpeed   : 0,
    deltaV     : 0,
    pet        : null,

    // ── conflict flags ──
    conflictDetected   : false,
    conflictTime       : null,
    brakeSpeedAtConflict: null,

    // ── zone tracking (for PET) ──
    egoZoneEntry   : null,
    egoZoneExit    : null,
    tgtZoneEntry   : null,
    tgtZoneExit    : null,
    petComputed    : false,

    // ── trajectory history (for trail) ──
    egoTrail : [],
    tgtTrail : [],
  };

  /* ── INITIALISE SCENARIO ────────────────────────────────── */
  function initScenario() {
    const s = sim;

    s.t = 0;
    s.curTTC           = Infinity;
    s.minTTC           = Infinity;
    s.pet              = null;
    s.conflictDetected = false;
    s.conflictTime     = null;
    s.brakeSpeedAtConflict = null;
    s.egoZoneEntry = s.egoZoneExit = s.tgtZoneEntry = s.tgtZoneExit = null;
    s.petComputed  = false;
    s.egoTrail = [];
    s.tgtTrail = [];

    // Compute initial positions so both arrive at conflict point (0,0)
    // with a 0.08 s gap → target arrives first (violates ROW)
    const T_EGO = 5.08;
    const T_TGT = 5.00;

    // Ego: moves +x, fixed y = -2 m
    s.ego.x  = -(s.vEgoInit * T_EGO);
    s.ego.y  = -2;
    s.ego.vx =  s.vEgoInit;
    s.ego.vy =  0;
    s.ego.ax =  0;

    // Target: moves +y, fixed x = 2 m
    s.tgt.x  =  2;
    s.tgt.y  = -(s.vTgtInit * T_TGT);
    s.tgt.vx =  0;
    s.tgt.vy =  s.vTgtInit;
    s.tgt.ay =  0;

    s.maxSpeed = Math.max(s.vEgoInit, s.vTgtInit);
    s.deltaV   = Math.sqrt(s.vEgoInit ** 2 + s.vTgtInit ** 2);

    updateSliderDisplays();
    resetTelemetryUI();
    drawFrame();
  }

  /* ── PHYSICS UPDATE ─────────────────────────────────────── */
  function update(dt) {
    const s   = sim;
    const e   = s.ego;
    const tgt = s.tgt;
    s.t += dt;

    // ── Move ego ──
    e.vx = Math.max(0, e.vx + e.ax * dt);
    e.x += e.vx * dt;

    // ── Move target ──
    tgt.vy = Math.max(0, tgt.vy + tgt.ay * dt);
    tgt.y  += tgt.vy * dt;

    // ── Compute instantaneous TTC ──
    const tE = (e.vx  > 0.01) ? (0 - e.x)   / e.vx   : Infinity;
    const tT = (tgt.vy > 0.01) ? (0 - tgt.y) / tgt.vy : Infinity;

    let ttc = Infinity;
    if (tE > 0 && tE < 20 && tT > 0 && tT < 20) {
      ttc = Math.abs(tE - tT);
    }
    s.curTTC = ttc;
    if (ttc < s.minTTC) s.minTTC = ttc;

    // ── Conflict detection ──
    if (ttc < s.ttcThreshold && !s.conflictDetected) {
      s.conflictDetected      = true;
      s.conflictTime          = s.t;
      s.brakeSpeedAtConflict  = e.vx;
      e.ax = -s.brakeRate;     // ego applies evasive braking
    }

    // ── Zone entry / exit (for PET) ──
    const halfL = VEH_L / 2;
    const eFrontX = e.x   + halfL;
    const eRearX  = e.x   - halfL;
    const tFrontY = tgt.y + halfL;
    const tRearY  = tgt.y - halfL;

    if (s.egoZoneEntry === null && eFrontX >= -ZONE_R) s.egoZoneEntry = s.t;
    if (s.egoZoneExit  === null && eRearX  >=  ZONE_R) s.egoZoneExit  = s.t;
    if (s.tgtZoneEntry === null && tFrontY >= -ZONE_R) s.tgtZoneEntry = s.t;
    if (s.tgtZoneExit  === null && tRearY  >=  ZONE_R) s.tgtZoneExit  = s.t;

    // ── Compute PET once both have passed through ──
    if (!s.petComputed && s.egoZoneExit !== null && s.tgtZoneEntry !== null) {
      s.pet = s.tgtZoneEntry - s.egoZoneExit;
      s.petComputed = true;
    }
    if (!s.petComputed && s.tgtZoneExit !== null && s.egoZoneEntry !== null) {
      s.pet = s.egoZoneEntry - s.tgtZoneExit;
      s.petComputed = true;
    }

    // ── Trail history ──
    if (s.t % 0.1 < dt * 2) {
      s.egoTrail.push({ x: e.x, y: e.y });
      s.tgtTrail.push({ x: tgt.x, y: tgt.y });
      if (s.egoTrail.length > 60) s.egoTrail.shift();
      if (s.tgtTrail.length > 60) s.tgtTrail.shift();
    }

    // ── Max speed ──
    s.maxSpeed = Math.max(s.maxSpeed, e.vx, tgt.vy);

    // ── Stop condition ──
    if (s.t > 14 || (e.x > 200 && tgt.y > 200)) {
      stopSim();
      showComplianceReport();
    }
  }

  /* ── DRAW ───────────────────────────────────────────────── */
  function drawFrame() {
    const W  = canvas.width;
    const H  = canvas.height;
    const CX = W / 2;
    const CY = H / 2;
    const s  = sim;

    // ── Background ──
    ctx.fillStyle = C.bg;
    ctx.fillRect(0, 0, W, H);

    // ── Grid (dot pattern) ──
    drawGrid(W, H, CX, CY);

    // ── Roads ──
    const rw = 24 * SCALE / 4; // road half-width in px  (road = ~12m wide)
    drawRoads(CX, CY, rw, W, H);

    // ── Conflict zone ──
    const zr = ZONE_R * SCALE;
    ctx.fillStyle = C.zone;
    ctx.strokeStyle = C.zoneEdge;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(CX - zr, CY - zr, zr * 2, zr * 2);
    ctx.fill();
    ctx.stroke();

    // ── Trails ──
    drawTrail(s.egoTrail, C.egoGlow, CX, CY);
    drawTrail(s.tgtTrail, 'rgba(204,32,32,0.2)', CX, CY);

    // ── Vehicles ──
    drawVehicle(s.ego, true,  CX, CY);
    drawVehicle(s.tgt, false, CX, CY);

    // ── TTC threat arc between vehicles ──
    if (s.curTTC < s.ttcThreshold && s.isPlaying) {
      drawThreatLine(s.ego, s.tgt, s.curTTC, CX, CY);
    }

    // ── HUD labels ──
    drawHUD(CX, CY, zr);
  }

  function worldToCanvas(wx, wy, CX, CY) {
    return { x: CX + wx * SCALE, y: CY - wy * SCALE };
  }

  function drawGrid(W, H, CX, CY) {
    const GRID_M = 10; // grid spacing in metres
    const GRID_PX = GRID_M * SCALE;
    ctx.fillStyle = C.grid;
    for (let x = (CX % GRID_PX); x < W; x += GRID_PX) {
      for (let y = (CY % GRID_PX); y < H; y += GRID_PX) {
        ctx.fillRect(x - 0.5, y - 0.5, 1.5, 1.5);
      }
    }
  }

  function drawRoads(CX, CY, rw, W, H) {
    // Road fill
    ctx.fillStyle = C.road;
    ctx.fillRect(0, CY - rw, W, rw * 2);       // horizontal
    ctx.fillRect(CX - rw, 0, rw * 2, H);        // vertical

    // Road edges
    ctx.strokeStyle = C.roadEdge;
    ctx.lineWidth = 1;
    ctx.setLineDash([]);

    // Horizontal road edges (top and bottom)
    ctx.beginPath();
    ctx.moveTo(0, CY - rw);  ctx.lineTo(W, CY - rw);
    ctx.moveTo(0, CY + rw);  ctx.lineTo(W, CY + rw);
    ctx.stroke();

    // Vertical road edges (left and right)
    ctx.beginPath();
    ctx.moveTo(CX - rw, 0);  ctx.lineTo(CX - rw, H);
    ctx.moveTo(CX + rw, 0);  ctx.lineTo(CX + rw, H);
    ctx.stroke();

    // Centre-line dashes
    ctx.strokeStyle = 'rgba(255,255,180,0.12)';
    ctx.setLineDash([12, 10]);
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.moveTo(0, CY);     ctx.lineTo(W, CY);
    ctx.moveTo(CX, 0);     ctx.lineTo(CX, H);
    ctx.stroke();
    ctx.setLineDash([]);

    // Axis crosshair
    ctx.strokeStyle = C.crosshair;
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(CX, 0);   ctx.lineTo(CX, H);
    ctx.moveTo(0, CY);   ctx.lineTo(W, CY);
    ctx.stroke();
  }

  function drawTrail(trail, color, CX, CY) {
    if (trail.length < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.setLineDash([3, 5]);
    ctx.beginPath();
    trail.forEach((pt, i) => {
      const p = worldToCanvas(pt.x, pt.y, CX, CY);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else         ctx.lineTo(p.x, p.y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function drawVehicle(v, isEgo, CX, CY) {
    const p = worldToCanvas(v.x, v.y, CX, CY);
    const angle = isEgo ? 0 : -Math.PI / 2;  // ego goes right, target goes up
    const lPx = VEH_L * SCALE;
    const wPx = VEH_W * SCALE;

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(angle);

    // Glow
    ctx.shadowColor  = isEgo ? C.egoGlow : C.tgtGlow;
    ctx.shadowBlur   = 18;

    // Body
    ctx.fillStyle = isEgo ? C.egoBody : C.tgtBody;
    ctx.fillRect(-lPx / 2, -wPx / 2, lPx, wPx);

    // Front indicator (bright strip)
    ctx.fillStyle = 'rgba(255,255,255,0.5)';
    ctx.fillRect(lPx / 2 - 4, -wPx / 2 + 2, 3, wPx - 4);

    // Roof
    ctx.fillStyle = 'rgba(0,0,0,0.3)';
    ctx.fillRect(-lPx / 2 + 5, -wPx / 2 + 2, lPx - 10, wPx - 4);

    ctx.shadowBlur = 0;

    // Label
    ctx.fillStyle = isEgo ? 'rgba(0,212,255,0.85)' : 'rgba(255,80,80,0.85)';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(isEgo ? 'EGO' : 'TGT', 0, -wPx / 2 - 5);

    ctx.restore();
  }

  function drawThreatLine(ego, tgt, ttc, CX, CY) {
    const pE = worldToCanvas(ego.x, ego.y, CX, CY);
    const pT = worldToCanvas(tgt.x, tgt.y, CX, CY);
    const alpha = Math.min(1, 1.2 - ttc / sim.ttcThreshold);

    ctx.strokeStyle = `rgba(255,59,59,${(alpha * 0.7).toFixed(2)})`;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(pE.x, pE.y);
    ctx.lineTo(pT.x, pT.y);
    ctx.stroke();
    ctx.setLineDash([]);

    // TTC label midpoint
    const mx = (pE.x + pT.x) / 2;
    const my = (pE.y + pT.y) / 2;
    ctx.fillStyle = '#ff5555';
    ctx.font = 'bold 10px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`TTC ${ttc.toFixed(2)}s`, mx, my - 6);
  }

  function drawHUD(CX, CY, zr) {
    // Conflict zone label
    ctx.fillStyle = 'rgba(255,140,0,0.4)';
    ctx.font = '8px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('CONFLICT ZONE', CX, CY - zr - 5);

    // Origin crosshair dot
    ctx.fillStyle = 'rgba(0,180,255,0.25)';
    ctx.beginPath();
    ctx.arc(CX, CY, 3, 0, Math.PI * 2);
    ctx.fill();
  }

  /* ── TELEMETRY UI UPDATE ────────────────────────────────── */
  function updateTelemetry() {
    const s   = sim;
    const e   = s.ego;
    const tgt = s.tgt;

    // Status bar
    const tStr   = s.t.toFixed(1);
    const ttcStr = s.curTTC < 10 ? s.curTTC.toFixed(2) + 's' : '10.00s';

    document.getElementById('status-time').textContent = `T=${tStr}s`;
    document.getElementById('status-ttc').textContent  = `TTC=${ttcStr}`;

    const lbl = document.getElementById('status-label');
    if (!s.conflictDetected) {
      lbl.textContent = '[NOMINAL]';
      lbl.className   = 'status-chip nominal';
    } else if (s.curTTC < s.ttcThreshold && s.curTTC < 10) {
      lbl.textContent = '[CONFLICT]';
      lbl.className   = 'status-chip conflict';
    } else {
      lbl.textContent = '[NOMINAL]';
      lbl.className   = 'status-chip nominal';
    }

    // Ego telemetry
    document.getElementById('tele-ego-v').textContent = e.vx.toFixed(2);
    document.getElementById('tele-ego-x').textContent = e.x.toFixed(1);
    document.getElementById('tele-ego-y').textContent = e.y.toFixed(1);
    document.getElementById('tele-ego-a').textContent = e.ax.toFixed(2);

    // Target telemetry
    document.getElementById('tele-tgt-v').textContent = tgt.vy.toFixed(2);
    document.getElementById('tele-tgt-x').textContent = tgt.x.toFixed(1);
    document.getElementById('tele-tgt-y').textContent = tgt.y.toFixed(1);
    document.getElementById('tele-tgt-a').textContent = tgt.ay.toFixed(2);

    // SSM
    const ttcDisp = (s.curTTC < 10) ? s.curTTC.toFixed(2) + ' s' : '—';
    const minDisp = isFinite(s.minTTC) ? s.minTTC.toFixed(2) + ' s' : '—';
    document.getElementById('ssm-cur-ttc').textContent = ttcDisp;
    document.getElementById('ssm-min-ttc').textContent = minDisp;

    // Conflict bar
    const bar = document.getElementById('conflict-bar');
    const txt = document.getElementById('conflict-status-txt');
    if (s.conflictDetected && s.curTTC < s.ttcThreshold && s.curTTC < 10) {
      bar.classList.add('active');
      txt.textContent = '● CONFLICT';
      txt.className = 'conflict-txt conflict-active';
    } else {
      bar.classList.remove('active');
      txt.textContent = '● NOMINAL';
      txt.className = 'conflict-txt nominal';
    }

    // Detail cards
    document.getElementById('det-pet').textContent =
      (s.pet !== null) ? s.pet.toFixed(2) + ' s' : '—';
    document.getElementById('det-brake').textContent =
      s.conflictDetected ? s.brakeRate.toFixed(2) + ' m/s²' : '—';
    document.getElementById('det-maxspeed').textContent =
      s.maxSpeed.toFixed(2) + ' m/s';
    document.getElementById('det-deltav').textContent =
      s.deltaV.toFixed(2) + ' m/s';
  }

  function resetTelemetryUI() {
    ['tele-ego-v','tele-ego-x','tele-ego-y','tele-ego-a',
     'tele-tgt-v','tele-tgt-x','tele-tgt-y','tele-tgt-a'].forEach(id => {
      document.getElementById(id).textContent = '—';
    });
    document.getElementById('ssm-cur-ttc').textContent = '—';
    document.getElementById('ssm-min-ttc').textContent = '—';
    document.getElementById('det-pet').textContent     = '—';
    document.getElementById('det-brake').textContent   = '—';
    document.getElementById('det-maxspeed').textContent = '—';
    document.getElementById('det-deltav').textContent  = '—';

    document.getElementById('status-time').textContent  = 'T=0.0s';
    document.getElementById('status-ttc').textContent   = 'TTC=—';

    const lbl = document.getElementById('status-label');
    lbl.textContent = '[READY]';
    lbl.className   = 'status-chip nominal';

    document.getElementById('conflict-bar').classList.remove('active');
    document.getElementById('conflict-status-txt').textContent = '● NOMINAL';
    document.getElementById('conflict-status-txt').className = 'conflict-txt nominal';

    document.getElementById('compliance-report').style.display = 'none';
  }

  function showComplianceReport() {
    const s   = sim;
    const rep = document.getElementById('compliance-report');
    const txt = document.getElementById('report-text');
    rep.style.display = 'block';

    if (s.conflictDetected) {
      const minTTCStr  = isFinite(s.minTTC) ? s.minTTC.toFixed(2) : '—';
      const petStr     = (s.pet !== null) ? s.pet.toFixed(2) + 's' : 'N/A';
      txt.className    = 'report-body';
      txt.innerHTML    =
        `VALID CONFLICT DETECTED | ` +
        `Min TTC = ${minTTCStr}s (threshold: ${s.ttcThreshold}s) | ` +
        `Evasive braking DR = ${s.brakeRate.toFixed(2)} m/s² | ` +
        `PET = ${petStr} | ` +
        `FHWA Classification: <strong>CONFIRMED</strong>`;
    } else {
      txt.className    = 'report-body nominal-report';
      txt.innerHTML    =
        `NO CONFLICT DETECTED | Min TTC = ${s.minTTC.toFixed(2)}s (threshold: ${s.ttcThreshold}s) | ` +
        `FHWA Classification: <strong>PASS — NOMINAL</strong>`;
    }
  }

  /* ── ANIMATION LOOP ─────────────────────────────────────── */
  function loop(ts) {
    if (!sim.isPlaying) return;
    const rawDt = Math.min((ts - sim.lastTs) / 1000, 0.05);
    const dt    = rawDt * sim.simSpeed;
    sim.lastTs  = ts;

    update(dt);
    drawFrame();
    updateTelemetry();

    sim.animId = requestAnimationFrame(loop);
  }

  function startSim() {
    if (sim.isPlaying) return;
    sim.isPlaying = true;
    sim.lastTs    = performance.now();
    document.getElementById('btn-play').textContent = '⏸  PAUSE';
    sim.animId = requestAnimationFrame(loop);
  }

  function pauseSim() {
    sim.isPlaying = false;
    cancelAnimationFrame(sim.animId);
    document.getElementById('btn-play').textContent = '▶  PLAY';
  }

  function stopSim() {
    sim.isPlaying = false;
    cancelAnimationFrame(sim.animId);
    document.getElementById('btn-play').textContent = '▶  PLAY';
  }

  function resetSim() {
    stopSim();
    readSliders();
    initScenario();
  }

  /* ── SLIDER BINDINGS ────────────────────────────────────── */
  function readSliders() {
    sim.vEgoInit     = parseFloat(document.getElementById('slider-v-ego').value);
    sim.vTgtInit     = parseFloat(document.getElementById('slider-v-target').value);
    sim.brakeRate    = Math.abs(parseFloat(document.getElementById('slider-brake').value));
    sim.ttcThreshold = parseFloat(document.getElementById('slider-ttc').value);
  }

  function updateSliderDisplays() {
    document.getElementById('val-v-ego').textContent    = sim.vEgoInit.toFixed(1);
    document.getElementById('val-v-target').textContent = sim.vTgtInit.toFixed(1);
    document.getElementById('val-brake').textContent    = (-sim.brakeRate).toFixed(1);
    document.getElementById('val-ttc').textContent      = sim.ttcThreshold.toFixed(2);
  }

  function bindSlider(id, key, factor) {
    document.getElementById(id).addEventListener('input', function () {
      sim[key] = factor ? Math.abs(parseFloat(this.value)) : parseFloat(this.value);
      updateSliderDisplays();
      if (!sim.isPlaying) { readSliders(); initScenario(); }
    });
  }

  bindSlider('slider-v-ego',    'vEgoInit',     false);
  bindSlider('slider-v-target', 'vTgtInit',     false);
  bindSlider('slider-brake',    'brakeRate',    true);
  bindSlider('slider-ttc',      'ttcThreshold', false);

  /* ── BUTTON HANDLERS ────────────────────────────────────── */
  document.getElementById('btn-play').addEventListener('click', () => {
    if (sim.isPlaying) { pauseSim(); }
    else               { readSliders(); if (sim.t === 0) initScenario(); startSim(); }
  });

  document.getElementById('btn-stop').addEventListener('click', () => {
    resetSim();
  });

  document.getElementById('scenario-type').addEventListener('change', () => {
    resetSim();
  });

  /* ── BOOT ───────────────────────────────────────────────── */
  resizeCanvas();
  readSliders();
  initScenario();

})();
