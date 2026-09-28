(function () {
  'use strict';

  var canvas = document.getElementById('preview');
  var ctx = canvas.getContext('2d');
  var warnEl = document.getElementById('warn');
  var readoutEl = document.getElementById('readout');
  var metricsEl = document.getElementById('metrics');
  var citationEl = document.getElementById('citation');
  var exportBtn = document.getElementById('exportBtn');
  var detailEl = document.getElementById('detailReadout');
  var measureBtn = document.getElementById('measureBtn');
  var playBtn = document.getElementById('playBtn');
  var speedRange = document.getElementById('speedRange');
  var hintEl = document.getElementById('hint');
  var defaultHint = hintEl.textContent;
  var waveGenToggle = document.getElementById('waveGenToggle');
  var autoHeightToggle = document.getElementById('autoHeightToggle');
  var ratioEl = document.getElementById('ratio');
  var linkBtn = document.getElementById('linkBtn');

  var style = 'cup';
  var profile = 'cycloid';
  var CTP = window.CTP;
  var includeWaveGen = false; // export/draw a solid-cam wave generator
  var autoHeight = false;     // drive HA = HD from the wave-generator stroke
  var exportTol = 0.01; // DXF simplification tolerance, mm (0 = full resolution)
  var parts = null;
  var view = { scale: 0, x: 0, y: 0 }; // scale 0 => fit on next draw
  var genTimer = null;
  var restoring = false; // suppress debounced regenerate while restoreURL writes fields

  // Distance measurement tool. Points are stored in world (mm) coordinates so
  // they stay pinned to the geometry across pan, zoom and regeneration.
  var measure = { on: false, a: null, b: null, hover: null };

  // Meshing animation. `omega` is the wave-generator angle (rad); `speed` is its
  // angular rate (rad/s) from the slider; `raf` is the pending frame id; `rest`
  // is the cached undeformed flexspline outline (rebuilt only on regenerate).
  var anim = { on: false, omega: 0, speed: 1, raf: 0, last: 0, rest: null };

  // Every numeric solver field. `ratio` is deliberately NOT here: it is a view of
  // zf and zc rather than an input to the solver, so it is derived on read and
  // never round-tripped through the URL.
  var ALL_FIELDS = ['module', 'zf', 'zc', 'w0Ratio', 'ha', 'hd', 'toothThickness',
    'toothAngle', 'tipHalfWidth', 'rootFillet', 'clearance', 'clearanceOutput',
    'pressureAngle', 'wallFlex', 'wallCirc', 'wallDyn'];

  function readParams() {
    var p = { style: style, profile: profile };
    ALL_FIELDS.forEach(function (id) {
      p[id] = parseFloat(document.getElementById(id).value);
    });
    return p;
  }

  // The clearance offset deflates the swept tooth along its own normals, so it
  // cannot exceed the smallest radius of curvature on the tooth or the offset
  // polyline folds through itself. For the S-tooth that is the tip cap, whose
  // radius IS tipHalfWidth. The cycloid has no cap: its crest radius falls out
  // of HA, the thickness and the tooth angle, so the bound has to be read back
  // off the solved profile.
  function tipRadiusOf(p) {
    var r = CTP.tipRadius(p);
    return isFinite(r) ? r : Infinity;
  }

  function validate(p) {
    var problems = [];
    ALL_FIELDS.forEach(function (id) {
      // clearanceOutput and wallDyn are pancake-only; skip them in cup mode
      // (their rows are hidden and the value is irrelevant).
      if (style !== 'pancake' && (id === 'clearanceOutput' || id === 'wallDyn')) return;
      if (!isFinite(p[id])) problems.push(id + ' is not a number');
    });
    if (problems.length) return problems;
    if (p.module <= 0) problems.push('module must be positive');
    if (p.w0Ratio <= 0) problems.push('deflection (w0*) must be positive');
    if (p.zf < 20) problems.push('Zf should be at least 20');
    if (p.zc <= p.zf) problems.push('Zc must be greater than Zf');
    if ((p.zc - p.zf) % 2 !== 0) problems.push('Zc − Zf must be even (typically 2)');
    if (p.ha < 0 || p.hd < 0) problems.push('addendum and dedendum must be non-negative');
    if (p.ha + p.hd <= 0) problems.push('tooth height must be positive');
    // Tooth thickness is a fraction of the circular pitch, and 0.5 (the even
    // split of pitch between tooth and space) is a hard ceiling: the dedendum is
    // the addendum turned 180° about the pitch point, so its root crest sits at
    // twice the half thickness, and past half the pitch that crest walks into
    // the neighbouring tooth.
    if (!(p.toothThickness > 0)) problems.push('tooth thickness must be positive');
    else if (p.toothThickness > 0.5) {
      problems.push('tooth thickness cannot exceed 0.50 of the pitch, past an ' +
        'even tooth/space split the point-symmetric dedendum overruns the neighbouring tooth');
    }
    if (p.rootFillet < 0) problems.push('root fillet must be non-negative');
    // The tooth angle truncates the cycloid at the joint point (paper §3.3).
    // Past its ceiling the truncation would consume the whole flank; the solver
    // clamps, but say so rather than silently ignoring what was typed.
    if (profile === 'cycloid') {
      var aMax = CTP.toothAngleLimit(p);
      if (p.toothAngle < 0) problems.push('tooth angle must be non-negative');
      else if (p.toothAngle >= aMax) {
        problems.push('tooth angle must be under ' + aMax.toFixed(1) +
          '° at HA = ' + p.ha + ', beyond that the truncation eats the whole flank');
      }
    }
    // The dedendum is the addendum turned 180° about the pitch point (paper
    // Eq. 12), so it simply cannot reach deeper than the addendum is tall.
    if (p.hd > p.ha) {
      problems.push('HD cannot exceed HA: the dedendum is the addendum mirrored about the pitch line');
    }
    if (profile === 's' && p.tipHalfWidth * p.module >= p.toothThickness * Math.PI * p.module / 2) {
      problems.push('tip radius must be under the half tooth thickness (' +
        (p.toothThickness * Math.PI / 2).toFixed(2) + '×m at this thickness)');
    }
    // Negative clearance = interference fit (preload). It deflates the swept
    // tooth used to cut the spline slots; once it passes the tightest radius on
    // the tooth (the tip) the offset polyline self-intersects and the geometry
    // degenerates. For the S-tooth that radius is the cap you typed; for the
    // cycloid it is the crest radius the profile came out at.
    var tipR = tipRadiusOf(p);
    if (isFinite(tipR)) {
      var tipMsg = profile === 's' ? 'the tip radius' :
        'the cycloid crest radius (' + (tipR / p.module).toFixed(2) + '×m)';
      if (p.clearance * p.module <= -tipR) {
        problems.push('clearance must be greater than −' + tipMsg);
      }
      if (style === 'pancake' && p.clearanceOutput * p.module <= -tipR) {
        problems.push('output clearance must be greater than −' + tipMsg);
      }
    }
    // The governing design inequality: the tooth has to be tall enough to cover
    // the wave generator's radial stroke, or the spline tooth tip bottoms out in
    // the flexspline root land at the major axis.
    var d = CTP.derive(p);
    var stroke = d.a - d.b;
    if (d.ha + d.hd + d.clearance <= stroke) {
      problems.push('tooth height (HA+HD = ' + (d.ha + d.hd).toFixed(2) +
        ' mm) must clear the radial stroke (' + stroke.toFixed(2) +
        ' mm ≈ 2·w₀*), raise HA/HD or lower the deflection');
    }
    if (p.w0Ratio > 2.5) {
      problems.push('deflection ratio above ~2.5 leaves the thin-ring elliptical model (and the flexspline)');
    }
    return problems;
  }

  function scheduleGenerate() {
    if (restoring) return; // restoreURL sets fields in bulk; the final generate() covers it
    clearTimeout(genTimer);
    genTimer = setTimeout(generate, 200);
  }

  function generate() {
    if (autoHeight) applyAutoHeight();
    syncRatioField();
    syncRootFilletState();
    var p = readParams();
    var problems = validate(p);
    exportBtn.disabled = problems.length > 0;
    if (problems.length) { warnEl.textContent = problems.join(' · '); return; }
    warnEl.textContent = '';

    parts = CTP.generate(p);
    parts.params = p;
    anim.rest = CTP.flexsplineRest(p); // cached outline for the animation
    // The wave-generator cam is exactly the flexspline bore in its deformed
    // (installed) state: the cam outer surface IS the shape the bore is held to.
    // deformFlexspline(...,0).inner gives that profile for free from the solver.
    parts.waveGen = includeWaveGen ? CTP.deformFlexspline(anim.rest, 0).inner : null;

    readoutEl.textContent =
      (profile === 'cycloid' ? 'cycloid (CTP)' : 'S-tooth') +
      ' · ' + (style === 'pancake' ? 'pancake' : 'cup');
    renderMetrics(CTP.metrics(p, parts));
    updateDetailReadout();
    draw();
    syncURL();
  }

  function renderMetrics(mt) {
    var rows = [
      ['reduction', mt.ratio.toFixed(0) + ' : 1'],
      ['pitch ø', mt.pitchDia.toFixed(1) + ' mm'],
      ['outer ø', mt.outerDia.toFixed(1) + ' mm'],
      ['radial deflection', mt.w0.toFixed(3) + ' mm (' + mt.deflPct.toFixed(1) + '%)'],
      ['teeth in mesh', '≈ ' + mt.teethMesh + ' (' + mt.teethPct.toFixed(0) + '%)']
    ];
    if (style === 'pancake') {
      rows.push(['backlash · fixed', '≈ ' + mt.backlashFixed.toFixed(3) + ' mm']);
      rows.push(['backlash · output', '≈ ' + mt.backlashOutput.toFixed(3) + ' mm']);
    } else {
      rows.push(['backlash', '≈ ' + mt.backlash.toFixed(3) + ' mm']);
    }
    rows.push(['flexspline strain', '≈ ' + mt.fsStrainPct.toFixed(2) + '%']);
    rows.push(['tooth height', mt.toothHeight.toFixed(2) + ' mm' +
      (autoHeight ? ' (fitted)' : '')]);
    // Thickness is an input, but in ×pitch; the millimetres it lands on are what
    // you actually check a print against. The root land beside it is the width
    // the thinning bought: zero at the even 0.50 split and full depth.
    rows.push(['tooth thickness', mt.toothThickness.toFixed(2) + ' mm of ' +
      mt.circularPitch.toFixed(2) + ' mm pitch']);
    rows.push(['root land', mt.rootLand.toFixed(2) + ' mm']);
    // The cycloid's tip radius is a RESULT: it falls out of HA, the tooth
    // thickness and the tooth angle, because the crest is the flat end of
    // Eq. (11) and the solver does not cap it. Worth showing: it is what a
    // cutter or a nozzle has to be able to follow.
    if (isFinite(mt.tipRadius)) {
      rows.push(['tip radius', mt.tipRadius.toFixed(3) + ' mm' +
        (profile === 'cycloid' ? ' (derived)' : '')]);
    }
    // How much tooth is left over the wave generator's radial stroke. This is
    // the constraint that decides whether the spline tooth tip bottoms out in
    // the flexspline root, so it is worth showing rather than only warning on.
    if (isFinite(mt.strokeMargin)) {
      rows.push(['stroke margin', (mt.strokeMargin * 1000).toFixed(0) + ' µm']);
    }
    if (includeWaveGen && parts && parts.waveGen) {
      var rmax = 0, rmin = Infinity;
      parts.waveGen.forEach(function (pt) {
        var r = Math.hypot(pt.x, pt.y);
        if (r > rmax) rmax = r;
        if (r < rmin) rmin = r;
      });
      rows.push(['wave gen major ø', (2 * rmax).toFixed(2) + ' mm']);
      rows.push(['wave gen minor ø', (2 * rmin).toFixed(2) + ' mm']);
    }
    var html = '<h4>performance</h4><dl>';
    rows.forEach(function (r) {
      html += '<dt>' + r[0] + '</dt><dd>' + r[1] + '</dd>';
    });
    html += '</dl><span class="est">backlash is geometric; teeth &amp; strain are first-order estimates</span>';
    metricsEl.innerHTML = html;
  }

  /* ---- drawing ---- */

  function resizeCanvas() {
    var r = canvas.parentElement.getBoundingClientRect();
    var dpr = window.devicePixelRatio || 1;
    canvas.width = r.width * dpr;
    canvas.height = r.height * dpr;
    draw();
  }

  function fitView() {
    if (!parts) return;
    var p = parts.params;
    var spline = parts.circular || parts.circularFixed;
    var rOut = spline ? spline.outerRadius : p.module * p.zf / 2 + 5;
    view.scale = Math.min(canvas.width, canvas.height) / (2.3 * rOut);
    view.x = 0;
    view.y = 0;
  }

  function drawLoop(pts, color, width) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width / view.scale;
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (var i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.closePath();
    ctx.stroke();
  }

  function drawCircle(r, color, width, dashed) {
    ctx.strokeStyle = color;
    ctx.lineWidth = width / view.scale;
    if (dashed) ctx.setLineDash([4 / view.scale, 4 / view.scale]);
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // The solid-cam wave generator: a filled oval inside the bore. In the static
  // view it visibly pokes past the round (relaxed) bore at the major axis, and that
  // overhang IS the radial deflection w0 the cam imposes on assembly.
  function drawCam(pts, color) {
    ctx.beginPath();
    ctx.moveTo(pts[0].x, pts[0].y);
    for (var i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
    ctx.closePath();
    ctx.fillStyle = 'rgba(232, 133, 74, 0.18)';
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.6 / view.scale;
    ctx.stroke();
  }

  function draw() {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!parts) return;
    if (!view.scale) fitView();

    ctx.translate(canvas.width / 2 + view.x, canvas.height / 2 + view.y);
    ctx.scale(view.scale, -view.scale); // y up

    var ink = '#1c1814';
    var apricot = '#e8854a';
    var soft = '#a0907a';

    var p = parts.params;
    // Deformed flexspline pose while animating; phiFs is the slow FS-body /
    // output-ring rotation that the splines must follow.
    var df = (anim.on && anim.rest) ? CTP.deformFlexspline(anim.rest, anim.omega) : null;
    var phiFs = df ? df.phiFs : 0;

    if (df) drawEngagementZones(p, anim.omega);

    if (style === 'pancake') {
      var fixed = parts.circularFixed;
      var output = parts.circularOutput;
      drawLoop(fixed.inner, apricot, 1.4);        // fixed ring, stationary
      drawCircle(fixed.outerRadius, apricot, 1.4);
      ctx.save();
      ctx.rotate(phiFs);                          // output ring co-rotates with FS
      drawLoop(output.inner, soft, 1.2);
      drawCircle(output.outerRadius, soft, 1.2, true);
      ctx.restore();
    } else {
      var cs = parts.circular;
      drawLoop(cs.inner, apricot, 1.4);
      drawCircle(cs.outerRadius, apricot, 1.4);
    }

    if (df) {
      drawLoop(df.outer, ink, 1.4);
      drawLoop(df.inner, ink, 1.4);
      // Wave-generator major axis.
      ctx.strokeStyle = soft;
      ctx.lineWidth = 0.7 / view.scale;
      ctx.setLineDash([3 / view.scale, 3 / view.scale]);
      ctx.beginPath();
      ctx.moveTo(-df.a * Math.cos(anim.omega), -df.a * Math.sin(anim.omega));
      ctx.lineTo(df.a * Math.cos(anim.omega), df.a * Math.sin(anim.omega));
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      var fs = parts.flexspline;
      drawLoop(fs.outer, ink, 1.4);
      drawCircle(fs.innerRadius, ink, 1.4);
      // Static view: the bore is round, so draw the cam explicitly. (While
      // animating, the deformed bore loop already traces the cam, so it isn't
      // drawn again.)
      if (includeWaveGen && parts.waveGen) drawCam(parts.waveGen, apricot);
    }
    // Pitch circle reference.
    drawCircle(p.module * p.zf / 2, soft, 0.7, true);

    drawMeasure();
  }

  /* ---- distance measurement ---- */

  // Pointer (device px) -> world (mm), inverse of the draw() transform.
  function screenToWorld(px, py) {
    return {
      x: (px - canvas.width / 2 - view.x) / view.scale,
      y: -(py - canvas.height / 2 - view.y) / view.scale
    };
  }

  function worldToScreen(w) {
    return {
      x: canvas.width / 2 + view.x + view.scale * w.x,
      y: canvas.height / 2 + view.y - view.scale * w.y
    };
  }

  function eventToWorld(e) {
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    return screenToWorld((e.clientX - rect.left) * dpr, (e.clientY - rect.top) * dpr);
  }

  // Drawn in device-pixel (identity) space so dots, dashes and the label stay a
  // constant on-screen size regardless of zoom, and the text isn't mirrored by
  // the y-up world transform.
  function drawMeasure() {
    if (!measure.on || !measure.a) return;
    var end = measure.b || measure.hover;
    if (!end) return;
    var dpr = window.devicePixelRatio || 1;
    var A = worldToScreen(measure.a);
    var B = worldToScreen(end);
    ctx.setTransform(1, 0, 0, 1, 0, 0);

    ctx.strokeStyle = '#e8854a';
    ctx.lineWidth = 1.5 * dpr;
    ctx.setLineDash([6 * dpr, 4 * dpr]);
    ctx.beginPath();
    ctx.moveTo(A.x, A.y);
    ctx.lineTo(B.x, B.y);
    ctx.stroke();
    ctx.setLineDash([]);

    [A, B].forEach(function (pt) {
      ctx.fillStyle = '#1c1814';
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, 3 * dpr, 0, 2 * Math.PI);
      ctx.fill();
    });

    var dist = Math.hypot(end.x - measure.a.x, end.y - measure.a.y);
    var label = dist.toFixed(2) + ' mm';
    var fs = 12 * dpr;
    ctx.font = '700 ' + fs + 'px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    var mx = (A.x + B.x) / 2;
    var my = (A.y + B.y) / 2 - 12 * dpr;
    var pad = 5 * dpr;
    var w = ctx.measureText(label).width;
    ctx.fillStyle = '#1c1814';
    ctx.fillRect(mx - w / 2 - pad, my - fs / 2 - pad, w + 2 * pad, fs + 2 * pad);
    ctx.fillStyle = '#faf4e8';
    ctx.fillText(label, mx, my);
    ctx.textAlign = 'start';
    ctx.textBaseline = 'alphabetic';
  }

  // Shade the two load-carrying zones at the wave-generator major axis. A tooth
  // carries load while its radial engagement is within 90% of the major-axis
  // maximum, i.e. |psi| < psiC about psi = 0 and psi = pi, the same threshold
  // the "teeth in mesh" metric uses. In world coordinates those zones are
  // centred on the major-axis angle omega (and omega + pi).
  function drawEngagementZones(p, omega) {
    var m = p.module, rp = m * p.zf / 2;
    var rIn = rp - p.hd * m;                       // tooth root
    var rOut = rp + p.w0Ratio * m + p.ha * m;      // engaged tip at the major axis
    var psiC = 0.5 * Math.acos(0.9);
    ctx.fillStyle = 'rgba(232, 133, 74, 0.26)';
    [omega, omega + Math.PI].forEach(function (c) {
      ctx.beginPath();
      ctx.arc(0, 0, rOut, c - psiC, c + psiC, false);
      ctx.arc(0, 0, rIn, c + psiC, c - psiC, true);
      ctx.closePath();
      ctx.fill();
    });
  }

  /* ---- pan & zoom ---- */

  canvas.addEventListener('wheel', function (e) {
    e.preventDefault();
    var dpr = window.devicePixelRatio || 1;
    var rect = canvas.getBoundingClientRect();
    var mx = (e.clientX - rect.left) * dpr - canvas.width / 2 - view.x;
    var my = (e.clientY - rect.top) * dpr - canvas.height / 2 - view.y;
    var f = Math.exp(-e.deltaY * 0.0012);
    view.x -= mx * (f - 1);
    view.y -= my * (f - 1);
    view.scale *= f;
    draw();
  }, { passive: false });

  var dragging = null;
  canvas.addEventListener('pointerdown', function (e) {
    if (measure.on) {
      // Click 1 sets the start; click 2 sets the end; click 3 starts over.
      if (!measure.a || (measure.a && measure.b)) {
        measure.a = eventToWorld(e);
        measure.b = null;
      } else {
        measure.b = eventToWorld(e);
      }
      draw();
      return;
    }
    dragging = { x: e.clientX, y: e.clientY };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', function (e) {
    if (measure.on) {
      if (measure.a && !measure.b) { measure.hover = eventToWorld(e); draw(); }
      return;
    }
    if (!dragging) return;
    var dpr = window.devicePixelRatio || 1;
    view.x += (e.clientX - dragging.x) * dpr;
    view.y += (e.clientY - dragging.y) * dpr;
    dragging = { x: e.clientX, y: e.clientY };
    draw();
  });
  canvas.addEventListener('pointerup', function () { dragging = null; });

  function setMeasure(on) {
    measure.on = on;
    measure.a = measure.b = measure.hover = null;
    measureBtn.classList.toggle('active', on);
    canvas.style.cursor = on ? 'crosshair' : 'grab';
    hintEl.textContent = on
      ? 'click two points to measure · scroll to zoom · esc to exit'
      : defaultHint;
    draw();
  }

  measureBtn.addEventListener('click', function () {
    var turnOn = !measure.on;
    if (turnOn && anim.on) stopAnim(); // measuring a moving target is no fun
    setMeasure(turnOn);
  });
  document.addEventListener('keydown', function (e) {
    if (e.key === 'Escape' && measure.on) setMeasure(false);
  });

  /* ---- meshing animation ---- */

  function frame(t) {
    if (!anim.on) { anim.raf = 0; return; }
    if (!anim.last) anim.last = t;
    var dt = (t - anim.last) / 1000;
    anim.last = t;
    if (dt > 0.1) dt = 0.1; // clamp catch-up after a backgrounded tab
    if (anim.speed > 0) {
      anim.omega += anim.speed * dt; // unbounded, keeps phiFs continuous
      draw();
    }
    anim.raf = requestAnimationFrame(frame);
  }

  function startAnim() {
    if (measure.on) setMeasure(false);
    anim.on = true;
    anim.last = 0;
    anim.speed = parseFloat(speedRange.value);
    playBtn.innerHTML = '&#9632; stop';
    playBtn.classList.add('active');
    speedRange.hidden = false;
    hintEl.textContent = 'wave generator drives the flexspline · slider sets speed';
    if (!anim.raf) anim.raf = requestAnimationFrame(frame);
    draw();
  }

  function stopAnim() {
    anim.on = false;
    if (anim.raf) { cancelAnimationFrame(anim.raf); anim.raf = 0; }
    anim.omega = 0;
    playBtn.innerHTML = '&#9654; mesh';
    playBtn.classList.remove('active');
    speedRange.hidden = true;
    if (!measure.on) hintEl.textContent = defaultHint;
    draw();
  }

  playBtn.addEventListener('click', function () { anim.on ? stopAnim() : startAnim(); });
  speedRange.addEventListener('input', function () {
    anim.speed = parseFloat(speedRange.value);
  });

  /* ---- export ---- */

  function stamp(p) {
    return p.profile + '_m' + p.module + '_zf' + p.zf + '_zc' + p.zc;
  }

  exportBtn.addEventListener('click', function () {
    if (!parts) return;
    var p = parts.params;
    var s = stamp(p);
    var opt = { tol: exportTol };
    var fs = parts.flexspline;

    DXF.download('drivemagic_' + style + '_flexspline_' + s + '.dxf',
      DXF.buildDxf([fs.outer, DXF.circle(fs.innerRadius)], opt));

    if (style === 'pancake') {
      var fixed = parts.circularFixed;
      var output = parts.circularOutput;
      DXF.download('drivemagic_pancake_fixed_cs_' + s + '.dxf',
        DXF.buildDxf([fixed.inner, DXF.circle(fixed.outerRadius)], opt));
      DXF.download('drivemagic_pancake_output_cs_' + s + '.dxf',
        DXF.buildDxf([output.inner, DXF.circle(output.outerRadius)], opt));
    } else {
      var cs = parts.circular;
      DXF.download('drivemagic_cup_circular_spline_' + s + '.dxf',
        DXF.buildDxf([cs.inner, DXF.circle(cs.outerRadius)], opt));
    }

    if (includeWaveGen && parts.waveGen) {
      DXF.download('drivemagic_' + style + '_wave_generator_' + s + '.dxf',
        DXF.buildDxf([parts.waveGen], opt));
    }
  });

  /* The toothed loop with the most vertices, used to preview export detail. */
  function heaviestLoop() {
    if (!parts) return null;
    if (style === 'pancake') return parts.circularFixed.inner;
    return (parts.circular || parts.flexspline).inner || parts.flexspline.outer;
  }

  function updateDetailReadout() {
    var loop = heaviestLoop();
    if (!loop) return;
    var full = loop.length;
    var n = DXF.countVertices([loop], exportTol);
    var pct = full ? Math.round(100 * (1 - n / full)) : 0;
    detailEl.textContent = exportTol
      ? '≈ ' + n + ' pts/spline (−' + pct + '%, ≤ ' + exportTol + ' mm dev)'
      : '≈ ' + full + ' pts/spline (full resolution)';
  }

  /* ---- controls ---- */

  function applyStyle(s) {
    style = (s === 'pancake') ? 'pancake' : 'cup';
    document.querySelectorAll('#styleSeg button').forEach(function (b) {
      b.classList.toggle('active', b.dataset.style === style);
    });
    var pancake = style === 'pancake';
    document.getElementById('wallDynRow').hidden = !pancake;
    document.getElementById('clearanceOutputRow').hidden = !pancake;
    document.getElementById('clearanceSym').innerHTML = pancake ? 'fixed CS, &times;m' : '&times;m';
    document.getElementById('wallCircLabel').textContent = pancake ? 'fixed CS' : 'circular spline';
  }
  document.querySelectorAll('#styleSeg button').forEach(function (btn) {
    btn.addEventListener('click', function () {
      applyStyle(btn.dataset.style);
      view.scale = 0;
      generate();
    });
  });

  function applyProfile(pr) {
    profile = (pr === 's') ? 's' : 'cycloid';
    document.querySelectorAll('#profileSeg button').forEach(function (b) {
      b.classList.toggle('active', b.dataset.profile === profile);
    });
    citationEl.hidden = profile !== 'cycloid';
    // Hide the knobs the inactive tooth form does not read. The tooth angle
    // truncates the cycloid and means nothing to the S-tooth; the pressure angle
    // sets the S-tooth's flank at the pitch line and the cycloid never consults
    // it; and the tip cap is built for the S-tooth alone, the cycloid's crest
    // being round already and reported in the metrics as a derived value.
    var cyc = profile === 'cycloid';
    document.getElementById('toothAngleRow').hidden = !cyc;
    document.getElementById('pressureAngleRow').hidden = cyc;
    document.getElementById('tipHalfWidthRow').hidden = cyc;
  }
  document.querySelectorAll('#profileSeg button').forEach(function (btn) {
    btn.addEventListener('click', function () {
      applyProfile(btn.dataset.profile);
      generate();
    });
  });

  function applyDetail(tol) {
    exportTol = isFinite(tol) ? tol : 0;
    document.querySelectorAll('#detailSeg button').forEach(function (b) {
      b.classList.toggle('active', parseFloat(b.dataset.tol) === exportTol);
    });
  }
  document.querySelectorAll('#detailSeg button').forEach(function (btn) {
    btn.addEventListener('click', function () {
      applyDetail(parseFloat(btn.dataset.tol));
      updateDetailReadout();
      syncURL();
    });
  });

  document.getElementById('advToggle').addEventListener('change', function (e) {
    document.getElementById('advanced').hidden = !e.target.checked;
  });

  waveGenToggle.addEventListener('change', function (e) {
    includeWaveGen = e.target.checked;
    generate();
  });

  /* ---- reduction ratio ---- */
  // The ratio is Zf / (Zc − Zf), so it is a VIEW of the two tooth counts rather
  // than a parameter of its own. Typing one holds the tooth difference dz and
  // solves Zf = ratio·dz, which lands exactly on every whole ratio because dz is
  // even, and Zc then follows as Zf + dz. Driving Zf rather than dz is what makes it
  // exact: rounding dz instead would snap 20:1 on a hundred-tooth flexspline to
  // 25:1 or 16.7:1 and quietly hand back a number nobody asked for.
  function currentDz() {
    var dz = Math.round(parseFloat(document.getElementById('zc').value) -
                        parseFloat(document.getElementById('zf').value));
    if (!isFinite(dz) || dz < 2) dz = 2;
    if (dz % 2 !== 0) dz += 1;
    return dz;
  }
  // Reflect Zf/Zc back into the ratio box, but never while it has focus: the
  // ratio is only rewritten once the edit is committed, so typing "2" on the way
  // to "20" does not get answered mid-keystroke.
  function syncRatioField() {
    if (document.activeElement === ratioEl) return;
    var zf = parseFloat(document.getElementById('zf').value);
    var dz = currentDz();
    if (!isFinite(zf) || !dz) return;
    var r = zf / dz;
    ratioEl.value = Math.abs(r - Math.round(r)) < 1e-9 ? String(Math.round(r)) : r.toFixed(2);
  }
  // 'change', not 'input': it fires on blur or Enter, once the whole number is
  // typed, so the field is not fighting the user a digit at a time.
  ratioEl.addEventListener('change', function () {
    var r = parseFloat(ratioEl.value);
    if (!isFinite(r) || r <= 0) { syncRatioField(); return; }
    var dz = currentDz();
    var zf = Math.round(r * dz);
    if (zf % 2 !== 0) zf += 1;          // keep Zf even, as the step implies
    if (zf < 20) zf = 20;               // the solver's own floor
    document.getElementById('zf').value = zf;
    document.getElementById('zc').value = zf + dz;
    view.scale = 0;                     // the gear changed size; refit
    generate();
  });

  /* ---- presets ---- */
  // Only the fields a preset actually speaks to are written; everything else is
  // reset to its markup default first, so a preset is a known state rather than
  // whatever the last one left behind.
  // One entry, deliberately: a preset is only worth shipping if it is a design
  // someone actually built and released files for. Anything else would be a
  // guess wearing the same clothes.
  var PRESETS = {
    // The drive documented at /actuators/hdp30: 30:1, module 0.55, cycloid,
    // pancake. Every field is pinned rather than left to fall back on the
    // markup defaults, because these exact numbers cut the DXFs published in
    // robrotics/hdp30, and a later change to a default must not silently move the
    // preset off the released geometry.
    hdp30: {
      style: 'pancake', profile: 'cycloid',
      fields: {
        module: 0.55, zf: 60, zc: 62, w0Ratio: 1.0, ha: 1.08, hd: 1.08,
        toothThickness: 0.50, toothAngle: 9.17, tipHalfWidth: 0.30,
        rootFillet: 0.15, clearance: 0, clearanceOutput: -0.05,
        pressureAngle: 30, wallFlex: 0.75, wallCirc: 3.0, wallDyn: 3.0
      }
    }
  };
  function applyPreset(name) {
    var ps = PRESETS[name];
    if (!ps) return;
    autoHeight = false;
    autoHeightToggle.checked = false;
    ALL_FIELDS.forEach(function (id) {
      var el = document.getElementById(id);
      el.value = (ps.fields[id] != null) ? ps.fields[id] : el.defaultValue;
    });
    applyStyle(ps.style);
    applyProfile(ps.profile);
    document.querySelectorAll('#presetSeg button').forEach(function (b) {
      b.classList.toggle('active', b.dataset.preset === name);
    });
    view.scale = 0;
    generate();
  }
  document.querySelectorAll('#presetSeg button').forEach(function (btn) {
    btn.addEventListener('click', function () { applyPreset(btn.dataset.preset); });
  });
  // Any hand edit means the design is no longer the preset that seeded it.
  function clearPresetHighlight() {
    document.querySelectorAll('#presetSeg button').forEach(function (b) {
      b.classList.remove('active');
    });
  }

  /* ---- tooth height fitted to the wave-generator stroke ---- */
  // HA + HD + clearance has to exceed the radial stroke a − b, or the spline
  // tooth tip bottoms out in the flexspline root land at the major axis. That
  // inequality is the single most common way to land in the warning strip, so
  // this solves it directly: split the height evenly (HA = HD, which also
  // satisfies the point-symmetry rule HD ≤ HA) and leave a fixed margin.
  var FIT_MARGIN = 0.15;   // module units of stroke margin to leave in hand
  function applyAutoHeight() {
    var m = parseFloat(document.getElementById('module').value);
    var clr = parseFloat(document.getElementById('clearance').value);
    if (!isFinite(m) || m <= 0 || !isFinite(clr)) return;
    var d;
    try { d = CTP.derive(readParams()); } catch (e) { return; }
    var strokeStar = (d.a - d.b) / m;                 // stroke in module units
    if (!isFinite(strokeStar)) return;
    var h = (strokeStar + FIT_MARGIN - clr) / 2;
    h = Math.max(0.2, Math.round(h * 100) / 100);
    document.getElementById('ha').value = h.toFixed(2);
    document.getElementById('hd').value = h.toFixed(2);
  }
  function setAutoHeight(on) {
    autoHeight = !!on;
    autoHeightToggle.checked = autoHeight;
    // Keep the numbers visible (they are the answer) but read-only, so it is
    // clear they are being driven rather than ignored.
    ['ha', 'hd'].forEach(function (id) {
      var el = document.getElementById(id);
      el.readOnly = autoHeight;
      el.title = autoHeight
        ? 'driven by the deflection fit, untick to edit'
        : '';
      el.parentElement.classList.toggle('driven', autoHeight);
    });
  }
  autoHeightToggle.addEventListener('change', function (e) {
    setAutoHeight(e.target.checked);
    generate();
  });

  /* ---- root fillet availability ---- */
  // At full depth (HD = HA) the two flanks meet at the root crest: there is no
  // root land for a fillet to blend, and the crest is already round by the
  // point symmetry. The solver zeroes the fillet in that case, so say so rather
  // than leaving a live-looking field that does nothing, which is what it did
  // at the page defaults.
  function syncRootFilletState() {
    var ha = parseFloat(document.getElementById('ha').value);
    var hd = parseFloat(document.getElementById('hd').value);
    var full = isFinite(ha) && isFinite(hd) && hd >= ha - 1e-9;
    var el = document.getElementById('rootFillet');
    el.disabled = full;
    document.getElementById('rootFilletRow').classList.toggle('inert', full);
    document.getElementById('rootFilletSym').innerHTML = full
      ? '(no root land at HD = HA)'
      : '&times;m';
  }

  /* ---- shareable permalink ---- */
  // The full design lives in the URL query string: every numeric field plus the
  // style / profile / export-detail / wave-generator toggles. syncURL rewrites it
  // on any change (replaceState, no history spam); restoreURL rebuilds the UI
  // from it on load so a pasted link reproduces the exact design.
  function syncURL() {
    try {
      var q = new URLSearchParams();
      q.set('style', style);
      q.set('profile', profile);
      q.set('tol', String(exportTol));
      q.set('wave', includeWaveGen ? '1' : '0');
      q.set('fit', autoHeight ? '1' : '0');
      ALL_FIELDS.forEach(function (id) {
        q.set(id, document.getElementById(id).value);
      });
      history.replaceState(null, '', location.pathname + '?' + q.toString());
    } catch (err) { /* e.g. file://, sharing just won't persist */ }
  }
  function restoreURL() {
    if (!location.search) return;
    restoring = true;
    try {
      var q = new URLSearchParams(location.search);
      // Links written while the rev A / rev B toggle shipped carry a `rev`; it is
      // ignored now that rev B is the only solver, which for a rev=B link is
      // lossless and for a rev=A one reproduces the design under the solver that
      // corrects its geometry. `crown` was rev A's only exclusive field and is
      // likewise dropped.
      ALL_FIELDS.forEach(function (id) {
        if (q.has(id)) document.getElementById(id).value = q.get(id);
      });
      if (q.has('wave')) includeWaveGen = q.get('wave') === '1';
      waveGenToggle.checked = includeWaveGen;
      if (q.has('style')) applyStyle(q.get('style'));
      if (q.has('profile')) applyProfile(q.get('profile'));
      if (q.has('tol')) applyDetail(parseFloat(q.get('tol')));
      if (q.has('fit')) setAutoHeight(q.get('fit') === '1');
      // Ratio is derived from Zf/Zc, not an independent input. Recompute it so
      // the field agrees with the restored tooth counts; otherwise a later
      // generate() re-derives Zf/Zc from the stale default ratio (50) and
      // silently discards the URL's tooth counts.
      syncRatioField();
    } finally {
      restoring = false;
    }
  }
  var LINK_LABEL = linkBtn.textContent;
  function flashLink(msg) {
    linkBtn.textContent = msg;
    setTimeout(function () { linkBtn.textContent = LINK_LABEL; }, 1400);
  }
  // Synchronous fallback for when the async Clipboard API is unavailable or
  // rejects (desktop browsers gate writeText on window focus / permissions far
  // more than mobile; the old code reported success unconditionally and so
  // "copied" nothing). Returns true only on an actual copy.
  function legacyCopy(text) {
    try {
      var ta = document.createElement('textarea');
      ta.value = text;
      ta.setAttribute('readonly', '');
      ta.style.position = 'fixed';
      ta.style.top = '-1000px';
      ta.style.opacity = '0';
      document.body.appendChild(ta);
      ta.focus();
      ta.select();
      var ok = document.execCommand('copy');
      document.body.removeChild(ta);
      return ok;
    } catch (e) {
      return false;
    }
  }
  linkBtn.addEventListener('click', function () {
    syncURL();
    var url = location.href;
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(url).then(
        function () { flashLink('link copied ✓'); },
        function () { flashLink(legacyCopy(url) ? 'link copied ✓' : 'press Ctrl+C to copy'); }
      );
    } else {
      flashLink(legacyCopy(url) ? 'link copied ✓' : 'press Ctrl+C to copy');
    }
  });

  ALL_FIELDS.forEach(function (id) {
    var input = document.getElementById(id);
    input.addEventListener('input', function () {
      clearPresetHighlight();
      scheduleGenerate();
    });
    // A per-field reset to the markup default (input.defaultValue is the value
    // attribute, untouched by user edits).
    var btn = document.createElement('button');
    btn.type = 'button';
    btn.className = 'reset-btn';
    btn.textContent = '↺';
    btn.title = 'reset to default (' + input.defaultValue + ')';
    btn.setAttribute('aria-label', 'reset ' + id + ' to default');
    btn.addEventListener('click', function () {
      if (input.value === input.defaultValue) return;
      input.value = input.defaultValue;
      clearPresetHighlight();
      scheduleGenerate();
    });
    input.insertAdjacentElement('afterend', btn);
  });

  // Keep the canvas bitmap matched to its CSS box. Toggling advanced mode (or
  // the style segment) reflows the panel, which restretches the 1fr viewport
  // column; a ResizeObserver catches every such layout change, not just window
  // resizes, so the drawing never gets squashed by a stale bitmap size.
  if (typeof ResizeObserver !== 'undefined') {
    new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  } else {
    window.addEventListener('resize', resizeCanvas);
  }
  applyProfile(profile);   // sets the tooth-form row visibility
  setAutoHeight(false);
  restoreURL();
  resizeCanvas();
  generate();
})();
