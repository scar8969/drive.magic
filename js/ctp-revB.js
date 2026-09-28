/*
 * Harmonic-drive tooth profile solver, rev B.
 *
 * Two tooth forms are supported:
 *
 *   'cycloid'  Fully-conjugate cycloid tooth profile (CTP) after
 *              Yao, Y.; Lu, L.; Chen, X.; Xie, Y.; Yang, Y.; Xing, J.
 *              "A Novel Cycloid Tooth Profile for Harmonic Drive with Fully
 *              Conjugate Features." Actuators 2025, 14, 187.
 *              https://doi.org/10.3390/act14040187
 *              The flank is the x-halved cycloid of Eqs. (11)/(12), truncated at
 *              the tooth angle alpha0 of Section 3.3: a convex addendum arc above
 *              the pitch line and a point-symmetric concave dedendum arc below it.
 *
 *   's'        Conventional "S" double-circular-arc tooth, the standard
 *              harmonic-drive form: a convex addendum arc and a concave
 *              dedendum arc tangent at the pitch line.
 *
 * Wave-generator deformation follows the elliptical neutral-line model of the
 * same paper (Eq. 8): the deformed flexspline midline is an ellipse whose
 * minor axis b is fixed by midline inextensibility, giving radial displacement
 * w(psi) = rho(psi) - rm and the exact section deflection mu = -arctan(rho'/rho)
 * (Eq. 4). The conjugate spline tooth space is the max-radius envelope of the
 * deformed flexspline tooth swept through one engagement.
 *
 * Length parameters HA, HD, tipHalfWidth, rootFillet, clearance are in units of
 * module. Output coordinates are millimetres.
 *
 * ---------------------------------------------------------------------------
 * CHANGES FROM REV A
 * ---------------------------------------------------------------------------
 * 1. CYCLOID ORIENTATION (root cause of the "square shoulder" tooth).
 *    Paper Eq. (11) is xd = m(t - sin t)/4, yd = m(1 + cos t)/2. dyd/dt = 0 at
 *    t = pi, so the CREST is the flat, naturally rounded end of the curve, and the
 *    cusp - where both derivatives vanish and the tangent stands radial - sits at
 *    t = 0, on the pitch line. Rev A had this right. The first cut of rev B
 *    "corrected" it to fx = (t + sin t)/pi, which is the curve mirrored
 *    end-for-end: a horizontal tangent at the PITCH LINE, i.e. the shelf halfway
 *    up the flank that made the tooth look stepped, and the cusp moved to the tip.
 *    At half tooth height the two differ by 625 um on a 1.25 module. Fixed in
 *    cycloidUnit(), which now reproduces Eq. (11) to within floating point.
 *
 * 2. TOOTH ANGLE alpha0 - the paper's own answer to the cusp, and the reason the
 *    orientation above is not a problem. Sections 3.3/3.4 never run the cycloid
 *    down to t = 0: both flanks are truncated at a joint parameter tE fixed by
 *    the tooth angle at the joint point ("the tangent angle of AB at point E is
 *    alpha0"), and Eqs. (18)-(21) each read "t takes values in the range from tE
 *    to pi". Table 2's design case uses alpha01 = 9.17 deg, which cuts the curve
 *    at tE = 39 deg and leaves a finite-radius flank at the pitch line instead of
 *    the cusp. The truncated curve is then rescaled into the tooth box - exactly
 *    what the paper's Ad/Au scaling matrices do (its fitted axu1 = 0.8819,
 *    ayu1 = 1.144, aru1 = 1.081 are that rescaling). toothAngle = 0 degenerates
 *    to the untruncated Eq. (11), cusp and all.
 *
 * 3. SECTION ROTATION SIGN. The local tooth frame is (x = tangential,
 *    y = radial), which is left-handed with respect to (e_r, e_theta), so
 *    applying the textbook rotation matrix there yields a physical rotation of
 *    -mu. Rev A therefore leaned every tooth the wrong way by 2*mu (up to 4.6
 *    degrees). mu itself is defined correctly and is unchanged; the three places
 *    that APPLY it now rotate by +mu. See rotateIntoSection().
 *
 * 4. NO TIP CAP ON THE CYCLOID; TANGENT ROOT FILLET. Because the crest is the
 *    flat end of the curve it is already round: its radius of curvature is
 *    4*s0^2*(1 + cos tE) / (ha*(pi - tE + sin tE)^2), 0.57 mm on the defaults.
 *    Capping it would cut away conjugate flank and glue on an arc that is not
 *    conjugate to anything, so the cap is now built for the S-tooth alone - whose
 *    flank does arrive at the pressure angle and does need one - and `metrics`
 *    reports the cycloid's natural tip radius instead of taking it as an input.
 *    Rev A's raised-cosine crown was in any case concave over the outer half of
 *    the tip (y'' flips sign at x = tipHalfWidth/2), which put a 20 um dimple
 *    either side of the tip. The new rootFillet parameter adds a tangent fillet
 *    where the flank meets the root land; rev A had a square corner there, which
 *    on a fatigue-loaded printed flexspline is the crack site.
 *
 * 5. ROOT LAND IS NOW SWEPT INTO THE CONJUGATE ENVELOPE. Rev A swept only the
 *    tooth, so nothing stopped the spline tooth tip from digging into the
 *    flexspline root land at the major axis: HD 1.25 -> 1.0 silently produced
 *    218 um of interference. The swept polyline now spans a full flexspline
 *    pitch, and rasterizeSegment wraps modulo the spline pitch instead of
 *    discarding whatever leaves the window.
 *
 * 6. CONSISTENT POLAR PLACEMENT. flexsplineProfile placed a tooth point at
 *    (radius rp + y, angle x/rp) while conjugateSlot placed it at a Cartesian
 *    offset (radial R + y, tangential x) - a ~13 um disagreement at the root
 *    corners. Both now use the polar convention, matching deformFlexspline.
 *
 * 7. Flexspline flank sampling raised from 16 to 48 (rev A shipped 30 degree
 *    corners between segments and 14 um of chord error, against a 10 um finest
 *    export tolerance that RDP can only ever coarsen).
 *
 * 8. Guards: b is clamped positive (rev A let the ellipse invert past
 *    w0Ratio ~ 36), and metrics reports the tooth-height/stroke margin.
 */
(function (global) {
  'use strict';

  var DEFAULTS = {
    module: 1.25,    // mm
    zf: 100,         // flexspline tooth count
    zc: 102,         // circular spline tooth count
    profile: 'cycloid', // 'cycloid' (CTP) or 's' (double circular arc)
    w0Ratio: 1.0,    // wave generator radial deflection / module (w0*)
    ha: 1.0,         // addendum height to the TIP APEX (module units)
    hd: 1.0,         // dedendum depth to the root circle (module units). Under
                     // the paper's point-symmetric profile the dedendum is the
                     // addendum turned through 180 degrees, so hd cannot exceed
                     // the addendum's full cycloid height; it is clamped if it
                     // does. Rev A's asymmetric 0.75/1.25 existed to work around
                     // the reversed flank and is no longer needed - and
                     // ha + hd = 2.0 still clears the 2*w0* radial stroke.
    toothAngle: 9.17, // cycloid tooth angle alpha0 at the joint point where the
                     // addendum and dedendum flanks meet (deg), the paper's
                     // alpha01. It truncates the cycloid at t = tE (Section 3.3)
                     // so the flank crosses the pitch line at a finite radius
                     // instead of running into the cusp at t = 0. 9.17 deg is the
                     // paper's own design case, Table 2. 0 = untruncated Eq. (11).
                     // Capped just below atan(s0 / 2*ha), where the flank would
                     // shrink to nothing.
    tipHalfWidth: 0.30, // S-TOOTH ONLY: tip cap radius (module units). The
                     // S-tooth flank tops out at ha - tipHalfWidth and a circular
                     // cap of this radius carries it to the apex. The cycloid's
                     // crest is round already (see note 4 above) and is left
                     // alone; metrics reports the radius it comes out at.
    toothThickness: 0.5, // tooth thickness at the pitch line as a fraction of the
                     // circular pitch (pi*m), so the half thickness is
                     // s0 = toothThickness * pi * m / 2. 0.5 is the paper's
                     // equal split of pitch between tooth and space, and is also
                     // the CEILING: the dedendum is the addendum turned 180
                     // degrees about the pitch point (Eq. 12), which puts the
                     // root crest at 2*s0 from the tooth axis, and that has to
                     // land at or inside the half pitch pi*m/2 or neighbouring
                     // teeth overlap. Below 0.5 the tooth thins against the
                     // space and a flat root land opens up between teeth,
                     // roughly pi*m*(1 - 2*toothThickness) wide at full depth.
                     //
                     // What this does NOT do is change backlash. The spline slot
                     // is the conjugate envelope of whatever tooth it is handed,
                     // so thinning the tooth thins the slot with it and the fit
                     // between them is unmoved - measured backlash drifts about
                     // 1% over the whole usable range. Backlash is `clearance`.
                     // What thickness buys is the root land itself: somewhere for
                     // grease and debris to go, and room under the mating tooth
                     // tip. See the regression in test/solver-test.js.
    rootFillet: 0.15, // fillet radius where the flank meets the root land
                     // (module units). Rev A had a square corner here, which on
                     // a fatigue-loaded printed flexspline is the crack site.
    clearance: 0.05, // backlash clearance cut into the splines (module units).
                     // For pancake this is the FIXED CS (zc, dz!=0) clearance,
                     // which converts to ~2x its value of rotational backlash.
                     // May be negative (interference fit / preload): the slot is
                     // cut undersize by that amount, e.g. to compensate FDM
                     // over-extrusion. Must stay above -tipHalfWidth or the
                     // offset tooth polyline collapses.
    clearanceOutput: 0.01, // pancake OUTPUT CS (zf, dz=0) clearance (module
                     // units); null => same as `clearance`. The dz=0 mesh makes
                     // near-point contact at the tooth TIP (non-conformal, since
                     // the flanks stand off), so this clearance is AMPLIFIED
                     // several-fold into rotational backlash (vs 2x for the
                     // distributed-flank-contact fixed CS); keep it roughly 1/5
                     // of `clearance` for a comparable feel. See measureBacklash.
    pressureAngle: 30, // S-tooth flank pressure angle at the pitch line (deg)
    wallFlex: 0.75,  // flexspline wall under the root (mm). Thin on purpose: the
                     // rim has to bend through 2*w0 every input turn, and peak
                     // bending strain runs with the wall (metrics' fsStrainPct is
                     // linear in it), so a thick flexspline is a stiff one that
                     // cracks. 0.75 is what our own HD20 runs.
    wallCirc: 3.0,   // circular spline wall behind the slot (mm)
    wallDyn: 3.0     // output CS (dynamic) spline wall (mm)
  };

  function derive(p) {
    var m = p.module;
    var rp = m * p.zf / 2;        // flexspline pitch / neutral radius
    var w0 = p.w0Ratio * m;       // radial deflection at the major axis
    // Elliptical neutral line (Eq. 8): a = major semi-axis, b from midline
    // inextensibility. b -> rm when w0 -> 0. Clamped positive: past
    // w0/rp ~ 0.71 the closed form goes negative and the ellipse inverts.
    var a = rp + w0;
    var b = (1 / 9) * ((12 * rp - 7 * a) + 4 * Math.sqrt(Math.max(0, a * (3 * rp - 2 * a))));
    if (!(b > 0)) b = 1e-6;
    // Half tooth thickness at the pitch line. Clamped to (0, pi*m/4]: at the top
    // of that range tooth and space split the pitch evenly (the paper's case),
    // and past it the point-symmetric dedendum's root crest, which sits at 2*s0,
    // would overrun the half pitch and collide with the neighbouring tooth.
    var thick = Math.min(0.5, Math.max(1e-3,
      p.toothThickness != null ? p.toothThickness : 0.5));
    var s0 = thick * Math.PI * m / 2;
    var tipHW = p.tipHalfWidth * m;
    var rf = (p.rootFillet != null ? p.rootFillet : 0) * m;
    var ha = p.ha * m;
    var hd = p.hd * m;
    // Tooth angle, clamped strictly inside its geometric ceiling: at the ceiling
    // the truncation eats the whole flank.
    var alpha0 = (p.toothAngle != null ? p.toothAngle : 0) * Math.PI / 180;
    alpha0 = Math.max(0, Math.min(alpha0, 0.98 * maxToothAngle(s0, ha)));
    return {
      m: m,
      rp: rp,
      rm: rp,                     // neutral circle radius
      a: a,
      b: b,
      w0: w0,
      ha: ha,                     // to the tip APEX
      hd: hd,
      tipHW: tipHW,               // = S-tooth tip cap radius
      rootFillet: rf,
      clearance: p.clearance * m, // = fixed CS clearance
      clearanceOutput: (p.clearanceOutput != null ? p.clearanceOutput : p.clearance) * m,
      pa: (p.pressureAngle != null ? p.pressureAngle : 30) * Math.PI / 180,
      alpha0: alpha0,             // cycloid tooth angle (rad), clamped. NOT `a`,
                                  // which is the ellipse's semi-major axis.
      tE: solveJointParam(s0, ha, alpha0), // cycloid truncation parameter
      s0: s0,
      toothThickness: thick,      // as clamped
      halfPitch: Math.PI * m / 2  // half the tooth pitch, arc length at rp
    };
  }

  /*
   * The joint parameter tE for a given tooth angle alpha0. Differentiating the
   * box-fitted flank of cycloidUnit(),
   *
   *     tan(alpha0) = s0 * sin(tE) / (ha * (pi - tE + sin tE))
   *
   * Here alpha0 is measured off the radial direction, as in the paper, where the slope
   * at the joint is -cot(alpha0). The right-hand side climbs monotonically from 0
   * at tE = 0 to s0/(2*ha) as tE -> pi (where the flank has been truncated out of
   * existence), so alpha0 has a hard ceiling of atan(s0 / 2*ha), 21.4 deg on a
   * standard tooth, and a plain bisection finds tE inside it.
   */
  function maxToothAngle(s0, ha) { return Math.atan(s0 / (2 * ha)); }
  function solveJointParam(s0, ha, alpha0) {
    if (!(alpha0 > 0)) return 0;
    var target = Math.tan(alpha0);
    function f(t) { return s0 * Math.sin(t) / (ha * (Math.PI - t + Math.sin(t))); }
    var lo = 0, hi = Math.PI, i;
    for (i = 0; i < 60; i++) {
      var mid = 0.5 * (lo + hi);
      if (f(mid) < target) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
  }

  /*
   * Elliptical wave-generator deformation at tooth angle psi from the major
   * axis. Returns radial displacement w, tangential displacement v and the
   * section deflection mu (Eqs. 4 and 8).
   */
  function deform(d, psi) {
    var a = d.a, b = d.b;
    var s = Math.sin(psi), c = Math.cos(psi);
    var D = a * a * s * s + b * b * c * c;
    var rho = a * b / Math.sqrt(D);
    var w = rho - d.rm;
    // rho'(psi) = -a*b*(a^2 - b^2)*sin(2psi) / (2*D^(3/2))
    var rhop = -a * b * (a * a - b * b) * Math.sin(2 * psi) / (2 * Math.pow(D, 1.5));
    var mu = -Math.atan2(rhop, rho);
    // Tangential displacement from midline inextensibility (dv/dpsi = -w). To
    // first order in w0/rm this integrates to the closed form below; the
    // higher-order term is O((w0/rm)^2) (~0.04% here) and is dropped to stay
    // consistent with the cited paper.
    var v = -(d.w0 / 2) * Math.sin(2 * psi);
    return { rho: rho, w: w, v: v, mu: mu };
  }

  /*
   * Carry a tooth-local point (x tangential, y radial from the pitch line) into
   * the deformed section whose neutral point sits at radius R with section
   * rotation mu, and return it as (radius, tangential offset).
   *
   * The local frame is (e_theta, e_r), which is a REFLECTION of the standard
   * right-handed (e_r, e_theta). A textbook rotation matrix applied in it turns
   * the tooth by -mu, which is the rev A bug. Rotating by +mu means the tooth's
   * radial axis lands along the outward normal of the deformed midline,
   * cos(mu)*e_r + sin(mu)*e_theta, which is what "plane sections stay normal to
   * the midline" requires.
   */
  function rotateIntoSection(x, y, cosM, sinM) {
    return {
      t: x * cosM + y * sinM,     // tangential
      r: -x * sinM + y * cosM     // radial, relative to the neutral point
    };
  }

  /* Place a sectioned point in the spline frame using the polar convention:
   * radius R + r, angle theta + t/R. Matches flexsplineProfile and
   * deformFlexspline so all three agree on where a tooth point lives. */
  function polarPlace(R, theta, t, r) {
    var rad = R + r;
    var ang = theta + t / (rad || 1);
    return { x: rad * Math.cos(ang), y: rad * Math.sin(ang), rad: rad, ang: ang };
  }

  /* ------------------------------------------------------------------ *
   * Tooth profile
   * ------------------------------------------------------------------ */

  /*
   * Normalised x-halved cycloid, paper Eq. (11)
   *
   *     xd = m(t - sin t)/4 ,   yd = m(1 + cos t)/2
   *
   * run from the joint point t = tE up to the crest at t = pi, and rescaled into
   * the tooth box so that u = 0 lands on the pitch point (s0, 0) and u = 1 on the
   * crest (0, H). Writing X(t) = t - sin t, the fraction of the way from the pitch
   * half-width to the crest and from the pitch line to the crest are
   *
   *     fx = (X(t) - X(tE)) / (pi - X(tE))      fy = (cos tE - cos t)/(1 + cos tE)
   *
   * dyd/dt vanishes at t = pi and dxd/dt does not, so the crest is the FLAT end of
   * the curve: the tip comes out naturally rounded and the flank meets the pitch
   * line at the tooth angle alpha0 (radial at alpha0 = 0, where tE = 0 and this
   * collapses to fx = (t - sin t)/pi, fy = (1 - cos t)/2, Eq. (11) untruncated,
   * cusp on the pitch line). Getting this end-for-end is what put a horizontal
   * shelf halfway up rev B's flank.
   */
  function cycloidUnit(u, tE) {
    tE = tE || 0;
    var t = tE + (Math.PI - tE) * u;
    var XE = tE - Math.sin(tE), cE = Math.cos(tE);
    return {
      fx: (t - Math.sin(t) - XE) / (Math.PI - XE),
      fy: (cE - Math.cos(t)) / (1 + cE)
    };
  }

  /* Circle through P0 with unit tangent T there, ending at Pend. Returns a
   * parameterisation so the flank can be sampled anywhere. Used by the S-tooth. */
  function arcParams(P0, T, Pend) {
    var Nx = -T.y, Ny = T.x;                  // unit normal to the tangent
    var dx = P0.x - Pend.x, dy = P0.y - Pend.y;
    var denom = Nx * dx + Ny * dy;
    if (Math.abs(denom) < 1e-9) {             // collinear -> straight flank
      return { line: true, P0: P0, Pend: Pend };
    }
    var s = -(dx * dx + dy * dy) / (2 * denom);
    var cx = P0.x + s * Nx, cy = P0.y + s * Ny;
    var R = Math.abs(s);
    var a0 = Math.atan2(P0.y - cy, P0.x - cx);
    var a1 = Math.atan2(Pend.y - cy, Pend.x - cx);
    var da = a1 - a0;                         // take the minor arc
    while (da > Math.PI) da -= 2 * Math.PI;
    while (da < -Math.PI) da += 2 * Math.PI;
    return { line: false, cx: cx, cy: cy, R: R, a0: a0, da: da };
  }
  function arcAt(ap, u) {
    if (ap.line) {
      return { x: ap.P0.x + (ap.Pend.x - ap.P0.x) * u, y: ap.P0.y + (ap.Pend.y - ap.P0.y) * u };
    }
    var ang = ap.a0 + ap.da * u;
    return { x: ap.cx + ap.R * Math.cos(ang), y: ap.cy + ap.R * Math.sin(ang) };
  }

  /*
   * Right-hand flank as a function of u in [0, 1]: u = 0 at the pitch point
   * (s0, 0), u = 1 at the crest. `kind` is 'add' (up to the tip) or 'ded'
   * (down to the root).
   */
  /*
   * The ADDENDUM flank, u = 0 at the pitch point (s0, 0), u = 1 on the tooth axis
   * at (0, haFull). For the cycloid that endpoint IS the finished tip: the crest
   * is the flat end of the curve, so it arrives there with a horizontal tangent
   * and a radius of curvature of its own (see crestRadius), and haFull is just
   * d.ha. The S-tooth arrives at the pressure angle instead and haFull is the
   * stretched height fitTipCap solves for, so that its cap apexes at d.ha.
   */
  function flankFn(d, profile, haFull) {
    if (profile === 's') {
      var P0 = { x: d.s0, y: 0 };
      var T = { x: -Math.sin(d.pa), y: Math.cos(d.pa) };   // pitch -> tip
      var ap = arcParams(P0, T, { x: 0, y: haFull });
      return function (u) { return arcAt(ap, u); };
    }
    return function (u) {
      var c = cycloidUnit(u, d.tE);
      return { x: d.s0 * (1 - c.fx), y: haFull * c.fy };
    };
  }

  /*
   * Radius of curvature of a flank at its crest (u = 1). For the cycloid this is
   * the tooth's tip radius, a RESULT of ha, s0 and the tooth angle rather than
   * an input, which is why rev B no longer caps the cycloid tip. Central
   * differences one step in from the end, where x'(u) is nonzero and y'(u) is not
   * quite zero, so the curvature is well conditioned.
   */
  function crestRadius(fn) {
    var h = 1e-4;
    var p0 = fn(1 - 2 * h), p1 = fn(1 - h), p2 = fn(1);
    var x1 = (p2.x - p0.x) / (2 * h), y1 = (p2.y - p0.y) / (2 * h);
    var x2 = (p2.x - 2 * p1.x + p0.x) / (h * h), y2 = (p2.y - 2 * p1.y + p0.y) / (h * h);
    var k = Math.abs(x1 * y2 - y1 * x2);
    return k > 1e-12 ? Math.pow(x1 * x1 + y1 * y1, 1.5) / k : Infinity;
  }

  /*
   * The DEDENDUM flank is the addendum rotated 180 degrees about the pitch point
   * - paper Eq. (12), "point symmetry of Equation (11) about the point (0, 0)".
   * Rev A instead gave the dedendum its own box, s0 -> rootHW by a heuristic,
   * which squashed it to an aspect ratio of 0.19 against the addendum's 0.71 and
   * left a 48 um radius right at the pitch line. Under point symmetry the two
   * flanks meet the pitch line with equal curvature magnitude and opposite sign,
   * which is the inflection the paper describes.
   *
   * It also fixes the root half-width at 2*s0 - no magic constant, no 0.92 cap.
   * At the default toothThickness of 0.5 that is exactly half the tooth pitch,
   * so at full depth neighbouring teeth meet precisely and the root land has
   * zero width. Thinning the tooth pulls the root crest inside the half pitch
   * and opens a flat root land between teeth, which flexsplineProfile and
   * toothWithRootLand already emit.
   */
  function dedendumFn(d, addFn) {
    return function (u) {
      var q = addFn(u);
      return { x: 2 * d.s0 - q.x, y: -q.y };
    };
  }

  /*
   * Sample a flank at n+1 points spaced evenly by ARC LENGTH over [0, uEnd].
   * The cycloid's parameter runs fast in x near the pitch line and fast in y
   * near the crest, so sampling uniformly in u leaves the two flanks meeting the
   * pitch line at very different point densities - a 3x jump that reads as a
   * 15 degree corner exactly where the curvature is highest.
   */
  function resampleByArc(fn, uEnd, n) {
    var M = 400, i, pts = [], cum = [0];
    for (i = 0; i <= M; i++) pts.push(fn(uEnd * i / M));
    for (i = 1; i <= M; i++) {
      cum.push(cum[i - 1] + Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y));
    }
    var total = cum[M], out = [], j = 0;
    for (i = 0; i <= n; i++) {
      var target = total * i / n;
      while (j < M - 1 && cum[j + 1] < target) j++;
      var seg = cum[j + 1] - cum[j];
      var f = seg > 1e-15 ? (target - cum[j]) / seg : 0;
      if (f > 1) f = 1;
      out.push({
        x: pts[j].x + (pts[j + 1].x - pts[j].x) * f,
        y: pts[j].y + (pts[j + 1].y - pts[j].y) * f
      });
    }
    out[n] = pts[M];
    return out;
  }

  /* Arc length of a flank over [0, uEnd], used to share the sample budget out
   * between the two flanks in proportion to how much curve each one has. */
  function flankLength(fn, uEnd) {
    var M = 120, L = 0, prev = fn(0), i;
    for (i = 1; i <= M; i++) {
      var q = fn(uEnd * i / M);
      L += Math.hypot(q.x - prev.x, q.y - prev.y);
      prev = q;
    }
    return L;
  }

  /* Closest approach of a point to a flank: coarse grid scan, then a ternary
   * refine on the bracketing interval so the tangency parameter is resolved far
   * below grid resolution. A grid-only answer leaves a visible micro-kink where
   * the cap or fillet meets the flank. */
  function closestOnFlank(fn, cx, cy, nGrid) {
    var n = nGrid || 96, best = Infinity, bu = 0, i;
    function dist(u) { var q = fn(u); return Math.hypot(q.x - cx, q.y - cy); }
    for (i = 0; i <= n; i++) {
      var dd = dist(i / n);
      if (dd < best) { best = dd; bu = i / n; }
    }
    var lo = Math.max(0, bu - 1 / n), hi = Math.min(1, bu + 1 / n);
    for (i = 0; i < 40; i++) {
      var m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
      if (dist(m1) < dist(m2)) hi = m2; else lo = m1;
    }
    var u = 0.5 * (lo + hi);
    return { d: dist(u), u: u };
  }

  /*
   * S-TOOTH tip cap: the circle of radius tipHalfWidth centred on the tooth axis,
   * tangent to the flank, with its apex at y = ha. The S-tooth's arc flank runs
   * into the axis at the pressure angle and needs this; the cycloid's crest is
   * already flat and round there and is left alone (see note 4 in the header).
   */
  function fitTipCap(d, profile) {
    // tipHalfWidth IS the tip radius. The cap is the circle of that radius
    // centred on the tooth axis and INSCRIBED against the flank, and the flank's
    // full height is solved so the cap apexes at exactly ha. Two nested
    // bisections: the inner one slides the centre up the axis until the circle
    // touches the flank, the outer one stretches the flank until the apex lands
    // on ha.
    var r = d.tipHW;
    if (!(r > 0)) return null;
    function capFor(haFull) {
      var fn = flankFn(d, profile, haFull);
      // Slide the centre up the axis until the circle touches the flank. Far
      // below the tooth the clearance is huge; level with the flank's own point
      // it is zero, so the touch height is bracketed.
      var lo = -4 * d.ha - r, hi = haFull, i;
      if (closestOnFlank(fn, 0, lo).d < r) return null;
      if (closestOnFlank(fn, 0, hi).d > r) return null;
      for (i = 0; i < 40; i++) {
        var m = 0.5 * (lo + hi);
        if (closestOnFlank(fn, 0, m).d > r) lo = m; else hi = m;
      }
      var cy = 0.5 * (lo + hi);
      var hit = closestOnFlank(fn, 0, cy);
      return { r: r, cy: cy, apex: cy + r, u: hit.u, tipY: haFull };
    }
    var lo = 1e-3, hi = d.ha * 6, i, c;
    var cLo = capFor(lo), cHi = capFor(hi);
    if (!cLo || !cHi || cLo.apex > d.ha || cHi.apex < d.ha) return null;
    for (i = 0; i < 40; i++) {
      var mid = 0.5 * (lo + hi);
      c = capFor(mid);
      if (!c) return null;
      if (c.apex < d.ha) lo = mid; else hi = mid;
    }
    c = capFor(0.5 * (lo + hi));
    return (c && c.u < 0.9999 && c.r > 0) ? c : null;
  }

  /*
   * Root fillet: an arc of radius rf tangent to the dedendum flank and to the
   * root land (the line y = -hd). Its centre sits at rf above the root land, on
   * the void side of the flank, so we walk the flank until the offset point
   * reaches that height. Tangency to both surfaces then holds by construction.
   */
  function fitRootFillet(hd, flankFn, rf) {
    if (!(rf > 0)) return null;
    var target = -hd + rf, i;
    var h = 1e-5;
    function centreAt(u) {
      var q = flankFn(u);
      var a = flankFn(Math.max(0, u - h)), b = flankFn(Math.min(1, u + h));
      var tx = b.x - a.x, ty = b.y - a.y, L = Math.hypot(tx, ty) || 1;
      // normal towards the void (+x side of a flank running pitch -> root)
      var nx = -ty / L, ny = tx / L;
      return { x: q.x + rf * nx, y: q.y + rf * ny, px: q.x, py: q.y };
    }
    var lo = 0, hi = 1;
    if (centreAt(lo).y < target || centreAt(hi).y > target) return null;
    for (i = 0; i < 80; i++) {
      var mid = 0.5 * (lo + hi);
      if (centreAt(mid).y > target) lo = mid; else hi = mid;
    }
    var c = centreAt(0.5 * (lo + hi));
    return { cx: c.x, cy: target, r: rf, u: 0.5 * (lo + hi), px: c.px, py: c.py };
  }

  /*
   * One full flexspline tooth polyline in local tooth coordinates, ordered from
   * the left root-land junction, up the left flank, across the tip cap, down the
   * right flank to the right root-land junction. `inflate` offsets the profile
   * outward along its normal (used to cut clearance into the conjugate splines).
   */
  /*
   * The profile-defining solve: flank height, tip cap and root fillet. It
   * depends only on the tooth-shape parameters, never on sampling density or
   * clearance, so it is memoised - the nested bisections are far too expensive
   * to redo on every toothProfile() call (conjugateSlot alone would trigger
   * several per regenerate).
   */
  var geomCache = new Map();
  function toothGeometry(d, kind) {
    var key = [kind, d.m, d.ha, d.hd, d.tipHW, d.rootFillet, d.pa, d.tE, d.s0].join('|');
    var g = geomCache.get(key);
    if (g) return g;

    // Cycloid crests are round by construction and must not be capped: the cap
    // would replace conjugate flank with an arc that is conjugate to nothing.
    var cap = (kind === 's') ? fitTipCap(d, kind) : null;
    var haFull = cap ? cap.tipY : d.ha;
    var addFn = flankFn(d, kind, haFull);
    var dedFn = dedendumFn(d, addFn);
    // The point-symmetric dedendum bottoms out at -haFull, so a deeper root is
    // not constructible; clamp rather than emit a broken tooth. validate()
    // surfaces this to the user before it gets here.
    var hdEff = Math.min(d.hd, haFull);
    // At FULL depth (hd = ha, the default) the two flanks meet at the root crest:
    // the root land has zero width, and that crest is already round - same radius
    // as the tip, by the point symmetry - so there is nothing for a fillet to
    // blend. fitRootFillet would only chase a degenerate tangency at u = 1.
    var atFullDepth = hdEff > haFull - 1e-9;
    var rf = atFullDepth ? 0 : Math.min(d.rootFillet, hdEff * 0.45);
    var fil = rf > 0 ? fitRootFillet(hdEff, dedFn, rf) : null;

    g = { cap: cap, haFull: haFull, hdEff: hdEff, addFn: addFn, dedFn: dedFn, fil: fil };
    if (geomCache.size > 64) geomCache.clear();
    geomCache.set(key, g);
    return g;
  }

  function toothProfile(d, inflate, samplesPerFlank, kind) {
    kind = kind || 'cycloid';
    var n = Math.max(6, samplesPerFlank || 24);
    var g = toothGeometry(d, kind);
    var cap = g.cap, fil = g.fil, addFn = g.addFn, dedFn = g.dedFn;
    var tipY = g.haFull, hdEff = g.hdEff;

    var uTip = cap ? cap.u : 1;
    var uRoot = fil ? fil.u : 1;

    var right = [], i;
    // Share the sample budget by arc length, then sample each flank evenly along
    // its own arc so both meet the pitch line at the same point density.
    var Ld = flankLength(dedFn, uRoot), La = flankLength(addFn, uTip);
    var nd = Math.max(3, Math.round(n * Ld / (Ld + La)));
    var na = Math.max(3, n - nd);
    var ded = resampleByArc(dedFn, uRoot, nd);
    var arcStep = Math.PI / Math.max(8, n / 2);   // consistent angular resolution
    // root fillet arc, from the flank tangency round to the root land. The
    // centre sits rf above the root land, so the arc ends straight below it.
    var filArc = [];
    if (fil) {
      var a0 = Math.atan2(fil.py - fil.cy, fil.px - fil.cx);
      var a1 = -Math.PI / 2;                      // straight down from the centre
      var da = a1 - a0;
      while (da > Math.PI) da -= 2 * Math.PI;
      while (da < -Math.PI) da += 2 * Math.PI;
      var nf = Math.max(4, Math.ceil(Math.abs(da) / arcStep));
      for (i = 1; i <= nf; i++) {
        var aa = a0 + da * (i / nf);
        filArc.push({ x: fil.cx + fil.r * Math.cos(aa), y: fil.cy + fil.r * Math.sin(aa) });
      }
    } else {
      filArc.push(dedFn(uRoot));
    }
    // addendum: pitch -> tip tangency (drop index 0, the shared pitch point)
    var add = resampleByArc(addFn, uTip, na).slice(1);

    // right flank, root-land junction -> tip tangency
    for (i = filArc.length - 1; i >= 0; i--) right.push(filArc[i]);
    for (i = ded.length - 1; i >= 0; i--) right.push(ded[i]);
    for (i = 0; i < add.length; i++) right.push(add[i]);

    // assemble: mirror the right flank for the left, then the tip cap. Uncapped
    // (cycloid), both flanks end ON the tooth axis at the same apex, so the left
    // copy stops one short of it rather than emitting the point twice.
    var pts = [];
    var nLeft = cap ? right.length : right.length - 1;
    for (i = 0; i < nLeft; i++) pts.push({ x: -right[i].x, y: right[i].y });
    if (cap) {
      // Sweep from the left tangency to the right one, always OVER the top.
      // Deriving both endpoints from atan2 invites a -0 sign flip that sends the
      // arc the long way round through the bottom, so take the right tangency
      // angle in [-pi/2, pi/2] and mirror it.
      var tip = right[right.length - 1];
      var phi = Math.atan2(tip.y - cap.cy, Math.abs(tip.x));
      var nc = Math.max(6, Math.ceil(Math.abs(Math.PI - 2 * phi) / arcStep));
      for (i = 1; i < nc; i++) {
        var ang = (Math.PI - phi) + (phi - (Math.PI - phi)) * (i / nc);
        pts.push({ x: cap.r * Math.cos(ang), y: cap.cy + cap.r * Math.sin(ang) });
      }
    }
    for (i = right.length - 1; i >= 0; i--) pts.push({ x: right[i].x, y: right[i].y });

    if (inflate && inflate !== 0) pts = offsetPolyline(pts, inflate);
    return pts;
  }

  /* Offset an open polyline along its (outward) normals. */
  function offsetPolyline(pts, dist) {
    var out = [];
    for (var i = 0; i < pts.length; i++) {
      var a = pts[Math.max(0, i - 1)];
      var b = pts[Math.min(pts.length - 1, i + 1)];
      var dx = b.x - a.x, dy = b.y - a.y;
      var len = Math.hypot(dx, dy) || 1;
      out.push({ x: pts[i].x - (dy / len) * dist, y: pts[i].y + (dx / len) * dist });
    }
    return out;
  }

  /* The tooth plus the root land out to half a flexspline pitch on each side.
   * This is what actually sweeps through a spline slot, so it is what the
   * conjugate envelope has to be built from - rev A swept the tooth alone and
   * left the root land free to collide with the spline tooth tip. */
  function toothWithRootLand(d, inflate, samplesPerFlank, kind) {
    var tooth = toothProfile(d, inflate, samplesPerFlank, kind);
    var yRoot = -toothGeometry(d, kind || 'cycloid').hdEff + (inflate || 0);
    var xEnd = d.halfPitch;
    var pts = [], i, nL = 10;
    var xL = tooth[0].x, xR = tooth[tooth.length - 1].x;
    // Land from -xEnd up to the tooth's own left junction xL. The span is
    // (xL + xEnd); writing it as (-xL + xEnd) ran the "land" forward across most
    // of the tooth at root depth instead. It was invisible because the envelope
    // takes the MAX radius per bin and the tooth sits above it everywhere it
    // trespassed, but it was not the polyline this is documented to sweep.
    // At full depth both lands are empty and are skipped rather than emitted as
    // a stack of coincident points.
    if (xL + xEnd > 1e-9) {
      for (i = 0; i < nL; i++) pts.push({ x: -xEnd + (xL + xEnd) * (i / nL), y: yRoot });
    }
    for (i = 0; i < tooth.length; i++) pts.push(tooth[i]);
    if (xEnd - xR > 1e-9) {
      for (i = 1; i <= nL; i++) pts.push({ x: xR + (xEnd - xR) * (i / nL), y: yRoot });
    }
    return pts;
  }

  /*
   * Full (undeformed, as-manufactured) flexspline 2D profile.
   * Returns { outer: [{x,y}...] closed CCW, innerRadius: r }.
   */
  function flexsplineProfile(p) {
    var d = derive(p);
    var tooth = toothProfile(d, 0, 48, p.profile);
    var rootR = d.rp - toothGeometry(d, p.profile || 'cycloid').hdEff;
    var pitchAngle = 2 * Math.PI / p.zf;
    var outer = [];
    var k, i;

    for (k = 0; k < p.zf; k++) {
      var theta0 = k * pitchAngle;
      for (i = 0; i < tooth.length; i++) {
        var ang = theta0 + tooth[i].x / d.rp;
        var r = d.rp + tooth[i].y;
        outer.push({ x: r * Math.cos(ang), y: r * Math.sin(ang) });
      }
      // Root land between this tooth and the next. At full depth the flanks
      // already meet at the root crest and this span is empty - emitting it would
      // stack ten copies of one vertex in the exported polyline.
      var a0 = theta0 + tooth[tooth.length - 1].x / d.rp;
      var a1 = theta0 + pitchAngle + tooth[0].x / d.rp;
      var arcN = 10;
      if ((a1 - a0) * rootR > 1e-9) {
        for (i = 1; i < arcN; i++) {
          var aa = a0 + (a1 - a0) * (i / arcN);
          outer.push({ x: rootR * Math.cos(aa), y: rootR * Math.sin(aa) });
        }
      }
    }
    return {
      outer: outer,
      innerRadius: rootR - p.wallFlex,
      tipRadius: d.rp + d.ha,
      rootRadius: rootR
    };
  }

  /*
   * Conjugate tooth-space envelope for a rigid spline with zs teeth
   * (zs = zc for the circular spline, zs = zf for the pancake dynamic spline).
   */
  function conjugateSlot(p, zs, bins, clearance) {
    var d = derive(p);
    var clr = (clearance != null) ? clearance : d.clearance;
    var nBins = bins || 256;
    var pitch = 2 * Math.PI / zs;
    var halfPitch = pitch / 2;
    var dz = zs - p.zf; // 0 for dynamic spline, >0 for circular spline
    var tooth = toothWithRootLand(d, clr, 32, p.profile);
    var env = new Float64Array(nBins);
    for (var b = 0; b < nBins; b++) env[b] = 0;

    var steps = 800;
    var phiMax = Math.PI / 2 / (1 + dz / p.zf);

    for (var s = 0; s <= steps; s++) {
      var phi = -phiMax + (2 * phiMax) * (s / steps);
      var thetaBody = -phi * dz / p.zf;        // gear-ratio drift of the FS body
      var psi = thetaBody - phi;               // tooth angle from the WG major axis
      var def = deform(d, psi);
      var theta = thetaBody + def.v / d.rm;     // tooth centre angle in spline frame
      var R = def.rho;
      var cosM = Math.cos(def.mu), sinM = Math.sin(def.mu);

      var prevA = null, prevR = null;
      for (var i = 0; i < tooth.length; i++) {
        var sec = rotateIntoSection(tooth[i].x, tooth[i].y, cosM, sinM);
        var pl = polarPlace(R, theta, sec.t, sec.r);
        if (prevA !== null) rasterizeSegment(env, nBins, halfPitch, prevA, prevR, pl.ang, pl.rad);
        prevA = pl.ang; prevR = pl.rad;
      }
    }

    // Enforce the symmetry the kinematics guarantee.
    for (var j = 0; j < nBins / 2; j++) {
      var mj = nBins - 1 - j;
      var mx = Math.max(env[j], env[mj]);
      env[j] = mx; env[mj] = mx;
    }

    // Spline tooth-tip floor: angular bins the flexspline never reaches must
    // still sit clear of the flexspline tip on the minor axis. Blend with a
    // smooth (hyperbolic) max rather than a hard clamp: a hard clamp leaves the
    // crest meeting the swept-envelope flanks at a sharp cusp. Because
    // soft(e, f) >= max(e, f) >= rFloor, the minor-axis tip clearance is always
    // preserved; the blend only ever adds clearance, never removes it.
    var rFloor = deform(d, Math.PI / 2).rho + d.ha + clr;
    var blend = Math.max(clr, 0.02 * d.m);
    for (var k = 0; k < nBins; k++) {
      var e = env[k] - rFloor;
      env[k] = rFloor + 0.5 * (e + Math.sqrt(e * e + blend * blend));
    }

    return { radii: env, halfPitch: halfPitch, nBins: nBins };
  }

  /* Accumulate a line segment into the angular max-radius envelope. Anything
   * that leaves this pitch window belongs to a neighbouring slot, and by
   * periodicity that slot is this one, so the segment is replayed shifted by
   * whole pitches rather than discarded. */
  function rasterizeSegment(env, nBins, halfPitch, a0, r0, a1, r1) {
    if (a0 > a1) { var t = a0; a0 = a1; a1 = t; t = r0; r0 = r1; r1 = t; }
    var pitch = 2 * halfPitch;
    var kLo = Math.floor((-halfPitch - a1) / pitch);
    var kHi = Math.ceil((halfPitch - a0) / pitch);
    for (var k = kLo; k <= kHi; k++) {
      rasterizeCore(env, nBins, halfPitch, a0 + k * pitch, r0, a1 + k * pitch, r1);
    }
  }
  function rasterizeCore(env, nBins, halfPitch, a0, r0, a1, r1) {
    if (a1 < -halfPitch || a0 > halfPitch) return;
    var binW = (2 * halfPitch) / nBins;
    var b0 = Math.max(0, Math.floor((a0 + halfPitch) / binW));
    var b1 = Math.min(nBins - 1, Math.floor((a1 + halfPitch) / binW));
    for (var b = b0; b <= b1; b++) {
      var ac = -halfPitch + (b + 0.5) * binW;
      var r;
      if (a1 - a0 < 1e-12) r = Math.max(r0, r1);
      else {
        var f = (ac - a0) / (a1 - a0);
        f = Math.max(0, Math.min(1, f));
        r = r0 + f * (r1 - r0);
      }
      if (r > env[b]) env[b] = r;
    }
  }

  /*
   * Full internal-spline 2D profile (circular spline or dynamic spline).
   */
  function splineProfile(p, zs, wall, clearance) {
    var slot = conjugateSlot(p, zs, null, clearance);
    var inner = [];
    var maxR = 0;
    var pitch = 2 * Math.PI / zs;
    for (var k = 0; k < zs; k++) {
      var base = k * pitch;
      for (var b = 0; b < slot.nBins; b++) {
        var ang = base - slot.halfPitch + (b + 0.5) * (2 * slot.halfPitch / slot.nBins);
        var r = slot.radii[b];
        if (r > maxR) maxR = r;
        inner.push({ x: r * Math.cos(ang), y: r * Math.sin(ang) });
      }
    }
    return {
      inner: inner,
      outerRadius: maxR + wall,
      slotBottomRadius: maxR
    };
  }

  /*
   * Undeformed flexspline outline as polar arrays, captured once (geometry is
   * omega-independent) so the animation only does the cheap per-point deform per
   * frame. `outer` is the full continuous profile (teeth AND root lands between
   * them); `inner` samples the bore circle.
   */
  function flexsplineRest(p) {
    var d = derive(p);
    var prof = flexsplineProfile(p); // continuous outer loop + bore radius
    var nOut = prof.outer.length;
    var outerR = new Float64Array(nOut), outerA = new Float64Array(nOut);
    for (var i = 0; i < nOut; i++) {
      var pt = prof.outer[i];
      outerR[i] = Math.hypot(pt.x, pt.y);
      outerA[i] = Math.atan2(pt.y, pt.x);
    }
    var nIn = Math.max(160, p.zf * 2);
    var innerR = new Float64Array(nIn), innerA = new Float64Array(nIn);
    for (i = 0; i < nIn; i++) {
      innerR[i] = prof.innerRadius;
      innerA[i] = (2 * Math.PI) * (i / nIn) - Math.PI;
    }
    return {
      outerR: outerR, outerA: outerA, innerR: innerR, innerA: innerA,
      rm: d.rm, a: d.a, b: d.b, w0: d.w0, dz: p.zc - p.zf, zf: p.zf
    };
  }

  /*
   * Deform a rest outline (from flexsplineRest) by the wave generator at input
   * angle `omega`. Every point is mapped through ONE continuous displacement
   * field (the elliptical neutral-line motion (rho, v) plus the section rotation
   * mu of Eqs. 4/8) evaluated at that point's own angle:
   *   - a point at body angle a sits at psi = (a + phiFs) - omega from the major
   *     axis, where phiFs = -omega*dz/zf is the slow flexspline-body rotation;
   *   - its height h above the neutral line rides the section: it lands at radius
   *     rho + h*cos(mu) with a tangential offset +h*sin(mu).
   * One smooth map of one continuous curve, so teeth and root lands always join
   * with no per-tooth seams.
   */
  function deformFlexspline(rest, omega) {
    var d = { a: rest.a, b: rest.b, rm: rest.rm, w0: rest.w0 };
    var rm = rest.rm;
    var phiFs = -omega * rest.dz / rest.zf;

    function mapLoop(rArr, aArr) {
      var out = new Array(rArr.length);
      for (var i = 0; i < rArr.length; i++) {
        var aBody = aArr[i] + phiFs;
        var h = rArr[i] - rm;
        var def = deform(d, aBody - omega);
        var cosM = Math.cos(def.mu), sinM = Math.sin(def.mu);
        var sec = rotateIntoSection(0, h, cosM, sinM);
        var R = def.rho, T = aBody + def.v / rm;
        var rad = R + sec.r;
        var ang = T + sec.t / (rad || 1);
        out[i] = { x: rad * Math.cos(ang), y: rad * Math.sin(ang) };
      }
      return out;
    }

    return {
      outer: mapLoop(rest.outerR, rest.outerA),
      inner: mapLoop(rest.innerR, rest.innerA),
      phiFs: phiFs, a: rest.a, b: rest.b
    };
  }

  /* Convenience: deform straight from params (rebuilds the rest outline). */
  function deformedFlexspline(p, omega) {
    return deformFlexspline(flexsplineRest(p), omega);
  }

  /* Assemble everything needed for a given configuration. */
  function generate(p) {
    var d = derive(p);
    var parts = { flexspline: flexsplineProfile(p) };
    if (p.style === 'pancake') {
      parts.circularFixed = splineProfile(p, p.zc, p.wallCirc, d.clearance);
      parts.circularOutput = splineProfile(p, p.zf, p.wallDyn, d.clearanceOutput);
    } else {
      parts.circular = splineProfile(p, p.zc, p.wallCirc, d.clearance);
    }
    return parts;
  }

  /*
   * Geometric backlash for one spline, as lost motion at the flexspline pitch
   * line (mm). We measure the LOAD-CARRYING tooth: the one at the major axis
   * (psi = 0), fully engaged. Holding the wave generator fixed, we rotate the
   * rigid spline about its axis until one of that tooth's flanks contacts; the
   * angular span over which it still fits, times the pitch radius, is the lost
   * motion you feel under load.
   */
  function measureBacklash(p, zs, clr) {
    var d = derive(p);
    var slot = conjugateSlot(p, zs, 1024, clr);
    var env = slot.radii, nBins = slot.nBins;
    var hp = slot.halfPitch, binW = 2 * hp / nBins, csPitch = 2 * hp;
    var tooth = toothProfile(d, 0, 28, p.profile); // actual (un-inflated) FS tooth

    function envAt(ang) { // slot radius at a local (within-pitch) angle, clamped
      var f = (ang + hp) / binW - 0.5;
      var i0 = Math.floor(f), t = f - i0;
      var lo = i0 < 0 ? 0 : (i0 >= nBins ? nBins - 1 : i0);
      var hi = i0 + 1 < 0 ? 0 : (i0 + 1 >= nBins ? nBins - 1 : i0 + 1);
      return env[lo] + (env[hi] - env[lo]) * t;
    }

    var def = deform(d, 0), R = def.rho;
    var cosM = Math.cos(def.mu), sinM = Math.sin(def.mu);
    var ang = [], rad = [];
    for (var i = 0; i < tooth.length; i++) {
      var sec = rotateIntoSection(tooth[i].x, tooth[i].y, cosM, sinM);
      var pl = polarPlace(R, 0, sec.t, sec.r);
      ang.push(pl.ang);
      rad.push(pl.rad);
    }

    function minGap(phi) {
      var g = Infinity;
      for (var j = 0; j < ang.length; j++) {
        var m = (ang[j] - phi) % csPitch;
        if (m > hp) m -= csPitch; else if (m < -hp) m += csPitch;
        var gg = envAt(m) - rad[j];
        if (gg < g) g = gg;
      }
      return g;
    }

    var nScan = 160, dphi = 2 * hp / nScan;
    var best = -Infinity, bestI = 0, gs = new Float64Array(nScan + 1);
    for (var s = 0; s <= nScan; s++) {
      gs[s] = minGap(-hp + s * dphi);
      if (gs[s] > best) { best = gs[s]; bestI = s; }
    }
    if (best <= 0) return 0; // interfering / degenerate
    function wall(dir) {
      var s2 = bestI;
      while (s2 + dir >= 0 && s2 + dir <= nScan && gs[s2 + dir] > 0) s2 += dir;
      var inPhi = -hp + s2 * dphi;
      var outPhi = inPhi + dir * dphi;
      if (s2 + dir < 0 || s2 + dir > nScan) return inPhi;
      for (var it = 0; it < 22; it++) {
        var mid = 0.5 * (inPhi + outPhi);
        if (minGap(mid) > 0) inPhi = mid; else outPhi = mid;
      }
      return inPhi;
    }
    return (wall(+1) - wall(-1)) * d.rp;
  }

  /*
   * Performance metrics for a generated design. `parts` is optional; when
   * supplied the outer diameter is read from the actual spline geometry.
   */
  function metrics(p, parts) {
    var d = derive(p);
    var dz = p.zc - p.zf;
    var ratio = dz !== 0 ? Math.abs(p.zf / dz) : Infinity;
    var pitchDia = p.module * p.zf;
    var spline = parts && (parts.circular || parts.circularFixed);
    var outerDia = spline ? 2 * spline.outerRadius : pitchDia + 2 * (d.w0 + d.ha + p.wallCirc);

    var psiC = 0.5 * Math.acos(0.9);                  // w(psi) = 0.9*w0
    var teethMesh = Math.round(p.zf * (2 * psiC) / Math.PI);

    var backlashFixed = measureBacklash(p, p.zc, d.clearance);
    var backlashOutput = p.style === 'pancake'
      ? measureBacklash(p, p.zf, d.clearanceOutput)
      : null;
    var backlash = backlashOutput != null
      ? Math.max(backlashFixed, backlashOutput)
      : backlashFixed;

    // Peak flexspline rim bending strain under elliptical deflection
    // (kappa ~ 3*w0/rm^2 at the major axis).
    var fsStrainPct = (p.wallFlex / 2) * (3 * d.w0 / (d.rm * d.rm)) * 100;

    // The governing design inequality: the tooth has to be tall enough to cover
    // the radial stroke, or the spline tooth tip bottoms out in the flexspline
    // root land at the major axis.
    var stroke = d.a - d.b;
    var strokeMargin = (d.ha + d.hd + d.clearance) - stroke;

    var tipR = tipRadius(p);

    // Flat root land left between neighbouring teeth, measured off the actual
    // tooth polyline rather than a closed form so it accounts for the root
    // fillet and for a shortened dedendum, not just the thickness. Zero at the
    // default (full depth, 50/50 thickness), where the flanks meet at the root
    // crest; thinning the tooth or lifting the root opens it up.
    var tp = toothProfile(d, 0, 28, p.profile);
    var rootLand = Math.max(0,
      2 * d.halfPitch - (tp[tp.length - 1].x - tp[0].x));

    return {
      ratio: ratio,                 // i : 1
      pitchDia: pitchDia,           // mm
      outerDia: outerDia,           // mm
      w0: d.w0,                     // mm
      w0Ratio: p.w0Ratio,           // w0*
      deflPct: d.w0 / d.rm * 100,   // radial deflection, % of pitch radius
      teethMesh: teethMesh,         // teeth in load contact (estimate)
      teethPct: 100 * teethMesh / p.zf,
      backlash: backlash,           // mm (governing spline, geometric)
      backlashFixed: backlashFixed, // mm (fixed CS, zc)
      backlashOutput: backlashOutput, // mm (output CS, zf; null for cup)
      fsStrainPct: fsStrainPct,     // % (estimate)
      toothHeight: d.ha + d.hd,     // mm
      tipRadius: tipR,              // mm (cycloid: derived, not an input)
      toothThickness: 2 * d.s0,     // mm, at the pitch line
      circularPitch: 2 * d.halfPitch, // mm, pi*m
      rootLand: rootLand,           // mm, flat between neighbouring teeth
      toothAngle: d.alpha0 * 180 / Math.PI, // deg, as clamped
      stroke: stroke,               // mm, a - b
      strokeMargin: strokeMargin    // mm, must stay > 0
    };
  }

  /*
   * The tooth's tip radius (mm): an INPUT for the S-tooth, whose cap radius is
   * tipHalfWidth, and a RESULT for the cycloid, whose crest is the naturally
   * round flat end of Eq. (11). Split out of metrics() so the UI can bound the
   * clearance offset against it on every keystroke without also paying for the
   * backlash sweep, which is the expensive half of metrics(). toothGeometry is
   * memoised, so repeated calls at fixed shape parameters are nearly free.
   */
  function tipRadius(p) {
    var d = derive(p);
    var g = toothGeometry(d, p.profile || 'cycloid');
    return g.cap ? g.cap.r : crestRadius(g.addFn);
  }

  /* Ceiling on the cycloid tooth angle for a given tooth, in degrees. derive()
   * clamps to just under it; the UI validates against it so the clamp never
   * silently overrides what someone typed. The module cancels, leaving
   * atan(s0* / 2*ha*) where s0* = toothThickness * pi / 2 - so 21.4 deg at the
   * default ha* = 1 and toothThickness = 0.5, and lower on a thinned tooth,
   * which has less flank to truncate in the first place. */
  function toothAngleLimit(p) {
    var thick = Math.min(0.5, Math.max(1e-3,
      p.toothThickness != null ? p.toothThickness : 0.5));
    return maxToothAngle(thick * Math.PI / 2, p.ha) * 180 / Math.PI;
  }

  global.CTP = {
    REV: 'B',
    DEFAULTS: DEFAULTS,
    derive: derive,
    tipRadius: tipRadius,
    toothAngleLimit: toothAngleLimit,
    deform: deform,
    toothProfile: toothProfile,
    toothWithRootLand: toothWithRootLand,
    flexsplineProfile: flexsplineProfile,
    conjugateSlot: conjugateSlot,
    splineProfile: splineProfile,
    flexsplineRest: flexsplineRest,
    deformFlexspline: deformFlexspline,
    deformedFlexspline: deformedFlexspline,
    measureBacklash: measureBacklash,
    generate: generate,
    metrics: metrics
  };
})(typeof window !== 'undefined' ? window : module.exports);
