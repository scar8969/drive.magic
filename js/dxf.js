/*
 * Minimal DXF R12 (AC1009) writer. Emits closed POLYLINE entities (and true
 * CIRCLE entities for round loops), which import cleanly into Onshape, Fusion
 * 360, FreeCAD, SolidWorks, LightBurn and most slicers. Units are millimetres.
 *
 * buildDxf(loops, opts) accepts a mix of:
 *   - point arrays [{x, y}, ...]            -> closed POLYLINE
 *   - circle descriptors { circle: radius } -> true CIRCLE entity (1 entity,
 *                                               not a 256-gon, best for CAD)
 * opts.tol (mm) runs Ramer-Douglas-Peucker simplification on the polyline
 * loops, collapsing near-collinear runs while holding every original point
 * within `tol` of the kept outline. tol = 0 (default) keeps full resolution.
 */
(function (global) {
  'use strict';

  function fmt(v) {
    return Number(v.toFixed(6)).toString();
  }

  /* Perpendicular distance from p to the infinite line through a and b. */
  function lineDist(p, a, b) {
    var dx = b.x - a.x, dy = b.y - a.y;
    var len = Math.hypot(dx, dy);
    if (len < 1e-12) return Math.hypot(p.x - a.x, p.y - a.y);
    return Math.abs((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
  }

  /* Ramer-Douglas-Peucker on an open polyline (endpoints always kept).
   * Iterative (explicit stack) so large profiles can't overflow recursion. */
  function rdpOpen(pts, tol) {
    var n = pts.length;
    if (n < 3) return pts.slice();
    var keep = new Array(n);
    keep[0] = keep[n - 1] = true;
    var stack = [[0, n - 1]];
    while (stack.length) {
      var seg = stack.pop(), s = seg[0], e = seg[1];
      var maxD = -1, idx = -1;
      for (var i = s + 1; i < e; i++) {
        var dd = lineDist(pts[i], pts[s], pts[e]);
        if (dd > maxD) { maxD = dd; idx = i; }
      }
      if (idx !== -1 && maxD > tol) {
        keep[idx] = true;
        stack.push([s, idx]);
        stack.push([idx, e]);
      }
    }
    var out = [];
    for (var k = 0; k < n; k++) if (keep[k]) out.push(pts[k]);
    return out;
  }

  /* RDP for a CLOSED loop (no repeated first/last vertex). Anchored at vertex 0
   * and the vertex farthest from it, so the wrap-around seam never cuts through
   * a feature and both chains are simplified consistently. */
  function simplifyClosed(pts, tol) {
    var n = pts.length;
    if (!tol || n < 5) return pts;
    var far = 0, maxd = -1;
    for (var i = 1; i < n; i++) {
      var d2 = (pts[i].x - pts[0].x) * (pts[i].x - pts[0].x) +
               (pts[i].y - pts[0].y) * (pts[i].y - pts[0].y);
      if (d2 > maxd) { maxd = d2; far = i; }
    }
    var chainA = pts.slice(0, far + 1);                 // 0 .. far
    var chainB = pts.slice(far).concat([pts[0]]);       // far .. n-1 .. 0
    var a = rdpOpen(chainA, tol);                        // [0, ..., far]
    var b = rdpOpen(chainB, tol);                        // [far, ..., 0]
    return a.concat(b.slice(1, b.length - 1));           // drop shared far & 0
  }

  /* loops: array of closed loops ({x,y} arrays) and/or circle descriptors. */
  function buildDxf(loops, opts) {
    opts = opts || {};
    var tol = opts.tol || 0;
    var out = [];
    function g(code, value) { out.push(code, value); }

    g(0, 'SECTION'); g(2, 'HEADER');
    g(9, '$ACADVER'); g(1, 'AC1009');
    g(9, '$INSUNITS'); g(70, 4); // millimetres
    g(0, 'ENDSEC');

    g(0, 'SECTION'); g(2, 'ENTITIES');
    loops.forEach(function (loop) {
      if (loop && loop.circle != null) {            // true circle entity
        g(0, 'CIRCLE');
        g(8, '0');
        g(10, fmt(loop.cx || 0));
        g(20, fmt(loop.cy || 0));
        g(30, '0');
        g(40, fmt(loop.circle));
        return;
      }
      var pts = tol ? simplifyClosed(loop, tol) : loop;
      g(0, 'POLYLINE');
      g(8, '0');      // layer
      g(66, 1);       // vertices follow
      g(70, 1);       // closed
      pts.forEach(function (p) {
        g(0, 'VERTEX');
        g(8, '0');
        g(10, fmt(p.x));
        g(20, fmt(p.y));
        g(30, '0');
      });
      g(0, 'SEQEND');
    });
    g(0, 'ENDSEC');
    g(0, 'EOF');

    return out.join('\r\n') + '\r\n';
  }

  /* Closed polyline approximation of a circle (kept for callers that want a
   * polygon rather than a true CIRCLE entity). */
  function circleLoop(radius, segments) {
    var n = segments || 256;
    var pts = [];
    for (var i = 0; i < n; i++) {
      var a = (2 * Math.PI * i) / n;
      pts.push({ x: radius * Math.cos(a), y: radius * Math.sin(a) });
    }
    return pts;
  }

  /* True-circle descriptor for buildDxf. */
  function circle(radius, cx, cy) {
    return { circle: radius, cx: cx || 0, cy: cy || 0 };
  }

  /* Total vertices a loop set will emit at a given tolerance (circles count
   * as 0, they're one analytic entity). Used by the UI to preview LOD. */
  function countVertices(loops, tol) {
    var total = 0;
    loops.forEach(function (loop) {
      if (loop && loop.circle != null) return;
      total += (tol ? simplifyClosed(loop, tol) : loop).length;
    });
    return total;
  }

  function download(filename, content) {
    var blob = new Blob([content], { type: 'application/dxf' });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    setTimeout(function () { URL.revokeObjectURL(url); }, 1000);
  }

  global.DXF = {
    buildDxf: buildDxf,
    circleLoop: circleLoop,
    circle: circle,
    simplifyClosed: simplifyClosed,
    countVertices: countVertices,
    download: download
  };
})(typeof window !== 'undefined' ? window : module.exports);
