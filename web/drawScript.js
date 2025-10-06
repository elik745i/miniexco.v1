const perspectiveConfig = {

	minWidth: 32,

	maxWidth: 1200,

	easingExponent: 1.2

};

const autoRoundConfig = {

	enabled: true,

	iterations: 10,

	minDistance: 1.2

};

function getAnchorLift() {

	return Math.max(perspectiveConfig.minWidth * 0.35, 12);

}

function normalizeAnchorPath(points, anchor, options = {}) {
if (!anchor || typeof anchor.x !== "number" || typeof anchor.y !== "number") {
	return Array.isArray(points) ? points.slice() : [];
}

// Tuning knobs
const tolerance    = Math.max(Number(options.tolerance ?? 0.5), 0.1);
const fallbackLift = Math.max(getAnchorLift(), 0.1);
const lift         = Math.max(Number(options.lift ?? fallbackLift), fallbackLift);

const safeAnchor = { x: Number(anchor.x) || 0, y: Number(anchor.y) || 0 };

// Sanitize inputs (finite numbers only)
const source = Array.isArray(points)
	? points
			.map(pt => ({ x: Number(pt?.x), y: Number(pt?.y) }))
			.filter(pt => Number.isFinite(pt.x) && Number.isFinite(pt.y))
	: [];

// Always start at the exact anchor; do not add the same point twice
const result = [{ x: safeAnchor.x, y: safeAnchor.y }];

// Skip any initial duplicates of the anchor
let cursor = 0;
while (
	cursor < source.length &&
	Math.abs(source[cursor].x - safeAnchor.x) <= tolerance &&
	Math.abs(source[cursor].y - safeAnchor.y) <= tolerance
) {
	cursor++;
}

// Determine the required vertical lift above anchor for the 2nd point
const targetY     = Math.max(0, safeAnchor.y - lift);        // full lift limit
const minHalfLift = Math.max(0, safeAnchor.y - lift * 0.5);  // at least half-lift

// Build/adjust the second point
let second = source[cursor];
if (!second) {
	// No user point yet -> create a clean upward segment
	second = { x: safeAnchor.x, y: targetY };
} else {
	const lockX = Math.abs(second.x - safeAnchor.x) <= tolerance;
	let y = second.y;

	// If flat/downward/too close to anchor, enforce at least half-lift upward
	if (y >= safeAnchor.y - tolerance || (safeAnchor.y - y) < (lift * 0.5)) {
		y = minHalfLift;
	}

	// Never allow the 2nd point to exceed the full lift upward clamp
	y = Math.min(y, targetY);

	second = { x: lockX ? safeAnchor.x : second.x, y };
}

result.push(second);

// Append the rest, skipping exact duplicates with the last appended point
for (let i = cursor + 1; i < source.length; i++) {
	const candidate = source[i];
	if (!candidate) continue;
	const last = result[result.length - 1];
	if (
		Math.abs(candidate.x - last.x) <= 1e-6 &&
		Math.abs(candidate.y - last.y) <= 1e-6
	) continue;

	result.push({ x: candidate.x, y: candidate.y });
}

return result;
}

function sanitizePoints(points, tolerance = autoRoundConfig.minDistance, dedupe = autoRoundConfig.enabled) {

	if (!Array.isArray(points)) return [];

	const cleaned = [];

	const safeTolerance = Number.isFinite(tolerance) && tolerance > 0 ? tolerance : 1;

	for (let i = 0; i < points.length; i++) {

		const source = points[i];

		if (!source) continue;

		const x = Number(source.x);

		const y = Number(source.y);

		if (!Number.isFinite(x) || !Number.isFinite(y)) continue;

		if (!dedupe || cleaned.length === 0) {

			cleaned.push({ x, y });

			continue;

		}

		const prev = cleaned[cleaned.length - 1];

		const dx = x - prev.x;

		const dy = y - prev.y;

		const dist = Math.hypot(dx, dy);

		if (dist >= safeTolerance) {

			cleaned.push({ x, y });

		} else if (i === points.length - 1 && (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6)) {

			cleaned[cleaned.length - 1] = { x, y };

		}

	}

	if (cleaned.length === 0) return [];

	return normalizeAnchorPath(cleaned, cleaned[0], { tolerance: safeTolerance, lift: getAnchorLift() });

}

function ensureAnchorHeadingUp(points, cfg = autoRoundConfig) {

	if (!Array.isArray(points) || points.length === 0) return [];

	const tolerance = Math.max(Number(cfg?.minDistance ?? 0.5), 0.5);

	const lift = Math.max(getAnchorLift(), tolerance);

	return normalizeAnchorPath(points, points[0], { tolerance, lift });

}

function autoRoundPoints(points, cfg = autoRoundConfig) {

	const sanitized = sanitizePoints(points, cfg.minDistance, cfg.enabled);

	if (!cfg.enabled || !Array.isArray(sanitized) || sanitized.length < 3 || cfg.iterations <= 0) {

		return ensureAnchorHeadingUp(sanitized, cfg);

	}

	let result = ensureAnchorHeadingUp(sanitized, cfg);

	for (let iter = 0; iter < cfg.iterations; iter++) {

		if (result.length < 3) break;

		const next = [result[0]];

		for (let i = 0; i < result.length - 1; i++) {

			const p0 = result[i];

			const p1 = result[i + 1];

			const q = {

				x: p0.x * 0.75 + p1.x * 0.25,

				y: p0.y * 0.75 + p1.y * 0.25

			};

			const r = {

				x: p0.x * 0.25 + p1.x * 0.75,

				y: p0.y * 0.25 + p1.y * 0.75

			};

			next.push(q, r);

		}

		next.push(result[result.length - 1]);

		result = ensureAnchorHeadingUp(next, cfg);

	}

	return result;

}

function getSurfaceBounds(surface) {

	if (!surface) return null;

	if (typeof surface.getBoundingClientRect === "function") {

		const rect = surface.getBoundingClientRect();

		return { width: rect.width, height: rect.height };

	}

	if (typeof surface.width === "number" && typeof surface.height === "number") {

		return { width: surface.width, height: surface.height };

	}

	return null;

}

function getPerspectiveWidth(y, bounds) {

	if (!bounds || !bounds.height) return perspectiveConfig.maxWidth;

	const safeY = typeof y === "number" ? y : 0;

	const normalized = Math.min(Math.max(bounds.height === 0 ? 0 : safeY / bounds.height, 0), 1);

	const eased = Math.pow(normalized, perspectiveConfig.easingExponent);

	return perspectiveConfig.minWidth + (perspectiveConfig.maxWidth - perspectiveConfig.minWidth) * eased;

}

function buildPerspectiveStrip(points, bounds) {
  if (!Array.isArray(points) || points.length < 2) return null;

  const safeBounds = bounds || { width: 0, height: 0 };
  const H = Number(safeBounds.height || 0);

  // centerline as you already compute it
  const centerline = autoRoundPoints(points);
  if (!Array.isArray(centerline) || centerline.length < 2) return null;

  const leftEdge  = [];
  const rightEdge = [];

  for (let i = 0; i < centerline.length; i++) {
    const p = centerline[i];

    // --- smoothed tangent (neighbors) ---
    let tx = 0, ty = 0;
    if (i > 0) { tx += p.x - centerline[i - 1].x; ty += p.y - centerline[i - 1].y; }
    if (i < centerline.length - 1) { tx += centerline[i + 1].x - p.x; ty += centerline[i + 1].y - p.y; }
    let mag = Math.hypot(tx, ty);
    if (mag < 1e-6) {                    // fallback to forward/back span
      const j = Math.min(centerline.length - 1, i + 1);
      const k = Math.max(0, i - 1);
      tx = centerline[j].x - centerline[k].x;
      ty = centerline[j].y - centerline[k].y;
      mag = Math.hypot(tx, ty) || 1;
    }
    tx /= mag; ty /= mag;

    // LEFT-hand normal (prevents flipping)
    const nx = -ty, ny = tx;

    // --- perspective width with Y taper (base untouched, top is 1/2) ---
    const baseW = getPerspectiveWidth(p.y, safeBounds);
    const t     = H > 0 ? Math.max(0, Math.min(1, (H - p.y) / H)) : 0; // 0 bottom, 1 top
    const yTaper = 1 - 0.5 * t;  // 1.0 at bottom -> 0.5 at top
    const trackW = baseW * yTaper;
    const half   = trackW * 0.5;

    leftEdge.push({  x: p.x - nx * half, y: p.y - ny * half });
    rightEdge.push({ x: p.x + nx * half, y: p.y + ny * half });
  }

  // outline uses right reversed to close (keeps compatibility with your code)
  const rightOutline = rightEdge.slice().reverse();
  return {
    outline: leftEdge.concat(rightOutline),
    leftEdge,
    rightEdge: rightOutline, // (reversed) matches your existing consumers
    centerline
  };
}

function drawPerspectiveStrip(ctxRef, points, bounds, styleOverrides = {}) {

	if (!ctxRef) return;

	const strip = buildPerspectiveStrip(points, bounds);

	if (!strip || strip.outline.length < 3) return;

	const style = Object.assign({

		fillStyle: "rgba(57, 255, 136, 0.22)",

		strokeStyle: "#39ff88",

		strokeWidth: 3.2

	}, styleOverrides || {});

	const { outline, centerline } = strip;

	ctxRef.save();

	ctxRef.fillStyle = style.fillStyle;

	ctxRef.beginPath();

	ctxRef.moveTo(outline[0].x, outline[0].y);

	for (let i = 1; i < outline.length; i++) {

		ctxRef.lineTo(outline[i].x, outline[i].y);

	}

	ctxRef.closePath();

	ctxRef.fill();

	ctxRef.strokeStyle = style.strokeStyle;

	ctxRef.lineWidth = style.strokeWidth;

	ctxRef.lineJoin = "round";

	ctxRef.lineCap = "round";

	const centerPath = Array.isArray(centerline) && centerline.length > 1 ? centerline : points;

	ctxRef.beginPath();

	ctxRef.moveTo(centerPath[0].x, centerPath[0].y);

	for (let i = 1; i < centerPath.length; i++) {

		ctxRef.lineTo(centerPath[i].x, centerPath[i].y);

	}

	ctxRef.stroke();

	ctxRef.restore();

}

// Triangle stream: speed ∝ size (smaller -> slower), no-overlap spacing in arc-length.
function attachTriangleStream(targetGroup, centerlinePoints, style = {}) {
  if (!targetGroup || !Array.isArray(centerlinePoints) || centerlinePoints.length < 2) return null;

  // ---- options ----
  const duration      = Math.max(400, Number(style.duration ?? 2400)); // ms for one loop
  const facing        = (style.facing || "down").toLowerCase();        // "up" | "down"
  const fill          = style.fill ?? "#ffe900";
  const fillOpacity   = Number.isFinite(style.fillOpacity) ? Number(style.fillOpacity) : 0.85;
  const stroke        = style.stroke ?? "#ffe900";
  const strokeWidth   = Math.max(0, Number(style.strokeWidth ?? 2));

  const widthFn       = typeof style.widthFn === "function" ? style.widthFn : null;
  const fallbackWidth = Number.isFinite(style.fallbackWidth) ? Number(style.fallbackWidth) : 60;
  const heightRatio   = Number.isFinite(style.heightRatio) ? Number(style.heightRatio) : 0.275;
  const edgeInset     = Number.isFinite(style.edgeInset) ? style.edgeInset : 0;

  const startOffsetPx = Math.max(0, Number(style.startOffsetPx ?? 120));
  const endOffsetPx   = Math.max(0, Number(style.endOffsetPx   ?? 110));
  const cullNarrowPx  = Math.max(0, Number(style.cullNarrowPx  ?? 8));

  const gapRatio      = Number.isFinite(style.gapRatio) ? style.gapRatio : 0.35; // gap vs height
  const minSpacingPx  = Math.max(6, Number(style.minSpacingPx ?? 18));

  // size->speed scaling: scale = minScale + (1 - minScale) * (H/Hmax)^exp
  // => smaller H => scale closer to minScale (slower), larger H => scale→1 (faster)
  const speedExponent = Number.isFinite(style.speedExponent) ? style.speedExponent : 1.2;
  const minScale      = Number.isFinite(style.minScale) ? style.minScale : 0.35;

  // lane edges (forward order preferred)
  const leftEdge  = Array.isArray(style.laneLeft)  && style.laneLeft.length  > 1 ? style.laneLeft  : null;
  const rightEdge = Array.isArray(style.laneRight) && style.laneRight.length > 1 ? style.laneRight : null;

  // ---- centerline arc-length table ----
  function buildTable(pts){ const segs=[]; let total=0;
    for (let i=0;i<pts.length-1;i++){
      const a=pts[i], b=pts[i+1], dx=b.x-a.x, dy=b.y-a.y, len=Math.hypot(dx,dy);
      if(len<1e-6) continue;
      segs.push({a,b,dx,dy,len,start:total,end:total+len}); total+=len;
    }
    return {segs,total};
  }
  const tbl = buildTable(centerlinePoints);
  if (!tbl.segs.length) return null;

  function sampleAtLen(s){
    if (s <= 0) { const f = tbl.segs[0]; const m=f.len||1; return {x:f.a.x,y:f.a.y,tx:f.dx/m,ty:f.dy/m, idx:0}; }
    if (s >= tbl.total) { const l = tbl.segs[tbl.segs.length-1]; const m=l.len||1; return {x:l.b.x,y:l.b.y,tx:l.dx/m,ty:l.dy/m, idx:tbl.segs.length-1}; }
    for (let i=0;i<tbl.segs.length;i++){
      const sg=tbl.segs[i];
      if (s <= sg.end){
        const t=(s - sg.start)/sg.len;
        let tx = sg.dx/sg.len, ty = sg.dy/sg.len;
        if (i > 0){ tx += tbl.segs[i-1].dx/tbl.segs[i-1].len; ty += tbl.segs[i-1].dy/tbl.segs[i-1].len; }
        if (i < tbl.segs.length-1){ tx += tbl.segs[i+1].dx/tbl.segs[i+1].len; ty += tbl.segs[i+1].dy/tbl.segs[i+1].len; }
        const m = Math.hypot(tx,ty) || 1; tx/=m; ty/=m;
        return { x: sg.a.x + sg.dx * t, y: sg.a.y + sg.dy * t, tx, ty, idx:i };
      }
    }
    const l = tbl.segs[tbl.segs.length-1], m=l.len||1;
    return { x:l.b.x, y:l.b.y, tx:l.dx/m, ty:l.dy/m, idx:tbl.segs.length-1 };
  }

  const widthAt = (pt,i)=> {
    const w = widthFn ? Number(widthFn(pt,i)) : fallbackWidth;
    return Math.max(10, Number.isFinite(w) ? w : fallbackWidth);
  };

  // ---- normal-line intersections to find base width ----
  const cross=(ax,ay,bx,by)=> ax*by - ay*bx;
  function lineSegIntersect(P,R,A,B){
    const Sx=B.x-A.x, Sy=B.y-A.y, denom = cross(R.x,R.y,Sx,Sy);
    if (Math.abs(denom) < 1e-9) return null;
    const Qx=A.x-P.x, Qy=A.y-P.y;
    const u = cross(Qx,Qy,Sx,Sy) / denom;     // along normal line
    const v = cross(Qx,Qy,R.x,R.y) / denom;   // along segment
    if (v < 0 || v > 1) return null;
    return {u, x:P.x + R.x*u, y:P.y + R.y*u};
  }
  function normalIntersections(C, n, poly){
    let pos=null, neg=null, up=Infinity, un=-Infinity;
    for (let i=0;i<poly.length-1;i++){
      const hit = lineSegIntersect(C, n, poly[i], poly[i+1]);
      if (!hit) continue;
      if (hit.u>0 && hit.u<up){ up=hit.u; pos={x:hit.x,y:hit.y}; }
      if (hit.u<0 && hit.u>un){ un=hit.u; neg={x:hit.x,y:hit.y}; }
    }
    return {pos,neg};
  }

  function baseWidthAtS(s){
    const c = sampleAtLen(s);
    let tx=c.tx, ty=c.ty; if (facing === "down") { tx=-tx; ty=-ty; }
    const n = { x:-ty, y:tx };
    if (leftEdge && rightEdge){
      const L = normalIntersections({x:c.x,y:c.y}, n, leftEdge);
      const R = normalIntersections({x:c.x,y:c.y}, n, rightEdge);
      let BL=null, BR=null;
      if (L.pos && R.neg) { BL = L.pos; BR = R.neg; }
      else if (L.neg && R.pos) { BL = L.neg; BR = R.pos; }
      if (BL && BR) return Math.hypot(BR.x - BL.x, BR.y - BL.y);
    }
    return widthAt({x:c.x,y:c.y}, c.idx);
  }
  const triHeightAtS = (s)=> Math.max(4, baseWidthAtS(s) * heightRatio);

  // ---- spacing (no overlap): use tallest triangle along usable path ----
  const usableLen = Math.max(0, tbl.total - startOffsetPx - endOffsetPx);
  let spacingPx = minSpacingPx, Hmax = 0;

  if (usableLen > 0) {
    const samples = Math.max(12, Math.floor(usableLen / 60)); // ~1 per 60px
    for (let i=0;i<=samples;i++){
      const s = startOffsetPx + (usableLen * i / samples);
      const H = triHeightAtS(s);
      if (H > Hmax) Hmax = H;
    }
    spacingPx = Math.max(minSpacingPx, Hmax * (1 + gapRatio));
  }

  // ---- triangle path at arc-length s ----
  function trianglePathAtS(s){
    const c = sampleAtLen(s);
    let tx=c.tx, ty=c.ty; if (facing === "down"){ tx=-tx; ty=-ty; }
    const n = { x:-ty, y:tx };

    // base from edge intersections (fallback to normal offsets)
    let BL=null, BR=null;
    if (leftEdge && rightEdge){
      const L = normalIntersections({x:c.x,y:c.y}, n, leftEdge);
      const R = normalIntersections({x:c.x,y:c.y}, n, rightEdge);
      if (L.pos && R.neg){
        BL = { x:L.pos.x - n.x*edgeInset, y:L.pos.y - n.y*edgeInset };
        BR = { x:R.neg.x + n.x*edgeInset, y:R.neg.y + n.y*edgeInset };
      } else if (L.neg && R.pos){
        BL = { x:L.neg.x - n.x*edgeInset, y:L.neg.y - n.y*edgeInset };
        BR = { x:R.pos.x + n.x*edgeInset, y:R.pos.y + n.y*edgeInset };
      }
    }
    if (!BL || !BR){
      const half = widthAt({x:c.x,y:c.y}, c.idx) * 0.5;
      BL = BL || { x:c.x - n.x*half, y:c.y - n.y*half };
      BR = BR || { x:c.x + n.x*half, y:c.y + n.y*half };
    }

    const baseW = Math.hypot(BR.x - BL.x, BR.y - BL.y);
    if (baseW < cullNarrowPx) return "";

    const H = Math.max(4, baseW * heightRatio);

    // apex ON centerline by arc-length
    const sApex = Math.max(0, Math.min(tbl.total, (facing === "up" ? s + H : s - H)));
    const aPt = sampleAtLen(sApex);
    const A = { x: aPt.x, y: aPt.y };

    return `M ${BL.x} ${BL.y} L ${BR.x} ${BR.y} L ${A.x} ${A.y} Z`;
  }

  // ---- create path elements (auto count from spacing) ----
  const count = (usableLen > 0) ? Math.max(1, Math.floor(usableLen / spacingPx)) : 1;

  if (targetGroup.__triangleAnimation?.stop) targetGroup.__triangleAnimation.stop();
  const layer = document.createElementNS("http://www.w3.org/2000/svg","g");
  targetGroup.appendChild(layer);

  const tris=[];
  for (let i=0;i<count;i++){
    const p=document.createElementNS("http://www.w3.org/2000/svg","path");
    p.setAttribute("fill", fill);
    p.setAttribute("fill-opacity", String(fillOpacity));
    p.setAttribute("stroke", stroke);
    p.setAttribute("stroke-width", String(strokeWidth));
    p.setAttribute("stroke-linejoin", "round");
    p.setAttribute("stroke-linecap", "round");
    layer.appendChild(p);
    tris.push(p);
  }

  // ---- animate: head moves in arc-length with speed ∝ size(head) ----
  const baseSpeed = (usableLen > 0) ? (usableLen / duration) : 0; // px per ms
  const state = { running: true, prevTs: 0, headS: startOffsetPx, raf: 0 };

	function speedScaleAt(s){
		if (Hmax <= 0) return 1;
		const H = triHeightAtS(s);
		const ratio = Math.max(0, Math.min(1, H / Hmax));

		// base size->speed: smaller H => closer to minScale, larger H => 1
		const base = minScale + (1 - minScale) * Math.pow(ratio, speedExponent);

		// sizeEffect: 1 = full effect, 0.5 = half (less variation), 2 = double (more variation)
		const sizeEffect = Number.isFinite(style.sizeEffect) ? style.sizeEffect : 1;

		const scaled = 1 - sizeEffect * (1 - base);
		return Math.max(0.01, Math.min(1, scaled));
	}

  function frame(ts){
    if (!state.running) return;
    if (!state.prevTs) state.prevTs = ts;

    const dt = ts - state.prevTs; // ms
    state.prevTs = ts;

    if (usableLen <= 0){
      for (const el of tris) el.setAttribute("d", "");
      state.raf = requestAnimationFrame(frame);
      return;
    }

    // advance head in s-space with size-based speed
    const scale = speedScaleAt(state.headS);
    state.headS += baseSpeed * scale * dt;

    // wrap to usable segment
    const spanStart = startOffsetPx, spanEnd = startOffsetPx + usableLen;
    if (state.headS >= spanEnd) state.headS = spanStart + ((state.headS - spanStart) % usableLen);
    if (state.headS <  spanStart) state.headS = spanEnd - ((spanStart - state.headS) % usableLen);

    // place triangles at fixed arc-length offsets (keeps spacing => no overlap)
    for (let i=0;i<tris.length;i++){
      const s = spanStart + ((state.headS - spanStart + i * spacingPx) % usableLen);
      const d = trianglePathAtS(s);
      if (d) tris[i].setAttribute("d", d); else tris[i].removeAttribute("d");
    }

    state.raf = requestAnimationFrame(frame);
  }

  state.stop = () => {
    state.running = false;
    if (state.raf) cancelAnimationFrame(state.raf);
    if (layer.isConnected) layer.remove();
    targetGroup.__triangleAnimation = null;
  };

  targetGroup.__triangleAnimation = state;
  state.raf = requestAnimationFrame(frame);
  return state;
}

function appendPerspectiveTrack(target, points, bounds, options = {}) {
  if (!target || !Array.isArray(points) || points.length < 2) return null;

  const strip = buildPerspectiveStrip(points, bounds);
  if (!strip || strip.outline.length < 3) return null;

  // --- build the outline path data ---
  const outline = strip.outline;
  const d = outline.map((p, i) => (i === 0 ? "M" : "L") + p.x + " " + p.y).join(" ") + " Z";

  // --- ensure a <defs> for clipPath ---
  const svgRoot = target.ownerSVGElement || target; // supports when target is the SVG itself
  let defs = svgRoot.querySelector("defs");
  if (!defs) {
    defs = document.createElementNS("http://www.w3.org/2000/svg", "defs");
    svgRoot.insertBefore(defs, svgRoot.firstChild || null);
  }

  // unique clip id per call (safe for multiple tracks)
  const clipId = `lane-clip-${Math.random().toString(36).slice(2)}`;
  const clip = document.createElementNS("http://www.w3.org/2000/svg", "clipPath");
  clip.setAttribute("id", clipId);
  const clipPath = document.createElementNS("http://www.w3.org/2000/svg", "path");
  clipPath.setAttribute("d", d);
  clip.appendChild(clipPath);
  defs.appendChild(clip);

  // --- group (clipped so triangles can't draw outside) ---
  const group = document.createElementNS("http://www.w3.org/2000/svg", "g");
  if (options.role) group.dataset.role = options.role;
  group.setAttribute("clip-path", `url(#${clipId})`);

  // --- filled lane outline ---
  const fillPath = document.createElementNS("http://www.w3.org/2000/svg", "path");
  fillPath.setAttribute("d", d);
  fillPath.setAttribute("fill", options.fill ?? "#39ff88");
  fillPath.setAttribute("fill-opacity", (options.fillOpacity ?? 0.35).toString());
  fillPath.setAttribute("stroke", options.stroke ?? "#39ff88");
  fillPath.setAttribute("stroke-width", (options.strokeWidth ?? 1.1).toString());
  fillPath.setAttribute("stroke-linejoin", "round");
  fillPath.setAttribute("stroke-linecap", "round");
  fillPath.classList.add("drawn-path");
  group.appendChild(fillPath);

  // --- centerline ---
  const centerPolyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");
  const centerlinePoints =
    Array.isArray(strip.centerline) && strip.centerline.length > 1 ? strip.centerline : points;

  centerPolyline.setAttribute("points", centerlinePoints.map(p => `${p.x},${p.y}`).join(" "));
  centerPolyline.setAttribute("fill", "none");
  centerPolyline.setAttribute("stroke", options.centerStroke ?? (options.stroke ?? "#39ff88"));
  centerPolyline.setAttribute("stroke-width", (options.centerStrokeWidth ?? 1).toString());
  centerPolyline.setAttribute("stroke-linecap", "round");
  centerPolyline.setAttribute("stroke-linejoin", "round");
  centerPolyline.setAttribute("vector-effect", "non-scaling-stroke");
  if (options.centerDash) centerPolyline.setAttribute("stroke-dasharray", options.centerDash);
  centerPolyline.classList.add("drawn-path");
  group.appendChild(centerPolyline);

  // --- triangles (now slower + shorter, and clipped to the lane) ---
  let triangleAnimation = null;
  if (options.animateTriangles !== false) {
    const widthResolver =
      typeof options.triangleWidthFn === "function"
        ? options.triangleWidthFn
        : (pt) => getPerspectiveWidth(pt.y, bounds);

		triangleAnimation = attachTriangleStream(group, centerlinePoints, {
			count: options.triangleCount,
			duration: Math.max(400, Number(options.triangleDuration ?? 2600) * 2),
			fill: options.triangleFill,
			stroke: options.triangleStroke,
			strokeWidth: options.triangleStrokeWidth,
			fillOpacity: options.triangleFillOpacity,
			widthFn: widthResolver,
			heightRatio: 0.275,         // half height
			facing: "up",
			laneLeft:  strip.leftEdge,
			laneRight: strip.rightEdge.slice().reverse(),
			edgeInset: Number.isFinite(options.edgeInset) ? options.edgeInset : 0,

			// NEW: start later & end earlier (in px along the centerline)
			startOffsetPx: Number.isFinite(options.triangleStartOffsetPx) ? options.triangleStartOffsetPx : 60,
			endOffsetPx:   Number.isFinite(options.triangleEndOffsetPx)   ? options.triangleEndOffsetPx   : 90,
			// (tweak 60/90 if you want more/less headroom)
		});

  }

  if (options.zIndex != null) group.style.zIndex = options.zIndex;
  target.appendChild(group);

  return { group, strip, animation: triangleAnimation };
}

function clearOverlayPreview() {
  if (!svgOverlay) svgOverlay = document.getElementById("drawingOverlay");
  svgOverlay?.querySelectorAll('[data-role="path-preview"]').forEach(n => n.remove());
}

function renderOverlayPreview(cursorX, cursorY) {

	if (!drawMode) return;

	if (!Array.isArray(currentPath) || currentPath.length === 0) {

		clearOverlayPreview();

		return;

	}

	if (cursorX === null || cursorY === null) {

		clearOverlayPreview();

		return;

	}

	if (!svgOverlay) svgOverlay = document.getElementById("drawingOverlay");

	if (!svgOverlay) return;

	clearOverlayPreview();

	const previewPoints = currentPath.concat({ x: cursorX, y: cursorY });

	const bounds = getSurfaceBounds(svgOverlay) || getSurfaceBounds(canvas);

	if (!bounds) return;

	appendPerspectiveTrack(svgOverlay, previewPoints, bounds, {

		role: "path-preview",

		fill: "#39ff88",

		fillOpacity: "0.18",

		stroke: "#39ff88",

		strokeWidth: 0.9,

		centerStrokeWidth: 1.8,

		centerDash: "10 10",

		animateTriangles: false

	});

}

window.toggleDrawMode = function() {

	drawMode = !drawMode;

	const btn = document.getElementById("drawPathBtn");

	btn.style.backgroundColor = drawMode ? "#4CAF50" : "#444";

	btn.innerText = drawMode ? "✏️ Drawing..." : "✏️ Draw Path";

	canvas = document.getElementById("drawingCanvas");

	svgOverlay = document.getElementById("drawingOverlay");

	if (drawMode) {

		// ✅ Show IMU/Encoder warning

		const old = document.getElementById("imuWarning");

		if (old) old.remove();

		const warning = document.createElement("div");

		warning.id = "imuWarning";

		warning.innerText = "⚠️ This feature requires MPU9250 and Encoders. DEMO mode only!";

		document.body.appendChild(warning);

		// Auto-remove warning after 4 seconds

		setTimeout(() => warning.remove(), 4000);

		// ✅ Initialize canvas

		canvas.width = window.innerWidth;

		canvas.height = window.innerHeight;

		canvas.style.pointerEvents = "auto";

		canvas.style.cursor = "crosshair";

		canvas.style.zIndex = "20";

		document.body.style.cursor = "crosshair";

		drawingPoints = [];

		currentPath = [];

		const cam = document.getElementById("cameraStream");

		const camRect = cam.getBoundingClientRect();

		const canvasRect = canvas.getBoundingClientRect();

		const anchorX = camRect.left + camRect.width / 2 - canvasRect.left;

		const anchorY = camRect.bottom - canvasRect.top;

		drawingPoints.push({ x: anchorX, y: anchorY });

		currentPath.push({ x: anchorX, y: anchorY });

		renderPath();          // draw START button immediately

		renderPath(false);     // force START rendering, even after wipe

		canvas.addEventListener("mousemove", handleMouseMove);

		canvas.addEventListener("mousedown", handleCanvasClick);

		canvas.addEventListener("contextmenu", stopDrawing);

		redrawCanvas();

	} else {

		cleanupDrawing(); // also disables pointerEvents

	}

}

function handleMouseMove(e) {
  if (!drawMode || drawingPoints.length === 0 || !canvas) return;

  const rect = canvas.getBoundingClientRect();
  const x = e.clientX - rect.left;
  const y = e.clientY - rect.top;

  if (handleMouseMove._rafPending) { handleMouseMove._nx = x; handleMouseMove._ny = y; return; }
  handleMouseMove._rafPending = true;

  requestAnimationFrame(() => {
    handleMouseMove._rafPending = false;
    const nx = handleMouseMove._nx ?? x;
    const ny = handleMouseMove._ny ?? y;
    handleMouseMove._nx = handleMouseMove._ny = null;
    redrawCanvas(nx, ny);
  });
}

function handleCanvasClick(e) {

	if (e.button === 0) { // Left click

		const rect = canvas.getBoundingClientRect();

		const x = e.clientX - rect.left;

		const y = e.clientY - rect.top;

		drawingPoints.push({ x, y });

		redrawCanvas();           // Optional: keep canvas for debugging

		addPointToPath(x, y);     // ✅ REQUIRED to draw to SVG overlay

		console.log("Point added to SVG:", x, y);

	}

}

function redrawCanvas(cursorX = null, cursorY = null) {

    if (!ctx) ctx = canvas.getContext("2d");

    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (!Array.isArray(drawingPoints) || drawingPoints.length === 0) {

      clearOverlayPreview();

      return;

    }

    const previewPoints = drawingPoints.slice();

    const hasCursor = cursorX !== null && cursorY !== null;

    if (hasCursor) {

      previewPoints.push({ x: cursorX, y: cursorY });

    }

    if (previewPoints.length < 2) {

      if (!hasCursor) clearOverlayPreview();

      return;

    }

    const bounds = getSurfaceBounds(canvas);

    drawPerspectiveStrip(ctx, previewPoints, bounds, {

      fillStyle: "rgba(57, 255, 136, 0.18)",

      strokeWidth: 3.2

    });

    if (hasCursor) {

      renderOverlayPreview(cursorX, cursorY);

    } else {

      clearOverlayPreview();

    }

  }

function drawPlayButton(x, y, size = 20, color = "yellow", outline = false) {
  if (!ctx) return;

  ctx.save();
  ctx.imageSmoothingEnabled = true;             // ✅ smooth edges
  ctx.globalCompositeOperation = "source-over"; // ✅ normal blend
  ctx.fillStyle = color;
  ctx.strokeStyle = color;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";

  // Center-based equilateral triangle (points right)
  const half = size / 2;
  const height = size * 0.6; // visual balance factor

  ctx.beginPath();
  ctx.moveTo(x + half, y);           // right tip
  ctx.lineTo(x - half, y - height);  // top-left
  ctx.lineTo(x - half, y + height);  // bottom-left
  ctx.closePath();

  if (outline) {
    ctx.lineWidth = 2;
    ctx.stroke();
  } else {
    ctx.fill();
  }

  ctx.restore();
}


function stopDrawing(e) {

	if (e) e.preventDefault();

	drawMode = false;

	canvas.removeEventListener("mousemove", handleMouseMove);

	canvas.removeEventListener("mousedown", handleCanvasClick);

	canvas.removeEventListener("contextmenu", stopDrawing);

	// ✅ Fix: stop blocking other UI

	canvas.style.pointerEvents = "none";

	canvas.style.cursor = "default";

	canvas.style.zIndex = "-1"; // ✅ Push canvas behind everything

	document.body.style.cursor = "default";

	// ✅ Fully clear canvas to remove overlay hitbox

	canvas.width = 0;

	canvas.height = 0;

	clearOverlayPreview();

	const btn = document.getElementById("drawPathBtn");

	btn.innerText = "✏️ Draw Path";

	btn.style.backgroundColor = "#444";

	renderPath(true); // ✅ show STOP button

}

function cleanupDrawing() {

	if (!canvas) return;

	canvas.removeEventListener("mousemove", handleMouseMove);

	canvas.removeEventListener("mousedown", handleCanvasClick);

	canvas.removeEventListener("contextmenu", stopDrawing);

	canvas.style.pointerEvents = "none";

	canvas.style.cursor = "default";

	document.body.style.cursor = "default";

	clearOverlayPreview();

}

window.addEventListener("resize", () => {

	const canvas = document.getElementById("drawingCanvas");

	if (canvas && drawMode) {

		canvas.width = window.innerWidth;

		canvas.height = window.innerHeight;

		redrawCanvas(); // <- redraw current path

	}

});

function clearDrawing() {

	if (!ctx || !canvas || !svgOverlay) return;

	// Clear path data

	drawingPoints = [];

	currentPath = [];

	// Clear visuals

	ctx.clearRect(0, 0, canvas.width, canvas.height);

	svgOverlay.innerHTML = ""; // ✅ Remove drawn polyline/arrow/nodes

	// Clear buttons

	const pathButtons = document.getElementById("pathControlButtons");

	pathButtons.innerHTML = "";                // ✅ Remove START/STOP buttons

	document.getElementById("startButton")?.remove(); // ✅ extra safety

	document.getElementById("stopButton")?.remove();

	pathButtons.style.pointerEvents = "none";  // ✅ Prevent hitbox bug

	// Reset cursors

	canvas.style.cursor = "default";

	document.body.style.cursor = "default";

	console.log("🧹 Drawing cleared");

}

function addPointToPath(x, y) {

	currentPath.push({ x, y });

	renderPath();

}

function rotateNodeAt(x, y) {

	alert(`Rotate node clicked at (${x.toFixed(0)}, ${y.toFixed(0)})`);

	// Later: open angle input or rotate arrow preview

}

function handleStopClick() {

	showToast("🛑 Stop triggered");

	console.log("Stop button clicked");

	// Add actual command logic here

}

function renderPath(showStopOnly = false) {

    // Grab overlay and button container

    if (!svgOverlay) svgOverlay = document.getElementById("drawingOverlay");

    const btnContainer = document.getElementById("pathControlButtons");

    if (!svgOverlay || !btnContainer) return;

    // Clear only the SVG overlay (NOT the buttons)

    svgOverlay.innerHTML = "";

    const svgBounds = getSurfaceBounds(svgOverlay) || getSurfaceBounds(canvas);

    if (!svgBounds) return;

    let latestStrip = null;

    // Default start point (bottom-center of overlay)

    const startPoint = { x: svgBounds.width / 2, y: svgBounds.height - 10 };

    // Ensure path starts from the anchor (with small tolerance)

    const anchorTolerance = 0.5;

    const normalizedPath = normalizeAnchorPath(currentPath, startPoint, { tolerance: anchorTolerance, lift: getAnchorLift() });

    currentPath = normalizedPath.length ? normalizedPath : [startPoint, { x: startPoint.x, y: Math.max(0, startPoint.y - getAnchorLift()) }];

    if (currentPath.length > 1) {

      const result = appendPerspectiveTrack(svgOverlay, currentPath, svgBounds, {

        role: "path-track",

        fill: "#39ff88",

        fillOpacity: "0.35",

        stroke: "#39ff88",

        strokeWidth: 1.1,

        centerStrokeWidth: 1.6,

        triangleCount: 5,

        triangleSize: 16,

        triangleSizeScale: 0.34,

        triangleMaxSize: 56,

        triangleDuration: 2600,

        triangleOpacity: 0.55,

        triangleFill: "#ffef75",

        triangleFillOpacity: 0.35,

        triangleStroke: "#ffd93b",

        triangleStrokeWidth: 2.1

      });

      latestStrip = result?.strip ?? null;

    }

    clearOverlayPreview();

    // START button (recreate cleanly unless we're only showing STOP)

    if (!showStopOnly) {

      document.getElementById("startButton")?.remove();

      const startBtn = document.createElement("button");

      startBtn.id = "startButton";

      startBtn.textContent = "Start";

      startBtn.style.pointerEvents = "auto";

      // keep it bottom-centered in case CSS is missing

      startBtn.style.position = "absolute";

      startBtn.style.bottom = "10px";

      startBtn.style.left = "50%";

      startBtn.style.transform = "translateX(-50%)";

      startBtn.onclick = () => {

        const sourcePath = latestStrip?.centerline && latestStrip.centerline.length > 1 ? latestStrip.centerline : currentPath;

        const payload = sourcePath.map(p => ({ x: p.x, y: p.y }));

        if (window.websocketCarInput && window.websocketCarInput.readyState === WebSocket.OPEN) {

          window.websocketCarInput.send("PATH," + JSON.stringify(payload));

          window.showToast?.("dYs- Path sent to robot!");

        } else {

          window.showToast?.("??O Robot not connected!", true);

        }

      };

      btnContainer.appendChild(startBtn);

    }

    // STOP button (draw after right-click; keep the size consistent)

    if (currentPath.length > 1) {

      const last = currentPath[currentPath.length - 1];

      let stopBtn = document.getElementById("stopButton");

      if (showStopOnly && !stopBtn) {

        stopBtn = document.createElement("button");

        stopBtn.id = "stopButton";

        stopBtn.textContent = "Stop";

        stopBtn.onclick = () => {

          try {

            window.websocketCarInput?.send("PATH_STOP");

            window.showToast?.("dY>` Stop sent");

          } catch (e) {

            console.warn(e);

          }

        };

        btnContainer.appendChild(stopBtn);

      }

      if (stopBtn) {

        stopBtn.style.position = "absolute";

        stopBtn.style.left = `${last.x}px`;

        stopBtn.style.top = `${last.y}px`;

        stopBtn.style.bottom = "auto";                    // prevent tall vertical stretching

        stopBtn.style.transform = "translate(-50%, -120%)"; // center above the point

        stopBtn.style.pointerEvents = "auto";

        stopBtn.style.minWidth = "auto";

        stopBtn.style.width = "auto";

      }

    }

    console.log("?o. Path rendered with", currentPath.length, "points");

  }

