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

	const centerline = autoRoundPoints(points);

	if (!Array.isArray(centerline) || centerline.length < 2) return null;

	const leftEdge = [];

	const rightEdge = [];

	for (let i = 0; i < centerline.length; i++) {

		const current = centerline[i];

		if (!current) continue;

		// Keep strip caps horizontal regardless of heading by offsetting along X only.

		const trackWidth = getPerspectiveWidth(current.y, safeBounds);

		const half = trackWidth / 2;

		leftEdge.push({

			x: current.x - half,

			y: current.y

		});

		rightEdge.push({

			x: current.x + half,

			y: current.y

		});

	}

	const rightOutline = rightEdge.slice().reverse();

	return {

		outline: leftEdge.concat(rightOutline),

		leftEdge,

		rightEdge: rightOutline,

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

function appendPerspectiveTrack(target, points, bounds, options = {}) {

	if (!target || !Array.isArray(points) || points.length < 2) return null;

	const strip = buildPerspectiveStrip(points, bounds);

	if (!strip || strip.outline.length < 3) return null;

	const group = document.createElementNS("http://www.w3.org/2000/svg", "g");

	if (options.role) group.dataset.role = options.role;

	const outline = strip.outline;

	let d = "";

	for (let i = 0; i < outline.length; i++) {

		const cmd = i === 0 ? "M" : "L";

		d += `${cmd}${outline[i].x} ${outline[i].y} `;

	}

	d += "Z";

	const fillPath = document.createElementNS("http://www.w3.org/2000/svg", "path");

	fillPath.setAttribute("d", d.trim());

	fillPath.setAttribute("fill", options.fill ?? "#39ff88");

	fillPath.setAttribute("fill-opacity", (options.fillOpacity ?? "0.35").toString());

	fillPath.setAttribute("stroke", options.stroke ?? "#39ff88");

	fillPath.setAttribute("stroke-width", (options.strokeWidth ?? 1.1).toString());

	fillPath.setAttribute("stroke-linejoin", "round");

	fillPath.setAttribute("stroke-linecap", "round");

	fillPath.classList.add("drawn-path");

	group.appendChild(fillPath);

	const centerPolyline = document.createElementNS("http://www.w3.org/2000/svg", "polyline");

	const centerlinePoints = Array.isArray(strip.centerline) && strip.centerline.length > 1 ? strip.centerline : points;

	centerPolyline.setAttribute("points", centerlinePoints.map(p => `${p.x},${p.y}`).join(" "));

	centerPolyline.setAttribute("fill", "none");

	centerPolyline.setAttribute("stroke", options.centerStroke ?? (options.stroke ?? "#39ff88"));

	centerPolyline.setAttribute("stroke-width", (options.centerStrokeWidth ?? 1).toString());

	centerPolyline.setAttribute("stroke-linecap", "round");

	centerPolyline.setAttribute("stroke-linejoin", "round");

	centerPolyline.setAttribute("vector-effect", "non-scaling-stroke");

	if (options.centerDash) {

		centerPolyline.setAttribute("stroke-dasharray", options.centerDash);

	}

	centerPolyline.classList.add("drawn-path");

	group.appendChild(centerPolyline);

	if (options.zIndex != null) {

		group.style.zIndex = options.zIndex;

	}

	target.appendChild(group);

	return { group, strip };

}

function clearOverlayPreview() {

	if (!svgOverlay) svgOverlay = document.getElementById("drawingOverlay");

	svgOverlay?.querySelector('[data-role="path-preview"]')?.remove();

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

		centerDash: "10 10"

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

	if (!drawMode || drawingPoints.length === 0) return;

	const rect = canvas.getBoundingClientRect();

	const x = e.clientX - rect.left;

	const y = e.clientY - rect.top;

	redrawCanvas(x, y); // live preview line with perspective

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

        centerStrokeWidth: 1.6

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

