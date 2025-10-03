  const perspectiveConfig = {

    minWidth: 32,

    maxWidth: 120,

    easingExponent: 1.2

  };

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

    const leftEdge = [];

    const rightEdge = [];

    for (let i = 0; i < points.length; i++) {

      const current = points[i];

      if (!current) continue;

      const prev = points[i - 1] || current;

      const next = points[i + 1] || current;

      let dirX = next.x - prev.x;

      let dirY = next.y - prev.y;

      if (!Number.isFinite(dirX) || !Number.isFinite(dirY) || (Math.abs(dirX) < 1e-6 && Math.abs(dirY) < 1e-6)) {

        if (i > 0) {

          dirX = current.x - prev.x;

          dirY = current.y - prev.y;

        } else if (i < points.length - 1) {

          dirX = next.x - current.x;

          dirY = next.y - current.y;

        } else {

          dirX = 0;

          dirY = -1;

        }

      }

      const length = Math.hypot(dirX, dirY) || 1;

      const normalX = -dirY / length;

      const normalY = dirX / length;

      const trackWidth = getPerspectiveWidth(current.y, safeBounds);

      const half = trackWidth / 2;

      leftEdge.push({

        x: current.x + normalX * half,

        y: current.y + normalY * half

      });

      rightEdge.push({

        x: current.x - normalX * half,

        y: current.y - normalY * half

      });

    }

    const rightOutline = rightEdge.slice().reverse();

    return {

      outline: leftEdge.concat(rightOutline),

      leftEdge,

      rightEdge: rightOutline

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

    const { outline } = strip;

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

    ctxRef.beginPath();

    ctxRef.moveTo(points[0].x, points[0].y);

    for (let i = 1; i < points.length; i++) {

      ctxRef.lineTo(points[i].x, points[i].y);

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

    centerPolyline.setAttribute("points", points.map(p => `${p.x},${p.y}`).join(" "));

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


  function drawPlayButton(x, y) {

    ctx.fillStyle = "yellow";

    ctx.beginPath();

    ctx.moveTo(x + 10, y);

    ctx.lineTo(x, y - 10);

    ctx.lineTo(x, y + 10);

    ctx.closePath();

    ctx.fill();

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

    // Default start point (bottom-center of overlay)

    const startPoint = { x: svgBounds.width / 2, y: svgBounds.height - 10 };

    // Ensure path starts from the anchor (with small tolerance)

    if (!Array.isArray(currentPath) || currentPath.length === 0) {

      currentPath = [startPoint];

    } else {

      const p0 = currentPath[0];

      if (Math.abs(p0.x - startPoint.x) > 0.5 || Math.abs(p0.y - startPoint.y) > 0.5) {

        currentPath.unshift(startPoint);

      }

    }

    if (currentPath.length > 1) {

      appendPerspectiveTrack(svgOverlay, currentPath, svgBounds, {

        role: "path-track",

        fill: "#39ff88",

        fillOpacity: "0.35",

        stroke: "#39ff88",

        strokeWidth: 1.1,

        centerStrokeWidth: 1.6

      });

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

        if (window.websocketCarInput && window.websocketCarInput.readyState === WebSocket.OPEN) {

          window.websocketCarInput.send("PATH," + JSON.stringify(currentPath));

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

