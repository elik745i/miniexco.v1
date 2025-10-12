// batteryGraph.js

(function () {
  // Avoid double-loading
  if (window.batteryGraphLoaded) return;
  window.batteryGraphLoaded = true;

  function createBatteryPopup() {
    if (document.getElementById('batteryPopupOverlay')) return;
    const overlay = document.createElement('div');
    overlay.id = 'batteryPopupOverlay';
    overlay.innerHTML = `
      <div id="batteryPopupContent">
        <button id="batteryPopupClose" title="Close">&times;</button>
        <div style="font-weight:bold; font-size:1.15em;">Battery Voltage History</div>
        <div id="batteryPopupGraph"></div>
      </div>
    `;
    document.body.appendChild(overlay);

    document.getElementById('batteryPopupClose').onclick = () => overlay.classList.remove('active');
    overlay.addEventListener('mousedown', e => {
      if (e.target === overlay) overlay.classList.remove('active');
    });
  }

  window.drawVoltageHistoryInPopup = async function() {
    const graphDiv = document.getElementById('batteryPopupGraph');
    graphDiv.innerHTML = `
      <div id="voltageEstimate"></div>
      <div id="voltageChart"></div>
      <div id="voltageButtonBar">
        <button id="voltageZoomIn" title="Zoom In">+</button>
        <button id="voltageZoomOut" title="Zoom Out">−</button>
        <button id="voltageResetZoom" title="Reset Zoom">⭯</button>
      </div>
      <div id="voltageCaption">Voltage and charge trend will be shown here.</div>
    `;

    const chartDiv = document.getElementById('voltageChart');
    const estimateDiv = document.getElementById('voltageEstimate');
    const captionDiv = document.getElementById('voltageCaption');

    // Fetch and parse telemetry as before...
    let files = [];
    try {
      let filesResp = await fetch('/list_telemetry_files');
      if (filesResp.ok) files = await filesResp.json();
    } catch (e) { /* ignore */ }
    if (!files.length) {
      estimateDiv.textContent = "No telemetry logs found!";
      captionDiv.textContent = "";
      return;
    }
    files.sort();
    let lastFile = files[files.length - 1];
    if (!lastFile.startsWith("/")) lastFile = "/telemetry/" + lastFile;

    let resp;
    try {
      resp = await fetch(lastFile);
      if (!resp.ok) throw new Error('Not found');
    } catch (e) {
      estimateDiv.textContent = 'Could not load telemetry log!';
      captionDiv.textContent = "";
      return;
    }
    const csv = await resp.text();
    const lines = csv.trim().split('\n');
    if (lines.length < 2) {
      estimateDiv.textContent = "No data in telemetry log!";
      captionDiv.textContent = "";
      return;
    }

    let tArr = [], vArr = [], tsArr = [];
    for (let i = 0; i < lines.length; ++i) {
      const cols = lines[i].split(',');
      if (cols.length < 3) continue;
      const timeLabel = cols[0].trim();
      const ts = Number(cols[1]);
      const v = Number(cols[2]);
      if (!timeLabel || isNaN(ts) || isNaN(v)) continue;
      tArr.push(timeLabel);
      tsArr.push(ts);
      vArr.push(v);
    }
    if (!vArr.length) {
      estimateDiv.textContent = "Not enough data.";
      captionDiv.textContent = "";
      return;
    }

    let lastTimestamp = tsArr[tsArr.length - 1] || 0;
    const minTime = lastTimestamp - 20 * 60;
    let filteredTArr = [], filteredVArr = [], filteredIndices = [];
    for (let i = 0; i < tArr.length; ++i) {
      if (tsArr[i] >= minTime) {
        filteredTArr.push(tArr[i]);
        filteredVArr.push(vArr[i]);
        filteredIndices.push(filteredTArr.length - 1);
      }
    }
    if (filteredVArr.length < 6) {
      filteredTArr = tArr;
      filteredVArr = vArr;
      filteredIndices = tArr.map((_, i) => i);
    }
    if (!filteredVArr.length) {
      estimateDiv.textContent = "Not enough data.";
      captionDiv.textContent = "";
      return;
    }

    let indices = filteredIndices;
    let minY = Math.min(...filteredVArr, 6);
    let maxY = Math.max(...filteredVArr, 9);

    let uplotOpts = {
      width: 600,
      height: 300,
      series: [
        {},
        {
          label: "Voltage (V)",
          stroke: "#45aaff",
          fill: null,
          width: 2,
          points: { show: true, size: 2 }
        }
      ],
      axes: [
        {
          label: "Time",
          stroke: "#bbb",
          grid: { stroke: "#333" },
          values: (u, vals, space) => vals.map(v => filteredTArr[Math.round(v)] || ''),
          space: 42,
          font: "13px sans-serif",
          ticks: { stroke: "#666" },
          size: 60,
          rotate: -35,
        },
        {
          label: "Voltage (V)",
          stroke: "#bbb",
          grid: { stroke: "#333" },
          scale: "y",
          space: 48,
          values: (u, vals) => vals.map(v=>v.toFixed(2)),
          font: "13px sans-serif",
          ticks: { stroke: "#666" }
        }
      ],
      scales: {
        x: { min: 0, max: Math.max(filteredIndices.length - 1, 1) },
        y: { min: Math.floor(minY * 10) / 10 - 0.1, max: Math.ceil(maxY * 10) / 10 + 0.1 }
      },
      cursor: { drag: { x:true, y:false }, focus: { prox: 24 } },
      hooks: {
        drawClear: [
          (u) => {
            u.ctx.save();
            u.ctx.fillStyle = "#181818";
            u.ctx.fillRect(0, 0, u.bbox.width, u.bbox.height);
            u.ctx.restore();
          }
        ],
        ready: [
          u => { window.voltageChartObj = u; }
        ]
      }
    };

    let data = [filteredIndices, filteredVArr];
    let uplot = new uPlot(uplotOpts, data, chartDiv);
    window.voltageChartObj = uplot;

    // Zoom/pan handlers
    let zoomFactor = 1.4;
    document.getElementById('voltageZoomIn').onclick = () => {
      let min = uplot.scales.x.min;
      let max = uplot.scales.x.max;
      let range = max - min;
      let center = (min + max) / 2;
      let newRange = range / zoomFactor;
      let minX = Math.max(0, Math.floor(center - newRange/2));
      let maxX = Math.min(filteredIndices.length - 1, Math.ceil(center + newRange/2));
      uplot.setScale('x', { min: minX, max: maxX });
    };
    document.getElementById('voltageZoomOut').onclick = () => {
      let min = uplot.scales.x.min;
      let max = uplot.scales.x.max;
      let range = max - min;
      let center = (min + max) / 2;
      let newRange = range * zoomFactor;
      let minX = Math.max(0, Math.floor(center - newRange/2));
      let maxX = Math.min(filteredIndices.length - 1, Math.ceil(center + newRange/2));
      uplot.setScale('x', { min: minX, max: maxX });
    };
    document.getElementById('voltageResetZoom').onclick = () => {
      uplot.setScale('x', { min: 0, max: Math.max(filteredIndices.length - 1, 1) });
    };

    // Estimate
    if (filteredVArr.length < 6) {
      estimateDiv.textContent = "Trend: not enough points.";
      return;
    }
    let sumX = 0, sumY = 0, sumXY = 0, sumXX = 0, N = filteredVArr.length;
    for(let i=0; i<N; ++i) {
      let x = i, y = filteredVArr[i];
      sumX += x; sumY += y; sumXY += x*y; sumXX += x*x;
    }
    let slope = (N*sumXY-sumX*sumY)/(N*sumXX-sumX*sumX);
    let flatV = 6.6, fullV = 8.4;
    let current = filteredVArr[filteredVArr.length-1];
    let lastT = filteredTArr[filteredTArr.length-1];
    let txt = "";
    if (slope < -0.0001) {
      let etaIdx = N-1 + (current-flatV)/slope;
      let mins = Math.max(0, Math.round((etaIdx-(N-1))));
      txt = `Discharging: est. ${mins} steps till flat (6.6V)`;
    } else if (slope > 0.0001) {
      let etaIdx = N-1 + (fullV-current)/slope;
      let mins = Math.max(0, Math.round((etaIdx-(N-1))));
      txt = `Charging: est. ${mins} steps till full (8.4V)`;
    } else {
      txt = "Voltage stable";
    }
    estimateDiv.textContent = txt;
  };

  window.showBatteryPopup = function() {
    ensureBatteryGraphCssLoaded();
    createBatteryPopup();
    document.getElementById('batteryPopupOverlay').classList.add('active');
    setTimeout(window.drawVoltageHistoryInPopup, 300);
  };

  function ensureBatteryGraphCssLoaded() {
    if (!document.getElementById("batteryGraphCss")) {
      const link = document.createElement("link");
      link.id = "batteryGraphCss";
      link.rel = "stylesheet";
      link.href = "/batteryGraph.css?v=" + Date.now();
      document.head.appendChild(link);
    }
  }
})();
