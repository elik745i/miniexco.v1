var websocketCarInput;
var auxSlider;
var bucketSlider;
var lastFPSUpdate = Date.now();
let emergencyOn = false;
let beaconActive = false;
let drawMode = false;
let drawingPoints = [];
let canvas = null;
let ctx = null;
let drawingActive = false;
let currentPath = [];
let svgOverlay;
let keymap = {};  // 🆕 Will hold dynamic keyboard mappings
// Make the same object visible via window for other scripts
window.keymap = keymap;

let imuScriptLoaded = false;
let ledOn = false;
let leftIndicator, rightIndicator, emergencyBtn, cameraStream;
let latestBatteryPercent = 0;
let micEnabled = false;
let micStream = null;
let micAudioElem = null;

let isRecordingVideo = false;
let videoRecordTimeout = null;

let cameraEnabled = true;

const BLANK_IMG =
  "data:image/svg+xml;utf8," + encodeURIComponent(`
    <svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">
      <rect width="100%" height="100%" fill="black"/>
      <g transform="translate(320,180) scale(6.67) translate(-8,-8)">
        <path d="m 6.5 0 c -0.265625 0 -0.519531 0.105469 -0.707031 0.292969 l -1.707031 1.707031 h -1.023438 l -1.53125 -1.53125 l -1.0625 1.0625 l 14 14 l 1.0625 -1.0625 l -0.386719 -0.386719 c 0.527344 -0.539062 0.855469 -1.277343 0.855469 -2.082031 v -7 c 0 -1.644531 -1.355469 -3 -3 -3 h -1.085938 l -1.707031 -1.707031 c -0.1875 -0.1875 -0.441406 -0.292969 -0.707031 -0.292969 z m 0.414062 2 h 2.171876 l 1.707031 1.707031 c 0.1875 0.1875 0.441406 0.292969 0.707031 0.292969 h 1.5 c 0.570312 0 1 0.429688 1 1 v 7 c 0 0.269531 -0.097656 0.503906 -0.257812 0.679688 l -2.4375 -2.4375 c 0.4375 -0.640626 0.695312 -1.414063 0.695312 -2.242188 c 0 -2.199219 -1.800781 -4 -4 -4 c -0.828125 0 -1.601562 0.257812 -2.242188 0.695312 l -0.808593 -0.808593 c 0.09375 -0.046875 0.183593 -0.105469 0.257812 -0.179688 z m -6.492187 1.484375 c -0.265625 0.445313 -0.421875 0.964844 -0.421875 1.515625 v 7 c 0 1.644531 1.355469 3 3 3 h 8.9375 l -2 -2 h -6.9375 c -0.570312 0 -1 -0.429688 -1 -1 v -6.9375 z m 7.578125 2.515625 c 1.117188 0 2 0.882812 2 2 c 0 0.277344 -0.058594 0.539062 -0.15625 0.78125 l -2.625 -2.625 c 0.242188 -0.097656 0.503906 -0.15625 0.78125 -0.15625 z m -3.90625 1.15625 c -0.058594 0.273438 -0.09375 0.554688 -0.09375 0.84375 c 0 2.199219 1.800781 4 4 4 c 0.289062 0 0.570312 -0.035156 0.84375 -0.09375 z m 0 0"
              fill="#888"/>
      </g>
    </svg>
  `);





	
window.headingDeg = 0;
window.rollDeg = 0;
window.pitchDeg = 0;
window.headingCanvas = null;
window.tiltCanvas = null;
window.headingCtx = null;
window.tiltCtx = null;
window.pendingIMU = null;


let  lastSentMotorValues = {
    Forward: 0,
    Backward: 0,
    Left: 0,
    Right: 0,
    ArmUp: 0,
    ArmDown: 0
};

// Track current indicator states and debounce timers
let turnSignalState = {
    left: false,
    right: false,
    leftTimer: null,
    rightTimer: null,
};

let lastDriveDir = ""; // "Forward", "Backward", or ""

const sliderAnimations = {};  // Store interval handles by direction
const keysDown = {};
const joystick = document.getElementById("joystickContainer");
const knob = document.getElementById("joystickKnob");
let active = false;

joystick.addEventListener("pointerdown", startDrag);
joystick.addEventListener("pointermove", drag);
joystick.addEventListener("pointerup", endDrag);
joystick.addEventListener("pointerleave", endDrag);


window.toggleDarkMode = function(isDark) {
	  document.body.classList.toggle("dark", isDark);
	  [".left", ".right", ".header"].forEach(sel => {
		const el = document.querySelector(sel);
		if (el) el.classList.toggle("dark", isDark);
	  });
	  localStorage.setItem("darkMode", isDark ? "1" : "0");
};


function startDrag(e) {
    active = true;
    joystick.style.cursor = "grabbing";
    drag(e);
}

function drag(e) {
    if (!active) return;

    const rect = joystick.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;
    const x = e.clientX - rect.left - centerX;
    const y = e.clientY - rect.top - centerY;

    const maxDist = rect.width / 2;
    const dist = Math.min(Math.hypot(x, y), maxDist);
    const angle = Math.atan2(y, x);
    const dx = Math.cos(angle) * dist;
    const dy = Math.sin(angle) * dist;

    knob.style.left = `${centerX + dx}px`;
    knob.style.top = `${centerY + dy}px`;

    // Normalize to -255 to +255 range
    const xVal = Math.round((dx / maxDist) * 255);
    const yVal = Math.round((-dy / maxDist) * 255); // negative = forward

    handle2DJoystick(xVal, yVal);
}

window.onload = function () {
  realOnLoad();
}



function endDrag() {
    active = false;
    knob.style.left = "50%";
    knob.style.top = "50%";

    // Explicitly force 0 speed in all directions
    lastSentMotorValues["Forward"] = -1;
    lastSentMotorValues["Backward"] = -1;
    lastSentMotorValues["Left"] = -1;
    lastSentMotorValues["Right"] = -1;
    joystick.style.cursor = "grab";
    handle2DJoystick(0, 0);
}


function handle2DJoystick(x, y) {
    x = Math.max(-255, Math.min(255, x));
    y = -Math.max(-255, Math.min(255, y)); // Invert Y

    const threshold = 20;
    let direction = null;
    let value = 0;
    let showLeft = false;
    let showRight = false;

    if (Math.abs(y) > Math.abs(x)) {
      if (y > threshold) {
        direction = "Backward";  // ⬅️ SWAPPED
        value = y;
      } else if (y < -threshold) {
        direction = "Forward";   // ⬅️ SWAPPED
        value = -y;
      }
    } else {
      if (x > threshold) {
        direction = "Right";
        value = x;
        showRight = true;
      } else if (x < -threshold) {
        direction = "Left";
        value = -x;
        showLeft = true;
      }
    }

    // Update left/right UI overlay based on direction
    const leftIndicator = document.getElementById("leftIndicator");
    const rightIndicator = document.getElementById("rightIndicator");

    if (direction === "Left") {
      leftIndicator.classList.add("visible", "blinking");
      rightIndicator.classList.remove("visible", "blinking");
    } else if (direction === "Right") {
      rightIndicator.classList.add("visible", "blinking");
      leftIndicator.classList.remove("visible", "blinking");
    } else {
      leftIndicator.classList.remove("visible", "blinking");
      rightIndicator.classList.remove("visible", "blinking");
    }

    // Send UDP trigger for turn signals (Left or Right)
    if (direction === "Left" || direction === "Right") {
      sendButtonInput("Slider", direction + "," + value);
    }

    // 🔁 Motor commands
    if (!direction) {
      sendMotorSpeed("Forward", 0);
      sendMotorSpeed("Backward", 0);
      sendMotorSpeed("Left", 0);
      sendMotorSpeed("Right", 0);
      return;
    }

    sendMotorSpeed(direction, Math.min(255, value));
}

function setJoystickKnob(x, y) {
    const rect = joystick.getBoundingClientRect();
    const centerX = rect.width / 2;
    const centerY = rect.height / 2;
    const maxDist = rect.width / 2;

    // Normalize to visual scale
    const dx = (x / 255) * maxDist;
    const dy = (-y / 255) * maxDist; // negative y = up

    knob.style.left = `${centerX + dx}px`;
    knob.style.top = `${centerY + dy}px`;
}

function realOnLoad() {
  // --- DARK MODE SYNC: Fetch from backend as soon as modal & DOM are ready ---
  fetch('/getsettings')
    .then(r => r.json())
    .then(data => {
      if ("darkMode" in data) {
        window.toggleDarkMode(data.darkMode == 1);
        const darkToggle = document.getElementById("darkToggle");
        if (darkToggle) darkToggle.checked = (data.darkMode == 1);
      }
      if ("horizontalScreen" in data) {
        const horizontalToggle = document.getElementById("horizontalToggle");
        if (horizontalToggle) horizontalToggle.checked = (data.horizontalScreen == 1);
      }
      if ("holdBucket" in data) {
        const holdBucketToggle = document.getElementById("holdBucketToggle");
        if (holdBucketToggle) holdBucketToggle.checked = (data.holdBucket == 1);
      }
      if ("holdAux" in data) {
        const holdAuxToggle = document.getElementById("holdAuxToggle");
        if (holdAuxToggle) holdAuxToggle.checked = (data.holdAux == 1);
      }
      if ("RecordTelemetry" in data) {
        const recordTelemetrySwitch = document.getElementById("recordTelemetryToggle");
        if (recordTelemetrySwitch) recordTelemetrySwitch.checked = (data.RecordTelemetry == 1);
      }
      if ("SystemSounds" in data) {
        const systemSoundsToggle = document.getElementById("systemSoundsToggle");
        if (systemSoundsToggle) systemSoundsToggle.checked = (data.SystemSounds == 1);
      }
      if ("SystemVolume" in data) {
        const systemVolume = document.getElementById("systemVolume");
        const systemVolumeLabel = document.getElementById("systemVolumeLabel");
        if (systemVolume) systemVolume.value = data.SystemVolume;
        if (systemVolumeLabel) systemVolumeLabel.innerText = data.SystemVolume;
      }
    })
    .catch(() => {
      window.toggleDarkMode(localStorage.getItem("darkMode") === "1");
    });

  // --- Your original code follows ---
  auxSlider = document.getElementById("AUX");
  bucketSlider = document.getElementById("Bucket");
  updateSliderValue("Bucket");
  updateSliderValue("AUX");

  // Arm slider logic
  const armSlider = document.getElementById("armVerticalSlider");
  const armValueLabel = document.getElementById("armVerticalValue");
  let lastSentArmValue = 0;

  if (armSlider && armValueLabel) {
    armSlider.addEventListener("input", function () {
      let value = parseInt(armSlider.value);
      armValueLabel.textContent = value;
      if (value !== lastSentArmValue) {
        sendMotorSpeed("Arm", value);
        lastSentArmValue = value;
      }
    });

    armSlider.addEventListener("change", function () {
      armSlider.value = 0;
      armValueLabel.textContent = 0;
      sendMotorSpeed("Arm", 0);
      lastSentArmValue = 0;
    });
  }

  // Restore emergency state if needed
  const emergencyBtn = document.getElementById("emergencyBtn");
  if (emergencyBtn?.classList.contains("active")) {
    emergencyOn = true;
    document.getElementById("leftIndicator")?.classList.add("visible", "blinking");
    document.getElementById("rightIndicator")?.classList.add("visible", "blinking");
    emergencyBtn.classList.add("blinking");
  }

  initWebSocket();

  // ---------- LIVE KEYMAP SUPPORT ----------
  // Map firmware (snake_case) names to UI action names used in handleKeyDown/Up
  const FW_TO_UI = {
    forward: "forward",
    backward: "backward",
    left: "left",
    right: "right",
    stop: "stop",
    arm_up: "armUp",
    arm_down: "armDown",
    bucket_up: "bucketUp",
    bucket_down: "bucketDown",
    aux_up: "auxUp",
    aux_down: "auxDown",
    light_toggle: "led",
    beacon_toggle: "beacon",
    emergency: "emergency",          // new stored key (≤15 chars)
    emergency_toggle: "emergency",    // legacy alias (if backend still returns it)
    horn: "horn",
  };

  // Global so modal can call it: window.refreshRuntimeKeymap()
	window.refreshRuntimeKeymap = async function refreshRuntimeKeymap() {
		try {
			const data = await (await fetch('/get_keymap')).json();
			// Clear the existing object so all references stay valid
			for (const k in keymap) delete keymap[k];

			// Map fw (snake_case) -> UI action name (used by handlers)
			const FW_TO_UI = {
				forward: "forward",
				backward: "backward",
				left: "left",
				right: "right",
				stop: "stop",
				arm_up: "armUp",
				arm_down: "armDown",
				bucket_up: "bucketUp",
				bucket_down: "bucketDown",
				aux_up: "auxUp",
				aux_down: "auxDown",
				light_toggle: "led",
				beacon: "beacon",
				beacon_toggle: "beacon",      // ← back-compat alias fixes your issue
				emergency: "emergency",
				emergency_toggle: "emergency", // ← optional back-compat
				horn: "horn",
			};

			Object.entries(data).forEach(([fwAction, key]) => {
				const uiAction = FW_TO_UI[fwAction] || fwAction;
				const normKey = (key || "").toLowerCase();
				if (normKey) keymap[normKey] = uiAction;
			});

			console.log('[keymap] refreshed', keymap);
		} catch (e) {
			console.error('refreshRuntimeKeymap failed', e);
		}
	};


  // Initial load of the live keymap
  window.refreshRuntimeKeymap();

  // If the ESP broadcasts KEYMAP_UPDATED over the CarInput WS, refresh immediately.
  // (Safe to attach even if initWebSocket also sets onmessage; addEventListener coexists.)
  (function attachKeymapWsListener(retries = 10) {
    const ws = window.wsCarInput || window.websocketCarInput;
    if (ws && !ws.__keymapListenerAttached) {
      ws.addEventListener('message', (ev) => {
        // Only handle string frames here
        if (typeof ev.data !== 'string') return;
        if (ev.data === 'KEYMAP_UPDATED') {
          window.refreshRuntimeKeymap && window.refreshRuntimeKeymap();
        }
      });
      ws.__keymapListenerAttached = true;
    } else if (retries > 0) {
      setTimeout(() => attachKeymapWsListener(retries - 1), 200);
    }
  })();
  // ---------- END LIVE KEYMAP SUPPORT ----------

  // Keyboard listeners
  document.addEventListener("keydown", handleKeyDown);
  document.addEventListener("keyup", handleKeyUp);

  // FPS placeholder
  document.getElementById("fpsOverlay").innerText = "FPS: ...";

  // Lane overlay
  const laneOverlay = document.getElementById("laneOverlay");
  if (laneOverlay) {
    laneOverlay.style.display =
      localStorage.getItem("laneOverlayVisible") === "true" ? "block" : "none";
  }
  fetchCameraStatus();
}



function updateLedButtonState(isOn) {
    const btn = document.getElementById("ledToggleBtn");
    ledOn = isOn;

    if (btn) {
      if (isOn) {
        btn.innerText = "💡 LED: ON";
        btn.style.backgroundColor = "#ffd700";
        btn.style.color = "black";
      } else {
        btn.innerText = "💡 LED: OFF";
        btn.style.backgroundColor = "#444";
        btn.style.color = "white";
      }
    }
}

function updateDeviceProgress(filename, elapsed, duration) {
	  if (typeof window.mediaPlayerUpdateDeviceProgress === "function") {
		window.mediaPlayerUpdateDeviceProgress(filename, elapsed, duration);
	  }
	  // Optionally: update a basic overlay, or ignore if only handled in mediaPlayer.js
	}

function setDevicePlayState(filename, isPlaying) {
	  if (typeof window.mediaPlayerSetDevicePlayState === "function") {
		window.mediaPlayerSetDevicePlayState(filename, isPlaying);
	  }
	  // Optionally: set global flags/UI outside media player if needed
}


function initWebSocket() {
  websocketCarInput = new WebSocket("ws://" + location.host + "/CarInput");
  window.wsCarInput = websocketCarInput;

  websocketCarInput.onopen  = () => console.log("WebSocket Connected");
  websocketCarInput.onclose = () => setTimeout(initWebSocket, 2000);

  // ✅ Robust onmessage: handles string, Blob, ArrayBuffer
  websocketCarInput.onmessage = function (event) {
    const deliver = (text) => {
      const msg = (typeof text === "string") ? text : String(text ?? "");

      // --- MEDIA PLAYER DEVICE EVENTS ---
      if (msg.startsWith("MEDIA_DEVICE_PROGRESS,")) {
        const parts = msg.split(",");
        if (parts.length >= 4) {
          const filename = decodeURIComponent(parts[1]);
          const elapsed  = parseFloat(parts[2]);
          const duration = parseFloat(parts[3]);
          updateDeviceProgress(filename, elapsed, duration);
        }
        return;
      }
      if (msg.startsWith("MEDIA_DEVICE_PLAYING,")) {
        const filename = decodeURIComponent(msg.split(",")[1] || "");
        setDevicePlayState(filename, true);
        return;
      }
      if (msg.startsWith("MEDIA_DEVICE_STOPPED")) {
        setDevicePlayState(null, false);
        return;
      }

      // --- IMU ---
      if (msg.startsWith("IMU,")) {
        const p = msg.split(",");
        if (p.length >= 8) {
          const [_, h, r, pch, mx, my, mz, temp] = p;
          if (!imuScriptLoaded) {
            const script = document.createElement("script");
            script.src = "/telemetryScript.js?v=" + Date.now();
            script.onload = () => { imuScriptLoaded = true; handleIMUMessage(h, r, pch, mx, my, mz, temp); };
            document.body.appendChild(script);
          } else {
            handleIMUMessage(h, r, pch, mx, my, mz, temp);
          }
        }
        return;
      }

      // --- General key,value CSV messages ---
      const parts = msg.split(",");
      const key   = (parts[0] || "").toString();
      const value = (parts[1] || "").toString();
      const normalizedKey = key.toUpperCase();

      if (key === "Light") {
        const on = (value === "1");
        ledOn = on;
        const btn = document.getElementById("ledToggleBtn");
        if (btn) {
          if (on) { btn.innerText = "💡 LED: ON";  btn.style.backgroundColor = "#ffd700"; btn.style.color = "black"; }
          else    { btn.innerText = "💡 LED: OFF"; btn.style.backgroundColor = "#444";    btn.style.color = "white"; }
        }
        return;
      }

      if (key === "GPIOCONF") {
        value.split(";").forEach(pair => {
          const [k, v] = pair.split(":");
          if (k === "Left")  document.getElementById("gpioLeft").value  = v;
          if (k === "Right") document.getElementById("gpioRight").value = v;
          if (k === "Servo") document.getElementById("gpioServo").value = v;
        });
        return;
      }

      if (key === "FPS") {
        document.getElementById('fpsOverlay').innerText = "FPS: " + value;
        lastFPSUpdate = Date.now();
        return;
      }

      if (key === "AUX") {
        auxSlider.value = parseInt(value || "0", 10);
        updateSliderValue("AUX");
        return;
      }
      if (key === "Bucket") {
        bucketSlider.value = parseInt(value || "0", 10);
        updateSliderValue("Bucket");
        return;
      }

      if (key === "BATT") {
        const batteryPercent = parseInt(parts[1] || "0", 10);
        latestBatteryPercent = batteryPercent;
        const voltage = parseFloat(parts[2] || "0");
        const wifiQuality = parseInt(parts[3] || "0", 10);

        const batteryText = document.getElementById('batteryText');
        const wifiText    = document.getElementById('wifiText');

        batteryText.innerText = "Batt: " + batteryPercent + "% (" + voltage.toFixed(2) + "V)";
        batteryText.className = "";
        if (batteryPercent > 70) batteryText.classList.add('batt-green');
        else if (batteryPercent > 40) batteryText.classList.add('batt-orange');
        else if (batteryPercent > 20) batteryText.classList.add('batt-red');
        else batteryText.classList.add('batt-critical');

        wifiText.innerText = "WiFi: " + wifiQuality + "%";
        wifiText.className = "";
        if (wifiQuality > 70) wifiText.classList.add('wifi-green');
        else if (wifiQuality > 40) wifiText.classList.add('wifi-orange');
        else wifiText.classList.add('wifi-red');
        return;
      }

      if (key === "CHARGE") {
        const chargeIcon = document.getElementById("chargeIcon");
        if (value === "YES") {
          chargeIcon.style.display = "inline";
          chargeIcon.innerText = "⚡";
          const m = document.getElementById("batteryText").innerText.match(/\(([\d.]+)V\)/);
          const voltage = m ? parseFloat(m[1]) : 0;
          if (voltage >= 8.4) { chargeIcon.style.animation = "none"; chargeIcon.style.color = "lime"; }
          else {
            chargeIcon.style.animation = "fadeCharge 1.5s infinite";
            const pm = document.getElementById("batteryText").innerText.match(/Batt: (\d+)%/);
            const percent = Math.min(100, Math.max(0, pm ? parseInt(pm[1],10) : 0));
            const red = Math.round(255 - percent * 2.55);
            const green = Math.round(percent * 2.55);
            chargeIcon.style.color = `rgb(${red}, ${green}, 0)`;
          }
        } else if (value === "FAULT") {
          chargeIcon.style.display = "inline";
          chargeIcon.innerText = "⚡🚫";
          chargeIcon.style.animation = "flashRed 1s infinite";
          chargeIcon.style.color = "red";
        } else {
          chargeIcon.style.display = "none";
        }
        return;
      }

      if (key === "STATS") {
        const uptimeSecs = parseInt(parts[1] || "0", 10);
        const chipTemp   = parseFloat(parts[2] || "0");
        const uptimeMins = Math.floor(uptimeSecs / 60);
        const stats = document.getElementById("statsOverlay");
        stats.innerText = "Uptime: " + uptimeMins + " min, Temp: " + chipTemp + "C";
        stats.className = "overlay-topright";
        if (chipTemp >= 70) stats.classList.add("temp-critical");
        else if (chipTemp >= 55) stats.classList.add("temp-warning");
        return;
      }

      if (normalizedKey === "TURN_LEFT" || normalizedKey === "TURN_RIGHT") {
        const el = document.getElementById(normalizedKey === "TURN_LEFT" ? "leftIndicator" : "rightIndicator");
        if (value === "1") { if (!emergencyOn) el.classList.add("blinking","visible"); }
        else { el.classList.remove("blinking","visible"); }
        return;
      }

      if (key === "Beacon") {
        beaconActive = (value === "1");
        const btn = document.getElementById("beaconBtn");
        if (btn) btn.classList.toggle("blinking", beaconActive);
        return;
      }

      if (normalizedKey === "EMERGENCY") {
        emergencyOn = (value === "1");
        const left  = document.getElementById("leftIndicator");
        const right = document.getElementById("rightIndicator");
        const btn   = document.getElementById("emergencyBtn");
        if (emergencyOn) {
          left.classList.add("blinking","visible");
          right.classList.add("blinking","visible");
          if (btn) { btn.classList.add("blinking","active"); btn.innerText = "⚠️ Emergency ON"; }
        } else {
          left.classList.remove("blinking","visible");
          right.classList.remove("blinking","visible");
          if (btn) { btn.classList.remove("blinking","active"); btn.innerText = "⚠️ Emergency"; }
        }
        return;
      }
    };

    const d = event.data;
    if (typeof d === "string") return deliver(d);
    if (d instanceof Blob)     return d.text().then(deliver).catch((e)=>console.warn("WS blob->text failed", e));
    if (d instanceof ArrayBuffer) {
      try { return deliver(new TextDecoder().decode(new Uint8Array(d))); }
      catch (e) { console.warn("WS buffer decode failed", e); return; }
    }
    console.warn("WS: ignoring non-text frame", d);
  };
}


function isInputFocused() {
  // Returns true if the focus is on a text, number, password, textarea, or contenteditable element
  const a = document.activeElement;
  if (!a) return false;
  if (a.tagName === "INPUT" || a.tagName === "TEXTAREA" || a.isContentEditable) {
    // Optionally: limit to input types where typing matters
    const types = ["text", "number", "password", "email", "search", "url"];
    return !a.disabled && !a.readOnly && (a.tagName !== "INPUT" || types.includes(a.type));
  }
  return false;
}

function isModalOpen() {
  const modal = document.getElementById("settingsModal");
  // style.display !== "none" && has "active" class, or just check display
  return modal && modal.style.display !== "none" && modal.classList.contains("active");
}

function sendButtonInput(key, value) {
	if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
	  websocketCarInput.send(key + "," + value);
	}
}

function handleKeyDown(e) {
	if (isModalOpen() || isInputFocused()) return;  // 👈 Block controls if modal or input
	const key = e.key.toLowerCase();
	if (keysDown[key]) return; // prevent re-processing held key
	keysDown[key] = true;

	const action = keymap[key];
	if (!action) return;

	switch (action) {
	  case "forward":
		sendButtonInput("MoveCar", "2");      // FORWARD
		setJoystickKnob(0, 255);
		break;
	  case "backward":
		sendButtonInput("MoveCar", "1");      // BACKWARD
		setJoystickKnob(0, -255);
		break;
	  case "left":
		sendButtonInput("MoveCar", "3");
		setJoystickKnob(-255, 0);
		break;
	  case "right":
		sendButtonInput("MoveCar", "4");
		setJoystickKnob(255, 0);
		break;
	  case "stop":
		sendButtonInput("MoveCar", "0");
		setJoystickKnob(0, 0);
		break;

	  case "bucketUp":
		bucketSlider.value = parseInt(bucketSlider.value) + 5;
		sendButtonInput("Bucket", bucketSlider.value);
		break;
	  case "bucketDown":
		bucketSlider.value = parseInt(bucketSlider.value) - 5;
		sendButtonInput("Bucket", bucketSlider.value);
		break;
	  case "auxUp":
		auxSlider.value = parseInt(auxSlider.value) + 5;
		sendButtonInput("AUX", auxSlider.value);
		break;
	  case "auxDown":
		auxSlider.value = parseInt(auxSlider.value) - 5;
		sendButtonInput("AUX", auxSlider.value);
		break;

	  case "led":
		toggleLed();
		break;
	  case "beacon":
		toggleBeacon();
		break;
	  case "emergency":
		toggleEmergency();
		break;
		case "horn":
		sendButtonInput("Horn", "1");  // or your horn-on command
		break;		
	}
}

function handleKeyUp(e) {
	if (isModalOpen() || isInputFocused()) return;  // 👈 Block controls if modal or input
	const key = e.key.toLowerCase();
	delete keysDown[key];

	const action = keymap[key];
	if (!action) return;

	// Stop movement on release
	if (["forward", "backward", "left", "right"].includes(action)) {
	  sendButtonInput("MoveCar", "0");
	  setJoystickKnob(0, 0);
	}
	if (action === "horn") {
		sendButtonInput("Horn", "0");  // or your horn-off command
	}	
}

  function updateSliderValue(id) {
    const slider = document.getElementById(id);
    const span = document.getElementById(id + "Value");
    if (slider && span) {
      span.innerText = slider.value;
    }
  }

  function sendMotorSpeed(direction, value) {
    value = parseInt(value);
      //if (lastSentMotorValues[direction] === value && value !== 0) return; // allow redundant zeroes
      if (value === 0 && lastSentMotorValues[direction] === 0) return; // skip repeated 0s
      if (value !== 0 && lastSentMotorValues[direction] === value) return; // skip repeated non-0s


    lastSentMotorValues[direction] = value;

    if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
      websocketCarInput.send("Motor," + direction + "," + value);


    }
  }

  function resetSlider(sliderElem, direction) {
    // Cancel any existing animation for this direction
    if (sliderAnimations[direction]) {
      clearInterval(sliderAnimations[direction]);
      delete sliderAnimations[direction];
    }

    let current = parseInt(sliderElem.value);
    const max = current;
    const duration = 500; // total animation time (ms)
    const steps = 30;
    let frame = 0;

    sliderAnimations[direction] = setInterval(() => {
      // End condition
      if (frame >= steps) {
        clearInterval(sliderAnimations[direction]);
        delete sliderAnimations[direction];
        sliderElem.value = 0;

        // Force send 0
        lastSentMotorValues[direction] = -1;
        sendMotorSpeed(direction, 0);
        updateSliderValue(sliderElem.id);

        // Handle turn signal deactivation
        if (direction === "Left" || direction === "Right") {
          sendButtonInput("Slider", direction + ",0");
          const indicator = document.getElementById(direction === "Left" ? "leftIndicator" : "rightIndicator");
          indicator.classList.remove("blinking");
          indicator.classList.remove("visible");
        }
        return;
      }

      // Ease out value
      const t = frame / steps;
      const eased = max * (1 - t * t);
      const newVal = Math.round(eased);

      if (parseInt(sliderElem.value) !== newVal) {
        sliderElem.value = newVal;
        updateSliderValue(sliderElem.id);
        sendMotorSpeed(direction, newVal);
      }

      frame++;
    }, duration / steps);
  }


	function showToast(message, type = "info") {
		const toast = document.getElementById("toast");
		toast.innerText = message;
		// Pick color by type
		toast.style.backgroundColor =
			type === "error" ? "#e53935" :
			type === "success" ? "#4CAF50" :
			"#1976d2"; // info (blue)
		toast.style.visibility = "visible";
		toast.style.opacity = "1";

		setTimeout(() => {
			toast.style.opacity = "0";
			setTimeout(() => toast.style.visibility = "hidden", 500);
		}, 3000);
	}


  function getResolutionName(code) {
    switch (code) {
      case 10: return "UXGA";
      case 6:  return "SVGA";
      case 5:  return "VGA";
      case 3:  return "QVGA";
      default: return "VGA";
    }
  }

  function controlLed(state) {
    sendButtonInput("Light", state ? 1 : 0); // ✅ Use WebSocket, not fetch()
    showToast(`💡 LED ${state ? "ON" : "OFF"}`);
  }

  setInterval(() => {
    const now = Date.now();
    const camStatus = document.getElementById("camStatusOverlay");
    if ((now - lastFPSUpdate) < 5000) {
      camStatus.style.display = "none";
    } else {
      camStatus.style.display = "block";  // No FPS in last 5 sec = Camera down
    }
  }, 2000); // Check every 2 seconds


	function updateLedBrightness(value) {
	  fetch(`/led?brightness=${value}`)
		.then(r => r.text())
		.then(txt => {
		  if (txt.startsWith("LEDSTATE:")) {
			const isOn = txt.includes("1");
			updateLedButtonState(isOn);
		  }
		  showToast(`💡 LED Brightness: ${value}`);
		})
		.catch(err => {
		  console.warn("LED brightness fetch error:", err);
		});
	}


  window.showTab = function(tabId, button = null) {
    const wrapper = document.getElementById('tabContentWrapper');
    const prev = wrapper.querySelector('.tabContent.active');
    const next = document.getElementById(tabId);

    if (prev === next) return;

    // Hide all tab contents
    document.querySelectorAll('.tabContent').forEach(div => {
      div.classList.remove('active');
      div.style.display = 'none'; // <--- HIDE explicitly
    });

    // Show selected tab
    next.classList.add('active');
    next.style.display = 'block'; // <--- SHOW explicitly

    // Animate height
    const prevHeight = prev ? prev.offsetHeight : 0;
    const nextHeight = next.offsetHeight;
    wrapper.style.height = prevHeight + 'px';

    requestAnimationFrame(() => {
      wrapper.style.height = nextHeight + 'px';
    });

    setTimeout(() => {
      wrapper.style.height = '';
    }, 300);

    // Toggle active tab button
    document.querySelectorAll('.tabButton').forEach(btn => btn.classList.remove('active'));
    if (button) button.classList.add('active');

    //requestAnimationFrame(updateFrameShape);

  }

  function toggleLed() {
    sendButtonInput("Light", 2);  // Use 2 as a toggle command
  }

  function toggleBeacon() {
    beaconActive = !beaconActive;
    sendButtonInput("Beacon", beaconActive ? 1 : 0);

    const btn = document.getElementById("beaconBtn");
    if (beaconActive) {
      btn.classList.add("blinking");
    } else {
      btn.classList.remove("blinking");
    }
  }

  function toggleEmergency() {
    emergencyOn = !emergencyOn;
    const btn = document.getElementById("emergencyBtn");
    btn.classList.toggle("active", emergencyOn);
    btn.innerText = emergencyOn ? "⚠️ Emergency ON" : "⚠️ Emergency";

    // Send UDP/WebSocket signal
    sendButtonInput("Emergency", emergencyOn ? 1 : 0);

    const left = document.getElementById("leftIndicator");
    const right = document.getElementById("rightIndicator");

    if (emergencyOn) {
      left.classList.add("visible", "blinking");
      right.classList.add("visible", "blinking");

      btn.classList.add("blinking"); // ✅ Restore blinking on Emergency button
    } else {
      left.classList.remove("visible", "blinking");
      right.classList.remove("visible", "blinking");

      btn.classList.remove("blinking"); // ✅ Stop blinking when off
    }
  }

  function toggleLaneOverlay() {
    const overlay = document.getElementById("laneOverlay");
    if (!overlay) return;

    const isVisible = overlay.style.display !== "none";
    overlay.style.display = isVisible ? "none" : "block";

    // Save state to localStorage
    localStorage.setItem("laneOverlayVisible", !isVisible);
  }

function toggleSettingsModal() {
	const mainContent = document.getElementById('mainContent');
	let modal = document.getElementById('settingsModal');
	const isVisible = modal && modal.classList.contains('active');

	if (isVisible) {
		// Remember which tab was active when closing
		try {
			const active = document.querySelector('#settingsModal .tabContent.active');
			if (active) localStorage.setItem('settingsLastTab', active.id);
		} catch (e) {}		
		// Hide with fade out, then REMOVE from DOM
		modal.classList.remove('active');
		setTimeout(() => {
			modal.style.display = 'none';
			modal.remove();
			if (typeof onunloadModal === "function") onunloadModal();
		}, 250);
		mainContent.classList.remove('blur');
		window.modalScriptLoaded = false;
	} else {
		// Modal not present, load HTML and JS fresh every time
		fetch('/settingsModal.html')
			.then(res => res.text())
			.then(html => {
				const div = document.createElement('div');
				div.innerHTML = html;
				document.body.appendChild(div.firstElementChild);

				const modal = document.getElementById('settingsModal');
				modal.style.display = 'block';
				loadModalCss();

				requestAnimationFrame(() => {
					modal.classList.add('active');
				});
				mainContent.classList.add('blur');

				// --- Attach close handlers immediately after injection ---
				// Click-away (backdrop)
				modal.addEventListener('mousedown', function(e) {
					if (e.target === modal) toggleSettingsModal();
				});
				// X button (adjust selector as needed)
				const closeBtn = modal.querySelector('.modal-close-btn, #settingsModalCloseBtn, .close');
				if (closeBtn) closeBtn.onclick = toggleSettingsModal;

				// --- Load modalScript.js fresh every time ---
				const oldScript = document.getElementById("modalScript");
				if (oldScript) oldScript.remove();

				const script = document.createElement('script');
				script.src = "/modalScript.js?v=" + Date.now();
				script.id = "modalScript";
				script.onload = () => {
					window.modalScriptLoaded = true;
					console.log("✅ Modal script loaded dynamically.");
					let startTab = localStorage.getItem('settingsLastTab') || 'keysTab';
					if (!document.getElementById(startTab)) startTab = 'keysTab'; // fallback safety
					showTab(startTab);

				};
				document.body.appendChild(script);
			});
	}
}




function loadModalCss() {
  if (!document.getElementById('modalCssLink')) {
    const link = document.createElement('link');
    link.rel = 'stylesheet';
    link.id = 'modalCssLink';
    link.href = '/modal.css?v=' + Date.now(); // Cache-busting
    document.head.appendChild(link);
  }
}


function toggleDrawMode() {
	// First-time load check
	if (!window.drawScriptLoaded) {
		const script = document.createElement("script");
		script.src = "/drawScript.js?v=" + Date.now();
		script.id = "drawScript";
		script.onload = () => {
			window.drawScriptLoaded = true;
			console.log("✅ Draw script loaded dynamically.");
			window.toggleDrawMode();  // Execute actual function
		};
		document.body.appendChild(script);
	} else {
		window.toggleDrawMode(); // Already loaded, just call it
	}
}

window.addEventListener("DOMContentLoaded", () => {
  const splash = document.getElementById("splashScreen");
  const text = document.getElementById("splashText");
  const main = document.getElementById("mainContent");

  const micBtn = document.getElementById("micBtn");
  micBtn.addEventListener("mousedown", () => micBtn.classList.add("pushed"));
  micBtn.addEventListener("mouseup", () => micBtn.classList.remove("pushed"));
  micBtn.addEventListener("mouseleave", () => micBtn.classList.remove("pushed"));
	
  // Main UI stays invisible and on black background until we fade in
  if (main) {
    main.style.opacity = "0";
    main.style.transition = "opacity 0.8s";
    main.style.background = "#000";
  }
  splash.style.opacity = "1";
  splash.style.transition = "opacity 1s";
  text.style.opacity = "0";
  text.style.transition = "opacity 0.7s";

  // 1. Wait 1s, fade in MINIEXCO
  setTimeout(() => {
    text.style.opacity = "1";

    // 2. Wait 1.2s, fade out MINIEXCO (over 1.4s)
    setTimeout(() => {
      text.style.transition = "opacity 1.4s"; // Set fade-out duration
      text.style.opacity = "0";

      // 3. Wait 1.4s for text to fade out, then fade out black splash (1s)
      setTimeout(() => {
        splash.style.opacity = "0";

        // 4. After splash fade (1s), show main content, keep bg black to prevent white flash
        setTimeout(() => {
          if (main) {
            main.style.background = "#000"; // Keep black background just in case
            main.style.opacity = "1";
          }
          splash.remove();

          // (Optional) After fade-in, if you want, you can set main's bg to original
          setTimeout(() => {
            if (main) main.style.background = "";
          }, 500);

        }, 1000);

      }, 1400);

    }, 1200);

  }, 1000);
	
	wireBatteryText();
});


// --- DYNAMIC MEDIA PLAYER LOADER ---

window.toggleMediaPlayer = function() {
  let modal = document.getElementById('mediaPlayerModal');
  let isVisible = modal && modal.style.display === 'block';

  if (isVisible) {
    // Hide and remove modal, clean up
    modal.style.display = 'none';
    setTimeout(() => {
      modal.remove();
      // Remove the script if you want true reload each time
      const oldScript = document.getElementById('mediaPlayerScript');
      if (oldScript) oldScript.remove();
      window.mediaPlayerScriptLoaded = false;
    }, 200);
    return;
  }

  // --- SHOW MEDIA PLAYER: Load CSS and script as needed ---
  // Dynamically load mediaPlayer.css if not yet loaded
  if (!document.getElementById("mediaPlayerCss")) {
    const link = document.createElement("link");
    link.id = "mediaPlayerCss";
    link.rel = "stylesheet";
    link.href = "/mediaPlayer.css?v=" + Date.now();
    document.head.appendChild(link);
  }

  // Now, load the JS as before
  if (!window.mediaPlayerScriptLoaded) {
    const oldScript = document.getElementById('mediaPlayerScript');
    if (oldScript) oldScript.remove();

    const s = document.createElement('script');
    s.src = '/mediaPlayer.js?v=' + Date.now();
    s.id = 'mediaPlayerScript';
    s.onload = function() {
      window.mediaPlayerScriptLoaded = true;
      if (typeof window.toggleMediaPlayerReal === "function") {
        window.toggleMediaPlayerReal();
      } else {
        showToast("Media player failed to initialize.");
      }
    };
    s.onerror = function() {
      showToast("Failed to load mediaPlayer.js from SD card!");
    };
    document.body.appendChild(s);
  } else {
    if (typeof window.toggleMediaPlayerReal === "function") {
      window.toggleMediaPlayerReal();
    } else {
      showToast("Media player script not ready!");
    }
  }
};


function toggleMic() {
  const micBtn = document.getElementById("micBtn");
  const micIcon = document.getElementById("micIcon");
  if (!micEnabled) {
    // Turn mic ON: tell backend to enable mic, pause speaker playback, start streaming mic audio
    micEnabled = true;
    micBtn.classList.add("active", "listening");
    micIcon.style.color = "#fff700";
    micBtn.title = "Listening to robot mic (click to disable)";
    // Pause media playback if needed
    pauseMediaPlayerIfPlaying();

    // Enable mic on backend
    fetch('/enable_mic')
      .then(() => {
        // Start streaming mic audio (assumes backend provides /mic_stream endpoint)
        if (micAudioElem) {
          micAudioElem.pause();
          micAudioElem.remove();
        }
        micAudioElem = document.createElement("audio");
        micAudioElem.id = "robotMicAudio";
        micAudioElem.src = `/mic_stream?${Date.now()}`; // Prevent caching
        micAudioElem.autoplay = true;
        micAudioElem.controls = false;
        micAudioElem.style.display = "none";
        document.body.appendChild(micAudioElem);
        micAudioElem.play().catch(()=>{});
      })
      .catch(() => {
        showToast("Failed to enable mic", "error");
        micBtn.classList.remove("active", "listening");
        micEnabled = false;
      });

  } else {
    // Turn mic OFF: tell backend to disable mic, stop audio stream
    micEnabled = false;
    micBtn.classList.remove("active", "listening");
    micIcon.style.color = "";
    micBtn.title = "🎤 Mic: listen to the robot audio (press to enable)";
    // Remove mic audio element
    if (micAudioElem) {
      micAudioElem.pause();
      micAudioElem.remove();
      micAudioElem = null;
    }
    fetch('/disable_mic');
  }
}


function pauseMediaPlayerIfPlaying() {
  // Pause browser media
  let mp = document.getElementById("mediaPlayerAudio");
  if (mp && !mp.paused && !mp.ended) mp.pause();

  // Pause player from dynamically loaded mediaPlayer.js if available
  if (typeof window.getPlayer === "function") {
    let player = window.getPlayer();
    if (player && !player.paused && !player.ended) player.pause();
  }

  // Always stop playback on device (ESP32)
  fetch('/stop_playback');
}




// Minimal toast implementation (if not already present)
window.showToast = window.showToast || function(msg, duration) {
  let toast = document.getElementById('toast');
  if (!toast) {
    toast = document.createElement('div');
    toast.id = 'toast';
    toast.style = 'visibility:hidden;min-width:250px;background:#333;color:#fff;text-align:center;border-radius:2px;padding:16px;position:fixed;z-index:9999;left:50%;bottom:30px;font-size:17px;opacity:0;transition:opacity 0.5s;';
    document.body.appendChild(toast);
  }
  toast.innerText = msg;
  toast.style.visibility = 'visible';
  toast.style.opacity = '1';
  setTimeout(() => {
    toast.style.opacity = '0';
    setTimeout(() => { toast.style.visibility = 'hidden'; }, 500);
  }, duration || 2200);
};




//Load Battery popup graph

function wireBatteryText() {
  const batteryText = document.getElementById("batteryText");
  if (!batteryText) {
    setTimeout(wireBatteryText, 200);
    return;
  }
  batteryText.title = "Click for more info";
  batteryText.style.cursor = "pointer";
	batteryText.addEventListener("click", () => {
	  // Load uPlot CSS if not already present
	  if (!document.getElementById("uPlotCss")) {
		const link = document.createElement("link");
		link.id = "uPlotCss";
		link.rel = "stylesheet";
		link.href = "/uPlot.min.css";
		document.head.appendChild(link);
	  }
	  // Load uPlot JS if not already present
	  function loadUPlotJs(callback) {
		if (window.uPlot) { callback(); return; }
		if (!document.getElementById("uPlotJs")) {
		  const script = document.createElement("script");
		  script.id = "uPlotJs";
		  script.src = "/uPlot.iife.min.js";
		  script.onload = callback;
		  document.body.appendChild(script);
		} else {
		  // If script tag exists but uPlot not ready, wait
		  const check = setInterval(() => {
			if (window.uPlot) { clearInterval(check); callback(); }
		  }, 50);
		}
	  }
	  // Load your graph.css if needed
	  if (!document.getElementById("batteryGraphCss")) {
		const link = document.createElement("link");
		link.id = "batteryGraphCss";
		link.rel = "stylesheet";
		link.href = "/graph.css?v=" + Date.now();
		document.head.appendChild(link);
	  }
	  // Load your own graph JS after uPlot is ready
	  loadUPlotJs(() => {
		if (!window.batteryGraphLoaded) {
		  let s = document.createElement("script");
		  s.src = "/batteryGraph.js?v=" + Date.now();
		  s.onload = () => {
			if (typeof window.showBatteryPopup === "function") window.showBatteryPopup();
		  };
		  document.body.appendChild(s);
		} else {
		  window.showBatteryPopup();
		}
	  });
	});

}



function capturePhoto() {
  fetch("/capture_photo")
    .then(r => r.json())
    .then(obj => {
      if (obj.status === "ok") {
        showToast("📸 Photo saved: " + obj.path, "success");
        // Reindex photo folder after saving
        fetch('/sd_reindex?path=' + encodeURIComponent('/media/capture/photo'), { method: 'POST' });
      } else {
        showToast("❌ Photo capture failed", "error");
      }
    })
    .catch(() => showToast("❌ Photo capture failed", "error"));
}


function toggleVideoRecording() {
  const btn = document.getElementById('videoCaptureBtn');
  if (!isRecordingVideo) {
    btn.classList.add('recording');
    btn.querySelector('.icon-record .record-outer').setAttribute('fill', '#ff3535');
    btn.querySelector('.icon-record .record-outer').setAttribute('stroke', '#fff');
    btn.childNodes[btn.childNodes.length-1].textContent = " Stop Recording";
    isRecordingVideo = true;
    fetch('/start_record_video?duration=60')
      .then(r => r.json())
      .then(resp => {
        if (resp.status !== "recording") {
          btn.classList.remove('recording');
          btn.childNodes[btn.childNodes.length-1].textContent = " Record Video";
          isRecordingVideo = false;
          showToast("❌ Failed to start recording", "error");
        }
      })
      .catch(() => {
        btn.classList.remove('recording');
        btn.childNodes[btn.childNodes.length-1].textContent = " Record Video";
        isRecordingVideo = false;
        showToast("❌ Failed to start recording", "error");
      });
  } else {
		fetch('/stop_record_video')
			.then(r => r.json())
			.then(resp => {
				btn.classList.remove('recording');
				btn.childNodes[btn.childNodes.length-1].textContent = " Record Video";
				isRecordingVideo = false;
				if (resp.status !== "stopped") {
					showToast("⚠️ Failed to stop recording", "error");
				} else {
					// Reindex video folder after recording
					fetch('/sd_reindex?path=' + encodeURIComponent('/media/capture/video'), { method: 'POST' });
				}
			})
			.catch(() => {
				btn.classList.remove('recording');
				btn.childNodes[btn.childNodes.length-1].textContent = " Record Video";
				isRecordingVideo = false;
				showToast("❌ Failed to stop recording", "error");
			});

  }
}

function updateCameraButton(state) {
  const btn = document.getElementById("cameraToggleBtn");
  cameraEnabled = state;
  if (btn) {
    btn.innerText = cameraEnabled ? "📷 Camera: ON" : "📷 Camera: OFF";
    btn.classList.toggle("active", cameraEnabled);
    btn.style.backgroundColor = cameraEnabled ? "#1976d2" : "#888";
  }
}

function fetchCameraStatus() {
  fetch('/camera_status')
    .then(res => res.json())
    .then(data => {
      updateCameraButton(data.enabled);
      const img = document.getElementById("cameraStream");
      img.src = data.enabled
        ? "http://" + location.hostname + ":81/stream"
        : BLANK_IMG;      // <- prevents browser retries and ERR_CONNECTION_REFUSED
      // Optional: when OFF, hide the “camera down” overlay
      if (!data.enabled) {
        document.getElementById("camStatusOverlay").style.display = "none";
        document.getElementById("fpsOverlay").innerText = "FPS: 0";
      }
    })
    .catch(() => {
      // If status endpoint fails, assume ON to preserve old behavior (or set BLANK_IMG if you prefer)
      updateCameraButton(true);
    });
}


function toggleCamera() {
  const img = document.getElementById("cameraStream");
  const newState = !cameraEnabled;

  // Optimistically blank first when turning OFF to stop any in-flight retries
  if (!newState) img.src = BLANK_IMG;

  // (Optional) prevent double clicks while toggling
  const btn = document.getElementById("cameraToggleBtn");
  btn?.setAttribute("disabled", "disabled");

  fetch(`/camera_enable?val=${newState ? 1 : 0}`)
    .then(res => res.json())
    .then(data => {
      if (["enabled", "disabled", "nochange"].includes(data.status)) {
        updateCameraButton(newState);
        showToast(`Camera ${newState ? "enabled" : "disabled"}`, "info");
        img.src = newState
          ? `http://${location.hostname}:81/stream?ts=${Date.now()}` // cache-bust
          : BLANK_IMG;
      } else {
        showToast("Camera toggle failed", "error");
        // revert UI if API rejected
        updateCameraButton(cameraEnabled);
        if (!cameraEnabled) img.src = BLANK_IMG;
      }
    })
    .catch(() => {
      showToast("Camera toggle failed", "error");
      // revert UI on error
      updateCameraButton(cameraEnabled);
      if (!cameraEnabled) img.src = BLANK_IMG;
    })
    .finally(() => btn?.removeAttribute("disabled"));
}

