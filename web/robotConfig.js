// --- ROBOT CONFIG TAB LOGIC ---


function saveRobotConfig() {
	const left = document.getElementById("gpioLeft").value;
	const right = document.getElementById("gpioRight").value;
	const servo = document.getElementById("gpioServo").value;

	const payload = `GPIO,Left,${left};Right,${right};Servo,${servo}`;
	if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
		websocketCarInput.send(payload);
		showToast("✅ GPIO config sent");
	} else {
		showToast("❌ WebSocket not connected", true);
	}
}


window.initRobotConfigTab = function() {
  // Dark Mode
  const darkToggle = document.getElementById("darkToggle");
  if (darkToggle) {
    // Sync checkbox on open
    darkToggle.checked = (localStorage.getItem("darkMode") === "1");
    // Add handler
    darkToggle.onchange = function() {
      toggleDarkMode(this.checked);
    };
  }

  // Horizontal Screen
  const horizontalToggle = document.getElementById("horizontalToggle");
  if (horizontalToggle) {
    // Initial sync: get value from server/localStorage/whatever (optional)
    // For demo, skip
    horizontalToggle.onchange = function() {
      sendButtonInput("Switch", this.checked ? 1 : 0);
    };
  }

  // Hold Bucket
  const holdBucketSwitch = document.getElementById("holdBucketToggle");
  if (holdBucketSwitch) {
    holdBucketSwitch.onchange = function() {
      sendButtonInput("HoldBucket", this.checked ? 1 : 0);
    };
  }

  // Hold Aux
  const holdAuxSwitch = document.getElementById("holdAuxToggle");
  if (holdAuxSwitch) {
    holdAuxSwitch.onchange = function() {
      sendButtonInput("HoldAux", this.checked ? 1 : 0);
    };
  }

	// Record Telemetry
	const recordTelemetrySwitch = document.getElementById("recordTelemetryToggle");
	if (recordTelemetrySwitch) {
		recordTelemetrySwitch.onchange = function() {
			sendButtonInput("RecordTelemetry", this.checked ? 1 : 0);
		};
	}

	// System Sounds Toggle
	const systemSoundsToggle = document.getElementById("systemSoundsToggle");
	if (systemSoundsToggle) {
		if (typeof window.cachedSystemSounds === "boolean") {
			systemSoundsToggle.checked = window.cachedSystemSounds;
		}
		systemSoundsToggle.onchange = function () {
			const nextState = !!this.checked;
			window.cachedSystemSounds = nextState;
			sendButtonInput("SystemSounds", nextState ? 1 : 0);
		};
	}

	// System Volume Slider
	const systemVolume = document.getElementById("systemVolume");
	const systemVolumeLabel = document.getElementById("systemVolumeLabel");
	if (systemVolume && systemVolumeLabel) {
		if (typeof window.cachedSystemVolume !== "undefined") {
			systemVolume.value = window.cachedSystemVolume;
			systemVolumeLabel.innerText = window.cachedSystemVolume;
		}
		systemVolume.oninput = function () {
			const val = this.value;
			window.cachedSystemVolume = val;
			systemVolumeLabel.innerText = val;
			sendButtonInput("SystemVolume", val);
		};
	paintRobotAudioControls();

	}
}

	// Refresh from backend to ensure latest values (in case modal opened early)
	try {
		fetch('/getsettings').then(r => r.json()).then(data => {
			if (typeof data.SystemSounds !== 'undefined') {
				window.cachedSystemSounds = (data.SystemSounds == 1);
			}
			if (typeof data.SystemVolume !== 'undefined') {
				window.cachedSystemVolume = data.SystemVolume;
			}
			paintRobotAudioControls();
		}).catch(() => {});
	} catch (e) {}


function paintRobotAudioControls() {
	const toggle = document.getElementById("systemSoundsToggle");
	if (toggle && typeof window.cachedSystemSounds === "boolean") {
		toggle.checked = window.cachedSystemSounds;
	}
	const volume = document.getElementById("systemVolume");
	const label = document.getElementById("systemVolumeLabel");
	if (volume && label) {
		let val;
		if (typeof window.cachedSystemVolume !== "undefined") {
			val = parseInt(window.cachedSystemVolume, 10);
		} else {
			val = parseInt(volume.value, 10);
		}
		if (Number.isNaN(val)) val = 0;
		volume.value = val;
		label.innerText = val;
	}
}

window.paintRobotAudioControls = paintRobotAudioControls;

function handleModalWebSocketMessage(key, value) {
	if (key === "darkMode") {
		const darkToggle = document.getElementById('darkToggle');
		if (darkToggle) darkToggle.checked = (value == "1");
		toggleDarkMode(value == "1");
	}
	if (key === "holdBucket") {
		const el = document.getElementById('holdBucketToggle');
		if (el) el.checked = (value == "1");
	}
	if (key === "holdAux") {
		const el = document.getElementById('holdAuxToggle');
		if (el) el.checked = (value == "1");
	}
	if (key === "horizontalScreen") {
		const el = document.getElementById('horizontalToggle');
		if (el) el.checked = (value == "1");
	}
	if (key === "SystemSounds") {
		window.cachedSystemSounds = (value == "1");
		const el = document.getElementById('systemSoundsToggle');
		if (el) el.checked = window.cachedSystemSounds;
		paintRobotAudioControls();
	}
	if (key === "SystemVolume") {
		window.cachedSystemVolume = value;
		const el = document.getElementById('systemVolume');
		const label = document.getElementById('systemVolumeLabel');
		if (el) el.value = value;
		if (label) label.innerText = value;
		paintRobotAudioControls();
	}
	if (key === "RecordTelemetry") {
		const el = document.getElementById('recordTelemetryToggle');
		if (el) el.checked = (value == "1");
	}
}



window.toggleDarkMode = function(isDark) {
	// Always safe
	document.body.classList.toggle("dark", isDark);

	// Only toggle if present (null check)
	[".left", ".right", ".header"].forEach(sel => {
	const el = document.querySelector(sel);
	if (el) el.classList.toggle("dark", isDark);
	});

	localStorage.setItem("darkMode", isDark ? "1" : "0");
	if (window.websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
	websocketCarInput.send("DarkMode," + (isDark ? 1 : 0));
	}
};



function syncDarkToggleCheckbox() {
	const darkToggle = document.getElementById("darkToggle");
	if (darkToggle) {
	darkToggle.checked = (localStorage.getItem("darkMode") === "1");
	}
}

