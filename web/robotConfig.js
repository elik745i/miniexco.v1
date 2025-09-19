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
		systemSoundsToggle.onchange = function() {
			if (window.websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
				websocketCarInput.send("SystemSounds," + (this.checked ? 1 : 0));
			}
		};
	}

	// System Volume Slider
	const systemVolume = document.getElementById("systemVolume");
	const systemVolumeLabel = document.getElementById("systemVolumeLabel");
	if (systemVolume && systemVolumeLabel) {
		systemVolume.oninput = function() {
			systemVolumeLabel.innerText = this.value;
			if (window.websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
				websocketCarInput.send("SystemVolume," + this.value);
			}
		};
	}
	
}

  
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
		const el = document.getElementById('systemSoundsToggle');
		if (el) el.checked = (value == "1");
	}
	if (key === "SystemVolume") {
		const el = document.getElementById('systemVolume');
		const label = document.getElementById('systemVolumeLabel');
		if (el) el.value = value;
		if (label) label.innerText = value;
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

