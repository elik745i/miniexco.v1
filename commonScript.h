// commonScript.h
#pragma once

const char* commonScript PROGMEM = R"HTMLSCRIPT(

  var websocketCarInput;
  var auxSlider;
  var bucketSlider;
  var lastFPSUpdate = Date.now();
  let cameraIP = "";
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
  let imuScriptLoaded = false;
  let ledOn = false;
  let leftIndicator, rightIndicator, emergencyBtn, cameraStream;

  let  lastSentMotorValues = {
    Forward: 0,
    Backward: 0,
    Left: 0,
    Right: 0,
    ArmUp: 0,
    ArmDown: 0
  };

  const sliderAnimations = {};  // Store interval handles by direction

  const joystick = document.getElementById("joystickContainer");
  const knob = document.getElementById("joystickKnob");
  let active = false;

  joystick.addEventListener("pointerdown", startDrag);
  joystick.addEventListener("pointermove", drag);
  joystick.addEventListener("pointerup", endDrag);
  joystick.addEventListener("pointerleave", endDrag);

  function startDrag(e) {
    active = true;
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

  function endDrag() {
    active = false;
    knob.style.left = "50%";
    knob.style.top = "50%";
    handle2DJoystick(0, 0);
  }

  function handle2DJoystick(x, y) {
    // Clamp to range -255..255
    x = Math.max(-255, Math.min(255, x));
    y = Math.max(-255, Math.min(255, y));

    // Invert Y (up is negative)
    y = -y;

    // Tank drive mixing
    let left = y + x;
    let right = y - x;

    // Clamp again after mix
    left = Math.max(-255, Math.min(255, left));
    right = Math.max(-255, Math.min(255, right));

    // Determine direction and magnitude for left motor
    if (Math.abs(left) > 5) {
      sendMotorSpeed(left > 0 ? "Backward" : "Forward", Math.abs(left));
    } else {
      sendMotorSpeed("Forward", 0);
      sendMotorSpeed("Backward", 0);
    }

    // Determine direction and magnitude for right motor
    if (Math.abs(right) > 5) {
      sendMotorSpeed(right > 0 ? "Right" : "Left", Math.abs(right));
      sendButtonInput("Slider", (right > 0 ? "Right" : "Left") + "," + Math.abs(right));
    } else {
      sendMotorSpeed("Left", 0);
      sendMotorSpeed("Right", 0);
      sendButtonInput("Slider", "Left,0");
      sendButtonInput("Slider", "Right,0");
    }
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

  window.onload = function () {
    // DOM references
    auxSlider = document.getElementById("AUX");
    bucketSlider = document.getElementById("Bucket");
    updateSliderValue("Bucket");
    updateSliderValue("AUX");

    // Only update sliders that exist
    ["armForwardSlider", "armBackwardSlider"].forEach(id => {
      const el = document.getElementById(id);
      if (el) updateSliderValue(id);
    });

    // ✅ You can optionally remove this if arm sliders are guaranteed
    const safeAdd = (id, evt, fn) => {
      const el = document.getElementById(id);
      if (el) el.addEventListener(evt, fn);
    };

    // 🦾 Arm motor release events
    safeAdd("armForwardSlider", "mouseup", () => resetSlider(document.getElementById("armForwardSlider"), "ArmUp"));
    safeAdd("armForwardSlider", "touchend", () => resetSlider(document.getElementById("armForwardSlider"), "ArmUp"));
    safeAdd("armBackwardSlider", "mouseup", () => resetSlider(document.getElementById("armBackwardSlider"), "ArmDown"));
    safeAdd("armBackwardSlider", "touchend", () => resetSlider(document.getElementById("armBackwardSlider"), "ArmDown"));

    // 🌙 Restore dark mode
    const storedDark = localStorage.getItem("darkMode");
    if (storedDark === "1") {
      document.getElementById("darkToggle").checked = true;
      toggleDarkMode(true);
    }

    // ⚠️ Restore emergency state
    const emergencyBtn = document.getElementById("emergencyBtn");
    if (emergencyBtn?.classList.contains("active")) {
      emergencyOn = true;
      document.getElementById("leftIndicator")?.classList.add("visible", "blinking");
      document.getElementById("rightIndicator")?.classList.add("visible", "blinking");
      emergencyBtn.classList.add("blinking");
    }

    initWebSocket();

    // ⌨️ Keymap fetch
    fetch("/get_keymap")
      .then(r => r.json())
      .then(data => {
        keymap = {};
        Object.entries(data).forEach(([action, key]) => {
          keymap[key.toLowerCase()] = action;
        });
      })
      .catch(err => console.error("❌ Failed to load keymap:", err));

    // 🎹 Keyboard listeners
    document.addEventListener("keydown", handleKeyDown);
    document.addEventListener("keyup", handleKeyUp);

    // FPS placeholder
    document.getElementById("fpsOverlay").innerText = "FPS: ...";

    // 📷 Load camera IP
    const camera = document.getElementById("cameraStream");
    fetch("/get_camera_ip")
      .then(r => r.json())
      .then(data => {
        if (data.ip) {
          cameraIP = data.ip;
          camera.src = `http://${cameraIP}:81/stream`;
          document.getElementById("camStatusOverlay").style.display = "none";
          console.log("✅ Loaded saved camera IP:", cameraIP);
        } else {
          console.log("⚠️ No saved camera IP.");
        }
      })
      .catch(err => console.error("Failed to fetch saved camera IP:", err));

    // 🛣️ Lane overlay
    const laneOverlay = document.getElementById("laneOverlay");
    if (laneOverlay) {
      laneOverlay.style.display =
        localStorage.getItem("laneOverlayVisible") === "true" ? "block" : "none";
    }
  };

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

  function initWebSocket() {
    websocketCarInput = new WebSocket("ws://" + location.host + "/CarInput");
    websocketCarInput.onopen = () => console.log("WebSocket Connected");
    websocketCarInput.onclose = () => setTimeout(initWebSocket, 2000);
    websocketCarInput.onmessage = function(event) {
      var message = event.data;
      var parts = message.split(',');
      var key = parts[0];
      var value = parts[1];

      const msg = event.data;

      // Handle IMU data
      if (msg.startsWith("IMU,")) {
        //console.log("📨 Raw IMU data received:", msg);  // <--- Add this line for debug
        const parts = msg.split(",");
        if (parts.length >= 8) {
          const [_, h, r, p, mx, my, mz, temp] = parts;

          if (!imuScriptLoaded) {
            const script = document.createElement("script");
            script.src = "/telemetryScript.js";
            script.onload = () => {
              imuScriptLoaded = true;
              console.log("✅ IMU telemetry script loaded");
              handleIMUMessage(h, r, p, mx, my, mz, temp);  // now safe to call
            };
            document.body.appendChild(script);
          } else {
            handleIMUMessage(h, r, p, mx, my, mz, temp);
          }
        }
        return;
      }


      if (key === "Light") {
        ledOn = (value == "1");
        const btn = document.getElementById("ledToggleBtn");
        if (ledOn) {
          btn.innerText = "💡 LED: ON";
          btn.style.backgroundColor = "#ffd700";
          btn.style.color = "black";
        } else {
          btn.innerText = "💡 LED: OFF";
          btn.style.backgroundColor = "#444"; // more visible on dark theme
          btn.style.color = "white";
        }

      }

      if (key === "CAMIP") {
          cameraIP = value;
          console.log("✅ Camera IP set to:", cameraIP);
          document.getElementById("statusOverlay").innerText =
              value ? `📡 Camera found: ${value}` : "❌ Camera not found";


        if (value) {
            cameraOnline = true;
            lastCamUpdateTime = Date.now();
            document.getElementById("cameraStream").src = "http://" + value + ":81/stream";
        }

      }

      if (key === "GPIOCONF") {
        const pairs = value.split(";");
        pairs.forEach(p => {
          const [k, v] = p.split(":");
          if (k === "Left") document.getElementById("gpioLeft").value = v;
          if (k === "Right") document.getElementById("gpioRight").value = v;
          if (k === "Servo") document.getElementById("gpioServo").value = v;
        });
      }

      if (key == "HoldBucket") document.getElementById('holdBucketSwitch').checked = (value == "1");
      if (key == "HoldAux") document.getElementById('holdAuxSwitch').checked = (value == "1");
      if (key == "Switch") document.getElementById('powerSwitch').checked = (value == "1");
      if (key === "FPS") {
        document.getElementById('fpsOverlay').innerText = "FPS: " + value;
        lastFPSUpdate = Date.now();  // ✅ use this for camera activity tracking
      }
      if (key === "AUX") {
        auxSlider.value = parseInt(value);
        updateSliderValue("AUX");
      }

      if (key === "Bucket") {
        bucketSlider.value = parseInt(value);
        updateSliderValue("Bucket");
      }


      if (key === "BATT") {
        let batteryPercent = parseInt(parts[1]);
        let voltage = parseFloat(parts[2]);
        let wifiQuality = parseInt(parts[3]);

        let batteryText = document.getElementById('batteryText');
        let wifiText = document.getElementById('wifiText');
        let chargeIcon = document.getElementById('chargeIcon');

        // Update battery
        batteryText.innerText = "Batt: " + batteryPercent + "% (" + voltage.toFixed(2) + "V)";
        batteryText.className = "";
        if (batteryPercent > 70) batteryText.classList.add('batt-green');
        else if (batteryPercent > 40) batteryText.classList.add('batt-orange');
        else if (batteryPercent > 20) batteryText.classList.add('batt-red');
        else batteryText.classList.add('batt-critical');

        // Update Wi-Fi
        wifiText.innerText = "WiFi: " + wifiQuality + "%";
        wifiText.className = "";
        if (wifiQuality > 70) wifiText.classList.add('wifi-green');
        else if (wifiQuality > 40) wifiText.classList.add('wifi-orange');
        else wifiText.classList.add('wifi-red');

        // Charger logic is handled in CHARGE
      }

      if (key === "CHARGE") {
        const chargeIcon = document.getElementById("chargeIcon");

        if (value === "YES") {
          chargeIcon.style.display = "inline";
          chargeIcon.innerText = "⚡";

          let voltage = parseFloat(document.getElementById("batteryText").innerText.match(/\(([\d.]+)V\)/)[1]);

          if (voltage >= 8.4) {
            chargeIcon.style.animation = "none";
            chargeIcon.style.color = "lime";
          } else {
            chargeIcon.style.animation = "fadeCharge 1.5s infinite";
            let percent = Math.min(100, Math.max(0, parseInt(document.getElementById("batteryText").innerText.match(/Batt: (\d+)%/)[1])));
            let red = Math.round(255 - percent * 2.55);
            let green = Math.round(percent * 2.55);
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
      }




      if (key == "STATS") {
        let uptimeSecs = parseInt(parts[1]);
        let chipTemp = parseFloat(parts[2]);
        let uptimeMins = Math.floor(uptimeSecs / 60);
        let stats = document.getElementById("statsOverlay");

        stats.innerText = "Uptime: " + uptimeMins + " min, Temp: " + chipTemp + "C";
        stats.className = "overlay-topright"; // reset base style

        if (chipTemp >= 70) {
          stats.classList.add("temp-critical");
        } else if (chipTemp >= 55) {
          stats.classList.add("temp-warning");
        }
      }


      if (key == "DarkMode") {
        const isDark = (value == "1");
        document.getElementById('darkToggle').checked = isDark;
        toggleDarkMode(isDark);
      }

      const normalizedKey = key.toUpperCase();

      if (normalizedKey === "TURN_LEFT") {
        const el = document.getElementById("leftIndicator");

        if (value === "1") {
          if (!emergencyOn) {
            el.classList.add("blinking", "visible");
          }
        } else {
          el.classList.remove("blinking");
          el.classList.remove("visible");
        }
      }


      if (normalizedKey === "TURN_RIGHT") {
        const el = document.getElementById("rightIndicator");

        if (value === "1") {
          if (!emergencyOn) {
            el.classList.add("blinking", "visible");
          }
        } else {
          el.classList.remove("blinking");
          el.classList.remove("visible");
        }
      }




      if (key === "Beacon") {
        beaconActive = (value === "1");
        const btn = document.getElementById("beaconBtn");
        if (beaconActive) {
          btn.classList.add("blinking");
        } else {
          btn.classList.remove("blinking");
        }
      }

      if (normalizedKey === "EMERGENCY") {
        emergencyOn = (value === "1");

        const left = document.getElementById("leftIndicator");
        const right = document.getElementById("rightIndicator");

        if (emergencyOn) {
          left.classList.add("blinking", "visible");
          right.classList.add("blinking", "visible");
        } else {
          left.classList.remove("blinking", "visible");
          right.classList.remove("blinking", "visible");
        }

        const btn = document.getElementById("emergencyBtn");
        if (btn) {
          if (emergencyOn) {
            btn.classList.add("blinking");
            btn.classList.add("active");
            btn.innerText = "⚠️ Emergency ON";
          } else {
            btn.classList.remove("blinking");
            btn.classList.remove("active");
            btn.innerText = "⚠️ Emergency";
          }
        }
      }


    };
  }

  function sendButtonInput(key, value) {
    if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
      websocketCarInput.send(key + "," + value);
    }
  }

  function handleKeyDown(e) {
    const key = e.key.toLowerCase();
    const action = keymap[key];
    if (!action) return;

    switch (action) {
      case "forward":
        sendButtonInput("MoveCar", "1");
        setJoystickKnob(0, 255);
        break;
      case "backward":
        sendButtonInput("MoveCar", "2");
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
    }
  }

  function handleKeyUp(e) {
    const key = e.key.toLowerCase();
    const action = keymap[key];

    if (["forward", "backward", "left", "right"].includes(action)) {
      sendButtonInput("MoveCar", "0");
      setJoystickKnob(0, 0);
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
      if (lastSentMotorValues[direction] === value && value !== 0) return; // allow redundant zeroes

    lastSentMotorValues[direction] = value;

    if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
      websocketCarInput.send("Motor," + direction + "," + value);

      // Optional: only reset arm sliders if they exist (prevents error)
      if (direction === "ArmUp") {
        const downSlider = document.getElementById("armBackwardSlider");
        if (downSlider) downSlider.value = 0;
      } else if (direction === "ArmDown") {
        const upSlider = document.getElementById("armForwardSlider");
        if (upSlider) upSlider.value = 0;
      }
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

  function toggleDarkMode(isDark) {
    document.body.classList.toggle("dark", isDark);
    document.querySelector(".left").classList.toggle("dark", isDark);
    document.querySelector(".right").classList.toggle("dark", isDark);
    document.querySelector(".header").classList.toggle("dark", isDark);
    localStorage.setItem("darkMode", isDark ? "1" : "0");
    if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
      websocketCarInput.send("DarkMode," + (isDark ? 1 : 0));
    }
  }

  function showToast(message, isError = false) {
    const toast = document.getElementById("toast");
    toast.innerText = message;
    toast.style.backgroundColor = isError ? "#e53935" : "#4CAF50";
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
    if (!cameraIP) {
      showToast("❌ Camera IP not set", true);
      return;
    }
    fetch(`http://${cameraIP}/led?brightness=${value}`)
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

    requestAnimationFrame(updateFrameShape);

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
    const modal = document.getElementById('settingsModal');
    const mainContent = document.getElementById('mainContent');
    const isVisible = modal.style.display === 'block';

    if (isVisible) {
      modal.style.display = 'none';
      mainContent.classList.remove('blur');

      // 🔄 Unload modal script
      if (typeof onunloadModal === "function") onunloadModal();  // optional cleanup
      const oldScript = document.getElementById("modalScript");
      if (oldScript) {
        document.body.removeChild(oldScript);
        window.modalScriptLoaded = false;
        console.log("❌ Modal script unloaded.");
      }

    } else {

      modal.style.display = 'block';
      mainContent.classList.add('blur');

      // 🔄 Load modal script only if not yet loaded
      if (!window.modalScriptLoaded) {
        const script = document.createElement('script');
        script.src = "/modalScript.js";
        script.id = "modalScript";
        script.onload = () => {
          window.modalScriptLoaded = true;
          console.log("✅ Modal script loaded dynamically.");
          openCamSettings();           // ✅ Safe to call now
          showTab('camTab');          
          requestAnimationFrame(updateFrameShape);
        };
        document.body.appendChild(script);
      } else {
        openCamSettings();            // ✅ Safe because already loaded
        showTab('camTab');
        requestAnimationFrame(updateFrameShape);
      }


    }
  }

  function toggleDrawMode() {
    // First-time load check
    if (!window.drawScriptLoaded) {
      const script = document.createElement("script");
      script.src = "/drawScript.js";
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

)HTMLSCRIPT";
