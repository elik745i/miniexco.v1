// modalScript.h
#pragma once

const char* modalScript = R"rawliteral(

  //if (window.modalScriptLoaded) return;
  //window.modalScriptLoaded = true;

  function onunloadModal() {
    console.log("🧹 Cleaning up modal...");

    // Remove resize listener (re-adds itself every time modal loads)
    window.removeEventListener('resize', modalResizeHandler);

    // Optionally clear DOM edits if needed (e.g., clear network containers)
    const container = document.getElementById('savedNetworksContainer');
    if (container) container.innerHTML = '<p>Loading saved networks...</p>';
  }

  // Save a named reference to remove easily
  function modalResizeHandler() {
    const modal = document.getElementById('settingsModal');
    if (modal && modal.offsetParent !== null) {
      updateFrameShape();
    }
  }

  // Register it on script load
  window.addEventListener('resize', modalResizeHandler);


  function scanNetworks() {
      let ssidSelect = document.getElementById('ssidList');

      // Always clear existing scanning text and dropdown
      let existingText = document.getElementById('scanningText');
      if (existingText) existingText.remove();

      ssidSelect.style.display = 'none';
      ssidSelect.innerHTML = '';  // clear old entries

      let scanningText = document.createElement('div');
      scanningText.id = 'scanningText';
      scanningText.innerText = "Scanning for Networks...";
      ssidSelect.parentNode.insertBefore(scanningText, ssidSelect);

      fetch('/listwifi')
        .then(response => response.json())
        .then(data => {
          let existingText = document.getElementById('scanningText');
          if (existingText) existingText.remove();

          if (data.length === 0) {
            let retryText = document.createElement('div');
            retryText.id = 'scanningText';
            retryText.innerText = "No networks found, retrying...";
            ssidSelect.parentNode.insertBefore(retryText, ssidSelect);

            setTimeout(scanNetworks, 3000); // retry after 3 seconds
            return;
          }

          data.forEach(function(network) {
            let option = document.createElement('option');
            option.value = network.ssid;
            option.innerText = network.ssid + " (" + network.rssi + "dBm)";
            ssidSelect.appendChild(option);
          });

          ssidSelect.style.display = 'block';
        })
        .catch(error => {
          console.error("Error fetching Wi-Fi list:", error);
          let existingText = document.getElementById('scanningText');
          if (existingText) existingText.innerText = "Error scanning Wi-Fi, retrying...";
          setTimeout(scanNetworks, 3000);
        });
  }

 function loadSavedNetworks() {
    fetch('/list_saved_wifi')
      .then(response => response.json())
      .then(data => {
        const container = document.getElementById('savedNetworksContainer');
        container.innerHTML = '';
        data.forEach(net => {
          const div = document.createElement('div');
          div.style.marginBottom = '10px';

          const nameSpan = document.createElement('span');
          nameSpan.innerText = net.ssid;
          div.appendChild(nameSpan);

          const connectBtn = document.createElement('button');
          connectBtn.innerText = 'Connect';
          connectBtn.style.marginLeft = '10px';
          connectBtn.onclick = () => connectToSavedNetwork(net.ssid);
          div.appendChild(connectBtn);

          const editBtn = document.createElement('button');
          editBtn.innerText = 'Edit';
          editBtn.style.marginLeft = '5px';
          editBtn.onclick = () => {
            if (editBtn.innerText === 'Edit') {
              const wrapper = document.createElement('span');
              wrapper.style.marginLeft = '5px';

              const input = document.createElement('input');
              input.type = 'password';
              input.placeholder = 'Password';
              input.value = net.password || '';
              input.style.marginRight = '5px';
              wrapper.appendChild(input);

              const toggle = document.createElement('span');
              toggle.innerText = '👁️';
              toggle.style.cursor = 'pointer';
              toggle.onclick = () => {
                input.type = input.type === 'password' ? 'text' : 'password';
              };
              wrapper.appendChild(toggle);

              const retryInput = document.createElement('input');
              retryInput.title = "Wi-Fi retry attempts";
              retryInput.type = 'number';
              retryInput.min = 1;
              retryInput.max = 10;
              retryInput.value = net.retry || 3;
              retryInput.style.width = '40px';
              retryInput.style.marginLeft = '10px';
              wrapper.appendChild(retryInput);

              div.appendChild(wrapper);

              editBtn.innerText = 'Save';
              editBtn.inputRef = input;
              editBtn.retryRef = retryInput;

            } else {
              const newPass = editBtn.inputRef.value;
              const newRetry = Math.min(10, Math.max(1, parseInt(editBtn.retryRef.value)));

              updateSavedPassword(net.ssid, newPass);
              updateRetryCount(net.ssid, newRetry);

              editBtn.innerText = 'Edit';
              editBtn.inputRef.parentElement.remove(); // remove wrapper
            }
          };

          div.appendChild(editBtn);

          container.appendChild(div);
        });
      })
      .catch(err => {
        console.error('Failed to load saved networks:', err);
        document.getElementById('savedNetworksContainer').innerText = 'Error loading saved networks.';
      });
 }

  function updateRetryCount(ssid, retries) {
    fetch(`/update_retry_count?ssid=${encodeURIComponent(ssid)}&count=${retries}`)
      .then(response => {
        if (response.ok) {
          showToast(`🔁 Retry count set to ${retries} for ${ssid}`);
        } else {
          showToast(`⚠️ Failed to set retry count for ${ssid}`, true);
        }
      })
      .catch(err => {
        console.error('Update retry count error:', err);
        showToast(`❌ Error updating retry count for ${ssid}`, true);
      });
  }

  function applyCamSettings() {
    if (!cameraIP) {
      showToast("❌ Camera IP not set", true);
      return;
    }

    let params = new URLSearchParams();

    const res = document.getElementById("camResolution").value;
    const fps = document.getElementById("camFPS").value;
    const rotate = document.getElementById("camRotate").value;
    const sat = document.getElementById("camSaturation").value;
    const gray = document.getElementById("camGrayscale").checked ? 1 : 0;
    const ledBrightness = document.getElementById("ledBrightness").value;

    const bright = document.getElementById("camBrightness").value;
    const contrast = document.getElementById("camContrast").value;
    const sharp = document.getElementById("camSharpness").value;
    const denoise = document.getElementById("camDenoise").value;

    const gamma = document.getElementById("camGamma").value;
    const compression = document.getElementById("camCompression").value;
    const quality = document.getElementById("camQuality").value;

    params.append("gamma", gamma);
    params.append("compression", compression);

    params.append("res", res);
    params.append("fps", fps);
    params.append("rot", rotate);
    params.append("sat", sat);
    params.append("gray", gray);
    params.append("bright", bright);
    params.append("contrast", contrast);
    params.append("sharp", sharp);
    params.append("denoise", denoise);
    params.append("quality", quality);

    const camURL = `http://${cameraIP}/control?`;

    fetch(camURL + params.toString())
      .then(r => {
        if (r.ok) {
          showToast("✅ Camera settings applied");
          updateLedBrightness(ledBrightness);  // 🔴 send LED brightness separately
        } else {
          showToast("⚠️ Failed to apply settings", true);
        }
      })
      .catch(err => {
        console.error(err);
        showToast("❌ Communication error", true);
      });

  } 

  function connectToSavedNetwork(ssid) {
      fetch(`/connect_saved_wifi?ssid=${encodeURIComponent(ssid)}`)
        .then(response => {
          if (response.ok) {
            showToast(`✅ Connecting to ${ssid}`);
          } else {
            showToast(`⚠️ Failed to connect to ${ssid}`, true);
          }
        })
        .catch(err => {
          console.error('Connect error:', err);
          showToast(`❌ Connection error for ${ssid}`, true);
        });
  }

  function updateSavedPassword(ssid, newPass) {
      fetch(`/update_wifi_password?ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(newPass)}`)
        .then(response => {
          if (response.ok) {
            showToast(`✅ Password updated for ${ssid}`);
          } else {
            showToast(`⚠️ Failed to update password for ${ssid}`, true);
          }
        })
        .catch(err => {
          console.error('Update password error:', err);
          showToast(`❌ Error updating password for ${ssid}`, true);
        });
  }

  window.openCamSettings = function() {
    if (!cameraIP) {
      fetch('/get_camera_ip')
        .then(r => r.json())
        .then(data => {
          cameraIP = data.ip;
          if (!cameraIP) {
            showToast("❌ Camera IP not set", true);
            return;
          }
          fetchCamSettings(cameraIP);
        })
        .catch(err => {
          console.error("Failed to fetch saved camera IP:", err);
          showToast("❌ Cannot retrieve camera IP", true);
        });
    } else {
      fetchCamSettings(cameraIP);
    }
  }

  function fetchCamSettings(ip) {
    fetch(`http://${ip}/getsettings`)
      .then(r => r.json())
      .then(data => {
        const resSelect = document.getElementById("camResolution");
        if (resSelect && data.res !== undefined) {
          resSelect.value = String(data.res); // Set the dropdown to match the value
        }
        document.getElementById("camFPS").value = data.fps || 15;
        updateSliderValue("camFPS");
        document.getElementById("camRotate").value = data.rot || 0;  // ✅ FIXED LINE
        updateSliderValue("camRotate");
        document.getElementById("camSaturation").value = data.sat || 0;
        updateSliderValue("camSaturation");
        document.getElementById("camGrayscale").checked = data.gray == 1;
        updateSliderValue("camGrayscale");
        document.getElementById("ledBrightness").value = data.led || 0;
        updateSliderValue("ledBrightness");
        document.getElementById("camBrightness").value = data.bright || 0;
        updateSliderValue("camBrightness");
        document.getElementById("camContrast").value = data.contrast || 0;
        updateSliderValue("camContrast");
        document.getElementById("camSharpness").value = data.sharp || 2;
        updateSliderValue("camSharpness");
        document.getElementById('settingsModal').style.display = 'block';
        if (document.getElementById("camGamma")) {
          document.getElementById("camGamma").value = data.gamma || 0;
          updateSliderValue("camGamma");
        }

        if (document.getElementById("camCompression")) {
          document.getElementById("camCompression").value = data.compression || 12;
          updateSliderValue("camCompression");
        }

        document.getElementById("camQuality").value = data.quality || 10;
        updateSliderValue("camQuality");

      })
      .catch(err => {
        console.error("Failed to load camera settings", err);
        showToast("❌ Cannot load current settings", true);
      });
  }

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

  window.updateFrameShape = function() {
    const modal = document.getElementById('settingsModal');
    const frame = document.getElementById('modalFrameShape');
    const activeBtn = document.querySelector('.tabButton.active');
    if (!modal || !frame || !activeBtn) return;

    const modalRect = modal.getBoundingClientRect();
    const btnRect = activeBtn.getBoundingClientRect();

    const offset = -10;      // 🔸 offset from left, right, bottom
    const topOffset = -50;   // 🔸 separate top offset
    const cut = 12;
    const notchHeight = 14;
    const notchPad = 6;

    const left = btnRect.left - modalRect.left - offset;
    const right = btnRect.right - modalRect.left - offset;

    const clip = `polygon(
      ${offset}px ${topOffset + cut}px,
      ${offset + cut}px ${topOffset}px,
      ${left - notchPad}px ${topOffset}px,
      ${left}px ${topOffset + notchHeight}px,
      ${right}px ${topOffset + notchHeight}px,
      ${right + notchPad}px ${topOffset}px,
      calc(100% - ${offset + cut}px) ${topOffset}px,
      calc(100% - ${offset}px) ${topOffset + cut}px,
      calc(100% - ${offset}px) calc(100% - ${cut}px),
      calc(100% - ${offset + cut}px) calc(100% - ${offset}px),
      ${offset + cut}px calc(100% - ${offset}px),
      ${offset}px calc(100% - ${offset + cut}px)
    )`;

    frame.style.clipPath = clip;
  }

  window.addEventListener('resize', () => {
    const modal = document.getElementById('settingsModal');
    if (modal && modal.offsetParent !== null) {
      updateFrameShape();
    }

  });

window.uploadFirmware = function() {
  const fileInput = document.getElementById("otaFile");
  const status = document.getElementById("otaStatus");
  const progress = document.getElementById("otaProgress");
  const file = fileInput.files[0];

  if (!file) {
    status.innerText = "Please select a file.";
    return;
  }

  const xhr = new XMLHttpRequest();
  xhr.open("POST", "/ota/upload", true);  // ✅ FIXED

  xhr.upload.onprogress = function (e) {
    if (e.lengthComputable) {
      progress.style.display = "block";
      progress.value = (e.loaded / e.total) * 100;
    }
  };

  xhr.onreadystatechange = function () {
    if (xhr.readyState === 4) {
      if (xhr.status === 200) {
        status.innerText = "✅ Upload successful. Rebooting...";
        setTimeout(() => location.reload(), 5000);
      } else if (xhr.status === 0) {
        // ESP32 likely rebooted too fast before HTTP finished
        status.innerText = "✅ Firmware uploaded. Waiting for reboot...";
        setTimeout(() => location.reload(), 6000);  // Give it a bit more time
      } else {
        status.innerText = "❌ Upload failed.";
      }
    }
  };


  const formData = new FormData();
  formData.append("update", file);
  xhr.send(formData);
}

let controlScriptLoaded = false;

function loadControlScript(callback) {
  if (controlScriptLoaded) {
    if (typeof callback === "function") callback();
    return;
  }

  const script = document.createElement("script");
  script.src = "/controlScript.js";
  script.onload = () => {
    controlScriptLoaded = true;
    console.log("✅ Control script loaded dynamically.");
    if (typeof callback === "function") callback();
  };
  document.body.appendChild(script);
}

//Telemetry tab----------------------------------------------------------------

function updateCalibrationStatus(sys, gyro, accel, mag) {
  document.getElementById("sysCal").textContent = sys;
  document.getElementById("gyroCal").textContent = gyro;
  document.getElementById("accelCal").textContent = accel;
  document.getElementById("magCal").textContent = mag;
}

function triggerCalibration() {
  fetch("/calibrate_imu", { method: "POST" })
    .then(res => res.text())
    .then(msg => {
      document.getElementById("calibStatus").textContent = msg;
    });
}

function loadCalibrationData() {
  fetch("/get_calibration")
    .then(res => res.json())
    .then(data => {
      updateCalibrationStatus(data.sys, data.gyro, data.accel, data.mag);
      document.getElementById("calibBtn").textContent = data.stored ? "Recalibrate" : "Calibrate";
    });
}

//show tab-----------------------------------------------------------------------------

window.showTab = function(tabId, button) {
  const tabs = document.querySelectorAll(".tabContent");
  const buttons = document.querySelectorAll(".tabButton");

  // Hide all tabs and remove active class
  tabs.forEach(tab => {
    tab.style.display = "none";
    tab.classList.remove("active");
  });
  buttons.forEach(btn => btn.classList.remove("active"));

  // Show selected tab and apply active class
  const selectedTab = document.getElementById(tabId);
  if (selectedTab) {
    selectedTab.style.display = "block";
    selectedTab.classList.add("active");
  }
  if (button) button.classList.add("active");

  // Special case for Controls tab to load key mappings
  if (tabId === "keysTab") {
    loadControlScript(() => {
      if (typeof loadKeyMappings === "function") loadKeyMappings();
    });
  }

  if (tabId === "wifiTab") {
    if (typeof loadSavedNetworks === "function") loadSavedNetworks();
  }

  if (tabId === "telemetryTab") {
    if (websocketCarInput && websocketCarInput.readyState === WebSocket.OPEN) {
      websocketCarInput.send("IMU,REQUEST_CALIB");
    }
  }


  if (typeof updateFrameShape === "function") updateFrameShape();
};



)rawliteral";
