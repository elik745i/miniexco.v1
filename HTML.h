#pragma once

const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>MiniExco</title>
<style>%COMMON_STYLE%</style>

</head>
<body class="noselect">
<div id="mainContent">
  <div class="header">MINIEXCO %MAINFW% | CAM %CAMFW%</div>


  <div class="container">

    <div class="left" align="center">

      <!-- MOVEMENT CONTROLS -->
      <h3>2D Joystick Control</h3>
      <div id="joystickContainer">
        <div id="joystickKnob"></div>
      </div>


      <h3>Arm Motor Control</h3>
      <div class="slidecontainer">
        <label for="armForwardSlider">Arm Up: <span id="armForwardSliderValue">0</span></label>
        <input type="range" min="0" max="255" value="0" class="slider" id="armForwardSlider" oninput='sendMotorSpeed("ArmUp", this.value); updateSliderValue("armForwardSlider")'>
      </div>
      <div class="slidecontainer">
        <label for="armBackwardSlider">Arm Down: <span id="armBackwardSliderValue">0</span></label>
        <input type="range" min="0" max="255" value="0" class="slider" id="armBackwardSlider" oninput='sendMotorSpeed("ArmDown", this.value); updateSliderValue("armBackwardSlider")'>
      </div>

      
      <h3>Bucket & AUX Control</h3>
      <div class="slidecontainer">
        <label for="Bucket">Bucket:<span id="BucketValue">140</span></label>
        <input type="range" min="0" max="180" value="140" class="slider" id="Bucket" oninput='sendButtonInput("Bucket", this.value); updateSliderValue("Bucket")'>
      </div>
      <div class="slidecontainer">
        <label for="AUX">AUX Arm: <span id="AUXValue">150</span></label>
        <input type="range" min="0" max="180" value="150" class="slider" id="AUX" oninput='sendButtonInput("AUX", this.value); updateSliderValue("AUX")'>    
      </div>


      <hr style="margin:30px 0;">
      <h3>ESP32-CAM Control</h3>
      <div style="margin-top: 20px;">
        <button id="ledToggleBtn" onclick="toggleLed()" class="ctrlBtn">
          💡 LED: OFF
        </button>

        <button onclick="toggleSettingsModal()" class="ctrlBtn">
          ⚙️ Settings
        </button>

        <button id="beaconBtn" onclick="toggleBeacon()" class="ctrlBtn beaconStyle">
          🚨 Beacon
        </button>

        <button id="emergencyBtn" onclick="toggleEmergency()" class="ctrlBtn emergencyStyle">
          ⚠️ Emergency
        </button>
      </div>


    </div>

    <div class="right">
      <img id="cameraStream" src="http://%CAMERAIP%:81/stream">
      <canvas id="drawingCanvas" style="position:absolute; top:0; left:0; z-index:15; pointer-events:none;"></canvas>
      <svg id="drawingOverlay" style="position:absolute; top:0; left:0; width:100%; height:100%; z-index:16; pointer-events:none;"></svg>

      <div id="camStatusOverlay" style="top: 40px; display:none; position:absolute; color:white; z-index:10;">
        <div class="spinner"></div>
        <div style="margin-top: 8px;">📷 Camera Unavailable</div>
      </div>

      <div class="overlay" id="fpsOverlay">FPS: 0</div>
      <div class="overlay-topright" id="batteryOverlay">
        <span id="batteryText">Batt: 0% (0.0V)</span>
        <span id="chargeIcon" style="display:none;">⚡</span>
        <span id="wifiText">WiFi: 0%</span>
      </div>
      <div class="overlay-topright" id="statsOverlay" style="top: 60px;">Uptime: 0 min, Temp: 0.0°C</div>
      <div id="statusOverlay" style="position:absolute; top:5px; right:5px; color:white; background:black; padding:5px; font-size:14px;"></div>

      <div id="pathControlButtons" style="position:absolute; top:0; left:0; width:100%; height:100%; z-index:999; pointer-events: none;"></div>


      <div id="drawPathBtn" 
          class="overlay-bottomright" 
          style="bottom: 150px; right: 10px;"
          onclick="toggleDrawMode()">
        ✏️ Draw Path
      </div>

      <div id="wipePathBtn" 
           class="overlay-bottomright" 
           style="bottom: 100px; right: 10px;" 
           onclick="clearDrawing()">
        🧹 Wipe
      </div>

      <div class="overlay-bottomright" 
          style="bottom: 60px; right: 10px;"
          onclick="toggleLaneOverlay()">
        🛣️ Toggle Lane
      </div>

      <div id="chargingOverlay">
        <div id="chargingFill"></div>
      </div>
      <!-- Turn Signal Overlays -->
      <div id="leftIndicator" class="turn-indicator left">&#x25C0;</div>
      <div id="rightIndicator" class="turn-indicator right">&#x25B6;</div>

      <div id="laneOverlay">
        <div class="lane left-lane"></div>
        <div class="lane right-lane"></div>
      </div>

      <!-- IMU Overlay (bottom-left corner) -->
      <div id="imuOverlay" style="position:absolute; bottom:10px; left:10px; z-index:20; color:white; font-family:sans-serif; pointer-events:none;">
        <div style="display:flex; gap:10px;">
          <div style="text-align:center;">
            <canvas id="tiltSphere" width="80" height="80" style="display:block;"></canvas>
            <div style="font-size:14px;">
              <div id="rollText">Roll: ---°</div>
              <div id="pitchText">Pitch: ---°</div>
            </div>
          </div>

          <div style="text-align:center;">
            <canvas id="compassCanvas" width="80" height="80" style="display:block;"></canvas>
            <div id="headingText" style="font-size:14px;">Heading: ---°</div>
          </div>

          <div style="text-align:center;">
            <canvas id="tempCanvas" width="30" height="80" style="display:block;"></canvas>
            <div id="imuTemp" style="font-size:14px;">Temp: --°C</div>
          </div>
        </div>

        <div id="magText" style="margin-top:4px; font-size:13px;">Mag: [--, --, --]</div>
      </div>


    </div>

  </div>

  <div id="toast" style="visibility:hidden; min-width:250px; margin-left:-125px;
    background-color:#333; color:#fff; text-align:center; border-radius:2px;
    padding:16px; position:fixed; z-index:9999; left:50%; bottom:30px;
    font-size:17px; opacity:0; transition:opacity 0.5s ease-in-out;">
  </div>
</div>
<script>%COMMON_SCRIPT%</script>

<!-- Settings modal placeholder -->
%MODAL%
%CONTROL_SCRIPT%
</body>
</html>
)HTMLHOMEPAGE";