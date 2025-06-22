#pragma once

const char* htmlSettingsModal PROGMEM = R"SETMODAL(
<!-- Unified Settings Modal -->
<div id="settingsModal" style="display:none;">
 <div id="modalFrameShape"></div>

  <div style="position: relative; z-index: 1002;">
    <div style="display: flex; justify-content: space-between; align-items: center;">
      <h2 style="margin: 0;">⚙️ Settings</h2>
      <button onclick="toggleSettingsModal()"
              style="background: none; border: none; color: white; font-size: 24px; cursor: pointer;">✖</button>

    </div>

    <div style="display:flex; justify-content:space-around; margin-bottom:10px;">
      <button class="tabButton" onclick="showTab('keysTab', this)">Controls</button>
      <button class="tabButton" onclick="showTab('camTab', this)">Camera</button>
      <button class="tabButton" onclick="showTab('robotTab', this)">Config</button>
      <button class="tabButton" onclick="showTab('telemetryTab', this)">Telemetry</button>
      <button class="tabButton" onclick="showTab('wifiTab', this)">Wi-Fi</button>
      <button class="tabButton" onclick="showTab('otaTab', this)">OTA</button>
    </div>

    <div id="tabContentWrapper" class="tabContentWrapper">

<!-- KEY CONFIG TAB -->
      <div id="keysTab" class="tabContent" style="display:none;">
        <h3>Keyboard Mapping</h3>
        <p>Edit keyboard bindings below (no duplicates allowed):</p>
        <div id="keyMappingInputs" style="line-height: 1.8;">
          <div><label>Forward:</label> <select data-action="forward"></select> <span class="dupWarn"></span></div>
          <div><label>Backward:</label> <select data-action="backward"></select> <span class="dupWarn"></span></div>
          <div><label>Left:</label> <select data-action="left"></select> <span class="dupWarn"></span></div>
          <div><label>Right:</label> <select data-action="right"></select> <span class="dupWarn"></span></div>
          <div><label>Stop (Space):</label> <select data-action="stop"></select> <span class="dupWarn"></span></div>

          <div><label>Arm Up:</label> <select data-action="armUp"></select> <span class="dupWarn"></span></div>
          <div><label>Arm Down:</label> <select data-action="armDown"></select> <span class="dupWarn"></span></div>

          <div><label>Bucket Up:</label> <select data-action="bucketUp"></select> <span class="dupWarn"></span></div>
          <div><label>Bucket Down:</label> <select data-action="bucketDown"></select> <span class="dupWarn"></span></div>
          <div><label>AUX Up:</label> <select data-action="auxUp"></select> <span class="dupWarn"></span></div>
          <div><label>AUX Down:</label> <select data-action="auxDown"></select> <span class="dupWarn"></span></div>
          <div><label>LED Toggle:</label> <select data-action="led"></select> <span class="dupWarn"></span></div>
          <div><label>Beacon Toggle:</label> <select data-action="beacon"></select> <span class="dupWarn"></span></div>
          <div><label>Emergency Toggle:</label> <select data-action="emergency"></select> <span class="dupWarn"></span></div>
        </div>
        <hr style="margin-top:20px;">
        <button onclick="saveKeyMappings()">Save Mappings</button>
        <button onclick="resetToDefaultKeymap()">Reset to Defaults</button>
        <div id="keySaveStatus" style="margin-top:10px; font-size:90%; color:lightgreen;"></div>      
      </div>

<!-- CAMERA CONFIG TAB -->
      <div id="camTab" class="tabContent active">
        <h3>Camera Settings</h3>
        <label for="camResolution">Resolution:</label>
        <select id="camResolution">
          <option value="0">96x96</option>
          <option value="1">QQVGA (160x120)</option>
          <option value="2">QCIF (176x144)</option>
          <option value="3">QVGA (320x240)</option>
          <option value="4">CIF (352x288)</option>
          <option value="5">VGA (640x480)</option>
          <option value="6">SVGA (800x600)</option>
          <option value="7">XGA (1024x768)</option>
          <option value="8">HD (1280x720)</option>
          <option value="9">SXGA (1280x1024)</option>
          <option value="10">UXGA (1600x1200)</option>
          <option value="11">FHD (1920x1080)</option>
          <option value="12">P_HD (720x1280)</option>
          <option value="13">P_3MP (864x1536)</option>
          <option value="14">QXGA (2048x1536)</option>
          <option value="15">QHD (2560x1440)</option>
          <option value="16">WQXGA (2560x1600)</option>
          <option value="17">P_FHD (1080x1920)</option>
          <option value="18">QSXGA (2560x1920)</option>
        </select><br><br>
        <label for="camFPS">Framerate:</label><input type="number" id="camFPS" value="15" min="1" max="60"><br><br>
        <label for="camRotate">Rotate:</label>
        <select id="camRotate">
          <option value="0">Normal</option>
          <option value="1">H Mirror</option>
          <option value="2">V Flip</option>
          <option value="3">Mirror + Flip</option>
        </select><br><br>

        <label for="camSaturation">Saturation: </label>
        <input type="range" min="-2" max="2" value="0" id="camSaturation" oninput="updateSliderValue('camSaturation')">
        <span id="camSaturationValue">0</span><br><br>

        <label for="ledBrightness">LED Brightness:</label>
        <input type="range" min="0" max="255" value="0" id="ledBrightness" oninput="updateSliderValue('ledBrightness')">
        <span id="ledBrightnessValue">0</span><br><br>

        <label for="camGrayscale">Grayscale:</label><input type="checkbox" id="camGrayscale"><br><br>

        <label for="camBrightness">Brightness:</label>
        <input type="range" min="-2" max="2" value="0" id="camBrightness" oninput="updateSliderValue('camBrightness')">
        <span id="camBrightnessValue">0</span><br><br>

        <label for="camContrast">Contrast:</label>
        <input type="range" min="-2" max="2" value="0" id="camContrast" oninput="updateSliderValue('camContrast')">
        <span id="camContrastValue">0</span><br><br>

        <label for="camGamma">Gamma:</label>
        <input type="range" min="0" max="3" value="0" id="camGamma" oninput="updateSliderValue('camGamma')">
        <span id="camGammaValue">0</span><br><br>

        <label for="camCompression">Compression:</label>
        <input type="range" min="0" max="63" value="12" id="camCompression" oninput="updateSliderValue('camCompression')">
        <span id="camCompressionValue">12</span><br><br>

        <label for="camSharpness">Sharpness:</label>
        <input type="range" min="0" max="3" value="2" id="camSharpness" oninput="updateSliderValue('camSharpness')">
        <span id="camSharpnessValue">2</span><br><br>

        <label for="camDenoise">Denoise:</label>
        <select id="camDenoise">
          <option value="0">Off</option>
          <option value="1" selected>On</option>
        </select><br><br>
        
        <label for="camQuality">JPEG Quality:</label>
        <input type="range" min="10" max="63" value="10" id="camQuality" oninput="updateSliderValue('camQuality')">
        <span id="camQualityValue">10</span>
        <small style="display:block; margin-bottom:10px;">10 = Best, 63 = Worst</small><br><br>


        <button onclick="applyCamSettings()">Apply</button>
      </div>
    </div>

<!-- TELEMETRY TAB -->
    <div id="telemetryTab" class="tabContent" style="display:none;">
      <h3>BNO055 Telemetry & Calibration</h3>
      <p>This shows current calibration status of the IMU. Values range from 0 (uncalibrated) to 3 (fully calibrated).</p>

      <table style="border-collapse: collapse; width: 100%; margin-top: 10px;">
        <tr><th>System</th><th>Gyro</th><th>Accel</th><th>Magnet</th></tr>
        <tr id="calibrationRow">
          <td id="sysCal">-</td>
          <td id="gyroCal">-</td>
          <td id="accelCal">-</td>
          <td id="magCal">-</td>
        </tr>
      </table>

      <br>
      <button id="calibBtn" onclick="triggerCalibration()">Calibrate</button>
      <div id="calibStatus" style="margin-top: 10px; font-size: 90%; color: lightgreen;"></div>
    </div>

<!-- WIFI TAB -->
    <div id="wifiTab" class="tabContent" style="display:none;">
      <h3>Wi-Fi Setup</h3>
      <div style="display: flex;">
        <!-- Left: Scanning and connect -->
        <div style="flex: 1; padding-right: 10px; border-right: 1px solid #555;">
          <form method="POST" action="/savewifi">
            <label for="ssidList">Select Wi-Fi:</label><br>
            <select name="ssid" id="ssidList"></select><br><br>
            <label for="password">Password:</label><br>
            <input type="password" name="password" id="password"><br><br>
            <input type="submit" value="Connect">
          </form>
          <button onclick="scanNetworks()">Scan Wi-Fi Networks</button>
        </div>

        <!-- Right: Saved networks -->
        <div style="flex: 1; padding-left: 10px;">
          <h4>Saved Networks</h4>
          <div id="savedNetworksContainer">
            <p>Loading saved networks...</p>
          </div>
        </div>
      </div>
    </div>

<!-- OTA TAB -->
    <div id="otaTab" class="tabContent" style="display:none;">
      <h3>OTA Update</h3>
      <form id="otaUploadForm" action="javascript:void(0);">
        <input type="file" id="otaFile" required><br><br>
        <button type="button" onclick="uploadFirmware()">Upload Firmware</button>
      </form>
      <progress id="otaProgress" value="0" max="100" style="width:100%; display:none;"></progress>
      <div id="otaStatus"></div>
    </div>

<!-- CONFIG TAB -->
    <div id="robotTab" class="tabContent" style="display:none;">
      <h3>Robot/Interface Config</h3>

      <label for="darkToggle">🌙 Dark Mode</label>
      <input type="checkbox" id="darkToggle" onchange='toggleDarkMode(this.checked)'><br><br>

      <label for="powerSwitch" style="font-size:20px;">HorizontalScreen:</label>
      <input type="checkbox" id="powerSwitch" onchange='sendButtonInput("Switch", this.checked ? 1 : 0)'><br><br>
      
      <label for="holdBucketSwitch" style="font-size:20px;">Hold Bucket:</label>
      <input type="checkbox" id="holdBucketSwitch" onchange='sendButtonInput("HoldBucket", this.checked ? 1 : 0)'><br><br>

      <label for="holdAuxSwitch" style="font-size:20px;">Hold AUX:</label>
      <input type="checkbox" id="holdAuxSwitch" onchange='sendButtonInput("HoldAux", this.checked ? 1 : 0)'><br><br>

      <label for="gpioLeft">Left Motor GPIO:</label>
      <input type="number" id="gpioLeft" min="0" max="39"><br><br>

      <label for="gpioRight">Right Motor GPIO:</label>
      <input type="number" id="gpioRight" min="0" max="39"><br><br>

      <label for="gpioServo">Servo GPIO:</label>
      <input type="number" id="gpioServo" min="0" max="39"><br><br>

      <button onclick="saveRobotConfig()">Apply Config</button>
    </div>
</div>
</div>

)SETMODAL";