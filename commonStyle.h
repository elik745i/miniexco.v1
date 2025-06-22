// commonStyle.h
#pragma once

const char* commonStyle PROGMEM = R"HTMLSTYLE(

  *, *::before, *::after {
    box-sizing: border-box;
  }

  html, body {
    margin: 0;
    padding: 0;
    height: 100%;
    overflow: hidden;
    font-family: Arial;
    background: white;
    box-sizing: border-box;
  }

  .header {
    height: 4vh; /* example value */
    display: flex;
    align-items: center;
    justify-content: center;
    line-height: 4vh;
    width: 100%;
    background: black;
    color: white;
    text-align: center;
    padding: 1vh;
    font-size: 1vw;
  }

  .container {
    display: flex;
    flex-direction: row;
    height: calc(100vh - 4vh);  /* if you prefer fixed unit offset */
    width: 100vw;
    overflow: hidden;
  }


  .left {
    flex: 1;
    min-width: 200px;
    background: white;
    padding: 1vh;
    overflow-y: auto;
    max-height: 100%;
    box-sizing: border-box;
    -webkit-overflow-scrolling: touch;
  }

  .right {
    flex: 4;
    display: flex;
    justify-content: center;
    align-items: center;
    background: black;
    position: relative;
    overflow: hidden;
  }

  #cameraStream {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    object-fit: cover;
    background-color: black;
    z-index: 1;
  }


  .overlay,
  .overlay-topright {
    position: absolute;
  }

  .overlay-bottomright {
    position: absolute;
    cursor: pointer;
    background: transparent !important;
    padding: 0 !important;
    border-radius: 0 !important;
    color: white !important;
    font-size: 1.2vw;
    text-shadow: 0 0 3px black;
    z-index: 20;
  }


  .overlay {
    top: 1vh;
    left: 1vh;
    z-index: 30;  /* was missing or too low */
  }

  .overlay-topright {
    top: 1vh;
    right: 1vh;
    text-align: right;
    z-index: 30;  /* elevate above drawing layers */
  }

  #statusOverlay {
    z-index: 30;  /* make sure it's above canvas too */
  }

  .arrows {
    font-size: 3vw;
    color: grey;
  }

  td.button {
    background-color: black;
    border-radius: 10%;
    box-shadow: 0.5vh 0.5vh #888;
  }

  td.button:active {
    transform: translate(0.5vh, 0.5vh);
    box-shadow: none;
  }

  .slidecontainer {
    width: 100%;
  }

  .slider {
    -webkit-appearance: none;
    width: 100%;
    height: 2vh;
    border-radius: 0.5vh;
    background: #d3d3d3;
    outline: none;
    opacity: 0.7;
    transition: opacity .2s;
  }

  .slider:hover {
    opacity: 1;
  }

  .slider::-webkit-slider-thumb,
  .slider::-moz-range-thumb {
    width: 4vh;
    height: 4vh;
    border-radius: 50%;
    background: red;
    cursor: pointer;
  }

  .batt-green { color: #00ff00; }
  .batt-orange { color: #ffaa00; }
  .batt-red { color: #ff0000; }
  .wifi-green { color: #00ff00; }
  .wifi-orange { color: #ffaa00; }
  .wifi-red { color: #ff0000; }

  @keyframes flashRed {
    0% { color: red; }
    50% { color: white; }
    100% { color: red; }
  }

  .batt-critical {
    animation: flashRed 1s infinite;
  }

  body.dark { background: #111; color: white; }
  .left.dark, .right.dark, .header.dark { background: #222; color: white; }

  #chargingOverlay {
    position: absolute;
    top: 1vh;
    right: 1vh;
    width: 4vw;
    height: 2vh;
    border: 2px solid lime;
    border-radius: 0.5vh;
    overflow: hidden;
    display: none;
    background: #222;
    z-index: 999;
  }

  #chargingFill {
    height: 100%;
    width: 0%;
    background-color: lime;
    animation: chargingAnim 1s linear infinite;
  }

  @keyframes chargingAnim {
    0% { width: 0%; }
    100% { width: 100%; }
  }

  #chargeIcon {
    margin-left: 0.5vw;
    font-size: 1.5vw;
    font-weight: bold;
    vertical-align: middle;
    animation: fadeCharge 1.5s infinite;
    transition: color 0.3s ease;
  }

  @keyframes fadeCharge {
    0%   { opacity: 0.2; }
    50%  { opacity: 1.0; }
    100% { opacity: 0.2; }
  }

  @keyframes flashYellow {
    0% { color: yellow; }
    50% { color: white; }
    100% { color: yellow; }
  }

  .temp-warning {
    animation: flashYellow 1s infinite;
    font-weight: bold;
  }

  .temp-critical {
    animation: flashRed 1s infinite;
    font-weight: bold;
  }

  @media (max-width: 800px), (max-height: 600px) {
    .arrows {
      font-size: 5vw;
    }

    .header {
      font-size: 2vw;
    }

    .overlay, .overlay-topright {
      font-size: 2vw;
    }

    td.button {
      padding: 0.5vw;
    }
    .turn-indicator {
      font-size: 12vw;
    }
  }

  .tabButton {
    background: #444;
    color: white;
    padding: 6px 12px;
    font-size: 14px;
    line-height: 18px;
    border: none;
    cursor: pointer;
    border-radius: 5px 5px 0 0;
    margin: 0 2px;
  }

  .tabButton.active {
    background: #222;
    color: white;
    border: 2px solid white;
    border-bottom: none;
    outline: none;
    position: relative;
    z-index: 1;
    padding: 6px 12px;
    clip-path: polygon(
      0 100%, 0 0, calc(100% - 10px) 0, 100% 10px, 100% 100%
    );
    box-shadow: 0 0 0 2px white;
  }

  .slider.active {
    box-shadow: 0 0 10px rgba(255, 0, 0, 0.7);
  }

  .spinner {
    width: 20px;
    height: 20px;
    border: 4px solid #ccc;
    border-top-color: #00f;
    border-radius: 50%;
    animation: spin 1s linear infinite;
  }

  @keyframes spin {
    to { transform: rotate(360deg); }
  }

  label span {
    font-weight: normal;
    margin-left: 6px;
    color: yellow;
  }

  .blur {
    filter: blur(5px);
    pointer-events: none;
    user-select: none;
  }

  .tabButton:hover {
    background: #666;
    box-shadow: 0 0 5px white;
  }

  #settingsModal {
    position: fixed;
    top: 50%;
    left: 50%;
    transform: translate(-50%, -50%);
    background: #333;
    color: white;
    width: 90%;
    max-width: 600px;
    max-height: 90vh;
    overflow-y: auto;
    margin-bottom: 2vh;
    padding: 20px;
    border-radius: 20px;
    z-index: 9999 !important;
    transition: box-shadow 0.2s ease, transform 0.3s ease;
  }

  #modalFrameShape {
    position: absolute;
    top: 0; left: 0;
    right: 0; bottom: 0; /* use this instead of width/height to track dynamic changes */
    z-index: 9998 !important;
    pointer-events: none;
    border: 4px solid white;
    border-radius: inherit; /* <-- Important for consistent rounded corners */
    transition: clip-path 0.2s ease; /* Optional for smooth shape change */
  }

  .tabContentWrapper {
    position: relative;
    overflow: hidden;
    transition: height 0.3s ease;
  }

  .tabContent {
    position: absolute;
    top: 0;
    left: 0;
    right: 0;
    opacity: 0;
    pointer-events: none;
    transition: opacity 0.3s ease;
  }

  .tabContent.active {
    position: relative;
    opacity: 1;
    pointer-events: auto;
  }

  @keyframes beaconFlash {
    0%   { background-color: red; }
    50%  { background-color: blue; }
    100% { background-color: red; }
  }

  #beaconBtn.blinking {
    animation: beaconFlash 0.6s infinite, emergencyPulse 1s infinite;
    color: white;
  }

  #beaconBtn.beacon-active {
    animation: beaconFlash 0.5s infinite;
  }

  #beaconBtn:active {
    transform: scale(0.95);
    box-shadow: 0 0 4px rgba(0,0,0,0.3);
  }

  /* Emergency button default style */
  #emergencyBtn {
    margin-top: 10px;
    padding: 10px 20px;
    background-color: yellow;
    color: black;
    border: none;
    border-radius: 5px;
    font-size: 16px;
    animation: none;
  }

  /* Emergency blinking effect */
  @keyframes emergencyBlink {
    0%   { background-color: yellow; opacity: 1; }
    50%  { background-color: #ffcc00; opacity: 0.4; }
    100% { background-color: yellow; opacity: 1; }
  }

  #emergencyBtn.blinking {
    animation: emergencyBlink 0.8s infinite;
  }

  #emergencyBtn:active {
    transform: scale(0.95);
    box-shadow: 0 0 4px rgba(0,0,0,0.3);
  }

  /* Optional: Pulsing transform like Beacon */
  @keyframes emergencyPulse {
    0% { transform: scale(1); }
    50% { transform: scale(1.05); }
    100% { transform: scale(1); }
  }

  #emergencyBtn.blinking {
    animation: emergencyBlink 0.8s infinite, emergencyPulse 1.2s infinite;
  }

  #ledToggleBtn {
    width: 100%;
    padding: 10px 20px;
    background-color: #222;
    color: white;
    border: 2px solid white;
    border-radius: 5px;
    font-size: 16px;
  }

  #ledToggleBtn.active {
    background-color: yellow;
    color: black;
    border: none;
  }

  .ctrlBtn {
    width: 100%;
    padding: 10px 20px;
    font-size: 16px;
    border: none;
    border-radius: 5px;
    margin-top: 10px;
    transition: transform 0.1s, background-color 0.2s;
  }

  .ctrlBtn:active {
    transform: scale(0.95);
  }

  .ctrlBtn.dark {
    background-color: #444;
    color: white;
  }

  .ctrlBtn.light {
    background-color: #ddd;
    color: black;
  }

  .beaconStyle {
    background-color: red;
    color: white;
  }

  .emergencyStyle {
    background-color: yellow;
    color: black;
  }

  .turn-indicator {
    position: absolute;
    top: 50%;
    font-size: 8vw;
    color: yellow;
    opacity: 0;
    z-index: 15;
    transform: translateY(-50%);
    pointer-events: none;
    background: transparent !important; /* force no background */
    text-shadow: 1px 1px 3px black;     /* add soft outline for visibility */
  }

  .turn-indicator.blinking {
    animation: flashTurn 1s infinite;

  }


  .turn-indicator.left {
    left: 2vw;
    z-index: 15;
  }

  .turn-indicator.right {
    right: 2vw;
    z-index: 15;
  }

  @keyframes flashTurn {
    0%, 100% { opacity: 1; }
    50% { opacity: 0; }
  }

  #leftIndicator, #rightIndicator {
    opacity: 0;
    visibility: hidden;
    transition: opacity 0.3s ease;
  }

  #leftIndicator.visible, #rightIndicator.visible {
    visibility: visible;
    opacity: 1;
  }

  #laneOverlay {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    pointer-events: none;
    z-index: 5;
  }

  .lane {
    position: absolute;
    bottom: 10px;
    height: 50%;
    width: 8px;
    background: rgba(255, 255, 0, 0.7); /* semi-transparent yellow line */
    opacity: 0.7;
  }

  .left-lane {
    left: 5%;
    transform: rotate(55deg);
    transform-origin: bottom center;
  }

  .right-lane {
    right: 5%;
    transform: rotate(-55deg);
    transform-origin: bottom center;
  }

  #drawPathBtn.active {
    background-color: lime;
    color: black;
    font-weight: bold;
  }

  #drawingCanvas {
    position: absolute;
    top: 0;
    left: 0;
    z-index: -1; /* default: not blocking anything */
    pointer-events: none;
  }

  #drawingOverlay {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
    z-index: 16;
    pointer-events: none;
  }

  .ctrlBtn:hover {
    background-color: #666;
  }

  .overlay-bottomright:hover {
    text-shadow: 0 0 5px white;
  }

  #startButton {
    z-index: 25;
    position: absolute;
    bottom: 10px;
    left: 50%;
    transform: translateX(-50%);
    background-color: lime;
    color: black;
    font-weight: bold;
    padding: 8px 14px;
    border-radius: 6px;
    font-size: 1vw;
    cursor: pointer;
    text-shadow: 1px 1px 2px black;
    transition: transform 0.2s, box-shadow 0.3s;
    animation: pulseStart 1.6s infinite;
  }

  #startButton:hover {
    box-shadow: 0 0 10px lime;
    transform: scale(1.05);
  }

  #stopButton {
    position: absolute;
    background-color: red;
    color: white;
    font-weight: bold;
    padding: 8px 14px;
    border-radius: 6px;
    font-size: 1vw;
    z-index: 25;
    cursor: pointer;
    text-shadow: 1px 1px 2px black;
    transition: transform 0.2s, box-shadow 0.3s;
    animation: pulseStop 1.6s infinite;
  }

  #stopButton:hover {
    box-shadow: 0 0 10px red;
    transform: scale(1.05);
  }

  @keyframes pulseStart {
    0%, 100% { box-shadow: 0 0 6px lime; }
    50% { box-shadow: 0 0 16px lime; }
  }

  @keyframes pulseStop {
    0%, 100% { box-shadow: 0 0 6px red; }
    50% { box-shadow: 0 0 16px red; }
  }



  .rotate-node {
    fill: orange;
    stroke: black;
    stroke-width: 1px;
    cursor: pointer;
    pointer-events: auto;
    r: 8;
    z-index: 24;
  }

  .arrowhead {
    fill: white;
    stroke: black;
    stroke-width: 1px;
  }

  .overlayBtn {
    background-color: #4CAF50;
    color: white;
    border: none;
    border-radius: 8px;
    padding: 6px 12px;
    font-size: 14px;
    cursor: pointer;
    transition: transform 0.1s ease-in-out;
  }

  .overlayBtn:active {
    transform: scale(0.95);
  }

  /* Style for SVG drawn polyline path */
  .drawn-path {
    stroke: lime;
    stroke-width: 10;
    fill: none;
    stroke-linecap: round;
    stroke-linejoin: round;
    stroke-dasharray: 20,10;          /* dash-gap */
    animation: moveDash 1s linear infinite;
    opacity: 1;
  }

  @keyframes moveDash {
    0% { stroke-dashoffset: 0; }
    100% { stroke-dashoffset: -30; }
  }

  #imuWarning {
    position: absolute;
    top: 10px;
    left: 50%;
    transform: translateX(-50%);
    background: red;
    color: white;
    font-weight: bold;
    padding: 12px 20px;
    border-radius: 8px;
    z-index: 9999;
    font-size: 16px;
    text-align: center;
    animation: blinkWarning 0.6s linear infinite;
    box-shadow: 0 0 10px rgba(255,0,0,0.7);
  }

  @keyframes blinkWarning {
    0%, 100% { opacity: 1; }
    50% { opacity: 0.3; }
  }

  #otaStatus {
    margin-top: 10px;
    color: white;
  }
  progress {
    height: 20px;
  }

#joystickContainer {
  width: 150px;
  height: 150px;
  background-color: #444;
  border-radius: 10px;
  position: relative;
  margin: 20px auto;
  touch-action: none;
}

#joystickKnob {
  position: absolute;
  width: 40px;
  height: 40px;
  background: #888;
  border-radius: 50%;
  left: 50%;
  top: 50%;
  transform: translate(-50%, -50%);
  transition: left 0.2s ease, top 0.2s ease;  /* 👈 Smooth transition */
}

)HTMLSTYLE";
