#pragma once

// Wi-Fi Setup Page
const char* htmlWiFiSetupPage PROGMEM = R"HTMLWIFISETUP(
<!DOCTYPE html>
<html>
<head>
  <title>Wi-Fi Setup</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <h2>Configure Wi-Fi</h2>
  <form method="POST" action="/savewifi">
    <label for="ssid">Select Wi-Fi:</label><br>
    <select name="ssid" id="ssidList">%OPTIONS%</select><br><br>
    <label for="password">Password:</label><br>
    <input type="password" name="password"><br><br>
    <input type="submit" value="Connect">
  </form>
  <button onclick="location.reload()">Refresh List</button>
</body>
</html>
)HTMLWIFISETUP";

// Success Page after Wi-Fi credentials are saved
const char* htmlWiFiSuccessPage PROGMEM = R"HTMLWIFISUCCESS(
<!DOCTYPE html>
<html>
<head>
  <title>Wi-Fi Saved</title>
  <meta http-equiv="refresh" content="3; url=/" />
</head>
<body>
  <h1>✅ Wi-Fi Credentials Saved</h1>
  <p>Rebooting and trying to connect...</p>
</body>
</html>
)HTMLWIFISUCCESS";
