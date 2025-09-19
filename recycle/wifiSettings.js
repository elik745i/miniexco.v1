// --- WiFi TAB LOGIC ---

window.loadWifiTab = function() {
  // Initial refresh of saved networks
  if (typeof loadSavedNetworks === "function") loadSavedNetworks();

  // Optional: Start scanning for new networks (if your UI supports it)
  // scanNetworks();

  // (Optional) Wire up scan/refresh button if you have one:
  // const scanBtn = document.getElementById("wifiScanBtn");
  // if (scanBtn) scanBtn.onclick = scanNetworks;
};


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
		div.className = 'wifi-row';

		// SSID name
		const nameSpan = document.createElement('span');
		nameSpan.innerText = net.ssid;
		nameSpan.style.fontWeight = 'bold';
		div.appendChild(nameSpan);

		// Pencil/Edit icon button
		const editBtn = document.createElement('button');
		editBtn.title = "Edit Wi-Fi settings";
		editBtn.style.marginLeft = '8px';
		editBtn.innerHTML = `
			<svg width="18" height="18" fill="currentColor" viewBox="0 0 20 20">
			<path d="M17.414 2.586a2 2 0 010 2.828l-10 10a2 2 0 01-.707.414l-4 1a1 1 0 01-1.265-1.265l1-4a2 2 0 01.414-.707l10-10a2 2 0 012.828 0zm-10 12.828L15 5.828l-2.828-2.828-10 10V15h2.172z"/>
			</svg>`;
		editBtn.onclick = () => showWifiEditModal(net);
		div.appendChild(editBtn);

		container.appendChild(div);
		});
	})
	.catch(err => {
		console.error('Failed to load saved networks:', err);
		document.getElementById('savedNetworksContainer').innerText = 'Error loading saved networks.';
	});
}

function showWifiEditModal(net) {
	
	let modal = document.getElementById('wifiEditModal');
	if (!modal) {
	modal = document.createElement('div');
	modal.id = 'wifiEditModal';
	modal.className = 'modal';
	modal.innerHTML = `
		<div class="modal-content">
		<h3>Edit Wi-Fi: <span id="wifiEditSSID"></span></h3>
		<label>Password:</label><br>
		<input type="password" id="wifiEditPassword" style="width:100%;margin-bottom:10px;">
		<span id="wifiEditShow" style="cursor:pointer;margin-left:4px;">👁️</span>
		<br><br>
		<label>Retry Count:</label><br>
		<input type="number" min="1" max="10" id="wifiEditRetry" style="width:60px;margin-bottom:14px;"><br>
		<label>
			<input type="checkbox" id="wifiEditAutoReconnect" style="margin-right:6px;vertical-align:middle;">
			Auto-reconnect
		</label>
		<br>
		<button id="wifiEditTryConnect" style="margin:12px 0 18px 0;float:right;">🔄 Try Connect Now</button>
		<div style="clear:both"></div>
		<div style="text-align:right;">
			<button id="wifiEditSave" style="margin-right:8px;">💾 Save</button>
			<button id="wifiEditCancel">✖ Cancel</button>
		</div>
		</div>
	`;
	document.body.appendChild(modal);

	// Dismiss by clicking outside
	modal.onclick = (e) => {
		if (e.target === modal) modal.style.display = 'none';
	};
	}

	// Update content for this network
	document.getElementById('wifiEditSSID').textContent = net.ssid;
	const passField = document.getElementById('wifiEditPassword');
	const retryField = document.getElementById('wifiEditRetry');
	const autoReconnect = document.getElementById('wifiEditAutoReconnect');
	const tryBtn = document.getElementById('wifiEditTryConnect');

	passField.value = net.password || '';
	retryField.value = net.retry || 3;
	autoReconnect.checked = net.autoReconnect !== false;

	// Show/hide password
	document.getElementById('wifiEditShow').onclick = () => {
	passField.type = passField.type === 'password' ? 'text' : 'password';
	};

	// Try Connect Now
	tryBtn.onclick = () => {
	fetch(`/wifi_try_connect?ssid=${encodeURIComponent(net.ssid)}`, { method: 'POST' })
		.then(r => r.text())
		.then(txt => {
		showToast(txt.startsWith("Connected") ? `🔄 Connecting to ${net.ssid}...` : `⚠️ ${txt}`, !txt.startsWith("Connected"));
		});
	};

	// Save changes
	document.getElementById('wifiEditSave').onclick = () => {
		const newPass = passField.value;
		const newRetry = Math.min(10, Math.max(1, parseInt(retryField.value) || 3));
		const autoRe = autoReconnect.checked;

		// Save password and retry count
		updateSavedPassword(net.ssid, newPass);
		updateRetryCount(net.ssid, newRetry);

		// Send autoReconnect using proper POST body
		const postData = `ssid=${encodeURIComponent(net.ssid)}&enabled=${autoRe ? 1 : 0}`;
		fetch(`/wifi_set_autoreconnect`, {
		method: 'POST',
		headers: {
			'Content-Type': 'application/x-www-form-urlencoded'
		},
		body: postData
		})
		.then(response => {
		if (!response.ok) {
			throw new Error(`Server responded with ${response.status}`);
		}
		modal.style.display = 'none';
		setTimeout(loadSavedNetworks, 400);
		})
		.catch(err => {
		console.error("AutoReconnect update failed:", err);
		showToast("❌ Failed to update auto-reconnect", true);
		});
	};


	// Cancel button
	document.getElementById('wifiEditCancel').onclick = () => {
	modal.style.display = 'none';
	};

	// Show modal
	modal.style.display = 'flex';
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

