let keyMap = {};  // Dynamic key-to-action map

const UI_TO_FW = {
  forward:"forward", backward:"backward", left:"left", right:"right",
  stop:"stop",
  armUp:"arm_up", armDown:"arm_down",
  bucketUp:"bucket_up", bucketDown:"bucket_down",
  auxUp:"aux_up", auxDown:"aux_down",
  led:"light_toggle", beacon:"beacon",
  emergency:"emergency",            // <- was "emergency_toggle"
  horn:"horn"
};

const FW_TO_UI = Object.fromEntries(Object.entries(UI_TO_FW).map(([ui, fw]) => [fw, ui]));

function uiKeyToWire(k) {
  if (k === " ") return " ";
  // ArrowUp -> arrowup (backend stores lowercase)
  if (k.startsWith("Arrow")) return k.toLowerCase();
  return k.length === 1 ? k.toLowerCase() : k;
}
function wireKeyToUi(k) {
  if (k === " ") return " ";
  if (k.startsWith("arrow")) {
    const tail = k.slice(5);                    // "up" | "down" | "left" | "right"
    return "Arrow" + tail.charAt(0).toUpperCase() + tail.slice(1);
  }
  return k;
}

const allKeys = [
  " ", "w", "a", "s", "d", "u", "j", "i", "k", "l", "b", "e",
  "q", "z", "x", "c", "v", "n", "m", "h",
  "ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight"
];

// --- WS proxy so legacy `ws.send(...)` never throws ---
(function () {
  // Queue for early sends
  window._wsQueue = window._wsQueue || [];

  // Create the socket on demand (single instance)
  window.ensureCarSocket = window.ensureCarSocket || function () {
    if (window.wsCarInput && window.wsCarInput.readyState <= 1) return window.wsCarInput;
    const s = new WebSocket(`ws://${location.host}/CarInput`);
    window.wsCarInput = s;
    window.websocketCarInput = s; // legacy alias
    s.addEventListener("open", () => {
      // flush anything queued before open
      const q = window._wsQueue.splice(0);
      q.forEach(m => s.send(m));
      window.dispatchEvent(new Event("car-socket-ready"));
    });
    return s;
  };

  // Global `ws` object that forwards to the live socket
  // Use a proxy-like object so `ws.send()` is always defined.
  window.ws = {
    send(msg) {
      const s = window.wsCarInput || window.websocketCarInput || window.ensureCarSocket();
      if (!s) return console.warn("car socket not available; dropped:", msg);
      if (s.readyState === WebSocket.OPEN) return s.send(msg);
      // Not open yet → queue and flush on open
      window._wsQueue.push(msg);
      const onOpen = () => { 
        const q = window._wsQueue.splice(0); 
        q.forEach(m => s.send(m)); 
        s.removeEventListener("open", onOpen); 
      };
      s.addEventListener("open", onOpen, { once: true });
    }
  };
})();


document.addEventListener("DOMContentLoaded", () => {
  loadKeyMappings();
  setupKeyInputValidation();
});

// 🧪 DEBUG: log every key pressed
document.addEventListener("keydown", e => console.log("Key pressed:", e.key));

// Map between UI tokens and stored/normalized keys
function uiToKey(v) {
  if (!v) return " ";
  if (v === "Space") return " ";
  if (v.startsWith("Arrow")) return v.toLowerCase();   // ArrowUp -> arrowup
  return v.length === 1 ? v.toLowerCase() : v;          // Letters -> lowercase
}
function keyToUI(k) {
  if (!k) return "Space";
  if (k === " ") return "Space";
  if (k.startsWith("arrow")) {
    const tail = k.slice(5);
    return "Arrow" + tail.charAt(0).toUpperCase() + tail.slice(1);
  }
  return k;
}

function controlsTabKeydown(e) {
  // Only active while the settings modal is open
  if (!window.isModalOpen || !window.isModalOpen()) return;

  // (Optional) keep preventDefault so arrows/space don't scroll the modal
  let key = e.key;
  if (key.length === 1) key = key.toLowerCase();
  if (["arrowup","arrowdown","arrowleft","arrowright"," "].includes(key)) {
    e.preventDefault();
  }

  // We intentionally DO NOT send any robot commands from here.
  // This script is for mapping UI only.
}

// Attach when the file loads
document.addEventListener("keydown", controlsTabKeydown);

// Make sure it’s removed when the modal closes (commonScript calls this)
window.onunloadModal = function () {
  document.removeEventListener("keydown", controlsTabKeydown);
};

function populateKeyDropdowns() {
  const selects = document.querySelectorAll("#keyMappingInputs select");
  selects.forEach(select => {
    select.innerHTML = "";
    allKeys.forEach(key => {
      const option = document.createElement("option");
      option.value = key;
      option.textContent = key === " " ? "Space" : key;
      select.appendChild(option);
    });
  });
}

function loadKeyMappings() {
  populateKeyDropdowns();
  fetch("/get_keymap")
    .then(res => res.json())
		.then(map => {
			keyMap = {};
			Object.entries(map).forEach(([fwAction, key]) => {
				const uiAction = FW_TO_UI[fwAction] || fwAction;     // snake_case -> camelCase
				const select = document.querySelector(`select[data-action="${uiAction}"]`);
				if (select) select.value = wireKeyToUi(key);          // arrowup -> ArrowUp
				keyMap[uiKeyToWire(key)] = uiAction;                  // keep keyMap normalized
			});
			console.log("Keymap loaded:", keyMap);
		});
}



function saveKeyMappings() {
	const selects = document.querySelectorAll("#keyMappingInputs select");
	const tempMap = {};
	let hasDuplicate = false;

	selects.forEach(select => {
		const ui = select.value.trim();
		const norm = uiToKey(ui);                  // normalize (space/arrow/letters → stored form)
		const action = select.dataset.action;
		const warn = select.nextElementSibling;
		warn.textContent = "";
		warn.style.color = "";

		if (!norm) return;
		if (tempMap[norm]) {
			warn.textContent = "⚠️ Duplicate";
			warn.style.color = "red";
			hasDuplicate = true;
		} else {
			tempMap[norm] = action;                  // use normalized key as the map key
		}
	});

  if (hasDuplicate) {
    document.getElementById("keySaveStatus").textContent = "Fix duplicates first!";
    document.getElementById("keySaveStatus").style.color = "red";
    return;
  }

	// Convert { normalizedKey: action } → { action: normalizedKey }
	const finalMap = {};
	Object.entries(tempMap).forEach(([key, action]) => {
		const fw = UI_TO_FW[action] || action;
		finalMap[fw] = uiKeyToWire(key);
	});

	fetch("/set_keymap", {
		method: "POST",
		headers: { "Content-Type": "application/json" },
		body: JSON.stringify(finalMap)
	})
		.then(res => res.text())
		.then(msg => {
			document.getElementById("keySaveStatus").textContent = "✔️ Saved!";
			document.getElementById("keySaveStatus").style.color = "lightgreen";
			loadKeyMappings();                         // repaint the dropdowns
			if (window.refreshRuntimeKeymap) window.refreshRuntimeKeymap(); // <-- add this
		});

}


function setupKeyInputValidation() {
  const selects = document.querySelectorAll("#keyMappingInputs select");

  selects.forEach(select => {
    select.addEventListener("change", () => {
      const seen = {};
      selects.forEach(sel => {
        const key = uiToKey(sel.value.trim());  // ✅ Preserve casing for correct duplicate detection
        const warn = sel.nextElementSibling;
        warn.textContent = "";
        warn.style.color = "";

        if (key && seen[key]) {
          warn.textContent = "⚠️ Duplicate";
          warn.style.color = "red";
        } else {
          seen[key] = true;
        }
      });
    });
  });
}


function resetToDefaultKeymap() {
  const defaultMap = {
    forward: "w",
    backward: "s",
    left: "a",
    right: "d",
    stop: " ",
    armUp: "n",
    armDown: "m",    
    bucketUp: "u",
    bucketDown: "j",
    auxUp: "i",
    auxDown: "k",
    led: "l",
    beacon: "b",
    emergency: "e",
		horn: "h"
  };

  const selects = document.querySelectorAll("#keyMappingInputs select");
  selects.forEach(select => {
    const action = select.dataset.action;
    if (defaultMap[action] !== undefined) {
      select.value = defaultMap[action];
    }
  });

  document.getElementById("keySaveStatus").textContent = "Default keys loaded. Click Save to apply.";
  document.getElementById("keySaveStatus").style.color = "orange";
}
