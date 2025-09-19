
// --- Azerbaijan radio stations (direct streams) ---
window.radioStations = [
  { name: "106 FM (Azad Azərbaycan)", url: "https://audiostr.atv.az/106fm" },
  { name: "Yurd FM",                   url: "https://icecast.livetv.az/yurdfm" },
  { name: "Anti Radio (Baku)",         url: "https://stream.antiradio.net/radio/8000/mp3" },
  { name: "Radio TMB 100.5",           url: "https://s39.myradiostream.com:5458/listen.mp3" }
];



if (typeof window.mediaFiles === "undefined") window.mediaFiles = [];
if (typeof window.currentMediaIndex === "undefined") window.currentMediaIndex = -1;
if (typeof window.currentDeviceMediaIndex === "undefined") window.currentDeviceMediaIndex = -1;
if (typeof window.fileListPageSize === "undefined") window.fileListPageSize = 20;
if (typeof window.totalMediaFiles === "undefined") window.totalMediaFiles = 0;
if (typeof window.loadingFiles === "undefined") window.loadingFiles = false;
if (typeof window.loopEnabled === "undefined") window.loopEnabled = false;
if (typeof window.shuffleEnabled === "undefined") window.shuffleEnabled = false;
if (typeof window.shuffleHistory === "undefined") window.shuffleHistory = [];
if (typeof window.shuffleHistoryPointer === "undefined") window.shuffleHistoryPointer = -1;



function fmt(s) {
  if (isNaN(s)) return '--:--';
  let m = Math.floor(s / 60), ss = Math.floor(s % 60);
  return `${m}:${ss < 10 ? '0' : ''}${ss}`;
}

  
window.mediaPlaybackMode = "browser"; // Keep track of which mode is active: "browser" or "device"

window.mediaPlayerUpdateDeviceProgress = function(filename, elapsed, duration) {
  // Only update if device mode active
  if (window.mediaPlaybackMode !== "device" && window.mediaPlaybackMode !== "both") return;
  // Optionally: check if filename matches current device song
  // Update progress bar:
  const progressFill = document.getElementById('progressFill');
  const progressThumb = document.getElementById('progressThumb');
  const progressTime = document.getElementById('progressTime');
  if (!progressFill || !progressThumb || !progressTime) return;

  const percent = duration > 0 ? (elapsed / duration) * 100 : 0;
  progressFill.style.width = percent + '%';
  progressThumb.style.left = percent + '%';
  progressTime.textContent = fmt(elapsed) + " / " + fmt(duration);

  // Optionally: update highlight, set device song title, etc.
};

window.mediaPlayerSetDevicePlayState = function(filename, isPlaying) {
  window.mediaPlaybackMode = isPlaying ? "device" : "browser";
  
  if (isPlaying && filename) {
    // Normalize both to just the filename part for robust comparison
    const target = filename.split('/').pop();
    const idx = mediaFiles.findIndex(f => {
      let name = (typeof f === "object" && f.name) ? f.name : f;
      // Compare either full path or just file name
      return name === filename || name.split('/').pop() === target;
    });
    if (idx !== -1) {
      window.currentDeviceMediaIndex = idx;
    } else {
      window.currentDeviceMediaIndex = -1;
    }
  } else {
    window.currentDeviceMediaIndex = -1;
  }
  
  renderMediaFileList();
};



function fetchFirstMediaFiles() {
  hideMediaIndexingProgress();
  mediaFiles = [];
  totalMediaFiles = 0;
  renderMediaFileList(true); // Clear existing list
  fetchNextMediaFiles();
  updateSongTitleMarquee();
}

function fetchNextMediaFiles() {
  if (loadingFiles) return;
  loadingFiles = true;
  fetch(`/list_media_files?start=${mediaFiles.length}&count=${fileListPageSize}`)
    .then(r => r.json())
	.then(data => {
		//console.log("Fetched data.files:", data.files);
	  if (data && data.status === "reindexing") {
		showMediaIndexingProgress(data.path || "/media", 0, 0, 0, true);
		pollMediaReindexStatus(data.path || "/media");
		return;
	  }
	  if (Array.isArray(data.files)) {
		mediaFiles = mediaFiles.concat(data.files);
		totalMediaFiles = data.total;
		renderMediaFileList(); // Just add, do NOT clear!
	  }
	})

    .finally(() => loadingFiles = false);
}


window.toggleMediaPlayerReal = function() {
  // Always remove old modal before injecting a new one!
  let oldModal = document.getElementById('mediaPlayerModal');
  if (oldModal) oldModal.remove();

  // Now, always load fresh from server:
  fetch('/mediaPlayer.html?v=' + Date.now())
    .then(r => r.text())
    .then(html => {
      document.body.insertAdjacentHTML('beforeend', html);
      window.mediaPlayerLoaded = true;  // Optional: this variable is now redundant if you always reload
      initMediaPlayer();
      showMediaPlayer();
    });
};



function showMediaPlayer() {
  const playerModal = document.getElementById('mediaPlayerModal');
  if (!playerModal) return showToast("Media player modal not found!");
  playerModal.style.display = 'block';
  //fetchFirstMediaFiles();     // instead of fetchMediaFiles()
}


function hideMediaPlayer() {
  hideMediaIndexingProgress();
  const playerModal = document.getElementById('mediaPlayerModal');
  if (playerModal) playerModal.style.display = 'none';
}

function initMediaPlayer() {
	document.getElementById('mediaPlayBtn').onclick = function() {
	  if (window.mediaPlaybackMode === "device") {
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send("MEDIA_RESUME");
		}
		return;
	  }
	  playCurrentMedia();
	};

	document.getElementById('mediaPauseBtn').onclick = function() {
	  if (window.mediaPlaybackMode === "device") {
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send("MEDIA_PAUSE");
		}
		return;
	  }
	  getPlayer().pause();
	};

	document.getElementById('mediaStopBtn').onclick = function() {
	  if (window.mediaPlaybackMode === "device") {
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send("MEDIA_STOP");
		}
		return;
	  }
	  getPlayer().pause();
	  getPlayer().currentTime = 0;
	};

	document.getElementById('mediaPrevBtn').onclick = function() {
	  if (window.mediaPlaybackMode === "device") {
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send("MEDIA_PREV");
		}
		return;
	  }
	  // --- Existing browser shuffle/prev logic ---
	  if (shuffleEnabled) {
		if (shuffleHistoryPointer > 0) {
		  selectMedia(shuffleHistory[shuffleHistoryPointer - 1], "prev");
		  shuffleHistoryPointer -= 2; // Will be re-incremented by selectMedia
		}
	  } else {
		if (currentMediaIndex > 0) selectMedia(currentMediaIndex - 1, true);
	  }
	};


	document.getElementById('mediaNextBtn').onclick = function() {
	  //console.log("NEXT clicked, mode=", window.mediaPlaybackMode);
	  if (window.mediaPlaybackMode === "device") {
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send("MEDIA_NEXT");
		}
		return;
	  }
	  // --- Existing browser shuffle/next logic ---
	  if (shuffleEnabled) {
		if (shuffleHistoryPointer < shuffleHistory.length - 1) {
		  selectMedia(shuffleHistory[shuffleHistoryPointer + 1], "next");
		} else {
		  let available = mediaFiles.map((_, i) => i).filter(i => i !== currentMediaIndex);
		  if (available.length === 0) return;
		  let next = available[Math.floor(Math.random() * available.length)];
		  selectMedia(next, true);
		}
	  } else {
		if (currentMediaIndex < mediaFiles.length - 1) selectMedia(currentMediaIndex + 1, true);
	  }
	};



	document.getElementById('mediaVolume').oninput = function(e) {
	  const browserVolume = parseFloat(e.target.value);
	  // Convert [0..1] slider to [0..21] for ESP32
	  const deviceVolume = Math.round(browserVolume * 21);

	  if (!window.mediaPlaybackMode) window.mediaPlaybackMode = "browser";

	  // Control volume as per playback mode
	  if (window.mediaPlaybackMode === "browser") {
		getPlayer().volume = browserVolume;
	  } else if (window.mediaPlaybackMode === "device") {
		fetch('/set_volume?value=' + deviceVolume);
	  } else if (window.mediaPlaybackMode === "both") {
		getPlayer().volume = browserVolume;
		fetch('/set_volume?value=' + deviceVolume);
	  }
	};


    // Loop/Shuffle buttons
	const loopBtn = document.getElementById('mediaLoopBtn');
	if (loopBtn) {
	  loopBtn.onclick = function() {
		loopEnabled = !loopEnabled;
		loopBtn.classList.toggle('active', loopEnabled);
		// If loop is on, turn shuffle off for clarity
		if (loopEnabled && shuffleEnabled) {
		  shuffleEnabled = false;
		  shuffleBtn.classList.remove('active');
		}
		if (window.mediaPlaybackMode === "device" && window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send(loopEnabled ? "MEDIA_LOOP_ON" : "MEDIA_LOOP_OFF");
		}
	  };
	}

	const shuffleBtn = document.getElementById('mediaShuffleBtn');
	if (shuffleBtn) {
	  shuffleBtn.onclick = function() {
		shuffleEnabled = !shuffleEnabled;
		shuffleBtn.classList.toggle('active', shuffleEnabled);
		// If shuffle is on, turn loop off for clarity
		if (shuffleEnabled && loopEnabled) {
		  loopEnabled = false;
		  loopBtn.classList.remove('active');
		}
		if (window.mediaPlaybackMode === "device" && window.wsCarInput && window.wsCarInput.readyState === 1) {
		  window.wsCarInput.send(shuffleEnabled ? "MEDIA_SHUFFLE_ON" : "MEDIA_SHUFFLE_OFF");
		}
	  };
	}


  // Only attach if exists
  var filesBtn = document.getElementById('mediaFilesBtn');
  if (filesBtn) {
    filesBtn.onclick = function() {
      const panel = document.getElementById('mediaFileListPanel');
      if (!panel) return;
      panel.classList.toggle('open');
      // Refresh and reset file list every time panel opens
			if (panel.classList.contains('open')) {
					// Only fetch if not already loaded
					if (mediaFiles.length === 0) fetchFirstMediaFiles();
			}

    };
  }

	// --- Radio dropdown + buttons ---
	const radioSel  = document.getElementById('radioSelect');
	const radioWeb  = document.getElementById('radioBrowserBtn');
	const radioDev  = document.getElementById('radioDeviceBtn');

	// populate dropdown once
	if (radioSel && !radioSel.options.length) {
		radioStations.forEach((s, i) => {
			const opt = document.createElement('option');
			opt.value = s.url;
			opt.textContent = s.name;
			radioSel.appendChild(opt);
		});
	}

	function setLiveTitle(name) {
		// show station name on the marquee/title area
		const titleDiv = document.getElementById('progressSongTitle');
		const wrap = document.getElementById('progressSongTitleWrap');
		if (!titleDiv || !wrap) return;
		currentMediaIndex = -1; // clear file highlight
		const live = `🔴 LIVE · ${name}`;
		titleDiv.innerHTML = live + '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;' + live;
		setTimeout(() => {
			const wrapWidth = wrap.offsetWidth;
			const textWidth = titleDiv.scrollWidth / 2;
			if (textWidth > wrapWidth) {
				const total = textWidth + 32;
				titleDiv.style.setProperty('--scroll-total', `-${total}px`);
				const speed = Math.max(8, (textWidth / 90) * 8);
				titleDiv.style.animation = `marquee-loop ${speed}s linear infinite`;
			} else {
				titleDiv.style.animation = 'none';
				titleDiv.textContent = live;
			}
		}, 30);
	}

	function playRadioInBrowser(url, name) {
		// stop device if it’s playing
		if (window.mediaPlaybackMode !== "browser" && window.wsCarInput && window.wsCarInput.readyState === 1) {
			window.wsCarInput.send("MEDIA_STOP");
		}
		window.mediaPlaybackMode = "browser";

		const audio = document.getElementById('mediaPlayerAudio');
		const video = document.getElementById('mediaPlayerVideo');
		if (video) { video.pause(); video.style.display = 'none'; }
		if (!audio) return;
		audio.style.display = '';
		const volSlider = document.getElementById('mediaVolume');
		if (volSlider) audio.volume = parseFloat(volSlider.value || "1");

		audio.src = url;     // direct stream URL
		audio.play().catch(()=>{ showToast("Couldn’t start radio in browser."); });
		setLiveTitle(name);
		renderMediaFileList(); // clears highlight
	}

	function playRadioOnDevice(url, name) {
		// stop browser player
		const audio = document.getElementById('mediaPlayerAudio');
		const video = document.getElementById('mediaPlayerVideo');
		if (audio) { audio.pause(); audio.currentTime = 0; }
		if (video) { video.pause(); video.currentTime = 0; }

		window.mediaPlaybackMode = "device";
		if (window.wsCarInput && window.wsCarInput.readyState === 1) {
			window.wsCarInput.send("RADIO_PLAY," + url);
			showToast("Asked ESP32 to play: " + name);
		} else {
			fetch('/play_radio?url=' + encodeURIComponent(url))
				.then(r => r.text()).then(t => showToast(t));
		}
		if (window.mediaPlayerSetDevicePlayState)
			window.mediaPlayerSetDevicePlayState(name, true);
		setLiveTitle(name);
	}

	if (radioWeb) {
		radioWeb.onclick = () => {
			const url = radioSel?.value || radioStations[0].url;
			const name = radioSel?.selectedOptions?.[0]?.textContent || radioStations[0].name;
			playRadioInBrowser(url, name);
		};
	}
	if (radioDev) {
		radioDev.onclick = () => {
			const url = radioSel?.value || radioStations[0].url;
			const name = radioSel?.selectedOptions?.[0]?.textContent || radioStations[0].name;
			playRadioOnDevice(url, name);
		};
	}


  document.getElementById('mediaPlayerModal').onclick = function(e) {
    if (e.target === this) hideMediaPlayer();
  };

  document.addEventListener('keydown', function(e) {
    const modal = document.getElementById('mediaPlayerModal');
    if (!modal || modal.style.display !== 'block') return;
    if (e.key === 'Escape') hideMediaPlayer();
    if (e.key === ' ') {
      const player = getPlayer();
      if (player.paused) player.play();
      else player.pause();
      e.preventDefault();
    }
  }, false);

  // --- Infinite Scroll Handler for File List ---
  const panel = document.getElementById('mediaFileListPanel');
  panel.onscroll = function() {
    if (panel.scrollTop + panel.clientHeight >= panel.scrollHeight - 50) {
      if (!loadingFiles && mediaFiles.length < totalMediaFiles) {
        fetchNextMediaFiles();
      }
    }
  };

  // --- Custom progress bar logic below ---
  const audio = document.getElementById('mediaPlayerAudio');
  const video = document.getElementById('mediaPlayerVideo');
  const progressTrack = document.getElementById('progressTrack');
  const progressFill = document.getElementById('progressFill');
  const progressThumb = document.getElementById('progressThumb');
  const progressTime = document.getElementById('progressTime');

  function updateProgressBar() {
    const player = getPlayer();
    if (!player || !progressFill || !progressThumb || !progressTime) return;
    let percent = 0;
    if (player.duration) percent = (player.currentTime / player.duration) * 100;
    progressFill.style.width = percent + '%';
    progressThumb.style.left = percent + '%';
    progressTime.textContent = `${fmt(player.currentTime)} / ${fmt(player.duration)}`;

    // --- Add thumb-over-text color logic (not needed if time is on right, but harmless):
    const timeRect = progressTime.getBoundingClientRect();
    const thumbRect = progressThumb.getBoundingClientRect();
    if (
      thumbRect.left < timeRect.right &&
      thumbRect.right > timeRect.left &&
      thumbRect.top < timeRect.bottom &&
      thumbRect.bottom > timeRect.top
    ) {
      progressTime.classList.add('covered');
    } else {
      progressTime.classList.remove('covered');
    }
  }
  
  updateSongTitleMarquee();
  // --- Drag/seek support ---
  let isDragging = false;
  let wasPlaying = false;

  function seekToEvent(e) {
    const player = getPlayer();
    if (!player.duration) return;
    const rect = progressTrack.getBoundingClientRect();
    let x;
    if (e.touches && e.touches.length) {
      x = e.touches[0].clientX - rect.left;
    } else if (e.changedTouches && e.changedTouches.length) {
      x = e.changedTouches[0].clientX - rect.left;
    } else {
      x = e.clientX - rect.left;
    }
    x = Math.max(0, Math.min(rect.width, x));
    const percent = x / rect.width;
    player.currentTime = percent * player.duration;
    updateProgressBar();
  }

  // Mouse events
  progressTrack.addEventListener('mousedown', function(e) {
    const player = getPlayer();
    if (!player.duration) return;
    isDragging = true;
    wasPlaying = !player.paused;
    player.pause();
    seekToEvent(e);
    document.body.style.userSelect = "none";
  });
  window.addEventListener('mousemove', function(e) {
    if (!isDragging) return;
    seekToEvent(e);
  });
  window.addEventListener('mouseup', function(e) {
    if (!isDragging) return;
    isDragging = false;
    seekToEvent(e);
    if (wasPlaying) getPlayer().play();
    document.body.style.userSelect = "";
  });

  // Touch events
  progressTrack.addEventListener('touchstart', function(e) {
    isDragging = true;
    wasPlaying = !getPlayer().paused;
    getPlayer().pause();
    seekToEvent(e);
    e.preventDefault();
  });
  window.addEventListener('touchmove', function(e) {
    if (!isDragging) return;
    seekToEvent(e);
    e.preventDefault();
  }, { passive: false });
  window.addEventListener('touchend', function(e) {
    if (!isDragging) return;
    isDragging = false;
    seekToEvent(e);
    if (wasPlaying) getPlayer().play();
    document.body.style.userSelect = "";
  });

  // --- Attach progress listeners to both audio and video
  ['timeupdate', 'durationchange', 'ended'].forEach(evt => {
    audio.addEventListener(evt, updateProgressBar);
    video.addEventListener(evt, updateProgressBar);
  });

  // --- Handle track end for loop/shuffle ---
  audio.addEventListener('ended', onTrackEnded);
  video.addEventListener('ended', onTrackEnded);

	function onTrackEnded() {
	  if (loopEnabled) {
		getPlayer().currentTime = 0;
		getPlayer().play();
	  } else if (shuffleEnabled) {
		// If in history, go forward
		if (shuffleHistoryPointer < shuffleHistory.length - 1) {
		  selectMedia(shuffleHistory[shuffleHistoryPointer + 1], "next");
		} else {
		  // Pick a new one
		  let available = mediaFiles.map((_, i) => i !== currentMediaIndex ? i : null).filter(i => i !== null);
		  if (available.length === 0) return;
		  let next = available[Math.floor(Math.random() * available.length)];
		  selectMedia(next, true);
		}
	  } else {
		// Next track if available
		let nextIndex = currentMediaIndex + 1;
		if (nextIndex >= totalMediaFiles) nextIndex = 0;
		if (nextIndex < totalMediaFiles) {
		  if (nextIndex >= mediaFiles.length) {
			fetch(`/list_media_files?start=${mediaFiles.length}&count=${fileListPageSize}`)
			  .then(r => r.json())
			  .then(data => {
				if (Array.isArray(data.files)) {
				  mediaFiles = mediaFiles.concat(data.files);
				  totalMediaFiles = data.total;
				  renderMediaFileList();
				  if (nextIndex < mediaFiles.length) {
					selectMedia(nextIndex, true);
				  }
				}
			  });
		  } else {
			selectMedia(nextIndex, true);
		  }
		}
	  }
	}


  // --- Key bit: update highlight & play button animation on actual play/pause events
  audio.addEventListener('play', renderMediaFileList);
  audio.addEventListener('pause', renderMediaFileList);
  video.addEventListener('play', renderMediaFileList);
  video.addEventListener('pause', renderMediaFileList);

  // Optionally: update when switching tracks
  updateProgressBar();
}


function getPlayer() {
  const audio = document.getElementById('mediaPlayerAudio');
  const video = document.getElementById('mediaPlayerVideo');

  // If neither exists (modal not open), return null safely
  if (!audio && !video) return null;

  // If only one exists, return it
  if (audio && !video) return audio;
  if (!audio && video) return video;

  // Both exist -> choose visible one
  return (audio.style.display !== 'none') ? audio : video;
}




// Main render function: append only new files, unless clear=true

function renderMediaFileList(clear = false) {
  const list = document.getElementById('mediaFileList');
  if (!list) return;

  list.innerHTML = '';

  let foundAny = false;
  for (let idx = 0; idx < mediaFiles.length; idx++) {
    const fileObj = mediaFiles[idx];
    const file = typeof fileObj === "string" ? fileObj : fileObj.name;
    const ext = file.split('.').pop().toLowerCase();
    const isAudio = ['mp3', 'wav', 'ogg'].includes(ext);
    const isVideo = ['mp4', 'webm', 'mov'].includes(ext);

    if (!(isAudio || isVideo)) continue;
    foundAny = true;

    const el = document.createElement('div');
    el.className = "media-file-item";

    // Browser play button
    const playBtn = document.createElement('button');
    playBtn.textContent = isAudio ? '▶️' : '🎬';
    playBtn.title = "Play in browser";
    playBtn.className = "play-anim";
    if (idx === currentMediaIndex && getPlayer() && !getPlayer().paused) {
      playBtn.classList.add('playing');
    }
    playBtn.onclick = () => selectMedia(idx);

    // Device play button
    const devBtn = document.createElement('button');
    devBtn.textContent = '🔊';
    devBtn.title = "Play on ESP32";
    devBtn.className = "play-anim";
    if (idx === window.currentDeviceMediaIndex) {
      devBtn.classList.add('playing');
    }
    devBtn.onclick = () => {
      window.currentDeviceMediaIndex = idx;
      playOnDevice(file);
      renderMediaFileList(); // Update UI immediately
    };

    // File name
    const fileSpan = document.createElement('span');
    fileSpan.textContent = ' ' + file;
    fileSpan.style.cursor = "pointer";
    fileSpan.onclick = () => selectMedia(idx);

    el.appendChild(playBtn);
    el.appendChild(devBtn);
    el.appendChild(fileSpan);

    if (idx === currentMediaIndex || idx === window.currentDeviceMediaIndex) el.classList.add('active');
    list.appendChild(el);
  }

  if (!foundAny) {
    list.innerHTML = '<div style="color:gray;padding:1em;text-align:center;">No audio/video files found</div>';
  }

  // Update summary
  const summary = document.getElementById('mediaFileListSummary');
  if (summary) {
    summary.textContent = `Showing ${mediaFiles.length} of ${totalMediaFiles}`;
  }

  // Spinner logic
  const spinnerId = 'mediaFileListSpinner';
  let oldSpinner = document.getElementById(spinnerId);
  if (loadingFiles && mediaFiles.length < totalMediaFiles) {
    if (!oldSpinner) {
      let spinner = document.createElement('div');
      spinner.id = spinnerId;
      spinner.style = "text-align:center;color:#6cf;padding:10px;";
      spinner.innerHTML = "Loading more files...";
      list.appendChild(spinner);
    }
  } else if (oldSpinner) {
    oldSpinner.remove();
  }
}


function playOnDevice(file) {
  // Instantly update UI: turn off mic if active
  if (window.micEnabled) {
    window.micEnabled = false;
    const micBtn = document.getElementById("micBtn");
    if (micBtn) micBtn.classList.remove("active", "listening");
    const micIcon = document.getElementById("micIcon");
    if (micIcon) micIcon.style.color = "";
  }

  // Stop browser playback
  const audio = document.getElementById('mediaPlayerAudio');
  const video = document.getElementById('mediaPlayerVideo');
  if (audio) { audio.pause(); audio.currentTime = 0; }
  if (video) { video.pause(); video.currentTime = 0; }

  const useWebSocket = window.wsCarInput && window.wsCarInput.readyState === 1;
  window.mediaPlaybackMode = "device";

  // NO MORE: fetch('/disable_mic')

  if (useWebSocket) {
    window.wsCarInput.send("MEDIA_PLAY," + file);
    showToast("Requested ESP32 to play: " + file);
  } else {
    fetch('/play_on_device?file=' + encodeURIComponent(file))
      .then(r => r.text())
      .then(txt => showToast(txt));
  }

  if (window.mediaPlayerSetDevicePlayState)
    window.mediaPlayerSetDevicePlayState(file, true);
}







function selectMedia(idx, fromUser = false) {
  // Handle shuffle history management
  if (shuffleEnabled && fromUser !== "prev") {
    // If we're not at the end of history, cut off future (like browser nav)
    if (shuffleHistoryPointer < shuffleHistory.length - 1) {
      shuffleHistory = shuffleHistory.slice(0, shuffleHistoryPointer + 1);
    }
    shuffleHistory.push(idx);
    shuffleHistoryPointer = shuffleHistory.length - 1;
  } else if (!shuffleEnabled) {
    shuffleHistory = [];
    shuffleHistoryPointer = -1;
  }

  currentMediaIndex = idx;
  const fileObj = mediaFiles[idx];
  const file = typeof fileObj === "string" ? fileObj : fileObj.name;
  const ext = file.split('.').pop().toLowerCase();
  const audio = document.getElementById('mediaPlayerAudio');
  const video = document.getElementById('mediaPlayerVideo');

  audio.style.display = 'none';
  video.style.display = 'none';
  audio.pause(); video.pause();

  // ----- Playback handling -----
  if (['mp3','wav','ogg'].includes(ext)) {
    audio.src = file.startsWith('/media/') ? encodeURI(file) : '/media/' + encodeURI(file);
    audio.style.display = '';
    // Set volume from slider (range 0..1)
    const volSlider = document.getElementById('mediaVolume');
    if (volSlider) audio.volume = parseFloat(volSlider.value || "1");
	
	// If switching to browser playback, stop device playback first
	if (window.mediaPlaybackMode !== "browser" && window.wsCarInput && window.wsCarInput.readyState === 1) {
	  window.wsCarInput.send("MEDIA_STOP");
	}
	window.mediaPlaybackMode = "browser";
	
    audio.play();
    // Set playback mode: if device is also playing, mode = both
    if (window.mediaPlaybackMode === "device") {
      window.mediaPlaybackMode = "both";
    } else {
      window.mediaPlaybackMode = "browser";
    }
  } else if (['mp4','webm','mov'].includes(ext)) {
    video.src = '/media/' + encodeURIComponent(file);
    video.style.display = '';
    video.play();
    // For video, you may want to set playback mode as "browser" too:
    window.mediaPlaybackMode = "browser";
  }

  renderMediaFileList();

  // Scroll into view
  const list = document.getElementById('mediaFileList');
  const items = list.children;
  if (items[idx]) {
    const panel = document.getElementById('mediaFileListPanel');
    const el = items[idx];
    const offset = el.offsetTop - panel.clientHeight / 2 + el.offsetHeight / 2;
    panel.scrollTop = Math.max(0, offset);
  }
  updateSongTitleMarquee();
}



function playCurrentMedia() {
  if (currentMediaIndex === -1 && mediaFiles.length > 0) {
    selectMedia(0);
  } else {
    getPlayer().play();
  }
}

function updateSongTitleMarquee() {
  const titleDiv = document.getElementById('progressSongTitle');
  const wrap = document.getElementById('progressSongTitleWrap');
  if (!titleDiv || !wrap) return;
  let fileObj = mediaFiles[currentMediaIndex] || '';
  let file = fileObj && typeof fileObj === "object" ? fileObj.name : fileObj;
  if (!file) {
    titleDiv.innerHTML = '';
    titleDiv.style.animation = 'none';
    return;
  }
  let name = file.split('/').pop();

  // Add spaces between repeats for smooth loop (repeat once)
  let spacer = '&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'; // 5 spaces
  titleDiv.innerHTML = name + spacer + name + spacer; // at least two cycles

  // Wait until DOM renders, then animate if overflow
  setTimeout(() => {
    const wrapWidth = wrap.offsetWidth;
    const textWidth = titleDiv.scrollWidth / 2; // Single name+gap width

    if (textWidth > wrapWidth) {
      // The total distance is textWidth + gap
      let total = textWidth + 32; // Add small fudge for smoother reset
      titleDiv.style.setProperty('--scroll-total', `-${total}px`);
      let speed = Math.max(8, (textWidth / 90) * 8); // 8s per 600px approx
      titleDiv.style.animation = `marquee-loop ${speed}s linear infinite`;
    } else {
      titleDiv.style.animation = 'none';
      titleDiv.innerHTML = name; // Remove repeat if no overflow
    }
  }, 50);
}

// Show/hide modal using SD File Manager logic if available
function showMediaIndexingProgress(path, percent, count, total, indeterminate) {
    if (window.showIndexingProgress) {
        // Reuse SD card modal (will show under SD tab too, that's fine!)
        showIndexingProgress(path, percent, count, total, indeterminate);
    } else {
        // Fallback: basic modal (never needed if SD logic loaded)
        let modal = document.getElementById("mediaIndexProgress");
        if (!modal) {
            modal = document.createElement("div");
            modal.id = "mediaIndexProgress";
            modal.style = "position:fixed;top:40%;left:50%;transform:translate(-50%,-50%);z-index:3001;background:#223;padding:22px 44px;border-radius:18px;color:#fff;font-size:1.2em;text-align:center;";
            modal.innerHTML = "<span id='mediaIndexText'></span><div id='mediaIndexBar' style='height:14px;background:#333;border-radius:7px;margin:12px 0;width:100%;overflow:hidden;'><div id='mediaIndexFill' style='background:#6cf;width:0;height:14px;border-radius:7px;transition:width .2s;'></div></div>";
            document.body.appendChild(modal);
        }
        let txt = indeterminate ? "Indexing media files..." : `Indexing folder: ${path}`;
        document.getElementById("mediaIndexText").textContent = txt;
        document.getElementById("mediaIndexBar").style.display = indeterminate ? "none" : "block";
        document.getElementById("mediaIndexFill").style.width = indeterminate ? "0" : (percent + "%");
        modal.style.display = "block";
    }
}

function hideMediaIndexingProgress() {
    if (window.hideIndexingProgress) {
        hideIndexingProgress();
    } else {
        let modal = document.getElementById("mediaIndexProgress");
        if (modal) modal.style.display = "none";
    }
}

function pollMediaReindexStatus(path) {
    function poll(currentPath) {
        fetch("/sd_reindex_status")
            .then(r => r.json())
            .then(info => {
                if (!info || !info.path) return;
                if (info.pending) {
                    let indeterminate = (info.counting === true) || (!info.total || info.total === 0);
                    let percent = (info.total > 0) ? Math.floor(info.count * 100 / info.total) : 0;
                    showMediaIndexingProgress(info.path, percent, info.count, info.total, indeterminate);
                    setTimeout(() => poll(info.path), 900); // Always update to latest
                } else {
                    hideMediaIndexingProgress();
                    fetchFirstMediaFiles(); // Reload after indexing
                }
            })
            .catch(e => {
                hideMediaIndexingProgress();
                showToast("❌ Error polling media index: " + e, true);
            });
    }
    poll(path);
}

