// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-streamer', {
  schema: {
    roomName: { type: 'string', default: 'test_room' },
    participantIdentity: { type: 'string', default: 'vr-viewer' },
    authServerPort: { type: 'number', default: 5050 },
  },

  init: function () {
    console.log('Teleop video streamer component initialized.');
    console.log('User agent:', navigator.userAgent);
    console.log('Window location:', window.location.href);

    this.room = null;
    this.videoElement = null;

    // Create debug text display for VR
    this.createVrLogDisplay();
    this.logToVR('VR log display created');

    this.roomName = this.data.roomName;
    this.participantIdentity = this.data.participantIdentity;

    // Create video element for the stream
    this.createVideoElement();

    // Start async initialization
    this.initializeAsync();
  },

  initializeAsync: async function () {
    try {
      this.logToVR('Starting initialization...');

      // Wait for LiveKit to be available
      await this.waitForLiveKit();

      // Get token and connect
      await this.connectToRoom();
    } catch (error) {
      this.logToVR('ERROR: Failed to initialize - ' + error.message);
      console.error('Failed to initialize video streamer:', error);
    }
  },
  waitForLiveKit: function () {
    return new Promise((resolve, reject) => {
      let attempts = 0;
      const maxAttempts = 50; // 5 seconds max

      const checkLiveKit = () => {
        attempts++;

        // Try different ways LiveKit might be available
        const liveKit =
          window.LiveKitClient ||
          window.LivekitClient || // Note: lowercase 'k'
          window.LiveKit ||
          (window.livekit && window.livekit.LiveKitClient);

        if (liveKit) {
          console.log('LiveKit client is available:', liveKit);
          window.LiveKitClient = liveKit; // Ensure it's available as LiveKitClient
          resolve();
        } else if (attempts >= maxAttempts) {
          console.error(
            'LiveKit client failed to load after',
            attempts,
            'attempts',
          );
          console.log(
            'Available window properties:',
            Object.keys(window).filter((k) => k.toLowerCase().includes('live')),
          );
          reject(new Error('LiveKit client not available'));
        } else {
          console.log(
            `Waiting for LiveKit client... (attempt ${attempts}/${maxAttempts})`,
          );
          setTimeout(checkLiveKit, 100);
        }
      };
      checkLiveKit();
    });
  },

  createVideoElement: function () {
    // Create video element to display the stream
    this.videoElement = document.createElement('video');
    this.videoElement.autoplay = true;
    this.videoElement.playsInline = true;
    this.videoElement.muted = true; // Required for autoplay in many browsers
    this.videoElement.style.display = 'none'; // Hide the actual video element
    this.videoElement.id = 'livekit-video-' + Date.now();

    // Add to document body (not A-Frame entity)
    document.body.appendChild(this.videoElement);

    // Set up video texture for A-Frame
    this.setupVideoTexture();
  },

  createVrLogDisplay: function () {
    // Create a text element to show debug info in VR
    const vrLogEntity = document.createElement('a-text');
    vrLogEntity.id = 'vr-log';
    vrLogEntity.setAttribute('value', 'Debug Log:\n');
    vrLogEntity.setAttribute('position', '0 2 -2');
    vrLogEntity.setAttribute('align', 'center');
    vrLogEntity.setAttribute('color', '#00ff00');
    vrLogEntity.setAttribute('scale', '0.3 0.3 0.3');
    document.querySelector('a-scene').appendChild(vrLogEntity);
  },

  logToVR: function (message) {
    const vrLogEntity = document.querySelector('#vr-log');
    if (vrLogEntity) {
      const currentValue = vrLogEntity.getAttribute('value');
      const lines = currentValue.split('\n');
      lines.push(`${new Date().toLocaleTimeString()}: ${message}`);
      // Keep only last 10 lines
      if (lines.length > 11) lines.splice(1, 1);
      vrLogEntity.setAttribute('value', lines.join('\n'));
    }
    console.log(message); // Also log normally
  },

  setupVideoTexture: function () {
    // Wait for video to be ready then apply as texture
    this.videoElement.addEventListener('loadeddata', () => {
      this.logToVR('Video data loaded, applying texture');
      this.applyVideoTexture();
    });

    this.videoElement.addEventListener('playing', () => {
      this.logToVR('Video is playing');
      this.applyVideoTexture();
    });
  },

  applyVideoTexture: function () {
    // Apply video as texture to the A-Frame plane
    const dimensions = `${this.videoElement.videoWidth}x${this.videoElement.videoHeight}`;
    this.logToVR(
      `Applying texture: ${dimensions}, ready: ${this.videoElement.readyState}, paused: ${this.videoElement.paused}`,
    );

    this.el.setAttribute('material', {
      shader: 'flat',
      src: `#${this.videoElement.id}`,
      transparent: false,
    });

    // Add visual feedback for debugging
    if (
      this.videoElement.videoWidth === 0 ||
      this.videoElement.videoHeight === 0
    ) {
      this.logToVR('WARNING: Video has no dimensions!');
      // Set a temporary red color to indicate issues
      this.el.setAttribute('material', { color: 'red' });
    } else {
      this.logToVR('Video texture applied successfully');
    }
  },

  connectToRoom: async function () {
    try {
      // Get token and LiveKit URL from Python backend
      const response = await this.getToken();
      const { token, livekit_url } = response;

      this.room = new window.LiveKitClient.Room();
      this.logToVR(`Created LiveKit room`);

      // Set up event listeners
      this.room
        .on(
          window.LiveKitClient.RoomEvent.TrackSubscribed,
          this.handleTrackSubscribed.bind(this),
        )
        .on(
          window.LiveKitClient.RoomEvent.TrackUnsubscribed,
          this.handleTrackUnsubscribed.bind(this),
        )
        .on(
          window.LiveKitClient.RoomEvent.Disconnected,
          this.handleDisconnect.bind(this),
        )
        .on(
          window.LiveKitClient.RoomEvent.Connected,
          this.handleConnected.bind(this),
        );

      // Connect to the room using the URL from the API
      this.logToVR(
        `Connecting to room with livekit url: ${livekit_url} and token: ${token} (room name: ${this.roomName})`,
      );
      await this.room.connect(livekit_url, token);
      this.logToVR(`Connected to room: ${this.roomName}`);
    } catch (error) {
      this.logToVR(
        `ERROR: Failed to connect to LiveKit room ${this.roomName} - ${error.message}`,
      );
      console.error('Failed to connect to LiveKit room:', error);
    }
  },

  getToken: async function () {
    try {
      const isVR =
        navigator.xr &&
        (await navigator.xr
          .isSessionSupported('immersive-vr')
          .catch(() => false));
      if (!isVR) {
        throw new Error(
          'VR session not detected. Camera streaming is only supported in the VR experience.',
        );
      }

      const authUrl = `https://${window.location.hostname}:${this.data.authServerPort}/api/subscriber-token`;
      this.logToVR(`Getting token from: ${authUrl} (VR: ${isVR})`);

      const response = await fetch(authUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          room_name: this.roomName,
          participant_identity: this.participantIdentity,
        }),
      });

      if (!response.ok) {
        const errorText = await response.text().catch(() => 'Unknown error');
        throw new Error(
          `HTTP error! status: ${response.status}, body: ${errorText}`,
        );
      }

      const data = await response.json();
      return data;
    } catch (error) {
      this.logToVR(`ERROR: Token request failed - ${error.message}`);
      console.error('Failed to get token:', error);
      throw error;
    }
  },

  handleTrackSubscribed: function (track, publication, participant) {
    this.logToVR(
      `Track subscribed: ${track.kind} from ${participant.identity}`,
    );

    if (track.kind === 'video') {
      // Attach video track to our video element
      track.attach(this.videoElement);
      this.logToVR('Video track attached');

      // Force video to play and apply texture
      setTimeout(() => {
        this.videoElement
          .play()
          .then(() => {
            this.logToVR('Video playback started');
            this.applyVideoTexture();
          })
          .catch((err) => {
            this.logToVR('ERROR: Video playback failed - ' + err.message);
          });
      }, 100);
    }
  },

  handleTrackUnsubscribed: function (track, publication, participant) {
    console.log(
      'Track unsubscribed:',
      track.kind,
      'from participant:',
      participant.identity,
    );

    if (track.kind === 'video') {
      // Detach video track
      track.detach();
    }
  },

  handleConnected: function () {
    this.logToVR('Connected to LiveKit room');
  },

  handleDisconnect: function () {
    this.logToVR('Disconnected from LiveKit room');
  },

  remove: function () {
    // Clean up LiveKit room connection
    if (this.room) {
      this.room.disconnect();
      this.room = null;
    }

    // Clean up video element
    if (this.videoElement) {
      // Remove video element from DOM
      if (this.videoElement.parentNode) {
        this.videoElement.parentNode.removeChild(this.videoElement);
      }
      this.videoElement = null;
    }
  },
});
