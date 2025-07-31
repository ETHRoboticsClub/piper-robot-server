import {Room, LocalVideoTrack, LocalAudioTrack, createLocalVideoTrack, createLocalAudioTrack} from 'livekit-client';  

// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-streamer', {
  schema: {
    roomName: { type: 'string', default: 'test_room' },
    participantIdentity: { type: 'string', default: '' },
    authServerPort: { type: 'number', default: 5050 },
  },

  init: function () {
    console.log('Teleop video streamer component initialized.');
    console.log('User agent:', navigator.userAgent);
    console.log('Window location:', window.location.href);

    this.room = null;
    this.videoElement = null;

    // Create debug text display for VR
    this.createDebugDisplay();

    this.roomName = this.data.roomName;
    this.participantIdentity =
      this.data.participantIdentity || `viewer-${Date.now()}`;

    // Create video element for the stream
    this.createVideoElement();

    // Start async initialization
    this.initializeAsync();
  },

  initializeAsync: async function () {
    try {
      this.addDebugMessage('Starting initialization...');

      // Wait for LiveKit to be available
      await this.waitForLiveKit();
      this.addDebugMessage('LiveKit loaded successfully');

      // Get token and connect
      await this.connectToRoom();
    } catch (error) {
      this.addDebugMessage('ERROR: Failed to initialize - ' + error.message);
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

  createDebugDisplay: function () {
    // Create a text element to show debug info in VR
    this.debugText = document.createElement('a-text');
    this.debugText.setAttribute('position', '0 0.5 0');
    this.debugText.setAttribute('align', 'center');
    this.debugText.setAttribute('color', '#00ff00');
    this.debugText.setAttribute('scale', '0.5 0.5 0.5');
    this.debugText.setAttribute(
      'value',
      'Video Stream Debug:\nInitializing...',
    );
    this.el.appendChild(this.debugText);

    this.debugMessages = [];
    this.maxDebugMessages = 8;
  },

  addDebugMessage: function (message) {
    console.log(message); // Still log to console

    // Add to debug messages array
    this.debugMessages.push(new Date().toLocaleTimeString() + ': ' + message);

    // Keep only the last N messages
    if (this.debugMessages.length > this.maxDebugMessages) {
      this.debugMessages.shift();
    }

    // Update the VR text display
    if (this.debugText) {
      this.debugText.setAttribute(
        'value',
        'Video Stream Debug:\n' + this.debugMessages.join('\n'),
      );
    }
  },

  setupVideoTexture: function () {
    // Wait for video to be ready then apply as texture
    this.videoElement.addEventListener('loadeddata', () => {
      this.addDebugMessage('Video data loaded, applying texture');
      this.applyVideoTexture();
    });

    this.videoElement.addEventListener('playing', () => {
      this.addDebugMessage('Video is playing');
      this.applyVideoTexture();
    });
  },

  applyVideoTexture: function () {
    // Apply video as texture to the A-Frame plane
    const dimensions = `${this.videoElement.videoWidth}x${this.videoElement.videoHeight}`;
    this.addDebugMessage(
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
      this.addDebugMessage('WARNING: Video has no dimensions!');
      // Set a temporary red color to indicate issues
      this.el.setAttribute('material', { color: 'red' });
    } else {
      this.addDebugMessage('Video texture applied successfully');
    }
  },

  connectToRoom: async function () {
    try {
      // Get token and LiveKit URL from Python backend
      const response = await this.getToken();
      const { token, livekit_url } = response;

      this.room = new LiveKitClient.Room();
      this.addDebugMessage(`Connecting to LiveKit room: ${this.roomName}`);

      // Set up event listeners
      this.room
        .on(
          LiveKitClient.RoomEvent.TrackSubscribed,
          this.handleTrackSubscribed.bind(this),
        )
        .on(
          LiveKitClient.RoomEvent.TrackUnsubscribed,
          this.handleTrackUnsubscribed.bind(this),
        )
        .on(
          LiveKitClient.RoomEvent.Disconnected,
          this.handleDisconnect.bind(this),
        )
        .on(LiveKitClient.RoomEvent.Connected, this.handleConnected.bind(this));

      // Connect to the room using the URL from the API
      await this.room.connect(livekit_url, token);
      this.addDebugMessage(`Connected to room: ${this.roomName}`);
    } catch (error) {
      this.addDebugMessage(`ERROR: Failed to connect to LiveKit room ${this.roomName} - ${error.message}`);
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

      let hostname = 'localhost'; // default for non-VR
      if (isVR) {
        hostname = window.location.hostname;
        this.addDebugMessage(`VR detected, using hostname: ${hostname}`);
      } else {
        this.addDebugMessage(`Non-VR mode, using hostname: ${hostname}`);
      }

      const authUrl = `https://${hostname}:${this.data.authServerPort}/api/subscriber-token`;
      this.addDebugMessage(`Getting token from: ${authUrl} (VR: ${isVR})`);

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
      this.addDebugMessage(`ERROR: Token request failed - ${error.message}`);
      console.error('Failed to get token:', error);
      throw error;
    }
  },

  handleTrackSubscribed: function (track, publication, participant) {
    this.addDebugMessage(
      `Track subscribed: ${track.kind} from ${participant.identity}`,
    );

    if (track.kind === 'video') {
      // Attach video track to our video element
      track.attach(this.videoElement);
      this.addDebugMessage('Video track attached');

      // Force video to play and apply texture
      setTimeout(() => {
        this.videoElement
          .play()
          .then(() => {
            this.addDebugMessage('Video playback started');
            this.applyVideoTexture();
          })
          .catch((err) => {
            this.addDebugMessage(
              'ERROR: Video playback failed - ' + err.message,
            );
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
    this.addDebugMessage('Connected to LiveKit room');
  },

  handleDisconnect: function () {
    this.addDebugMessage('Disconnected from LiveKit room');
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
