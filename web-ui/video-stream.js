// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-streamer', {
  schema: {
    roomName: { type: 'string', default: 'test_room' },
    participantIdentity: { type: 'string', default: '' },
    authServerPort: { type: 'number', default: 5000 },
  },

  init: async function () {
    console.log('Teleop video streamer component initialized.');

    this.room = null;
    this.videoElement = null;

    // Get room name from component attributes or use default from streamer.py
    this.roomName = this.data.roomName;
    this.participantIdentity =
      this.data.participantIdentity || `viewer-${Date.now()}`;

    // Create video element for the stream
    this.createVideoElement();

    // Wait for LiveKit to be available
    await this.waitForLiveKit();

    // Get token and connect
    await this.connectToRoom();
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

  setupVideoTexture: function () {
    // Wait for video to be ready then apply as texture
    this.videoElement.addEventListener('loadeddata', () => {
      console.log('Video data loaded, applying texture');
      this.applyVideoTexture();
    });

    this.videoElement.addEventListener('playing', () => {
      console.log('Video is playing');
      this.applyVideoTexture();
    });
  },

  applyVideoTexture: function () {
    // Apply video as texture to the A-Frame plane
    this.el.setAttribute('material', {
      shader: 'flat',
      src: `#${this.videoElement.id}`,
      transparent: false,
    });
    console.log('Applied video texture to A-Frame plane');
  },

  connectToRoom: async function () {
    try {
      // Get token and LiveKit URL from Python backend
      const response = await this.getToken();
      const { token, livekit_url } = response;

      this.room = new LiveKitClient.Room();

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
      console.log('Connected to LiveKit room:', this.roomName);
    } catch (error) {
      console.error('Failed to connect to LiveKit room:', error);
    }
  },

  getToken: async function () {
    try {
      const authUrl = `http://localhost:${this.data.authServerPort}/api/token`;
      console.log('Requesting token from:', authUrl);

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
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      return data;
    } catch (error) {
      console.error('Failed to get token:', error);
      throw error;
    }
  },

  handleTrackSubscribed: function (track, publication, participant) {
    console.log(
      'Track subscribed:',
      track.kind,
      'from participant:',
      participant.identity,
    );

    if (track.kind === 'video') {
      // Attach video track to our video element
      track.attach(this.videoElement);
      console.log('Video track attached to display element');

      // Force video to play and apply texture
      setTimeout(() => {
        this.videoElement
          .play()
          .then(() => {
            console.log('Video playback started');
            this.applyVideoTexture();
          })
          .catch((err) => {
            console.error('Failed to start video playback:', err);
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
    console.log('Connected to LiveKit room');
  },

  handleDisconnect: function () {
    console.log('Disconnected from LiveKit room');
  },

  remove: function () {
    if (this.room) {
      this.room.disconnect();
    }
  },
});
