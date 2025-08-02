// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-stream', {
  init: function () {
    console.log('🎥 Teleop video streamer component initialized.');
    console.log('User agent:', navigator.userAgent);
    console.log('Window location:', window.location.href);

    this.videoElement = null;

    // Create video element for the stream
    this.createVideoElement();

    // Load config and initialize asynchronously
    this.initializeAsyncWithConfig();
  },

  initializeAsyncWithConfig: async function () {
    // defaults, if loading from backend fails
    let roomName = 'robot-vr-teleop-room';
    let participantIdentity = 'vr-viewer';
    let debug = false;

    try {
      console.log('🔧 Loading LiveKit config from backend...');
      const config = await window.LiveKitUtils.loadLiveKitConfig();
      roomName = config.livekit_room;
      participantIdentity = config.vr_viewer_participant;
      debug = config.vr_viewer_debug;
      console.log('✅ Loaded LiveKit config from backend:', config);
      console.log('🏠 (for video stream) Using roomName:', roomName);
      console.log(
        '👤 (for video stream) Using participantIdentity:',
        participantIdentity,
      );
      console.log('🐛 (for video stream) Using debug:', debug);
    } catch (error) {
      console.warn('⚠️ Failed to load LiveKit config, using defaults:', error);
    }

    // Create debug text display for VR
    if (debug) {
      window.LiveKitUtils.createVrLogDisplay();
      window.LiveKitUtils.logToVR('VR log display created');
    }

    // connect to livekit room
    console.log('🔌 Attempting to connect to LiveKit room...');
    window.LiveKitUtils.asyncRoomConnect(this, roomName, participantIdentity);
  },

  // --- LiveKit Event Handlers ---
  handleTrackSubscribed: function (track, publication, participant) {
    window.LiveKitUtils.logToVR(
      `Track subscribed: ${track.kind} from ${participant.identity}`,
    );

    if (track.kind === 'video') {
      track.attach(this.videoElement);
      window.LiveKitUtils.logToVR('Attached video track to video element');

      // Force video to play and apply texture
      setTimeout(() => {
        this.videoElement
          .play()
          .then(() => {
            window.LiveKitUtils.logToVR('Video playback started');
            this.applyVideoTexture();
          })
          .catch((err) => {
            window.LiveKitUtils.logToVR(
              'ERROR: Video playback failed - ' + err.message,
            );
          });
      }, 100);
    }
  },

  handleTrackUnsubscribed: function (track, publication, participant) {
    window.LiveKitUtils.logToVR(
      'Track unsubscribed:',
      track.kind,
      'from participant:',
      participant.identity,
    );

    if (track.kind === 'video') {
      track.detach();
    }
  },

  handleConnected: function () {
    window.LiveKitUtils.logToVR('Connected to LiveKit room');

    // Log current room state
    const room = window.LiveKitUtils.room;
    if (room) {
      window.LiveKitUtils.logToVR(
        `Local participant: ${room.localParticipant.identity}`,
      );
      window.LiveKitUtils.logToVR(
        `Remote participants: ${room.remoteParticipants.size}`,
      );

      // Check for existing participants and their tracks (like camera-streamer)
      room.remoteParticipants.forEach((participant) => {
        window.LiveKitUtils.logToVR(
          `Found existing participant: ${participant.identity}`,
        );
        participant.trackPublications.forEach((publication) => {
          window.LiveKitUtils.logToVR(
            `  - Track: ${publication.kind} (${
              publication.source || 'unknown'
            })`,
          );
          if (publication.isSubscribed && publication.track) {
            window.LiveKitUtils.logToVR(`    Already subscribed, attaching...`);
            if (publication.track.kind === 'video') {
              this.handleTrackSubscribed(
                publication.track,
                publication,
                participant,
              );
            }
          }
        });
      });
    }
  },

  handleDisconnect: function () {
    window.LiveKitUtils.logToVR('Disconnected from LiveKit room');
  },
  // --- End LiveKit Event Handlers ---

  remove: function () {
    // Clean up video element
    if (this.videoElement) {
      if (this.videoElement.parentNode) {
        this.videoElement.parentNode.removeChild(this.videoElement);
      }
      this.videoElement = null;
    }
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
      window.LiveKitUtils.logToVR('Video data loaded, applying texture');
      this.applyVideoTexture();
    });

    this.videoElement.addEventListener('playing', () => {
      window.LiveKitUtils.logToVR('Video is playing');
      this.applyVideoTexture();
    });
  },

  applyVideoTexture: function () {
    // Apply video as texture to the A-Frame plane
    const dimensions = `${this.videoElement.videoWidth}x${this.videoElement.videoHeight}`;
    window.LiveKitUtils.logToVR(
      `Applying texture: ${dimensions}, ready: ${this.videoElement.readyState}, paused: ${this.videoElement.paused}`,
    );

    // Add visual feedback for debugging
    if (
      this.videoElement.videoWidth === 0 ||
      this.videoElement.videoHeight === 0
    ) {
      window.LiveKitUtils.logToVR(
        'WARNING: Video has no dimensions! Setting red placeholder.',
      );
      // Set a temporary red color to indicate issues
      this.el.setAttribute('material', {
        shader: 'flat',
        color: 'red',
        transparent: false,
      });
    } else {
      window.LiveKitUtils.logToVR('Video texture applied successfully');
      this.el.setAttribute('material', {
        shader: 'flat',
        src: `#${this.videoElement.id}`,
        transparent: false,
      });
    }
  },
});
