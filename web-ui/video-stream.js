// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-streamer', {
  schema: {
    participantIdentity: { type: 'string', default: 'vr-teleop-viewer' },
    authServerPort: { type: 'number', default: 5050 },
    debug: { type: 'boolean', default: false },
  },

  init: function () {
    console.log('Teleop video streamer component initialized.');
    console.log('User agent:', navigator.userAgent);
    console.log('Window location:', window.location.href);

    this.room = null;
    this.videoElement = null;

    // Create debug text display for VR
    if (this.data.debug) {
      window.LiveKitUtils.createVrLogDisplay();
      window.LiveKitUtils.logToVR('VR log display created');
    }

    // Create video element for the stream
    this.createVideoElement();

    // Start async initialization
    window.LiveKitUtils.asyncRoomConnect(this.data.participantIdentity);
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
      window.LiveKitUtils.logToVR('WARNING: Video has no dimensions!');
      // Set a temporary red color to indicate issues
      this.el.setAttribute('material', { color: 'red' });
    } else {
      window.LiveKitUtils.logToVR('Video texture applied successfully');
    }
  },
});
