// This component receives the video stream from the Python camera streamer
AFRAME.registerComponent('teleop-video-stream', {
  init: function () {
    console.log('🎥 Teleop video streamer component initialized.');
    console.log('User agent:', navigator.userAgent);
    console.log('Window location:', window.location.href);

    this.videoElement = null;
   
    // Load config and initialize asynchronously
    this.initializeAsyncWithConfig();

    // Create video element for the stream
    this.createVideoElement();
  },

  initializeAsyncWithConfig: async function () {
    // defaults, if loading from backend fails
    let roomName = 'robot-vr-teleop-room';
    let participantIdentity = 'vr-viewer';

    try {
      console.log('🔧 Loading LiveKit config from backend...');
      const config = await window.LiveKitUtils.loadLiveKitConfig();
      roomName = config.livekit_room;
      participantIdentity = config.vr_viewer_participant;
      console.log('✅ Loaded LiveKit config from backend:', config);
      console.log('🏠 (for video stream) Using roomName:', roomName);
      console.log(
        '👤 (for video stream) Using participantIdentity:',
        participantIdentity,
      );
    } catch (error) {
      console.warn('⚠️ Failed to load LiveKit config, using defaults:', error);
    }
    // connect to livekit room
    console.log('🔌 Attempting to connect to LiveKit room...');
    window.LiveKitUtils.asyncRoomConnect(this, roomName, participantIdentity);
  },

  
  createVideoElement: function () {
    // Create video element to display the stream
    this.videoElement = document.createElement('video');
    this.videoElement.autoplay = true;
    this.videoElement.playsInline = true;
    this.videoElement.muted = true;
    this.videoElement.style.display = 'none';
    this.videoElement.id = 'calibrated-livekit-video-' + Date.now();
    
    // Add to document body
    document.body.appendChild(this.videoElement);
    
    // Create canvas elements for split video
    this.createSplitCanvases();
    
    // Set up video texture for A-Frame
    this.setupVideoTexture();
  },

  createSplitCanvases: function () {
    // Create canvas elements for left and right video halves
    this.leftCanvas = document.createElement('canvas');
    this.rightCanvas = document.createElement('canvas');
    
    this.leftCanvas.id = 'calibrated-left-video-canvas-' + Date.now();
    this.rightCanvas.id = 'calibrated-right-video-canvas-' + Date.now();
    
    this.leftCanvas.style.display = 'none';
    this.rightCanvas.style.display = 'none';
    
    // Add to document body
    document.body.appendChild(this.leftCanvas);
    document.body.appendChild(this.rightCanvas);
    
    // Get canvas contexts
    this.leftCtx = this.leftCanvas.getContext('2d');
    this.rightCtx = this.rightCanvas.getContext('2d');
    
    this.isRendering = false;
  },

  setupVideoTexture: function () {
    this.videoElement.addEventListener('loadeddata', () => {
      this.setupCanvasDimensions();
      this.setupStereoPlanes();
      this.applyVideoTexture();
    });
    
    this.videoElement.addEventListener('playing', () => {
      this.setupCanvasDimensions();
      this.setupStereoPlanes();
      this.applyVideoTexture();
      this.startSplitRendering();
    });
  },
  
  setupCanvasDimensions: function () {
    if (this.videoElement.videoWidth > 0 && this.videoElement.videoHeight > 0) {
      const halfWidth = this.videoElement.videoWidth / 2;
      const height = this.videoElement.videoHeight;
      
      this.leftCanvas.width = halfWidth;
      this.leftCanvas.height = height;
      this.rightCanvas.width = halfWidth;
      this.rightCanvas.height = height;
      
      console.log(`Canvas dimensions: ${halfWidth}x${height} per eye`);
    }
  },
  
  setupStereoPlanes: function () {
    // Get or create stereo plane elements with proper positioning
    let leftVideoPlane = document.querySelector('#leftVideoPlane');
    let rightVideoPlane = document.querySelector('#rightVideoPlane');
    
    // Calculate dimensions based on actual video aspect ratio
    let planeWidth = 3.2;  // Default width in VR units
    let planeHeight = 2.4; // Default height in VR units
    
    if (this.videoElement.videoWidth > 0 && this.videoElement.videoHeight > 0) {
      const halfWidth = this.videoElement.videoWidth / 2;
      const height = this.videoElement.videoHeight;
      const aspectRatio = halfWidth / height;
      
      // Maintain consistent height, adjust width based on aspect ratio
      planeHeight = 1.5;
      planeWidth = planeHeight * aspectRatio;
      
      console.log(`Video aspect ratio: ${aspectRatio.toFixed(3)}:1 (${halfWidth}x${height} per eye)`);
      console.log(`VR plane dimensions: ${planeWidth.toFixed(2)}x${planeHeight.toFixed(2)}`);
    }
    
    if (!leftVideoPlane) {
      leftVideoPlane = document.createElement('a-plane');
      leftVideoPlane.id = 'leftVideoPlane';
      leftVideoPlane.setAttribute('position', `0 0 -2`);
      leftVideoPlane.setAttribute('rotation', '0 0 0');
      leftVideoPlane.setAttribute('width', planeWidth.toString());
      leftVideoPlane.setAttribute('height', planeHeight.toString());
      leftVideoPlane.setAttribute('stereo', 'eye: left');
      document.querySelector('a-scene').appendChild(leftVideoPlane);
    } else {
      // Update existing plane dimensions
      leftVideoPlane.setAttribute('width', planeWidth.toString());
      leftVideoPlane.setAttribute('height', planeHeight.toString());
    }
    
    if (!rightVideoPlane) {
      rightVideoPlane = document.createElement('a-plane');
      rightVideoPlane.id = 'rightVideoPlane';
      rightVideoPlane.setAttribute('position', `0 0 -2`);
      rightVideoPlane.setAttribute('rotation', '0 0 0');
      rightVideoPlane.setAttribute('width', planeWidth.toString());
      rightVideoPlane.setAttribute('height', planeHeight.toString());
      rightVideoPlane.setAttribute('stereo', 'eye: right');
      document.querySelector('a-scene').appendChild(rightVideoPlane);
    } else {
      // Update existing plane dimensions
      rightVideoPlane.setAttribute('width', planeWidth.toString());
      rightVideoPlane.setAttribute('height', planeHeight.toString());
    }
    
    console.log(`Stereo planes positioned with correct aspect ratio for cropped video`);
  },
  
  startSplitRendering: function () {
    if (this.isRendering) {
      return;
    }
    
    this.isRendering = true;
    
    // Get references to the planes for texture updates
    const leftVideoPlane = document.querySelector('#leftVideoPlane');
    const rightVideoPlane = document.querySelector('#rightVideoPlane');
    
    const renderFrame = () => {
      if (!this.isRendering || this.videoElement.paused || this.videoElement.ended) {
        if (this.vrRenderTimeout) {
          clearTimeout(this.vrRenderTimeout);
          this.vrRenderTimeout = null;
        }
        return;
      }
      
      if (this.videoElement.videoWidth > 0 && this.videoElement.videoHeight > 0) {
        const halfWidth = this.videoElement.videoWidth / 2;
        const height = this.videoElement.videoHeight;
        
        try {
          // Draw left half of video to left canvas (already rectified by Python)
          this.leftCtx.drawImage(
            this.videoElement,
            0, 0, halfWidth, height,
            0, 0, halfWidth, height
          );
          
          // Draw right half of video to right canvas (already rectified by Python)
          this.rightCtx.drawImage(
            this.videoElement,
            halfWidth, 0, halfWidth, height,
            0, 0, halfWidth, height
          );
          
          // Force A-Frame to update the textures
          this.updateAFrameTextures(leftVideoPlane, rightVideoPlane);
          
        } catch (error) {
          console.error('Canvas drawing error:', error);
        }
      }
      
      // Use setTimeout for consistent rendering in both VR and desktop
      try {
        this.vrRenderTimeout = setTimeout(renderFrame, 16); // ~60fps
      } catch (error) {
        console.error('setTimeout failed:', error);
      }
    };
    
    renderFrame();
  },
  
  updateAFrameTextures: function (leftPlane, rightPlane) {
    try {
      if (leftPlane && leftPlane.getObject3D('mesh')) {
        const leftMaterial = leftPlane.getObject3D('mesh').material;
        if (leftMaterial && leftMaterial.map) {
          leftMaterial.map.needsUpdate = true;
        }
      }
      
      if (rightPlane && rightPlane.getObject3D('mesh')) {
        const rightMaterial = rightPlane.getObject3D('mesh').material;
        if (rightMaterial && rightMaterial.map) {
          rightMaterial.map.needsUpdate = true;
        }
      }
    } catch (error) {
      console.error('Texture update error:', error);
    }
  },
  
  applyVideoTexture: function () {
    const leftVideoPlane = document.querySelector('#leftVideoPlane');
    const rightVideoPlane = document.querySelector('#rightVideoPlane');
    
    if (!leftVideoPlane || !rightVideoPlane) {
      console.error('Could not find video planes');
      return;
    }
    
    if (this.videoElement.videoWidth === 0 || this.videoElement.videoHeight === 0) {
      // Set fallback colors to indicate issues
      leftVideoPlane.setAttribute('material', { color: 'red' });
      rightVideoPlane.setAttribute('material', { color: 'blue' });
      console.warn('Video dimensions are zero, showing fallback colors');
    } else {
      // Apply left canvas to left plane
      leftVideoPlane.setAttribute('material', {
        shader: 'flat',
        src: `#${this.leftCanvas.id}`,
        transparent: false,
      });
      
      // Apply right canvas to right plane  
      rightVideoPlane.setAttribute('material', {
        shader: 'flat',
        src: `#${this.rightCanvas.id}`,
        transparent: false,
      });
      
      console.log('Applied calibrated stereo textures to VR planes');
    }
  },
  
  connectToRoom: async function () {
    try {
      const response = await this.getToken();
      const { token, livekit_url } = response;
      
      this.room = new window.LiveKitClient.Room();
      
      // Set up event listeners
      this.room
      .on(window.LiveKitClient.RoomEvent.TrackSubscribed, this.handleTrackSubscribed.bind(this))
      .on(window.LiveKitClient.RoomEvent.TrackUnsubscribed, this.handleTrackUnsubscribed.bind(this))
      .on(window.LiveKitClient.RoomEvent.Disconnected, this.handleDisconnect.bind(this))
      .on(window.LiveKitClient.RoomEvent.Connected, this.handleConnected.bind(this));
      
      await this.room.connect(livekit_url, token);
      console.log('Connected to LiveKit room for calibrated stereo streaming');
    } catch (error) {
      console.error('Failed to connect to room:', error);
    }
  },
  
  getToken: async function () {
    try {
      const isVR = navigator.xr && (await navigator.xr.isSessionSupported('immersive-vr').catch(() => false));
      if (!isVR) {
        throw new Error('VR session not detected. Calibrated camera streaming is only supported in VR.');
      }
      
      const authUrl = `https://${window.location.hostname}:${this.data.authServerPort}/api/subscriber-token`;
      
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
        throw new Error(`HTTP error! status: ${response.status}, body: ${errorText}`);
      }
      
      return await response.json();
    } catch (error) {
      console.error('Token request failed:', error);
      throw error;
    }
  },
  
  handleTrackSubscribed: function (track, publication, participant) {
    if (track.kind === 'video') {
      track.attach(this.videoElement);
      console.log('Subscribed to calibrated stereo video track');
      
      // Add track-level event listeners
      track.on('muted', () => {
        console.warn('Calibrated video track muted');
      });
      
      track.on('unmuted', () => {
        console.log('Calibrated video track unmuted');
      });
      
      track.on('ended', () => {
        console.log('Calibrated video track ended');
      });
      
      // Force video to play and apply texture
      setTimeout(() => {
        this.videoElement
        .play()
        .then(() => {
          this.setupCanvasDimensions();
          this.setupStereoPlanes();
          this.applyVideoTexture();
          this.startSplitRendering();
          console.log('Started calibrated stereo video playback');
        })
        .catch((err) => {
          console.error('Calibrated video playback failed:', err);
        });
      }, 100);
    }
  },
  
  handleTrackUnsubscribed: function (track, publication, participant) {
    if (track.kind === 'video') {
      track.detach();
      console.log('Unsubscribed from calibrated video track');
    }
  },
  
  handleConnected: function () {
    console.log('Connected to LiveKit room for calibrated stereo viewing');
  },

  handleDisconnect: function () {
    console.log('Disconnected from LiveKit room');
  },

  remove: function () {
    // Stop rendering loop
    this.isRendering = false;

    // Clean up VR render timeouts
    if (this.vrRenderTimeout) {
      clearTimeout(this.vrRenderTimeout);
      this.vrRenderTimeout = null;
    }

    // Clean up LiveKit room connection
    if (this.room) {
      this.room.disconnect();
      this.room = null;
    }

    // Clean up video element
    if (this.videoElement && this.videoElement.parentNode) {
      this.videoElement.parentNode.removeChild(this.videoElement);
      this.videoElement = null;
    }

    // Clean up canvas elements
    if (this.leftCanvas && this.leftCanvas.parentNode) {
      this.leftCanvas.parentNode.removeChild(this.leftCanvas);
      this.leftCanvas = null;
      this.leftCtx = null;
    }

    if (this.rightCanvas && this.rightCanvas.parentNode) {
      this.rightCanvas.parentNode.removeChild(this.rightCanvas);
      this.rightCanvas = null;
      this.rightCtx = null;
    }

    console.log('Cleaned up calibrated stereo video streamer');
  },
});

// Enhanced stereo component that understands eye layers
AFRAME.registerComponent('stereo', {
  schema: {
    eye: { type: 'string', default: "left"},
  },

  update: function(oldData){
    var object3D = this.el.object3D.children[0];
    var data = this.data;

    if(data.eye === "both"){
      object3D.layers.set(0);
    }
    else{
      object3D.layers.set(data.eye === 'left' ? 1:2);
    }
    
    // Log stereo layer info
    if (data.eye !== oldData.eye) {
      console.log(`Set stereo layer for ${data.eye} eye`);
    }
  },
}); 