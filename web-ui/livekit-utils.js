window.LiveKitUtils = {
  AUTH_SERVER_PORT: 5050,
  DEBUG: false,

  asyncRoomConnect: async function (component, roomName, participantIdentity) {
    try {
      this.logToVR('Starting initialization...');

      // Wait for LiveKit to be available
      await this.waitForLiveKit();

      // Get token and connect
      await window.LiveKitUtils.connectToRoom(
        component,
        roomName,
        participantIdentity,
      );
    } catch (error) {
      this.logToVR('ERROR: Failed to initialize - ' + error.message);
      console.error('Failed to initialize video streamer:', error);
    }
  },

  connectToRoom: async function (
    component,
    roomName,
    participantIdentity,
    canPublish = false,
  ) {
    try {
      // Get token and LiveKit URL from Python backend
      const response = await this.getToken(
        roomName,
        participantIdentity,
        canPublish,
      );
      const { token, livekit_url } = response;

      this.room = new window.LiveKitClient.Room();
      this.logToVR(`Created LiveKit room`);

      // Set up event listeners
      this.room
        .on(
          window.LiveKitClient.RoomEvent.TrackSubscribed,
          component.handleTrackSubscribed,
        )
        .on(
          window.LiveKitClient.RoomEvent.TrackUnsubscribed,
          component.handleTrackUnsubscribed,
        )
        .on(
          window.LiveKitClient.RoomEvent.Disconnected,
          component.handleDisconnect,
        )
        .on(
          window.LiveKitClient.RoomEvent.Connected,
          component.handleConnected,
        );

      this.logToVR(
        `Connecting to room with livekit url: ${livekit_url} and token: ${token} (room name: ${roomName})`,
      );
      await this.room.connect(livekit_url, token);
      this.logToVR(`Connected to room: ${roomName}`);
    } catch (error) {
      this.logToVR(
        `ERROR: Failed to connect to LiveKit room ${roomName} - ${error.message}`,
      );
      console.error('Failed to connect to LiveKit room:', error);
    }
  },

  getToken: async function (roomName, participantIdentity, canPublish = false) {
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

      const authUrl = `https://${window.location.hostname}:${AUTH_SERVER_PORT}/api/get-token`;
      this.logToVR(`Getting token from: ${authUrl} (VR: ${isVR})`);

      const response = await fetch(authUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          room_name: roomName,
          participant_identity: participantIdentity,
          canPublish: canPublish,
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
    if (DEBUG) {
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
    }
  },
};
