// Wait for A-Frame scene to load

AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log('🎮 Controller updater component initialized.');
    console.log('🔍 Checking for controller elements...');
    // Controllers are enabled

    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');

    // --- Controller State ---
    this.leftGripDown = false;
    this.rightGripDown = false;
    this.leftTriggerDown = false;
    this.rightTriggerDown = false;
    this.leftXButtonDown = false;
    this.rightAButtonDown = false;

    // --- Status reporting ---
    this.lastStatusUpdate = 0;
    this.statusUpdateInterval = 5000; // 5 seconds

    // --- Relative rotation tracking ---
    this.leftGripInitialRotation = null;
    this.rightGripInitialRotation = null;
    this.leftRelativeRotation = { x: 0, y: 0, z: 0 };
    this.rightRelativeRotation = { x: 0, y: 0, z: 0 };

    // --- Quaternion-based Z-axis rotation tracking ---
    this.leftGripInitialQuaternion = null;
    this.rightGripInitialQuaternion = null;
    this.leftZAxisRotation = 0;
    this.rightZAxisRotation = 0;

    // set up text encoder and decoder
    this.encoder = new TextEncoder();
    this.decoder = new TextDecoder();

    // Load config and initialize asynchronously
    this.initializeAsyncWithConfig().catch((e) =>
      console.error('Error during async initialization:', e),
    );

    // --- VR Status Reporting Function ---
    this.reportVRStatus = (connected) => {
      // Update global status if available (for desktop interface)
      if (typeof updateStatus === 'function') {
        updateStatus({ vrConnected: connected });
      }

      // Also try to notify parent window if in iframe
      try {
        if (window.parent && window.parent !== window) {
          window.parent.postMessage(
            {
              type: 'vr_status',
              connected: connected,
            },
            '*',
          );
        }
      } catch (e) {
        // Ignore cross-origin errors
      }
    };

    if (
      !this.leftHand ||
      !this.rightHand ||
      !this.leftHandInfoText ||
      !this.rightHandInfoText
    ) {
      console.error('Controller or text entities not found!');
      // Check which specific elements are missing
      if (!this.leftHand) console.error('Left hand entity not found');
      if (!this.rightHand) console.error('Right hand entity not found');
      if (!this.leftHandInfoText)
        console.error('Left hand info text not found');
      if (!this.rightHandInfoText)
        console.error('Right hand info text not found');
      return;
    }

    // Apply initial rotation to combined text elements
    const textRotation = '-90 0 0'; // Rotate -90 degrees around X-axis
    if (this.leftHandInfoText)
      this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText)
      this.rightHandInfoText.setAttribute('rotation', textRotation);

    // --- Create axis indicators ---
    console.log('🔧 Creating axis indicators...');
    this.createAxisIndicators();
    console.log('✅ Axis indicators created successfully');

    // --- Helper function to send grip release message ---
    this.sendGripRelease = (hand) => {
      // encode gripper release message
      const releaseMessage = {
        hand: hand,
        gripReleased: true,
      };
      this.sendMessageToControlServer(releaseMessage);
      console.log(`Sent grip release for ${hand} hand`);
    };

    // --- Helper function to send trigger release message ---
    this.sendTriggerRelease = (hand) => {
      const releaseMessage = {
        hand: hand,
        triggerReleased: true,
      };
      this.sendMessageToControlServer(releaseMessage);
      console.log(`Sent trigger release for ${hand} hand`);
    };

    // --- Helper function to send X button release message ---
    this.sendXButtonRelease = (hand) => {
      const releaseMessage = {
        hand: hand,
        xButtonReleased: true,
        resetEvent: true,
      };
      this.sendMessageToControlServer(releaseMessage);
      console.log(`Sent X button release (reset event) for ${hand} hand`);
    };

    // --- Helper function to send A button release message ---
    this.sendAButtonRelease = (hand) => {
      const releaseMessage = {
        hand: hand,
        aButtonReleased: true,
        resetEvent: true,
      };
      this.sendMessageToControlServer(releaseMessage);
      console.log(`Sent A button release (reset event) for ${hand} hand`);
    };

    // --- Helper function to calculate relative rotation ---
    this.calculateRelativeRotation = (currentRotation, initialRotation) => {
      return {
        x: currentRotation.x - initialRotation.x,
        y: currentRotation.y - initialRotation.y,
        z: currentRotation.z - initialRotation.z,
      };
    };

    // --- Helper function to calculate Z-axis rotation from quaternions ---
    this.calculateZAxisRotation = (currentQuaternion, initialQuaternion) => {
      // Calculate relative quaternion (from initial to current)
      const relativeQuat = new THREE.Quaternion();
      relativeQuat.multiplyQuaternions(
        currentQuaternion,
        initialQuaternion.clone().invert(),
      );

      // Get the controller's current forward direction (local Z-axis in world space)
      const forwardDirection = new THREE.Vector3(0, 0, 1);
      forwardDirection.applyQuaternion(currentQuaternion);

      // Convert relative quaternion to axis-angle representation
      const angle = 2 * Math.acos(Math.abs(relativeQuat.w));

      // Handle case where there's no rotation (avoid division by zero)
      if (angle < 0.0001) {
        return 0;
      }

      // Get the rotation axis
      const sinHalfAngle = Math.sqrt(1 - relativeQuat.w * relativeQuat.w);
      const rotationAxis = new THREE.Vector3(
        relativeQuat.x / sinHalfAngle,
        relativeQuat.y / sinHalfAngle,
        relativeQuat.z / sinHalfAngle,
      );

      // Project the rotation axis onto the forward direction to get the component
      // of rotation around the forward axis
      const projectedComponent = rotationAxis.dot(forwardDirection);

      // The rotation around the forward axis is the angle times the projection
      const forwardRotation = angle * projectedComponent;

      // Convert to degrees and handle the sign properly
      let degrees = THREE.MathUtils.radToDeg(forwardRotation);

      // Normalize to -180 to +180 range to avoid sudden jumps
      while (degrees > 180) degrees -= 360;
      while (degrees < -180) degrees += 360;

      return degrees;
    };

    // --- Modify Event Listeners ---
    this.leftHand.addEventListener('triggerdown', (evt) => {
      console.log('Left Trigger Pressed');
      this.leftTriggerDown = true;
    });
    this.leftHand.addEventListener('triggerup', (evt) => {
      console.log('Left Trigger Released');
      this.leftTriggerDown = false;
      this.sendTriggerRelease('left'); // Send trigger release message
    });
    this.leftHand.addEventListener('gripdown', (evt) => {
      console.log('Left Grip Pressed');
      this.leftGripDown = true; // Set grip state

      // Store initial rotation for relative tracking
      if (this.leftHand.object3D.visible) {
        const leftRotEuler = this.leftHand.object3D.rotation;
        this.leftGripInitialRotation = {
          x: THREE.MathUtils.radToDeg(leftRotEuler.x),
          y: THREE.MathUtils.radToDeg(leftRotEuler.y),
          z: THREE.MathUtils.radToDeg(leftRotEuler.z),
        };

        // Store initial quaternion for Z-axis rotation tracking
        this.leftGripInitialQuaternion =
          this.leftHand.object3D.quaternion.clone();

        console.log(
          'Left grip initial rotation:',
          this.leftGripInitialRotation,
        );
        console.log(
          'Left grip initial quaternion:',
          this.leftGripInitialQuaternion,
        );
      }
    });
    this.leftHand.addEventListener('gripup', (evt) => {
      // Add gripup listener
      console.log('Left Grip Released');
      this.leftGripDown = false; // Reset grip state
      this.leftGripInitialRotation = null; // Reset initial rotation
      this.leftGripInitialQuaternion = null; // Reset initial quaternion
      this.leftRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
      this.leftZAxisRotation = 0; // Reset Z-axis rotation
      this.sendGripRelease('left'); // Send grip release message
    });

    this.leftHand.addEventListener('xbuttondown', (evt) => {
      console.log('Left X Button Pressed (Reset Event)');
      this.leftXButtonDown = true;
    });
    this.leftHand.addEventListener('xbuttonup', (evt) => {
      console.log('Left X Button Released (Reset Event)');
      this.leftXButtonDown = false;
      this.sendXButtonRelease('left'); // Send X button release message
    });

    this.rightHand.addEventListener('triggerdown', (evt) => {
      console.log('Right Trigger Pressed');
      this.rightTriggerDown = true;
    });
    this.rightHand.addEventListener('triggerup', (evt) => {
      console.log('Right Trigger Released');
      this.rightTriggerDown = false;
      this.sendTriggerRelease('right'); // Send trigger release message
    });
    this.rightHand.addEventListener('gripdown', (evt) => {
      console.log('Right Grip Pressed');
      this.rightGripDown = true; // Set grip state

      // Store initial rotation for relative tracking
      if (this.rightHand.object3D.visible) {
        const rightRotEuler = this.rightHand.object3D.rotation;
        this.rightGripInitialRotation = {
          x: THREE.MathUtils.radToDeg(rightRotEuler.x),
          y: THREE.MathUtils.radToDeg(rightRotEuler.y),
          z: THREE.MathUtils.radToDeg(rightRotEuler.z),
        };

        // Store initial quaternion for Z-axis rotation tracking
        this.rightGripInitialQuaternion =
          this.rightHand.object3D.quaternion.clone();

        console.log(
          'Right grip initial rotation:',
          this.rightGripInitialRotation,
        );
        console.log(
          'Right grip initial quaternion:',
          this.rightGripInitialQuaternion,
        );
      }
    });
    this.rightHand.addEventListener('gripup', (evt) => {
      // Add gripup listener
      console.log('Right Grip Released');
      this.rightGripDown = false; // Reset grip state
      this.rightGripInitialRotation = null; // Reset initial rotation
      this.rightGripInitialQuaternion = null; // Reset initial quaternion
      this.rightRelativeRotation = { x: 0, y: 0, z: 0 }; // Reset relative rotation
      this.rightZAxisRotation = 0; // Reset Z-axis rotation
      this.sendGripRelease('right'); // Send grip release message
    });
    // --- End Modify Event Listeners ---

    this.rightHand.addEventListener('abuttondown', (evt) => {
      console.log('Right A Button Pressed (Reset Event)');
      this.rightAButtonDown = true;
    });
    this.rightHand.addEventListener('abuttonup', (evt) => {
      console.log('Right A Button Released (Reset Event)');
      this.rightAButtonDown = false;
      this.sendAButtonRelease('right'); // Send A button release message
    });
    // --- End Modify Event Listeners ---
  },

  initializeAsyncWithConfig: async function () {
    // defaults, if loading from backend fails
    let roomName = 'robot-vr-teleop-room';
    let participantIdentity = 'controllers-publishing';
    let destinationParticipant = 'controllers-processing';
    let debug = false;

    try {
      const config = await window.LiveKitUtils.loadLiveKitConfig();
      roomName = config.livekit_room;
      participantIdentity = config.controllers_publishing_participant;
      destinationParticipant = config.controllers_processing_participant;
      debug = config.vr_viewer_debug;
      console.log('Loaded LiveKit config from backend:', config);
      console.log(
        '🎯 Participant identities - Publishing:',
        participantIdentity,
        'Processing:',
        destinationParticipant,
      );

      // Store as component property for use in sendMessageToControlServer
      this.destinationParticipant = destinationParticipant;
    } catch (error) {
      console.warn('Failed to load LiveKit config, using defaults:', error);
      // Set fallback value
      this.destinationParticipant = destinationParticipant;
    }

    // connect to livekit room
    window.LiveKitUtils.asyncRoomConnect(this, roomName, participantIdentity);
  },

  // --- LiveKit Event Handlers ---
  handleTrackSubscribed: function (track, publication, participant) {
    window.LiveKitUtils.logToVR(
      `Track subscribed: ${track.kind} from ${participant.identity}`,
    );
  },

  handleTrackUnsubscribed: function (track, publication, participant) {
    window.LiveKitUtils.logToVR(
      `Track unsubscribed: ${track.kind} from ${participant.identity}`,
    );
  },

  handleConnected: function () {
    window.LiveKitUtils.logToVR('Connected to LiveKit room');
  },

  handleDisconnect: function () {
    window.LiveKitUtils.logToVR('Disconnected from LiveKit room');
  },
  // --- End LiveKit Event Handlers ---

  sendMessageToControlServer: function (message) {
    if (!window.LiveKitUtils || !window.LiveKitUtils.room) {
      console.error('❌ LiveKit not connected, cannot send message:', message);
      return;
    }

    const strData = JSON.stringify(message);
    const encodedData = this.encoder.encode(strData);

    try {
      // Check if target participant is connected before sending targeted message
      const targetConnected = Array.from(
        window.LiveKitUtils.room.remoteParticipants.values(),
      ).some((p) => p.identity === this.destinationParticipant);

      if (targetConnected) {
        window.LiveKitUtils.room.localParticipant.publishData(encodedData, {
          reliable: true,
          destinationIdentities: [this.destinationParticipant],
          topic: 'vr-controller-data',
        });
        console.log('📤 Sent TARGETED message to control server:', message);
      } else {
        // Fallback to broadcast if target not connected yet
        window.LiveKitUtils.room.localParticipant.publishData(encodedData, {
          reliable: true,
          topic: 'vr-controller-data',
        });
        console.log(
          '📤 Sent BROADCAST message (target not connected):',
          message,
        );
      }
    } catch (error) {
      console.error('❌ Failed to send message:', error, message);
    }
  },

  createAxisIndicators: function () {
    // Create XYZ axis indicators for both controllers

    // Left Controller Axes
    // X-axis (Red)
    const leftXAxis = document.createElement('a-cylinder');
    leftXAxis.setAttribute('id', 'leftXAxis');
    leftXAxis.setAttribute('height', '0.08');
    leftXAxis.setAttribute('radius', '0.003');
    leftXAxis.setAttribute('color', '#ff0000'); // Red for X
    leftXAxis.setAttribute('position', '0.04 0 0');
    leftXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.leftHand.appendChild(leftXAxis);

    const leftXTip = document.createElement('a-cone');
    leftXTip.setAttribute('height', '0.015');
    leftXTip.setAttribute('radius-bottom', '0.008');
    leftXTip.setAttribute('radius-top', '0');
    leftXTip.setAttribute('color', '#ff0000');
    leftXTip.setAttribute('position', '0.055 0 0');
    leftXTip.setAttribute('rotation', '0 0 90');
    this.leftHand.appendChild(leftXTip);

    // Y-axis (Green) - Up
    const leftYAxis = document.createElement('a-cylinder');
    leftYAxis.setAttribute('id', 'leftYAxis');
    leftYAxis.setAttribute('height', '0.08');
    leftYAxis.setAttribute('radius', '0.003');
    leftYAxis.setAttribute('color', '#00ff00'); // Green for Y
    leftYAxis.setAttribute('position', '0 0.04 0');
    leftYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.leftHand.appendChild(leftYAxis);

    const leftYTip = document.createElement('a-cone');
    leftYTip.setAttribute('height', '0.015');
    leftYTip.setAttribute('radius-bottom', '0.008');
    leftYTip.setAttribute('radius-top', '0');
    leftYTip.setAttribute('color', '#00ff00');
    leftYTip.setAttribute('position', '0 0.055 0');
    this.leftHand.appendChild(leftYTip);

    // Z-axis (Blue) - Forward
    const leftZAxis = document.createElement('a-cylinder');
    leftZAxis.setAttribute('id', 'leftZAxis');
    leftZAxis.setAttribute('height', '0.08');
    leftZAxis.setAttribute('radius', '0.003');
    leftZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    leftZAxis.setAttribute('position', '0 0 0.04');
    leftZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.leftHand.appendChild(leftZAxis);

    const leftZTip = document.createElement('a-cone');
    leftZTip.setAttribute('height', '0.015');
    leftZTip.setAttribute('radius-bottom', '0.008');
    leftZTip.setAttribute('radius-top', '0');
    leftZTip.setAttribute('color', '#0000ff');
    leftZTip.setAttribute('position', '0 0 0.055');
    leftZTip.setAttribute('rotation', '90 0 0');
    this.leftHand.appendChild(leftZTip);

    // Right Controller Axes
    // X-axis (Red)
    const rightXAxis = document.createElement('a-cylinder');
    rightXAxis.setAttribute('id', 'rightXAxis');
    rightXAxis.setAttribute('height', '0.08');
    rightXAxis.setAttribute('radius', '0.003');
    rightXAxis.setAttribute('color', '#ff0000'); // Red for X
    rightXAxis.setAttribute('position', '0.04 0 0');
    rightXAxis.setAttribute('rotation', '0 0 90'); // Rotate to point along X-axis
    this.rightHand.appendChild(rightXAxis);

    const rightXTip = document.createElement('a-cone');
    rightXTip.setAttribute('height', '0.015');
    rightXTip.setAttribute('radius-bottom', '0.008');
    rightXTip.setAttribute('radius-top', '0');
    rightXTip.setAttribute('color', '#ff0000');
    rightXTip.setAttribute('position', '0.055 0 0');
    rightXTip.setAttribute('rotation', '0 0 90');
    this.rightHand.appendChild(rightXTip);

    // Y-axis (Green) - Up
    const rightYAxis = document.createElement('a-cylinder');
    rightYAxis.setAttribute('id', 'rightYAxis');
    rightYAxis.setAttribute('height', '0.08');
    rightYAxis.setAttribute('radius', '0.003');
    rightYAxis.setAttribute('color', '#00ff00'); // Green for Y
    rightYAxis.setAttribute('position', '0 0.04 0');
    rightYAxis.setAttribute('rotation', '0 0 0'); // Default up orientation
    this.rightHand.appendChild(rightYAxis);

    const rightYTip = document.createElement('a-cone');
    rightYTip.setAttribute('height', '0.015');
    rightYTip.setAttribute('radius-bottom', '0.008');
    rightYTip.setAttribute('radius-top', '0');
    rightYTip.setAttribute('color', '#00ff00');
    rightYTip.setAttribute('position', '0 0.055 0');
    this.rightHand.appendChild(rightYTip);

    // Z-axis (Blue) - Forward
    const rightZAxis = document.createElement('a-cylinder');
    rightZAxis.setAttribute('id', 'rightZAxis');
    rightZAxis.setAttribute('height', '0.08');
    rightZAxis.setAttribute('radius', '0.003');
    rightZAxis.setAttribute('color', '#0000ff'); // Blue for Z
    rightZAxis.setAttribute('position', '0 0 0.04');
    rightZAxis.setAttribute('rotation', '90 0 0'); // Rotate to point along Z-axis
    this.rightHand.appendChild(rightZAxis);

    const rightZTip = document.createElement('a-cone');
    rightZTip.setAttribute('height', '0.015');
    rightZTip.setAttribute('radius-bottom', '0.008');
    rightZTip.setAttribute('radius-top', '0');
    rightZTip.setAttribute('color', '#0000ff');
    rightZTip.setAttribute('position', '0 0 0.055');
    rightZTip.setAttribute('rotation', '90 0 0');
    this.rightHand.appendChild(rightZTip);

    console.log(
      'XYZ axis indicators created for both controllers (RGB for XYZ)',
    );
  },

  tick: function () {
    // Update controller text if controllers are visible
    if (!this.leftHand || !this.rightHand) return; // Added safety check

    // Initialize debug counter if not exists
    if (!this.debugCounter) this.debugCounter = 0;
    this.debugCounter++;

    // --- BEGIN DETAILED LOGGING (every 60 frames = ~1 second) ---
    if (this.debugCounter % 60 === 0) {
      if (this.leftHand.object3D) {
        console.log(
          `👈 Left Hand - Visible: ${
            this.leftHand.object3D.visible
          }, Pos: ${this.leftHand.object3D.position.x.toFixed(
            2,
          )},${this.leftHand.object3D.position.y.toFixed(
            2,
          )},${this.leftHand.object3D.position.z.toFixed(2)}`,
        );
      }
      if (this.rightHand.object3D) {
        console.log(
          `👉 Right Hand - Visible: ${
            this.rightHand.object3D.visible
          }, Pos: ${this.rightHand.object3D.position.x.toFixed(
            2,
          )},${this.rightHand.object3D.position.y.toFixed(
            2,
          )},${this.rightHand.object3D.position.z.toFixed(2)}`,
        );
      }
    }
    // --- END DETAILED LOGGING ---

    // Collect data from both controllers
    const leftController = {
      hand: 'left',
      position: null,
      rotation: null,
      gripActive: false,
      trigger: 0,
      xButtonDown: false,
    };

    const rightController = {
      hand: 'right',
      position: null,
      rotation: null,
      gripActive: false,
      trigger: 0,
      aButtonDown: false,
    };

    // Update Left Hand Text & Collect Data
    if (this.leftHand.object3D.visible) {
      const leftPos = this.leftHand.object3D.position;
      const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
      // Convert to degrees without offset
      const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
      const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
      const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);

      // Calculate relative rotation if grip is held
      if (this.leftGripDown && this.leftGripInitialRotation) {
        this.leftRelativeRotation = this.calculateRelativeRotation(
          { x: leftRotX, y: leftRotY, z: leftRotZ },
          this.leftGripInitialRotation,
        );

        // Calculate Z-axis rotation using quaternions
        if (this.leftGripInitialQuaternion) {
          this.leftZAxisRotation = this.calculateZAxisRotation(
            this.leftHand.object3D.quaternion,
            this.leftGripInitialQuaternion,
          );
        }

        console.log('Left relative rotation:', this.leftRelativeRotation);
        console.log(
          'Left Z-axis rotation:',
          this.leftZAxisRotation.toFixed(1),
          'degrees',
        );
      }

      // Create display text including relative rotation when grip is held
      let combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(
        2,
      )} ${leftPos.z.toFixed(2)}\\nRot: ${leftRotX.toFixed(
        0,
      )} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
      if (this.leftGripDown && this.leftGripInitialRotation) {
        combinedLeftText += `\\nZ-Rot: ${this.leftZAxisRotation.toFixed(1)}°`;
      }

      if (this.leftHandInfoText) {
        this.leftHandInfoText.setAttribute('value', combinedLeftText);
      }

      // Collect left controller data
      leftController.position = { x: leftPos.x, y: leftPos.y, z: leftPos.z };
      leftController.rotation = { x: leftRotX, y: leftRotY, z: leftRotZ };
      leftController.quaternion = {
        x: this.leftHand.object3D.quaternion.x,
        y: this.leftHand.object3D.quaternion.y,
        z: this.leftHand.object3D.quaternion.z,
        w: this.leftHand.object3D.quaternion.w,
      };
      leftController.trigger = this.leftTriggerDown ? 1 : 0;
      leftController.gripActive = this.leftGripDown;
      leftController.xButtonDown = this.leftXButtonDown;
    }

    // Update Right Hand Text & Collect Data
    if (this.rightHand.object3D.visible) {
      const rightPos = this.rightHand.object3D.position;
      const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
      // Convert to degrees without offset
      const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
      const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
      const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);

      // Calculate relative rotation if grip is held
      if (this.rightGripDown && this.rightGripInitialRotation) {
        this.rightRelativeRotation = this.calculateRelativeRotation(
          { x: rightRotX, y: rightRotY, z: rightRotZ },
          this.rightGripInitialRotation,
        );

        // Calculate Z-axis rotation using quaternions
        if (this.rightGripInitialQuaternion) {
          this.rightZAxisRotation = this.calculateZAxisRotation(
            this.rightHand.object3D.quaternion,
            this.rightGripInitialQuaternion,
          );
        }

        console.log('Right relative rotation:', this.rightRelativeRotation);
        console.log(
          'Right Z-axis rotation:',
          this.rightZAxisRotation.toFixed(1),
          'degrees',
        );
      }

      // Create display text including relative rotation when grip is held
      let combinedRightText = `Pos: ${rightPos.x.toFixed(
        2,
      )} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(
        2,
      )}\\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(
        0,
      )} ${rightRotZ.toFixed(0)}`;
      if (this.rightGripDown && this.rightGripInitialRotation) {
        combinedRightText += `\\nZ-Rot: ${this.rightZAxisRotation.toFixed(1)}°`;
      }

      if (this.rightHandInfoText) {
        this.rightHandInfoText.setAttribute('value', combinedRightText);
      }

      // Collect right controller data
      rightController.position = {
        x: rightPos.x,
        y: rightPos.y,
        z: rightPos.z,
      };
      rightController.rotation = { x: rightRotX, y: rightRotY, z: rightRotZ };
      rightController.quaternion = {
        x: this.rightHand.object3D.quaternion.x,
        y: this.rightHand.object3D.quaternion.y,
        z: this.rightHand.object3D.quaternion.z,
        w: this.rightHand.object3D.quaternion.w,
      };
      rightController.trigger = this.rightTriggerDown ? 1 : 0;
      rightController.gripActive = this.rightGripDown;
      rightController.aButtonDown = this.rightAButtonDown;
    }

    // Send combined packet if at least one controller has valid data
    const hasValidLeft =
      leftController.position &&
      (leftController.position.x !== 0 ||
        leftController.position.y !== 0 ||
        leftController.position.z !== 0);
    const hasValidRight =
      rightController.position &&
      (rightController.position.x !== 0 ||
        rightController.position.y !== 0 ||
        rightController.position.z !== 0);

    if (hasValidLeft || hasValidRight) {
      const dualControllerData = {
        timestamp: Date.now(),
        leftController: leftController,
        rightController: rightController,
      };

      // Debug log every 60 frames to avoid spam
      if (this.debugCounter % 60 === 0) {
        console.log('📡 Sending controller data:', {
          hasValidLeft,
          hasValidRight,
          leftPos: leftController.position,
          rightPos: rightController.position,
        });
      }

      this.sendMessageToControlServer(dualControllerData);
    } else if (this.debugCounter % 60 === 0) {
      console.log('⚠️ No valid controller data to send');
    }
  },
});

// Add the component to the scene after it's loaded
document.addEventListener('DOMContentLoaded', (event) => {
  const scene = document.querySelector('a-scene');

  if (scene) {
    // Listen for controller connection events
    scene.addEventListener('controllerconnected', (evt) => {
      console.log(
        '🎮 Controller CONNECTED:',
        evt.detail.name,
        evt.detail.component.data.hand,
      );
      console.log('📊 Controller details:', evt.detail);
    });
    scene.addEventListener('controllerdisconnected', (evt) => {
      console.log(
        '🎮 Controller DISCONNECTED:',
        evt.detail.name,
        evt.detail.component.data.hand,
      );
    });

    // Add controller-updater component when scene is loaded (A-Frame manages session)
    console.log('🔄 Scene loaded status:', scene.hasLoaded);
    if (scene.hasLoaded) {
      scene.setAttribute('controller-updater', '');
      console.log('✅ controller-updater component added immediately.');
    } else {
      console.log('⏳ Waiting for scene to load...');
      scene.addEventListener('loaded', () => {
        scene.setAttribute('controller-updater', '');
        console.log(
          '✅ controller-updater component added after scene loaded.',
        );
      });
    }
  } else {
    console.error('A-Frame scene not found!');
  }

  // VR button logic is now handled by start-vr.js
  console.log('Controllers initialized. VR entry point handled by start-vr.js');
});

// VR button functionality moved to start-vr.js
// This keeps controllers.js focused on controller tracking logic only
