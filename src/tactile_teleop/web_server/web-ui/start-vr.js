// VR Experience Entry Point
// This file handles the VR mode activation and controller tracking initialization

class VRTeleopStarter {
  constructor() {
    this.isVRActive = false;
    this.startButton = null;
    this.sceneEl = null;
    this.controllersInitialized = false;
  }

  // Initialize VR starter - only called for VR-capable browsers
  async init() {
    console.log('VR Teleop Starter initializing...');

    try {
      // Wait for A-Frame scene to be ready
      await this.waitForScene();

      // Create and show the VR start button
      this.createStartButton();

      // Set up VR session event listeners
      this.setupVRSessionListeners();

      console.log('VR Teleop Starter ready');
    } catch (error) {
      console.error('Failed to initialize VR Teleop Starter:', error);
    }
  }

  // Wait for A-Frame scene to be fully loaded
  waitForScene() {
    return new Promise((resolve, reject) => {
      const checkScene = () => {
        this.sceneEl = document.querySelector('a-scene');
        if (this.sceneEl && this.sceneEl.hasLoaded) {
          resolve();
        } else if (this.sceneEl) {
          this.sceneEl.addEventListener('loaded', resolve, { once: true });
        } else {
          setTimeout(checkScene, 100);
        }
      };
      checkScene();

      // Timeout after 10 seconds
      setTimeout(() => reject(new Error('Scene loading timeout')), 10000);
    });
  }

  // Create the "Start VR Experience" button
  createStartButton() {
    this.startButton = document.createElement('button');
    this.startButton.id = 'vr-start-button';
    this.startButton.textContent = 'Start VR Experience';

    // Style the button
    Object.assign(this.startButton.style, {
      position: 'fixed',
      top: '50%',
      left: '50%',
      transform: 'translate(-50%, -50%)',
      padding: '20px 40px',
      fontSize: '24px',
      fontWeight: 'bold',
      backgroundColor: '#4CAF50',
      color: 'white',
      border: 'none',
      borderRadius: '12px',
      cursor: 'pointer',
      zIndex: '10000',
      boxShadow: '0 8px 20px rgba(76, 175, 80, 0.4)',
      transition: 'all 0.3s ease',
      fontFamily:
        '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
    });

    // Add hover effects
    this.startButton.addEventListener('mouseenter', () => {
      this.startButton.style.backgroundColor = '#45a049';
      this.startButton.style.transform = 'translate(-50%, -50%) scale(1.05)';
      this.startButton.style.boxShadow = '0 12px 25px rgba(76, 175, 80, 0.5)';
    });

    this.startButton.addEventListener('mouseleave', () => {
      this.startButton.style.backgroundColor = '#4CAF50';
      this.startButton.style.transform = 'translate(-50%, -50%) scale(1)';
      this.startButton.style.boxShadow = '0 8px 20px rgba(76, 175, 80, 0.4)';
    });

    // Add click handler
    this.startButton.addEventListener('click', () => this.startVRExperience());

    // Add to page
    document.body.appendChild(this.startButton);
    console.log('VR start button created and added to page');
  }

  // Main VR experience activation
  async startVRExperience() {
    console.log('Starting VR experience...');

    try {
      // Hide the start button
      this.startButton.style.display = 'none';

      // Enter VR mode
      await this.enterVRMode();

      // Initialize controller tracking
      this.initializeControllerTracking();

      // Show VR interface elements
      this.showVRInterface();

      this.isVRActive = true;
      console.log('VR experience started successfully');
    } catch (error) {
      console.error('Failed to start VR experience:', error);

      // Show error and restore button
      alert(`Failed to start VR experience: ${error.message}`);
      this.startButton.style.display = 'block';
    }
  }

  // Enter VR mode using A-Frame
  async enterVRMode() {
    console.log('Entering VR mode...');

    if (!this.sceneEl) {
      throw new Error('A-Frame scene not found');
    }

    // Use A-Frame's enterVR method to request VR session
    try {
      await this.sceneEl.enterVR(true);
      console.log('VR mode activated');
    } catch (error) {
      throw new Error(`VR session failed: ${error.message}`);
    }
  }

  // Initialize controller tracking
  initializeControllerTracking() {
    if (this.controllersInitialized) {
      return;
    }

    console.log('Initializing controller tracking...');

    // Get controller entities
    const leftController = document.getElementById('leftHand');
    const rightController = document.getElementById('rightHand');

    if (leftController && rightController) {
      // Controllers are already set up in the HTML
      // The actual tracking logic is handled by the existing controllers.js
      console.log('Controller entities found and ready for tracking');
      this.controllersInitialized = true;
    } else {
      console.warn('Controller entities not found');
    }
  }

  // Show VR interface elements (video streams, etc.)
  showVRInterface() {
    console.log('Showing VR interface elements...');

    // Video streams are already set up in the HTML
    // They will automatically become visible once VR session starts
    const videoEntity = document.querySelector('[teleop-video-stream]');
    if (videoEntity) {
      console.log('Video stream entity found and ready');
    }

    // Any additional VR-specific UI can be shown here
  }

  // Set up VR session event listeners
  setupVRSessionListeners() {
    if (!this.sceneEl) return;

    // Listen for VR session start
    this.sceneEl.addEventListener('enter-vr', () => {
      console.log('VR session started');
      this.onVRSessionStart();
    });

    // Listen for VR session end
    this.sceneEl.addEventListener('exit-vr', () => {
      console.log('VR session ended');
      this.onVRSessionEnd();
    });
  }

  // Handle VR session start
  onVRSessionStart() {
    console.log('VR session active - hiding desktop UI');

    // Hide desktop interface
    const desktopInterface = document.getElementById('desktopInterface');
    if (desktopInterface) {
      desktopInterface.style.display = 'none';
    }
  }

  // Handle VR session end
  onVRSessionEnd() {
    console.log('VR session ended - showing desktop UI');

    // Show desktop interface
    const desktopInterface = document.getElementById('desktopInterface');
    if (desktopInterface) {
      desktopInterface.style.display = 'block';
    }

    // Show start button again
    if (this.startButton) {
      this.startButton.style.display = 'block';
    }

    this.isVRActive = false;
  }

  // Check if currently in VR
  isInVR() {
    return this.isVRActive;
  }
}

// Initialize VR Teleop Starter when DOM is ready
document.addEventListener('DOMContentLoaded', async function () {
  // Only initialize if this is a VR-capable browser
  if (navigator.xr) {
    try {
      const isVRSupported = await navigator.xr.isSessionSupported(
        'immersive-ar',
      );
      if (isVRSupported) {
        console.log('VR support detected, initializing VR Teleop Starter...');
        const vrStarter = new VRTeleopStarter();
        await vrStarter.init();

        // Make it globally available
        window.vrTeleopStarter = vrStarter;
      } else {
        console.log('VR not supported on this device');
      }
    } catch (error) {
      console.error('Error checking VR support:', error);
    }
  } else {
    console.log('WebXR not available in this browser');
  }
});
