// Wait for A-Frame scene to load

AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log("Controller updater component initialized.");
    // Controllers are enabled

    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandInfoText = document.querySelector('#leftHandInfo');
    this.rightHandInfoText = document.querySelector('#rightHandInfo');

    if (!this.leftHand || !this.rightHand || !this.leftHandInfoText || !this.rightHandInfoText) {
      console.error("Controller or text entities not found!");
      // Check which specific elements are missing
      if (!this.leftHand) console.error("Left hand entity not found");
      if (!this.rightHand) console.error("Right hand entity not found");
      if (!this.leftHandInfoText) console.error("Left hand info text not found");
      if (!this.rightHandInfoText) console.error("Right hand info text not found");
      return;
    }

    // Apply initial rotation to combined text elements
    const textRotation = '-90 0 0'; // Rotate -90 degrees around X-axis
    if (this.leftHandInfoText) this.leftHandInfoText.setAttribute('rotation', textRotation);
    if (this.rightHandInfoText) this.rightHandInfoText.setAttribute('rotation', textRotation);

  },

  tick: function () {
    // Update controller text if controllers are visible
    if (!this.leftHand || !this.rightHand) return; // Added safety check

    // --- BEGIN DETAILED LOGGING ---
    if (this.leftHand.object3D) {
      // console.log(`Left Hand Raw - Visible: ${this.leftHand.object3D.visible}, Pos: ${this.leftHand.object3D.position.x.toFixed(2)},${this.leftHand.object3D.position.y.toFixed(2)},${this.leftHand.object3D.position.z.toFixed(2)}`);
    }
    if (this.rightHand.object3D) {
      // console.log(`Right Hand Raw - Visible: ${this.rightHand.object3D.visible}, Pos: ${this.rightHand.object3D.position.x.toFixed(2)},${this.rightHand.object3D.position.y.toFixed(2)},${this.rightHand.object3D.position.z.toFixed(2)}`);
    }
    // --- END DETAILED LOGGING ---


    // Update Left Hand Text (Combined)
    if (this.leftHand.object3D.visible && this.leftHandInfoText) {
        const leftPos = this.leftHand.object3D.position;
        const leftRotEuler = this.leftHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const leftRotX = THREE.MathUtils.radToDeg(leftRotEuler.x);
        const leftRotY = THREE.MathUtils.radToDeg(leftRotEuler.y);
        const leftRotZ = THREE.MathUtils.radToDeg(leftRotEuler.z);
        const combinedLeftText = `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}\nRot: ${leftRotX.toFixed(0)} ${leftRotY.toFixed(0)} ${leftRotZ.toFixed(0)}`;
        this.leftHandInfoText.setAttribute('value', combinedLeftText);
    }

    // Update Right Hand Text (Combined)
    if (this.rightHand.object3D.visible && this.rightHandInfoText) {
        const rightPos = this.rightHand.object3D.position;
        const rightRotEuler = this.rightHand.object3D.rotation; // Euler angles in radians
        // Convert to degrees without offset
        const rightRotX = THREE.MathUtils.radToDeg(rightRotEuler.x);
        const rightRotY = THREE.MathUtils.radToDeg(rightRotEuler.y);
        const rightRotZ = THREE.MathUtils.radToDeg(rightRotEuler.z);
        const combinedRightText = `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}\nRot: ${rightRotX.toFixed(0)} ${rightRotY.toFixed(0)} ${rightRotZ.toFixed(0)}`;
        this.rightHandInfoText.setAttribute('value', combinedRightText);
    }

  }
});


// Add the component to the scene after it's loaded
document.addEventListener('DOMContentLoaded', (event) => {
    const scene = document.querySelector('a-scene');

    if (scene) {
        // Listen for controller connection events
        scene.addEventListener('controllerconnected', (evt) => {
            console.log('Controller CONNECTED:', evt.detail.name, evt.detail.component.data.hand);
        });
        scene.addEventListener('controllerdisconnected', (evt) => {
            console.log('Controller DISCONNECTED:', evt.detail.name, evt.detail.component.data.hand);
        });

        // Add controller-updater component when scene is loaded (A-Frame manages session)
        if (scene.hasLoaded) {
            scene.setAttribute('controller-updater', '');
            console.log("controller-updater component added immediately.");
        } else {
            scene.addEventListener('loaded', () => {
                scene.setAttribute('controller-updater', '');
                console.log("controller-updater component added after scene loaded.");
            });
        }
    } else {
        console.error('A-Frame scene not found!');
    }


    // Add custom AR button logic
    addCustomARButton();
});

function addCustomARButton() {
    if (navigator.xr) {
        navigator.xr.isSessionSupported('immersive-ar').then((supported) => {
            if (supported) {
                const button = document.createElement('button');
                button.textContent = 'Enter AR (Custom Button)';
                button.style.position = 'fixed';
                button.style.bottom = '20px';
                button.style.left = '50%';
                button.style.transform = 'translateX(-50%)';
                button.style.padding = '12px';
                button.style.fontSize = '16px';
                button.style.backgroundColor = '#ff9800';
                button.style.color = 'white';
                button.style.border = 'none';
                button.style.borderRadius = '4px';
                button.style.cursor = 'pointer';
                button.style.zIndex = '9999'; // Ensure it's on top

                button.onclick = () => {
                    console.log('Custom AR button clicked. Requesting session via A-Frame...');
                    const sceneEl = document.querySelector('a-scene');
                    if (sceneEl) {
                        // Use A-Frame's enterVR to handle session start
                        sceneEl.enterVR(true).catch((err) => {
                            console.error('A-Frame failed to enter VR/AR:', err);
                            alert(`Failed to start AR session via A-Frame: ${err.message}`);
                        });
                    } else {
                         console.error('A-Frame scene not found for enterVR call!');
                    }
                    // Remove manual session request
                    // navigator.xr.requestSession('immersive-ar')
                    //     .then(onSessionStarted)
                    //     .catch((err) => {
                    //         console.error('Failed to start immersive-ar session:', err);
                    //         alert(`Failed to start AR session: ${err.message}`);
                    //     });
                };

                document.body.appendChild(button);
                console.log('Custom AR button added (uses scene.enterVR).');
            } else {
                console.warn('immersive-ar session not supported by this browser/device.');
            }
        }).catch((err) => {
            console.error('Error checking immersive-ar support:', err);
        });
    } else {
        console.warn('WebXR not supported by this browser.');
    }
}

// REMOVED onSessionStarted function as A-Frame handles session now
/*
function onSessionStarted(session) {
    console.log('Immersive AR session started successfully!');
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl && sceneEl.renderer) {
        sceneEl.renderer.xr.setReferenceSpaceType('local'); // Reverted back to 'local'
        sceneEl.renderer.xr.setSession(session).then(() => {
            console.log('A-Frame renderer set with AR session using local reference space.'); // Reverted log message

            // ADD controller-updater component AFTER session is set
            if (!sceneEl.hasAttribute('controller-updater')) {
                sceneEl.setAttribute('controller-updater', '');
                console.log("controller-updater component added AFTER session was set.");
            }

            // Optionally hide the custom button after entering AR
            const customButton = document.querySelector('button');
            if (customButton && customButton.textContent.includes('Custom Button')) {
                customButton.style.display = 'none';
            }
        }).catch(err => {
             console.error('Error setting A-Frame renderer session:', err);
        });

        session.addEventListener('end', () => {
            console.log('AR session ended.');
             // Optionally show the button again
            const customButton = document.querySelector('button');
            if (customButton && customButton.textContent.includes('Custom Button')) {
                customButton.style.display = 'block';
            }
        });
    } else {
        console.error('A-Frame scene or renderer not ready when session started.');
    }
} 
*/ 