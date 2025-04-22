// Wait for A-Frame scene to load
AFRAME.registerComponent('controller-updater', {
  init: function () {
    console.log("Controller updater component initialized (but controllers are disabled for now).");
    // Controllers are disabled for debugging session start
    /*
    this.leftHand = document.querySelector('#leftHand');
    this.rightHand = document.querySelector('#rightHand');
    this.leftHandPosText = document.querySelector('#leftHandPos');
    this.leftHandRotText = document.querySelector('#leftHandRot');
    this.rightHandPosText = document.querySelector('#rightHandPos');
    this.rightHandRotText = document.querySelector('#rightHandRot');

    if (!this.leftHand || !this.rightHand) {
      console.error("Controller entities not found!");
      return;
    }
    */
  },

  tick: function () {
    // Controllers are disabled for debugging session start
    /*
    if (!this.leftHand.object3D.visible && !this.rightHand.object3D.visible) {
        return; // Don't update if controllers aren't tracking
    }

    // Update Left Hand Text
    if (this.leftHand.object3D.visible) {
        const leftPos = this.leftHand.object3D.position;
        const leftRot = this.leftHand.object3D.rotation; // Euler angles in radians
        this.leftHandPosText.setAttribute('value', `Pos: ${leftPos.x.toFixed(2)} ${leftPos.y.toFixed(2)} ${leftPos.z.toFixed(2)}`);
        this.leftHandRotText.setAttribute('value', `Rot: ${THREE.MathUtils.radToDeg(leftRot.x).toFixed(0)} ${THREE.MathUtils.radToDeg(leftRot.y).toFixed(0)} ${THREE.MathUtils.radToDeg(leftRot.z).toFixed(0)}`);
    }

    // Update Right Hand Text
    if (this.rightHand.object3D.visible) {
        const rightPos = this.rightHand.object3D.position;
        const rightRot = this.rightHand.object3D.rotation; // Euler angles in radians
        this.rightHandPosText.setAttribute('value', `Pos: ${rightPos.x.toFixed(2)} ${rightPos.y.toFixed(2)} ${rightPos.z.toFixed(2)}`);
        this.rightHandRotText.setAttribute('value', `Rot: ${THREE.MathUtils.radToDeg(rightRot.x).toFixed(0)} ${THREE.MathUtils.radToDeg(rightRot.y).toFixed(0)} ${THREE.MathUtils.radToDeg(rightRot.z).toFixed(0)}`);
    }
    */
  }
});

// Add the component to the scene after it's loaded
document.addEventListener('DOMContentLoaded', (event) => {
    const scene = document.querySelector('a-scene');
    if (scene) {
        if (scene.hasLoaded) {
            // Don't add the controller-updater component for now
            // scene.setAttribute('controller-updater', '');
        } else {
            scene.addEventListener('loaded', () => {
                 // Don't add the controller-updater component for now
                // scene.setAttribute('controller-updater', '');
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
                    console.log('Custom AR button clicked. Requesting session...');
                    navigator.xr.requestSession('immersive-ar')
                        .then(onSessionStarted)
                        .catch((err) => {
                            console.error('Failed to start immersive-ar session:', err);
                            alert(`Failed to start AR session: ${err.message}`);
                        });
                };

                document.body.appendChild(button);
                console.log('Custom AR button added.');
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

function onSessionStarted(session) {
    console.log('Immersive AR session started successfully!');
    const sceneEl = document.querySelector('a-scene');
    if (sceneEl && sceneEl.renderer) {
        sceneEl.renderer.xr.setReferenceSpaceType('local');
        sceneEl.renderer.xr.setSession(session).then(() => {
            console.log('A-Frame renderer set with AR session.');
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