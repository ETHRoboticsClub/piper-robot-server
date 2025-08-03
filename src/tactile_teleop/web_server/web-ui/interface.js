// VR interface detection and switching
function updateUIForDevice() {
    const desktopInterface = document.getElementById('desktopInterface');
  
    // Check if this is a VR-capable device
    if (navigator.xr) {
      navigator.xr
        .isSessionSupported('immersive-vr')
        .then((supported) => {
          if (supported) {
            // VR-capable device - hide desktop interface
            desktopInterface.style.display = 'none';
          } else {
            // Not VR-capable - show desktop interface
            desktopInterface.style.display = 'block';
          }
        })
        .catch(() => {
          // Fallback to desktop interface if XR check fails
          desktopInterface.style.display = 'block';
        });
    } else {
      // No XR support - show desktop interface
      desktopInterface.style.display = 'block';
    }
  }
  
  // Initialize when page loads
  document.addEventListener('DOMContentLoaded', function () {
    // Update VR server URL dynamically
    const vrUrlElement = document.getElementById('vrServerUrl');
    if (vrUrlElement) {
      const currentUrl = window.location.origin;
      vrUrlElement.textContent = currentUrl + '/';
    }
  
    // Set up VR interface detection
    updateUIForDevice();
  
    // Handle VR session changes
    if (navigator.xr) {
      navigator.xr.addEventListener('sessionstart', updateUIForDevice);
      navigator.xr.addEventListener('sessionend', updateUIForDevice);
    }
  });
  
  // Handle window resize
  window.addEventListener('resize', updateUIForDevice);
  