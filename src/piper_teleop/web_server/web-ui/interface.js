// Flow State Manager
const FlowState = {
  LANDING: 'landing',
  BOOKING: 'booking',
};

let currentState = FlowState.LANDING;

// Local storage keys
const STORAGE_KEYS = {
  HAS_SCHEDULED: 'tactile_has_scheduled_demo',
  LAST_STATE: 'tactile_last_state',
};

// Check if user has previously indicated they scheduled a demo
function hasScheduledDemo() {
  return localStorage.getItem(STORAGE_KEYS.HAS_SCHEDULED) === 'true';
}

// Save that user has scheduled a demo
function setScheduledDemo(scheduled) {
  localStorage.setItem(STORAGE_KEYS.HAS_SCHEDULED, scheduled.toString());
}

// Get elements
function getElements() {
  return {
    landingScreen: document.getElementById('landingScreen'),
    desktopInterface: document.getElementById('desktopInterface'),
    calendlySection: document.getElementById('calendlySection'),
    backToLanding: document.getElementById('backToLanding'),
  };
}

// Update UI based on current state
function updateUIForState() {
  const elements = getElements();

  // Hide all screens first
  elements.landingScreen.style.display = 'none';
  elements.desktopInterface.style.display = 'none';
  elements.calendlySection.style.display = 'none';
  elements.backToLanding.style.display = 'none';

  if (currentState === FlowState.LANDING) {
    // Show landing screen
    elements.landingScreen.style.display = 'flex';
  } else if (currentState === FlowState.BOOKING) {
    // Show booking screen
    elements.desktopInterface.style.display = 'block';
    elements.calendlySection.style.display = 'block';
    elements.backToLanding.style.display = 'block';
  }
}

// Transition to a new state
function transitionToState(newState) {
  currentState = newState;
  localStorage.setItem(STORAGE_KEYS.LAST_STATE, newState);
  updateUIForState();
}

// Initialize UI
function initializeUI() {
  // Check URL hash for direct booking link
  if (window.location.hash === '#booking') {
    transitionToState(FlowState.BOOKING);
  } else {
    // Start with landing screen
    transitionToState(FlowState.LANDING);
  }
}

// Event handlers for buttons
function setupEventHandlers() {
  // Desktop landing buttons
  const continueToDemo = document.getElementById('continueToDemo');
  const bookDemo = document.getElementById('bookDemo');
  const backToLanding = document.getElementById('backToLanding');
  const quickBookDemo = document.getElementById('quickBookDemo');

  if (continueToDemo) {
    continueToDemo.addEventListener('click', () => {
      setScheduledDemo(true);
      // Redirect to VR teleop page
      window.location.href = 'vr-teleop.html';
    });
  }

  if (bookDemo) {
    bookDemo.addEventListener('click', () => {
      transitionToState(FlowState.BOOKING);
      // Ensure Calendly widget loads
      if (window.Calendly && window.Calendly.initInlineWidget) {
        setTimeout(() => {
          const calendlyElement = document.querySelector(
            '.calendly-inline-widget',
          );
          if (calendlyElement) {
            window.Calendly.initInlineWidget({
              url: calendlyElement.getAttribute('data-url'),
              parentElement: calendlyElement,
            });
          }
        }, 500);
      }
    });
  }

  if (backToLanding) {
    backToLanding.addEventListener('click', () => {
      setScheduledDemo(false);
      transitionToState(FlowState.LANDING);
    });
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

  // Set up event handlers
  setupEventHandlers();

  // Initialize UI
  initializeUI();
});
