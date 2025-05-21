// Global variables for browser camera
let browserVideo;
let browserStream;
let browserCamActive = false;
let captureInterval = null;

// Performance monitoring
let frameCounter = 0;
let lastFpsCheck = Date.now();
let currentFps = 0;

// Function to send commands to the robot
function sendCommand(command) {
    // Commands can now be sent regardless of any "mode"
    fetch('/command?cmd=' + command) // Removed '&manual=true' as it's no longer needed
        .then(response => response.text())
        .then(data => {
            console.log('Command sent:', data);
        });
}

// Update status periodically
setInterval(function() {
    fetch('/status')
        .then(response => response.json())
        .then(data => {
            document.getElementById('detection-status').innerText = 
                data.detected ? 'Person Detected!' : 'No Person Detected';
        });
}, 1000);

// Fetch sensor data periodically
setInterval(function() {
    fetch('/sensor_data')
        .then(response => response.json())
        .then(data => {
            document.getElementById('moisture-value').innerText = data.moisture;
            document.getElementById('moisture-status').innerText = 
                data.plantNeedsWater ? 'Needs Water' : 'Moist';
            document.getElementById('distance-value').innerText = data.distance;
            // document.getElementById('robot-state').innerText = data.state; // 'state' might not be available if robot_state was removed from Python
            // If 'data.state' is indeed removed from the Python backend, you should comment out or remove the line above.
            // For now, I'll assume it might still be part of the sensor data for other purposes.
            // If you confirm it's gone, remove the line above.
            if (data.state !== undefined) {
                 document.getElementById('robot-state').innerText = data.state;
            } else {
                 document.getElementById('robot-state').innerText = "--"; // Default if state is not sent
            }
        })
        .catch(error => {
            console.error('Error fetching sensor data:', error);
        });
}, 1000);

// Add a timestamp to video feed URL to prevent caching
function refreshVideoStream() {
    const videoFeed = document.getElementById('video-feed');
    if (videoFeed) {
        const timestamp = new Date().getTime();
        const baseUrl = videoFeed.src.split('?')[0];
        videoFeed.src = baseUrl + '?t=' + timestamp;
    }
}

// Performance monitoring
function updateFps() {
    frameCounter++;
    const now = Date.now();
    const elapsed = now - lastFpsCheck;
    
    if (elapsed >= 1000) {  // Update every second
        currentFps = Math.round((frameCounter * 1000) / elapsed);
        document.getElementById('performance-stats').textContent = 'FPS: ' + currentFps;
        frameCounter = 0;
        lastFpsCheck = now;
    }
}

// Toggle between browser cam and ESP32-CAM
function toggleMode() {
    fetch('/toggle_mode') // Assuming you have a '/toggle_mode' endpoint in your Flask app
        .then(response => response.json())
        .then(data => {
            let modeName = data.mode === 'browser_cam' ? 'Browser Camera' : 'ESP32-CAM';
            document.getElementById('mode-name').innerText = modeName;
            
            // Update toggle button text
            let toggleButton = document.querySelector('.mode-button');
            toggleButton.innerText = 'Switch to ' + (data.mode === 'browser_cam' ? 'ESP32-CAM' : 'Browser Camera');
            
            // Stop browser camera if switching to ESP32-CAM mode
            if (data.mode === 'esp32cam' && browserCamActive) {
                stopBrowserCamera();
            }
            
            // Show browser camera option only in browser camera mode
            document.getElementById('browser-cam-section').style.display = 
                (data.mode === 'browser_cam') ? 'block' : 'none';
                
            // Start browser camera automatically in browser cam mode
            if (data.mode === 'browser_cam' && !browserCamActive) {
                setTimeout(startBrowserCamera, 500);
            }
            
            // Refresh video stream
            refreshVideoStream();
        })
        .catch(error => {
            console.error('Error toggling mode:', error);
            // Fallback or error message to user
            document.getElementById('mode-name').innerText = "Error switching mode";
        });
}

// Browser camera functions
function startBrowserCamera() {
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
        navigator.mediaDevices.getUserMedia({ 
            video: { 
                width: { ideal: 640 },
                height: { ideal: 480 },
                frameRate: { ideal: 20 }  // Higher frame rate
            } 
        })
        .then(function(stream) {
            browserStream = stream;
            browserVideo.srcObject = stream;
            browserCamActive = true;
            
            // Update UI
            document.getElementById('browser-start-btn').disabled = true;
            document.getElementById('browser-stop-btn').disabled = false;
            document.getElementById('browser-status').innerText = 'Camera active. Detection running automatically.';
            
            // Start capture at higher frequency for smoother video
            captureInterval = setInterval(captureBrowserFrame, 50);
        })
        .catch(function(error) {
            console.error('Error accessing browser camera:', error);
            document.getElementById('browser-status').innerText = 
                'Error accessing camera: ' + error.message;
        });
    } else {
        document.getElementById('browser-status').innerText = 
            'Browser does not support camera access.';
    }
}

function stopBrowserCamera() {
    if (browserStream) {
        browserStream.getTracks().forEach(track => track.stop());
        browserVideo.srcObject = null;
        browserCamActive = false;
        
        // Stop the automatic capture
        if (captureInterval) {
            clearInterval(captureInterval);
            captureInterval = null;
        }
        
        // Update UI
        document.getElementById('browser-start-btn').disabled = false;
        document.getElementById('browser-stop-btn').disabled = true;
        document.getElementById('browser-status').innerText = 'Camera stopped.';
    }
}

// Throttle variables to prevent overwhelming the server
let lastCaptureSent = 0;
const captureMinInterval = 80; // 12.5 fps target (1000/12.5 ~= 80ms)

function captureBrowserFrame() {
    if (!browserCamActive) {
        return;
    }
    
    // Check if enough time has passed since last capture
    const now = Date.now();
    if (now - lastCaptureSent < captureMinInterval) {
        return;
    }
    
    // Create a canvas to capture the frame
    const canvas = document.createElement('canvas');
    canvas.width = browserVideo.videoWidth;
    canvas.height = browserVideo.videoHeight;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(browserVideo, 0, 0, canvas.width, canvas.height);
    
    // Get the data URL with reduced quality for better performance
    const dataURL = canvas.toDataURL('image/jpeg', 0.6);
    
    // Update timestamp
    lastCaptureSent = now;
    
    // Send to server without updating status (to avoid flickering)
    // Assuming you have a '/browser_upload' endpoint
    fetch('/browser_upload', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({ image: dataURL })
    })
    .then(response => response.json())
    .then(data => {
        if (data.error) {
            document.getElementById('browser-status').innerText = 'Error: ' + data.error;
        }
    })
    .catch(error => {
        console.error('Error sending image:', error);
    });
}

// Set up on document load
document.addEventListener('DOMContentLoaded', function() {
    browserVideo = document.getElementById('browser-video');
    
    // Monitor video feed performance
    const videoFeedElement = document.getElementById('video-feed');
    if (videoFeedElement) {
        videoFeedElement.onload = function() {
            updateFps();
        };
    }
    
    // Force video feed refresh every 5 seconds in case of stalls
    setInterval(function() {
        refreshVideoStream();
    }, 5000);
    
    // Initialize the UI based on server state
    const useWebcamInitial = document.body.getAttribute('data-webcam') === 'true';
    
    // Update mode displays
    // Assuming the initial mode is ESP32-CAM unless 'data-webcam' is explicitly true
    // or you fetch this initial state from the server.
    // For simplicity, let's assume default is ESP32-CAM if not specified by 'data-webcam'
    let initialModeName = useWebcamInitial ? 'Browser Camera' : 'ESP32-CAM';
    document.getElementById('mode-name').innerText = initialModeName;
    let toggleButton = document.querySelector('.mode-button');
    if (toggleButton) {
        toggleButton.innerText = 'Switch to ' + (initialModeName === 'Browser Camera' ? 'ESP32-CAM' : 'Browser Camera');
    }
    
    const browserCamSection = document.getElementById('browser-cam-section');
    if (browserCamSection) {
        browserCamSection.style.display = useWebcamInitial ? 'block' : 'none';
    }
    
    // Start browser camera if we're in webcam mode
    if (useWebcamInitial && browserVideo) { // ensure browserVideo is found
        setTimeout(startBrowserCamera, 500);
    }
});