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
    fetch('/command?cmd=' + command)
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
    fetch('/toggle_mode')
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
    document.getElementById('video-feed').onload = function() {
        updateFps();
    };
    
    // Force video feed refresh every 5 seconds in case of stalls
    setInterval(function() {
        refreshVideoStream();
    }, 5000);
    
    // Initialize the UI based on server state
    const useWebcam = document.body.getAttribute('data-webcam') === 'true';
    
    // Update mode displays
    document.getElementById('mode-name').innerText = useWebcam ? 'Browser Camera' : 'ESP32-CAM';
    document.querySelector('.mode-button').innerText = 'Switch to ' + (useWebcam ? 'ESP32-CAM' : 'Browser Camera');
    
    // Show/hide browser camera section
    document.getElementById('browser-cam-section').style.display = useWebcam ? 'block' : 'none';
    
    // Start browser camera if we're in webcam mode
    if (useWebcam) {
        setTimeout(startBrowserCamera, 500);
    }
}); 