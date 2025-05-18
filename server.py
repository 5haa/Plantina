import cv2
import numpy as np
from flask import Flask, request, jsonify, render_template, Response
import os
import datetime
import socket
import serial
import threading
import time
import base64
import io
from queue import Queue
from threading import Lock

app = Flask(__name__)

# Create directories for storing images
os.makedirs('static/images', exist_ok=True)
os.makedirs('static/detections', exist_ok=True)
os.makedirs('models', exist_ok=True)

# Global variables
latest_frame = None
latest_detection_frame = None
frame_lock = Lock()
person_detected = False
use_webcam = True  # Only for browser camera toggle
use_browser_cam = True  # Default to browser camera
detection_counter = 0  # Counter for detection frequency reduction
detection_frequency = 2  # Only detect every N frames (reduced for better detection)

# Initialize MobileNet SSD for human detection
# Download model files if they don't exist
model_file = os.path.join('models', 'mobilenet_ssd.caffemodel')
config_file = os.path.join('models', 'mobilenet_ssd.prototxt')

# Check if model files exist, download them if not
if not os.path.exists(model_file) or not os.path.exists(config_file):
    print("=== Downloading human detection model files... ===")
    import urllib.request
    
    # Create models directory if it doesn't exist
    os.makedirs('models', exist_ok=True)
    
    # Download model files
    print("Downloading MobileNet SSD model file (this may take a minute)...")
    urllib.request.urlretrieve(
        "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/mobilenet_iter_73000.caffemodel",
        model_file
    )
    print(f"Model downloaded to {model_file}")
    
    print("Downloading MobileNet SSD config file...")
    urllib.request.urlretrieve(
        "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt",
        config_file
    )
    print(f"Config downloaded to {config_file}")
    print("=== Model files downloaded successfully ===")
else:
    print(f"Found existing model files at {model_file} and {config_file}")

# Load MobileNet SSD model
print("Loading MobileNet SSD model for human detection...")
detection_net = cv2.dnn.readNetFromCaffe(config_file, model_file)
print("Model loaded successfully!")

# Class names for MobileNet SSD
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

# Serial communication with Arduino (optional direct control)
serial_port = None
try:
    # Update COM port as needed - use device manager to find the correct port
    serial_port = serial.Serial('COM3', 9600, timeout=1)
    print("Connected to Arduino")
except:
    print("Failed to connect to Arduino - commands will only be sent via ESP32")

def detect_people(image, force_detection=False):
    """Detect people in the image using MobileNet SSD model"""
    global person_detected, detection_counter
    
    # Only run detection every N frames to improve performance
    # unless force_detection is True
    detection_counter += 1
    run_detection = force_detection or (detection_counter % detection_frequency == 0)
    
    # Create a copy for annotation
    detection_img = image.copy()
    
    if run_detection:
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Prepare image for DNN
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, 
                                     (300, 300), 127.5)
        
        # Set the input to the pre-trained DNN model
        detection_net.setInput(blob)
        
        # Get the detections
        detections = detection_net.forward()
        
        # Track if any people were detected
        people_found = False
        people_count = 0
        
        # Process the detections
        for i in range(0, detections.shape[2]):
            # Extract the confidence
            confidence = detections[0, 0, i, 2]
            
            # Filter out weak detections
            if confidence > 0.5:
                # Extract the index of the class label
                class_index = int(detections[0, 0, i, 1])
                
                # Check if the detected object is a person (class 15)
                if CLASSES[class_index] == "person":
                    people_found = True
                    people_count += 1
                    
                    # Compute the bounding box
                    box = detections[0, 0, i, 3:7] * np.array([width, height, width, height])
                    (startX, startY, endX, endY) = box.astype("int")
                    
                    # Make sure box coordinates are valid
                    startX = max(0, startX)
                    startY = max(0, startY)
                    endX = min(width, endX)
                    endY = min(height, endY)
                    
                    # Draw a more visible bounding box
                    cv2.rectangle(detection_img, (startX, startY), (endX, endY), (0, 255, 0), 3)
                    
                    # Add label with confidence
                    label = f"Person: {confidence * 100:.1f}%"
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    
                    # Make text more visible with background
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(detection_img, (startX, y - label_size[1] - 10), 
                                 (startX + label_size[0], y + 5), (0, 0, 0), -1)
                    cv2.putText(detection_img, label, (startX, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Update detection status
        person_detected = people_found
        
        # Print debug info when people are detected
        if people_found:
            print(f"Detected {people_count} people in the frame")
    
    # Add text with detection status (even on frames without detection)
    status_text = f"Human detected: {'YES' if person_detected else 'NO'}"
    
    # Make status text more visible
    cv2.rectangle(detection_img, (5, 5), (300, 40), (0, 0, 0), -1)
    cv2.putText(detection_img, status_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    # Add timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cv2.putText(detection_img, timestamp, (10, image.shape[0] - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return detection_img, person_detected

@app.route('/')
def index():
    """Render the main page"""
    return render_template('index.html', use_webcam=use_webcam, use_browser_cam=use_browser_cam)

@app.route('/upload', methods=['POST'])
def upload():
    """Receive and process images from ESP32-CAM"""
    global latest_frame, latest_detection_frame, person_detected
    
    if use_webcam:
        return "Using browser camera mode, ESP32-CAM upload ignored"
    
    # Get image data from request
    img_data = request.data
    
    # Convert to OpenCV format
    nparr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if img is None:
        return jsonify({"error": "Invalid image data"}), 400
    
    # Process image - force detection on ESP32-CAM images for better responsiveness
    processed_img, detected = detect_people(img, force_detection=True)
    
    # Store frames in memory instead of writing to disk
    with frame_lock:
        latest_frame = img.copy()
        latest_detection_frame = processed_img.copy()
    
    # Prepare minimal response based on detection
    if detected:
        response = "person_detected"
        if serial_port:
            serial_port.write(b'P')  # Send person detected command to Arduino
    else:
        response = "no_person"
        if serial_port:
            serial_port.write(b'N')  # Send no person command to Arduino
    
    return response

@app.route('/toggle_mode')
def toggle_mode():
    """Toggle between browser camera and ESP32-CAM modes"""
    global use_webcam
    
    use_webcam = not use_webcam
    
    return jsonify({"mode": "browser_cam" if use_webcam else "esp32cam", 
                    "success": True, 
                    "browser_cam": use_webcam})

@app.route('/browser_upload', methods=['POST'])
def browser_upload():
    """Process images captured from browser webcam"""
    global latest_frame, latest_detection_frame, person_detected
    
    # Get image data from request
    try:
        # The image is sent as a data URL, so we need to extract the base64 part
        img_data = request.json.get('image', '')
        if not img_data or not img_data.startswith('data:image'):
            return jsonify({"error": "Invalid image data"}), 400
        
        # Extract the base64 part
        img_data = img_data.split(',')[1]
        
        # Decode base64 to bytes
        img_bytes = base64.b64decode(img_data)
        
        # Convert to OpenCV format
        nparr = np.frombuffer(img_bytes, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if img is None or img.size == 0:
            return jsonify({"error": "Invalid image data"}), 400
        
        # Detect people - regular detection frequency for browser
        processed_img, detected = detect_people(img)
        
        # Store frames in memory
        with frame_lock:
            latest_frame = img.copy()
            latest_detection_frame = processed_img.copy()
        
        # Prepare minimal response based on detection
        result = {
            "detected": detected
        }
        
        # Send command to Arduino if connected
        if detected and serial_port:
            serial_port.write(b'P')  # Send person detected command to Arduino
        elif not detected and serial_port:
            serial_port.write(b'N')  # Send no person command to Arduino
        
        return jsonify(result)
    
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# Pre-generate empty frame with message for initial display
def create_empty_frame(width=640, height=480, message="Initializing camera..."):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(frame, message, (width//2-100, height//2), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

# Initialize with empty frame
with frame_lock:
    latest_detection_frame = create_empty_frame()

@app.route('/video_feed')
def video_feed():
    """Video streaming route for the web interface"""
    def generate():
        global latest_detection_frame
        
        # Frame rate control
        fps_target = 15
        frame_time = 1.0 / fps_target
        last_frame_time = time.time()
        
        # Initialize with debug message in frame
        debug_frame = create_empty_frame(message="Initializing detection system...")
        
        while True:
            # Limit frame rate to target FPS
            current_time = time.time()
            elapsed = current_time - last_frame_time
            if elapsed < frame_time:
                # Small sleep to reduce CPU usage while waiting
                time.sleep(max(0, (frame_time - elapsed) / 3))
                continue
            
            try:
                with frame_lock:
                    if latest_detection_frame is not None:
                        # Use in-memory frame
                        frame = latest_detection_frame.copy()
                    else:
                        # Use debug frame if no detection frame is available
                        frame = debug_frame
                        
                    # Convert to JPEG for HTTP streaming with optimized quality
                    # Better quality for detection but still optimized for performance
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ret:
                        continue
                        
                    frame_bytes = buffer.tobytes()
                    
                    # Update frame time
                    last_frame_time = time.time()
                    
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            except Exception as e:
                print(f"Error in video feed generation: {e}")
                time.sleep(0.1)  # Prevent busy-waiting in case of error
    
    return Response(generate(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# Create a simple HTML template
@app.route('/create_template')
def create_template():
    """This endpoint is deprecated since we now use the template system"""
    return render_template('index.html', use_webcam=use_webcam)

@app.route('/command')
def command():
    """Send manual commands to Arduino"""
    cmd = request.args.get('cmd', '')
    if serial_port and cmd:
        serial_port.write(cmd.encode())
        return f"Command {cmd} sent to Arduino"
    return "Failed to send command"

@app.route('/status')
def status():
    """Return current detection status"""
    return jsonify({
        "detected": person_detected,
        "mode": "browser_cam" if use_webcam else "esp32cam"
    })

if __name__ == '__main__':
    # Start the server on all network interfaces with highest performance settings
    app.run(host='0.0.0.0', port=5000, threaded=True, processes=1) 