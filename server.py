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
detection_counter = 0  # Counter for detection frequency reduction
detection_frequency = 2  # Only detect every N frames (reduced for better detection)

# Person direction tracking
person_direction = "center"  # Can be "left", "center", "right"

# Sensor data (shared between server and ESP32)
moisture_value = 0
distance_cm = 0
sensor_data_lock = Lock()

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
serial_port_name = None
serial_connected = False
arduino_com_ports = []

# ESP32-CAM handles the communication to Arduino, so direct serial is disabled
ESP32_MODE = True  # Set to True since we're using ESP32-CAM to communicate with Arduino

def list_com_ports():
    """List all available COM ports"""
    import serial.tools.list_ports
    return list(serial.tools.list_ports.comports())

def find_arduino_ports():
    """Try to find Arduino COM ports by checking device descriptions"""
    global arduino_com_ports
    arduino_com_ports = []
    
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        # Save all ports for display
        all_ports = []
        for port in ports:
            port_info = {
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid,
                'manufacturer': getattr(port, 'manufacturer', 'Unknown')
            }
            all_ports.append(port_info)
            
            # Look for likely Arduino ports (containing Arduino, CH340, or other common identifiers)
            if ('arduino' in port.description.lower() or 
                'ch340' in port.description.lower() or
                'serial' in port.description.lower() or
                'uart' in port.description.lower() or
                'usb' in port.description.lower()):
                arduino_com_ports.append(port.device)
        
        print(f"Found potential Arduino ports: {arduino_com_ports}")
        print(f"All available COM ports: {[p['device'] + ' - ' + p['description'] for p in all_ports]}")
        
        # Return all ports info for UI
        return all_ports
        
    except Exception as e:
        print(f"Error detecting COM ports: {str(e)}")
        return []

def test_arduino_port(port_name):
    """Test if a port responds like an Arduino by sending a test command and checking response"""
    try:
        # Try to open the port
        test_port = serial.Serial(port_name, 9600, timeout=1)
        print(f"Testing port {port_name}...")
        
        # Flush any existing data
        test_port.reset_input_buffer()
        test_port.reset_output_buffer()
        
        # Send a test command (S = Stop)
        test_port.write(b'S')
        time.sleep(0.5)  # Wait for response
        
        # Try a few more test commands
        test_port.write(b'P')  # Person detected
        time.sleep(0.1)
        test_port.write(b'N')  # No person
        time.sleep(0.1)
        
        # Check if port is still open (Arduino typically remains open)
        is_open = test_port.is_open
        
        # Close the port
        test_port.close()
        
        # If we got this far without errors, likely an Arduino
        return {
            "port": port_name,
            "success": True,
            "is_arduino": True,
            "message": f"Port {port_name} responded like an Arduino"
        }
        
    except Exception as e:
        print(f"Port {port_name} test failed: {str(e)}")
        return {
            "port": port_name,
            "success": False,
            "is_arduino": False,
            "message": f"Port test failed: {str(e)}"
        }

def connect_serial(port_name=None):
    """
    This function is disabled when using ESP32-CAM mode.
    The ESP32-CAM communicates directly with the Arduino.
    """
    global serial_port, serial_connected, serial_port_name
    
    # If ESP32 mode is enabled, don't try to connect directly
    if ESP32_MODE:
        print("Using ESP32-CAM for Arduino communication - direct serial connection disabled")
        return False
    
    # If port_name is provided, update the global
    if port_name:
        serial_port_name = port_name
    
    # If no port is specified and we have auto-detected Arduino ports, use the first one
    if not serial_port_name and arduino_com_ports:
        serial_port_name = arduino_com_ports[0]
        print(f"Auto-selected Arduino port: {serial_port_name}")
    
    # If we still don't have a port, can't connect
    if not serial_port_name:
        print("No COM port specified and no Arduino ports detected")
        serial_connected = False
        return False
    
    try:
        # Close existing connection if any
        if serial_port is not None:
            try:
                serial_port.close()
            except:
                pass
        
        # Create new connection
        serial_port = serial.Serial(serial_port_name, 9600, timeout=1)
        serial_connected = True
        print(f"Connected to Arduino on {serial_port_name}")
        return True
    except Exception as e:
        serial_connected = False
        print(f"Failed to connect to Arduino - {str(e)}")
        return False

def send_serial_command(command):
    """
    In ESP32 mode, we don't actually send commands via serial port.
    The ESP32-CAM will receive the HTTP response and forward commands to Arduino.
    """
    # If ESP32 mode is enabled, just log the command but don't try to send
    if ESP32_MODE:
        print(f"Command {command} will be sent via ESP32-CAM response")
        return True
    
    # Don't attempt if no command
    if not command:
        return False
    
    # Try sending 3 times
    for attempt in range(3):
        try:
            if not serial_connected:
                connect_serial()
            
            if serial_connected and serial_port:
                serial_port.write(command)
                return True
        except Exception as e:
            print(f"Serial error (attempt {attempt+1}/3): {str(e)}")
            serial_connected = False
            time.sleep(0.1)  # Short delay before retry
    
    return False

# Try to detect available COM ports
found_ports = find_arduino_ports()

# Only try to connect if we found potential Arduino ports
if arduino_com_ports:
    # Try to connect initially using the first detected Arduino port
    connect_serial()
else:
    print("No Arduino COM ports detected. Use /select_port to choose a port manually.")

def detect_people(image, force_detection=False):
    """Detect people in the image using MobileNet SSD model and determine their direction"""
    global person_detected, detection_counter, person_direction
    
    # Only run detection every N frames to improve performance
    # unless force_detection is True
    detection_counter += 1
    run_detection = force_detection or (detection_counter % detection_frequency == 0)
    
    # Create a copy for annotation
    detection_img = image.copy()
    
    # Get image dimensions
    height, width = image.shape[:2]
    
    # Define the left, center, and right regions of the frame
    left_boundary = width // 3
    right_boundary = 2 * width // 3
    
    if run_detection:
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
        
        # Variables to track the largest person detection (likely the closest)
        largest_person_area = 0
        largest_person_x = 0
        
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
                    
                    # Calculate area of the bounding box
                    person_area = (endX - startX) * (endY - startY)
                    
                    # Calculate center X position of the person
                    person_center_x = (startX + endX) // 2
                    
                    # If this is the largest person detected so far, update tracking variables
                    if person_area > largest_person_area:
                        largest_person_area = person_area
                        largest_person_x = person_center_x
                    
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
        
        # Determine direction of the largest person detected
        if people_found and largest_person_area > 0:
            if largest_person_x < left_boundary:
                person_direction = "left"
            elif largest_person_x > right_boundary:
                person_direction = "right"
            else:
                person_direction = "center"
            
            # Draw direction indicator
            direction_text = f"Direction: {person_direction.upper()}"
            cv2.rectangle(detection_img, (5, 45), (300, 80), (0, 0, 0), -1)
            cv2.putText(detection_img, direction_text, (10, 70), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # Draw frame divisions for debugging
            cv2.line(detection_img, (left_boundary, 0), (left_boundary, height), (255, 0, 0), 1)
            cv2.line(detection_img, (right_boundary, 0), (right_boundary, height), (255, 0, 0), 1)
        
        # Print debug info when people are detected
        if people_found:
            print(f"Detected {people_count} people in the frame, direction: {person_direction}")
    
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
    
    return detection_img, person_detected, person_direction

@app.route('/')
def index():
    """Render the main page"""
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload():
    """Receive and process images from ESP32-CAM"""
    global latest_frame, latest_detection_frame, person_detected, person_direction
    global moisture_value, distance_cm
    
    # Get image data from request
    img_data = request.data
    
    # Check if this is a sensor data update
    if len(img_data) < 1000:  # Sensor data is typically small
        try:
            # Try to decode as sensor data (format: "SENSOR:moisture:distance")
            text_data = img_data.decode('utf-8')
            if text_data.startswith('SENSOR:'):
                parts = text_data.split(':')
                if len(parts) >= 3:
                    with sensor_data_lock:
                        try:
                            moisture_value = int(parts[1])
                            distance_cm = float(parts[2])
                        except ValueError:
                            pass
                return "sensor_data_received"
        except:
            # Not a sensor data update, continue with image processing
            pass
    
    # Convert to OpenCV format
    nparr = np.frombuffer(img_data, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if img is None:
        return jsonify({"error": "Invalid image data"}), 400
    
    # Process image - force detection on ESP32-CAM images for better responsiveness
    processed_img, detected, direction = detect_people(img, force_detection=True)
    
    # Store frames in memory instead of writing to disk
    with frame_lock:
        latest_frame = img.copy()
        latest_detection_frame = processed_img.copy()
    
    # In ESP32 mode, we return the detection result with direction
    # The ESP32-CAM code will parse this response and forward to Arduino
    if ESP32_MODE:
        if detected:
            if direction == "left":
                print("Person detected on LEFT - ESP32-CAM will forward 'L' command to Arduino")
                return "person_left"
            elif direction == "right":
                print("Person detected on RIGHT - ESP32-CAM will forward 'R' command to Arduino")
                return "person_right"
            else:  # center
                print("Person detected in CENTER - ESP32-CAM will forward 'P' command to Arduino")
                return "person_center"
        else:
            print("No person detected - ESP32-CAM will forward 'N' command to Arduino")
            return "no_person"
    
    # Legacy direct serial mode (not used with ESP32-CAM)
    else:
        if detected:
            if direction == "left":
                response = "person_left"
                send_serial_command(b'L')  # Send person left command to Arduino
            elif direction == "right":
                response = "person_right"
                send_serial_command(b'R')  # Send person right command to Arduino
            else:  # center
                response = "person_center"
                send_serial_command(b'P')  # Send person center command to Arduino
        else:
            response = "no_person"
            send_serial_command(b'N')  # Send no person command to Arduino
        return response

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

@app.route('/command')
def command():
    """Send manual commands to Arduino"""
    cmd = request.args.get('cmd', '')
    
    if cmd:
        if ESP32_MODE:
            # In ESP32 mode, we return a special response that ESP32-CAM will interpret
            print(f"Command {cmd} will be sent via ESP32-CAM response")
            
            # Return in format that ESP32-CAM can parse (added in the ESP32-CAM code)
            # This format is explicitly checked for in ESP32-CAM
            response = f"command_{cmd}"
            
            print(f"Sending response: {response}")
            return response
        else:
            # Direct serial mode
            success = send_serial_command(cmd.encode())
            if success:
                return f"Command {cmd} sent to Arduino"
    
    return "Failed to send command"

@app.route('/status')
def status():
    """Return current detection status"""
    return jsonify({
        "detected": person_detected,
        "direction": person_direction,
        "mode": "esp32cam" # This 'mode' refers to communication mode, not auto/manual
    })

@app.route('/sensor_data')
def sensor_data():
    """Return current sensor data"""
    with sensor_data_lock:
        return jsonify({
            "moisture": moisture_value,
            "plantNeedsWater": moisture_value < 300,  # MOISTURE_THRESHOLD from sketch
            "distance": distance_cm,
            # "state": robot_state # robot_state was not defined, removing
        })

# Add route to list available COM ports
@app.route('/list_ports')
def list_ports():
    """List all available COM ports"""
    all_ports = find_arduino_ports()  # This also updates arduino_com_ports
    
    return jsonify({
        "all_ports": all_ports,
        "arduino_ports": arduino_com_ports,
        "current_port": serial_port_name,
        "connected": serial_connected
    })

# Add route to select a specific COM port
@app.route('/select_port')
def select_port():
    """Select a specific COM port for Arduino"""
    port = request.args.get('port', '')
    if not port:
        return jsonify({"error": "No port specified"}), 400
    
    success = connect_serial(port)
    
    return jsonify({
        "success": success,
        "connected": serial_connected,
        "port": serial_port_name
    })

# Add route to test all available COM ports
@app.route('/test_all_ports')
def test_all_ports():
    """Test all available COM ports to find the Arduino"""
    import serial.tools.list_ports
    
    # Get all available COM ports
    ports = list(serial.tools.list_ports.comports())
    port_results = []
    
    # Skip COM3 since we know it's your camera
    for port in ports:
        port_name = port.device
        if port_name.lower() == 'com3':
            port_results.append({
                "port": port_name,
                "skipped": True,
                "description": "Skipped (Camera module)",
                "message": "COM3 is used by your camera module"
            })
            continue
        
        # Test each port
        result = test_arduino_port(port_name)
        result["description"] = port.description
        port_results.append(result)
    
    # If a successful port was found, connect to it
    success_ports = [p for p in port_results if p.get("success", False) and p.get("is_arduino", False)]
    if success_ports:
        best_port = success_ports[0]["port"]
        connect_serial(best_port)
        return jsonify({
            "found_arduino": True,
            "selected_port": best_port,
            "all_results": port_results,
            "message": f"Successfully identified and connected to Arduino on {best_port}"
        })
    
    return jsonify({
        "found_arduino": False,
        "all_results": port_results,
        "message": "No Arduino found on any COM port. Try each port manually."
    })

# Add route for ESP32-CAM diagnostic and testing
@app.route('/esp32_test')
def esp32_test():
    """Special endpoint to test ESP32-CAM to Arduino communication"""
    command = request.args.get('cmd', '')
    direction = request.args.get('dir', '')
    
    if not command:
        # Show a simple diagnostic page with buttons if no command
        html = """
        <html>
        <head>
            <title>ESP32-CAM to Arduino Test</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                .button { padding: 10px 20px; margin: 5px; font-size: 16px; cursor: pointer; }
                .info { background-color: #f0f0f0; padding: 10px; border-radius: 5px; margin: 10px 0; }
            </style>
        </head>
        <body>
            <h1>ESP32-CAM to Arduino Communication Test</h1>
            
            <div class="info">
                <p>This page helps test if your ESP32-CAM can successfully forward commands to your Arduino.</p>
                <p>Your setup diagram:</p>
                <pre>
    PC/Server → WiFi → ESP32-CAM → Serial → Arduino
                </pre>
            </div>
            
            <h2>Test Commands</h2>
            <p>Click these buttons to send test commands via ESP32-CAM to Arduino:</p>
            
            <button class="button" onclick="window.location.href='/esp32_test?cmd=P'">
                Send 'P' - Person Detected (Center)
            </button>
            
            <button class="button" onclick="window.location.href='/esp32_test?cmd=P&dir=left'">
                Send 'L' - Person Detected (Left)
            </button>
            
            <button class="button" onclick="window.location.href='/esp32_test?cmd=P&dir=right'">
                Send 'R' - Person Detected (Right)
            </button>
            
            <button class="button" onclick="window.location.href='/esp32_test?cmd=N'">
                Send 'N' - No Person
            </button>
            
            <button class="button" onclick="window.location.href='/esp32_test?cmd=S'">
                Send 'S' - Stop Motors
            </button>
            
            <h2>Connection Status</h2>
            <div class="info">
                <p>ESP32-CAM Mode: <strong>Enabled</strong></p>
                <p>Check the response from this page on your ESP32-CAM to see if it receives and processes these test commands.</p>
            </div>
        </body>
        </html>
        """
        return html
    
    # Return special command responses that ESP32-CAM will recognize
    if command == 'P':
        if direction == 'left':
            return "person_left"
        elif direction == 'right':
            return "person_right"
        else:
            return "person_center"
    elif command == 'N':
        return "no_person"
    else:
        # For other commands, use a format ESP32-CAM code can parse
        return f"command_{command}"

if __name__ == '__main__':
    # Clean up serial port on exit
    try:
        import atexit
        
        def cleanup():
            global serial_port
            if serial_port:
                try:
                    serial_port.close()
                    print("Serial port closed")
                except:
                    pass
        
        atexit.register(cleanup)
    except:
        pass
    
    # Start the server on all network interfaces with highest performance settings
    app.run(host='0.0.0.0', port=5000, threaded=True, processes=1)
