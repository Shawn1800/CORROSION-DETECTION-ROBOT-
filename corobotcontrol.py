import os
import serial
import time
import io
import cv2
import numpy as np
import threading
from flask import Flask, render_template, Response, jsonify, send_from_directory
from picamera2 import Picamera2
from ultralytics import YOLO

# ----------------------- Global Variables -----------------------
ultrasonic_distance1 = None  # Front sensor
ultrasonic_distance2 = None  # Back sensor
gps_data = None
ULTRASONIC_THRESHOLD = 50  # cm threshold for obstacle detection
SERIAL_RECONNECT_DELAY = 1  # seconds delay before reconnecting serial
AUTONOMOUS_DECISION_DELAY = 0.1  # delay between decisions in autonomous mode
DEFAULT_SPEED = 70  # Default speed (0-255)
MIN_SPEED = 70      # Minimum speed for movement
MAX_SPEED = 100      # Maximum speed

start_time = time.time()
frame_count = 0
last_fps_time = time.time()
fps = 0

autonomous_mode = False
corrosion_alert = False
alert_image_path = None

last_command = None
last_command_time = time.time()
command_count = 0

YOLO_SKIP = 10
last_yolo_result = None
frame_counter = 0

# ----------------------- Serial Connection -----------------------
def init_serial_connection():
    ports = ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']
    for port in ports:
        try:
            ser = serial.Serial(port, 2000000, timeout=1)
            print(f"Connected to {port}")
            time.sleep(2)
            return ser
        except serial.SerialException as e:
            print(f"Failed to connect to {port}: {e}")
    print("Warning: No serial connection found, using dummy serial")
    class DummySerial:
        def write(self, data): pass
        def readline(self): return b""
        def close(self): pass
        @property
        def in_waiting(self): return 0
    return DummySerial()

ser = init_serial_connection()

# ----------------------- Sensor Data Listener -----------------------
def serial_listener():
    global ultrasonic_distance1, ultrasonic_distance2, gps_data, ser
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                print(f"Received: {line}")
                if "ULTRASONIC1:" in line and "ULTRASONIC2:" in line:
                    try:
                        parts = line.split(" | ")
                        ultrasonic_distance1 = float(parts[0].split("ULTRASONIC1: ")[1].strip())
                        ultrasonic_distance2 = float(parts[1].split("ULTRASONIC2: ")[1].strip())
                    except (ValueError, IndexError) as e:
                        print(f"Error parsing ultrasonic data: {e}")
                elif "GPS:" in line:
                    gps_data = line.split("GPS:")[1].strip()
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Serial error: {e}. Attempting to reconnect...")
            try:
                ser.close()
            except Exception as ex:
                print(f"Error closing serial: {ex}")
            time.sleep(SERIAL_RECONNECT_DELAY)
            ser = init_serial_connection()

listener_thread = threading.Thread(target=serial_listener, daemon=True)
listener_thread.start()

# ----------------------- Flask Application -----------------------
app = Flask(__name__, static_folder='alerts', static_url_path='/alerts')
picam = None

# ----------------------- Camera Initialization -----------------------
def initialize_camera():
    global picam
    try:
        picam = Picamera2()
        config = picam.create_video_configuration(main={"size": (320, 240)})
        picam.configure(config)
        picam.start()
        print("Camera initialized successfully")
        return True
    except Exception as e:
        print(f"Camera initialization failed: {e}")
        return False

# ----------------------- YOLO Model Loading -----------------------
try:
    yolo_model = YOLO("best14_ncnn_model", task='detect')
    print("YOLO model loaded successfully")
except Exception as e:
    print(f"YOLO model loading failed: {e}")
    class DummyYOLO:
        def __call__(self, img):
            class DummyResult:
                def __init__(self):
                    self.boxes = None
                    self.names = {0: "Dummy"}
                def plot(self):
                    return img
            return [DummyResult()]
    yolo_model = DummyYOLO()

# ----------------------- Command Sending with Speed -----------------------
def send_command(command, speed=DEFAULT_SPEED):
    global last_command, last_command_time, command_count
    if command in ['F', 'B', 'L', 'R', 'S']:
        speed = max(MIN_SPEED, min(speed, MAX_SPEED))  # Clamp speed between MIN and MAX
        full_command = f"{command}{speed:03d}"  # e.g., "F150" for forward at speed 150
        current_time = time.time()
        if full_command != last_command or (current_time - last_command_time) > 0.5:
            try:
                ser.write(full_command.encode())
                command_count += 1
                last_command = full_command
                last_command_time = current_time
                print(f"Sent command: {full_command}")
                return True
            except Exception as e:
                print(f"Command send failed: {e}")
                return False
    return False

# ----------------------- YOLO Processing -----------------------
def run_yolo_on_frame(img):
    try:
        img_resized = cv2.resize(img, (416, 416)) if img.shape[:2] != (416, 416) else img
        results = yolo_model(img_resized)
        annotated_img = results[0].plot()
        alert_triggered = False
        if results[0].boxes:
            labels = [results[0].names[int(cls)].lower() for cls in results[0].boxes.cls.tolist()]
            if "corrosion" in labels:
                alert_triggered = True
        return annotated_img, alert_triggered
    except Exception as e:
        print(f"YOLO processing error: {e}")
        return img, False

# ----------------------- Autonomous Navigation Logic -----------------------
def autonomous_navigation():
    global ultrasonic_distance1, ultrasonic_distance2
    if ultrasonic_distance1 is None or ultrasonic_distance2 is None:
        print("Ultrasonic sensor data not available.")
        send_command('S')  # Stop if no sensor data
        return

    # Obstacle avoidance logic
    if ultrasonic_distance1 < ULTRASONIC_THRESHOLD:
        if ultrasonic_distance2 < ULTRASONIC_THRESHOLD:
            print("Obstacles front and back: stopping")
            send_command('S')  # Stop if surrounded
        else:
            print("Front obstacle detected: turning right")
            send_command('R', speed=100)  # Turn right at moderate speed
            time.sleep(0.5)  # Brief turn duration
            send_command('F', speed=DEFAULT_SPEED)  # Resume forward
    elif ultrasonic_distance2 < ULTRASONIC_THRESHOLD:
        print("Rear obstacle detected: turning left")
        send_command('L', speed=100)  # Turn left at moderate speed
        time.sleep(0.5)  # Brief turn duration
        send_command('F', speed=DEFAULT_SPEED)  # Resume forward
    else:
        print("No obstacles: moving forward")
        send_command('F', speed=DEFAULT_SPEED)  # Move forward at default speed

# ----------------------- Frame Processing -----------------------
def process_frame(frame):
    global last_yolo_result, frame_count, last_fps_time, frame_counter, fps, corrosion_alert, alert_image_path
    try:
        img = np.frombuffer(frame, dtype=np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        
        frame_counter += 1
        if frame_counter % YOLO_SKIP == 0:
            annotated_img, alert_triggered = run_yolo_on_frame(img)
            last_yolo_result = annotated_img
            if alert_triggered and not corrosion_alert:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                os.makedirs("alerts", exist_ok=True)
                filename = f"alert_{timestamp}.jpg"
                alert_image_path = f"/alerts/{filename}"
                cv2.imwrite(os.path.join("alerts", filename), annotated_img)
                corrosion_alert = True
                print("Corrosion alert triggered!")
        else:
            annotated_img = last_yolo_result if last_yolo_result is not None else img

        if autonomous_mode:
            autonomous_navigation()
            time.sleep(AUTONOMOUS_DECISION_DELAY)

        cv2.putText(annotated_img, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(annotated_img, f"CMD: {last_command or 'None'}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(annotated_img, f"Front US: {ultrasonic_distance1 if ultrasonic_distance1 is not None else 'N/A'} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(annotated_img, f"Rear US: {ultrasonic_distance2 if ultrasonic_distance2 is not None else 'N/A'} cm", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(annotated_img, f"Mode: {'AUTO' if autonomous_mode else 'MANUAL'}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        frame_count += 1
        current_time = time.time()
        if current_time - last_fps_time >= 1.0:
            fps = frame_count / (current_time - last_fps_time)
            frame_count = 0
            last_fps_time = current_time

        ret, buffer = cv2.imencode('.jpg', annotated_img)
        return buffer.tobytes()
    except Exception as e:
        print(f"Frame processing error: {e}")
        blank = np.ones((480, 640, 3), dtype=np.uint8) * 255
        cv2.putText(blank, "Processing Error", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        ret, buffer = cv2.imencode('.jpg', blank)
        return buffer.tobytes()

# ----------------------- Frame Generator -----------------------
def generate_frames():
    while True:
        try:
            if picam is None:
                blank = np.ones((480, 640, 3), dtype=np.uint8) * 255
                cv2.putText(blank, "Camera not initialized", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                ret, buffer = cv2.imencode('.jpg', blank)
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                time.sleep(1)
                continue
                
            stream = io.BytesIO()
            picam.capture_file(stream, format='jpeg')
            stream.seek(0)
            frame = stream.getvalue()
            processed_frame = process_frame(frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + processed_frame + b'\r\n')
            stream.close()
            time.sleep(0.05)
        except Exception as e:
            print(f"Frame generation error: {e}")
            time.sleep(1)

# ----------------------- Flask Routes -----------------------
@app.route('/')
def index():
    return render_template('diddy1.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/control/<command>', methods=['POST'])
def control_car(command):
    cmd_map = {'forward': 'F', 'backward': 'B', 'left': 'L', 'right': 'R', 'stop': 'S'}
    if command in cmd_map:
        success = send_command(cmd_map[command])  # Default speed
        return jsonify({"success": success, "command": command}), 200
    return jsonify({"success": False, "error": "Invalid command"}), 400

@app.route('/control/<command>/<int:speed>', methods=['POST'])
def control_car_with_speed(command, speed):
    cmd_map = {'forward': 'F', 'backward': 'B', 'left': 'L', 'right': 'R', 'stop': 'S'}
    if command in cmd_map:
        success = send_command(cmd_map[command], speed)
        return jsonify({"success": success, "command": command, "speed": speed}), 200
    return jsonify({"success": False, "error": "Invalid command"}), 400

@app.route('/toggle_autonomous', methods=['POST'])
def toggle_autonomous():
    global autonomous_mode
    autonomous_mode = not autonomous_mode
    if not autonomous_mode:
        send_command('S')
    return jsonify({'autonomous': autonomous_mode}), 200

@app.route('/get_alert')
def get_alert():
    global corrosion_alert, alert_image_path
    if corrosion_alert:
        alert_info = {"alert": True, "image": alert_image_path}
        corrosion_alert = False
        alert_image_path = None
    else:
        alert_info = {"alert": False}
    return jsonify(alert_info)

@app.route('/status')
def get_status():
    current_time = time.time()
    runtime = current_time - start_time
    return jsonify({
        "fps": round(fps, 2),
        "runtime": int(runtime),
        "lastCommand": last_command,
        "mode": "Autonomous" if autonomous_mode else "Manual",
        "ultrasonic1": ultrasonic_distance1,
        "ultrasonic2": ultrasonic_distance2,
        "gps": gps_data
    })

@app.route('/sensors')
def get_sensors():
    return jsonify({
        "ultrasonic1": ultrasonic_distance1,
        "ultrasonic2": ultrasonic_distance2,
        "gps": gps_data
    })

@app.route('/alerts/<path:filename>')
def serve_alert_image(filename):
    return send_from_directory('alerts', filename)

# ----------------------- Main Entry Point -----------------------
if __name__ == '__main__':
    try:
        camera_ready = initialize_camera()
        if not camera_ready:
            print("Warning: Running without camera")
        app.run(host='0.0.0.0', port=8080, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down gracefully...")
        if picam:
            picam.stop()
        ser.close()
    except Exception as e:
        print(f"Fatal error: {e}")
        if picam:
            picam.stop()
        ser.close()
