import os
import csv
import datetime
import time
import threading
import math
import numpy as np
from scipy.fft import rfft, rfftfreq
from scipy.signal import find_peaks
from flask import Flask, jsonify, render_template, request
import websocket  # websocket-client library

# --- Configuration ---
# The ESP32 hotspot assigns itself 192.168.4.1 by default
ESP32_WS_URL = 'ws://192.168.4.1:81'

app = Flask(__name__)

# Create dataset directory if it doesn't exist
DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dataset')
os.makedirs(DATASET_DIR, exist_ok=True)

# --- Thread-Safe Data Storage ---
sensor_data = {
    'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
    'raw_ax': 0, 'raw_ay': 0, 'raw_az': 0,
    'raw_gx': 0, 'raw_gy': 0, 'raw_gz': 0,
    'ax_g': 0.0, 'ay_g': 0.0, 'az_g': 0.0,
    'gx_dps': 0.0, 'gy_dps': 0.0, 'gz_dps': 0.0,
    'dsp_freq': 0.0, 'dsp_amp': 0.0, 'dsp_axis': 'None',
    'needs_label': False,
    'episode_duration': 0.0,
    'compound_signal': False,
    'xai_fft_amps': [],
    'xai_fft_freqs': []
}
data_lock = threading.Lock()

# --- Active Learning / Episode Logic ---
pending_episode_snapshots = []
episode_active = False
episode_start_time = 0
episode_max_intensity = 0.0
episode_peak_fft = [] 
episode_peak_freqs = []
compound_signal_flag = False

# --- DSP Buffers ---
BUFFER_SIZE = 256
SAMPLING_RATE = 50.0 # 50 Hz
buffer_x = []
buffer_y = []
buffer_z = []
buffer_gx = []
buffer_gy = []
buffer_gz = []

# --- Temporal Consistency Filter Variables ---
last_dom_freq = 0.0
freq_streak_count = 0

# --- Complementary Filter Variables ---
dt = 0.02  
pitch = 0.0
roll = 0.0
yaw = 0.0

ACCEL_SENSITIVITY = 16384.0
GYRO_SENSITIVITY = 131.0


def process_sensor_line(line):
    """Processes a single CSV line of sensor data."""
    global sensor_data, pitch, roll, yaw

    try:
        values = [int(v) for v in line.split(',')]
        
        if len(values) == 6:
            raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = values
            
            ax_g = raw_ax / ACCEL_SENSITIVITY
            ay_g = raw_ay / ACCEL_SENSITIVITY
            az_g = raw_az / ACCEL_SENSITIVITY
            
            gx_dps = raw_gx / GYRO_SENSITIVITY
            gy_dps = raw_gy / GYRO_SENSITIVITY
            gz_dps = raw_gz / GYRO_SENSITIVITY

            accel_pitch = -math.degrees(math.atan2(ay_g, math.sqrt(ax_g**2 + az_g**2)))
            accel_roll = math.degrees(math.atan2(-ax_g, az_g))

            alpha = 0.96
            pitch = alpha * (pitch + gx_dps * dt) + (1.0 - alpha) * accel_pitch
            roll = alpha * (roll + gy_dps * dt) + (1.0 - alpha) * accel_roll
            yaw = yaw + gz_dps * dt

            # Update DSP buffers
            if len(buffer_x) >= BUFFER_SIZE:
                buffer_x.pop(0)
                buffer_y.pop(0)
                buffer_z.pop(0)
                buffer_gx.pop(0)
                buffer_gy.pop(0)
                buffer_gz.pop(0)
            buffer_x.append(ax_g)
            buffer_y.append(ay_g)
            buffer_z.append(az_g)
            buffer_gx.append(gx_dps)
            buffer_gy.append(gy_dps)
            buffer_gz.append(gz_dps)

            with data_lock:
                sensor_data['pitch'] = pitch
                sensor_data['roll'] = roll
                sensor_data['yaw'] = yaw
                sensor_data['raw_ax'] = raw_ax
                sensor_data['raw_ay'] = raw_ay
                sensor_data['raw_az'] = raw_az
                sensor_data['raw_gx'] = raw_gx
                sensor_data['raw_gy'] = raw_gy
                sensor_data['raw_gz'] = raw_gz
                sensor_data['ax_g'] = ax_g
                sensor_data['ay_g'] = ay_g
                sensor_data['az_g'] = az_g
                sensor_data['gx_dps'] = gx_dps
                sensor_data['gy_dps'] = gy_dps
                sensor_data['gz_dps'] = gz_dps
        
    except (ValueError, IndexError):
        pass 


def websocket_reader():
    """Connects to the ESP32 WebSocket server and reads sensor data."""
    def on_message(ws_conn, message):
        line = message.strip()
        if "Initialized" in line or not line:
            return
        process_sensor_line(line)

    def on_error(ws_conn, error):
        print(f"WebSocket error: {error}")

    def on_close(ws_conn, close_status_code, close_msg):
        print("WebSocket connection closed. Reconnecting in 3s...")

    def on_open(ws_conn):
        print(f"Successfully connected to ESP32 at {ESP32_WS_URL}")

    while True:
        try:
            ws_conn = websocket.WebSocketApp(
                ESP32_WS_URL,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close,
                on_open=on_open
            )
            ws_conn.run_forever()
        except Exception as e:
            print(f"WebSocket connection failed: {e}")
        
        print(f"Waiting for ESP32 hotspot... Retrying in 3s.")
        time.sleep(3)


def dsp_worker():
    """Runs FFT on the sliding window every 0.5s to find dominant tremor frequencies."""
    global sensor_data, buffer_x, buffer_y, buffer_z
    global episode_active, episode_max_intensity, episode_peak_fft, episode_peak_freqs, pending_episode_snapshots, compound_signal_flag, episode_start_time
    global last_dom_freq, freq_streak_count
    
    while True:
        time.sleep(0.5)
        
        if len(buffer_x) < BUFFER_SIZE:
            continue
            
        x_data = np.array(buffer_x)
        y_data = np.array(buffer_y)
        z_data = np.array(buffer_z)
        
        x_data = x_data - np.mean(x_data)
        y_data = y_data - np.mean(y_data)
        z_data = z_data - np.mean(z_data)

        freqs = rfftfreq(BUFFER_SIZE, d=1.0/SAMPLING_RATE)
        valid_idx = np.where((freqs >= 4.0) & (freqs <= 12.0))[0]
        
        if len(valid_idx) == 0:
            continue
            
        fft_x = np.abs(rfft(x_data))
        fft_y = np.abs(rfft(y_data))
        fft_z = np.abs(rfft(z_data))
        
        band_freqs = freqs[valid_idx]
        band_x = fft_x[valid_idx]
        band_y = fft_y[valid_idx]
        band_z = fft_z[valid_idx]
        
        highest_amp = 0
        dom_freq = 0
        dom_axis = 'None'
        active_peaks = 0
        best_band_raw = []

        # Find multiple peaks! Prominence=0.8 effectively ignores noise flutter.
        for ax_name, band_data in [('X', band_x), ('Y', band_y), ('Z', band_z)]:
            peaks, properties = find_peaks(band_data, prominence=0.8)
            
            if len(peaks) > 0:
                active_peaks += len(peaks)
                
            max_idx = np.argmax(band_data)
            if band_data[max_idx] > highest_amp:
                highest_amp = band_data[max_idx]
                dom_freq = band_freqs[max_idx]
                dom_axis = ax_name
                best_band_raw = list(band_data)
                
        is_compound = (active_peaks > 1)
        
        STREAK_THRESHOLD = 11 
        
        activity_window = 25
        recent_x = x_data[-activity_window:]
        recent_y = y_data[-activity_window:]
        recent_z = z_data[-activity_window:]
        
        is_moving_now = False
        if dom_axis == 'X':
            is_moving_now = (np.max(recent_x) - np.min(recent_x)) > 0.1
        elif dom_axis == 'Y':
            is_moving_now = (np.max(recent_y) - np.min(recent_y)) > 0.1
        elif dom_axis == 'Z':
            is_moving_now = (np.max(recent_z) - np.min(recent_z)) > 0.1

        if highest_amp > 1.5 and is_moving_now: 
            if abs(dom_freq - last_dom_freq) <= 0.6: 
                freq_streak_count += 1
            else:
                freq_streak_count = 1 
                
            last_dom_freq = dom_freq
            
            if freq_streak_count >= STREAK_THRESHOLD:
                # WE ARE IN AN EPISODE
                if not episode_active:
                    episode_active = True
                    episode_start_time = time.time()
                    pending_episode_snapshots = []
                    episode_max_intensity = 0.0
                    compound_signal_flag = False
                    print(f"--- EPISODE STARTED! Frequency: {dom_freq:.1f}Hz ---")
                
                if is_compound:
                    compound_signal_flag = True
                
                if highest_amp > episode_max_intensity:
                    episode_max_intensity = highest_amp
                    episode_peak_fft = list(best_band_raw)
                    episode_peak_freqs = list(band_freqs)
                
                # Take 4 diverse snapshots periodically
                if len(pending_episode_snapshots) == 0 or (freq_streak_count % 10 == 0 and len(pending_episode_snapshots) < 4):
                    pending_episode_snapshots.append({
                        'x': list(buffer_x), 'y': list(buffer_y), 'z': list(buffer_z),
                        'gx': list(buffer_gx), 'gy': list(buffer_gy), 'gz': list(buffer_gz)
                    })
                    print(f"Episode: Saved snapshot #{len(pending_episode_snapshots)}")

                with data_lock:
                    sensor_data['dsp_freq'] = dom_freq
                    sensor_data['dsp_amp'] = highest_amp / (BUFFER_SIZE/2)
                    sensor_data['dsp_axis'] = 'Accel ' + dom_axis
            else:
                with data_lock:
                    sensor_data['dsp_freq'] = 0.0
                    sensor_data['dsp_amp'] = 0.0
                    sensor_data['dsp_axis'] = 'None'
        else:
            # END OF EPISODE LOGIC
            if episode_active and freq_streak_count > 0:
                duration = time.time() - episode_start_time
                print(f"--- EPISODE ENDED! Duration: {duration:.1f}s. Triggering UI ---")
                
                with data_lock:
                    sensor_data['needs_label'] = True
                    sensor_data['episode_duration'] = duration
                    sensor_data['compound_signal'] = compound_signal_flag
                    sensor_data['xai_fft_amps'] = episode_peak_fft
                    sensor_data['xai_fft_freqs'] = episode_peak_freqs
                    
                episode_active = False

            freq_streak_count = 0
            last_dom_freq = 0.0
            with data_lock:
                sensor_data['dsp_freq'] = 0.0
                sensor_data['dsp_amp'] = 0.0
                sensor_data['dsp_axis'] = 'None'


# --- Routes ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/log_data', methods=['POST'])
def log_data():
    global pending_episode_snapshots
    data = request.json
    label = data.get('label', 'Unknown')
    
    with data_lock:
        sensor_data['needs_label'] = False # Reset the UI flag
        
    if not pending_episode_snapshots:
        return jsonify({"status": "error", "message": "No pending data"})
        
    base_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    for idx, window in enumerate(pending_episode_snapshots):
        filename = f"{label}_slice{idx+1}_{base_timestamp}.csv"
        filepath = os.path.join(DATASET_DIR, filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps'])
            for i in range(len(window['x'])):
                writer.writerow([
                    window['x'][i], window['y'][i], window['z'][i],
                    window['gx'][i], window['gy'][i], window['gz'][i]
                ])
                
    saved_count = len(pending_episode_snapshots)
    pending_episode_snapshots = []
    print(f"Data successfully saved {saved_count} slices to disk!")
    return jsonify({"status": "success", "message": f"Saved {saved_count} files"})

@app.route('/data')
def get_data():
    with data_lock:
        return jsonify(sensor_data)


if __name__ == '__main__':
    thread = threading.Thread(target=websocket_reader, daemon=True)
    thread.start()
    
    dsp_thread = threading.Thread(target=dsp_worker, daemon=True)
    dsp_thread.start()
    
    app.run(host='0.0.0.0', port=5000, debug=False)
