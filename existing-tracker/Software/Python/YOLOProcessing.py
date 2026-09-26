import cv2
import socket
import threading
import time
import numpy as np
import argparse
from ultralytics import YOLO

# ─── CLI ARGUMENTS ────────────────────────────────────────────────────────────
ap = argparse.ArgumentParser(description="Camera Node Tracker – Simulation Mode")
ap.add_argument("--source", default="0",
                help="Video source: 0=webcam, or path to a video file")
ap.add_argument("--model", default="yolov8n.pt",
                help="YOLO model (default: yolov8n.pt, auto-downloads ~6 MB)")
ap.add_argument("--class-id", dest="class_id", type=int, default=None,
                help="COCO class to track: 0=person 2=car 15=cat (omit = highest conf)")
args = ap.parse_args()

VIDEO_SOURCE = int(args.source) if args.source.isdigit() else args.source

# ─── SYSTEM CONFIG (simulation – localhost) ───────────────────────────────────
SYSTEM_CONFIG = [
    {"id": 0, "name": "Node 0", "motor_ip": "127.0.0.1", "motor_port": 3333},
    {"id": 1, "name": "Node 1", "motor_ip": "127.0.0.1", "motor_port": 3334},
]

# ─── TUNING ───────────────────────────────────────────────────────────────────
YOLO_MODEL_PATH    = args.model
TRACK_CLASS_ID     = args.class_id   # None → track any class
YOLO_SIZE          = 320
CONFIDENCE         = 0.40
LOOKAHEAD_TIME     = 0.40
PREDICTION_TIMEOUT = 0.50
INFERENCE_INTERVAL = 0.050           # ~20 FPS detection

# ─── DISPLAY ──────────────────────────────────────────────────────────────────
DISPLAY_WIDTH      = 640
DISPLAY_HEIGHT     = 480
BUTTON_AREA_HEIGHT = 60
BTN_W, BTN_H       = 160, 40
BTN_X = (DISPLAY_WIDTH - BTN_W) // 2
BTN_Y = (BUTTON_AREA_HEIGHT - BTN_H) // 2


# ─── KALMAN TRACKER ───────────────────────────────────────────────────────────
class KalmanTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array(
            [[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array(
            [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov     = np.eye(4, dtype=np.float32) * 0.01
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1.0
        self.last_time = time.time()
        self.found = False

    def predict_only(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()
        self.kalman.transitionMatrix[0, 2] = dt
        self.kalman.transitionMatrix[1, 3] = dt
        self.kalman.predict()
        self.kalman.statePost = self.kalman.statePre.copy()

    def correct(self, x, y):
        self.kalman.correct(np.array([[np.float32(x)], [np.float32(y)]]))
        self.found = True

    def stop_prediction(self):
        self.kalman.statePost[2] = 0
        self.kalman.statePost[3] = 0
        self.found = False

    def get_prediction(self, lookahead=0.0):
        if not self.found:
            return 320, 240
        s = self.kalman.statePost
        return s[0][0] + s[2][0] * lookahead, s[1][0] + s[3][0] * lookahead


# ─── VIDEO STREAM ─────────────────────────────────────────────────────────────
class VideoStream:
    def __init__(self, src=0):
        self.src = src
        self.stream = None
        self.stopped = False
        self.frame = None
        self.lock = threading.Lock()
        self.connected = False

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def force_reconnect(self):
        with self.lock:
            if self.stream:
                self.stream.release()
            self.stream = None
            self.connected = False

    def _update(self):
        while not self.stopped:
            if self.stream is None or not self.stream.isOpened():
                self.connected = False
                try:
                    self.stream = cv2.VideoCapture(self.src)
                    self.stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    if not self.stream.isOpened():
                        time.sleep(2.0)
                        continue
                    self.connected = True
                    print(f"[Stream] Connected: {self.src}")
                except Exception as e:
                    print(f"[Stream] Error: {e}")
                    time.sleep(2.0)
                    continue

            grabbed, frame = self.stream.read()

            if grabbed:
                with self.lock:
                    self.frame = frame
            else:
                # Video file: loop; live stream: reconnect
                is_file = isinstance(self.src, str) and not self.src.startswith("http")
                if is_file:
                    self.stream.set(cv2.CAP_PROP_POS_FRAMES, 0)
                else:
                    self.stream.release()
                    self.stream = None
                    self.connected = False
                    time.sleep(0.1)

    def read(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.stopped = True
        if self.stream:
            self.stream.release()


# ─── DRONE CONTROLLER ─────────────────────────────────────────────────────────
class DroneController:
    def __init__(self, config, sock, shared_stream=None):
        self.config = config
        self.sock   = sock
        self.name   = config["name"]

        # Use shared stream so both nodes see the same feed on one camera
        self.stream      = shared_stream if shared_stream is not None else VideoStream(VIDEO_SOURCE).start()
        self._owns_stream = shared_stream is None

        self.tracker   = KalmanTracker()
        self.is_active = False

        self.running    = True
        self.udp_thread = threading.Thread(target=self._udp_sender_loop, daemon=True)
        self.udp_thread.start()

        self.last_inference_time = 0
        self.last_detection_time = 0

        # Auto-activate the simulator node
        self._send_cmd("active")
        self.is_active = True

    def _send_cmd(self, cmd):
        try:
            self.sock.sendto(cmd.encode(),
                             (self.config["motor_ip"], self.config["motor_port"]))
        except Exception:
            pass

    def _udp_sender_loop(self):
        ip   = self.config["motor_ip"]
        port = self.config["motor_port"]
        while self.running:
            px, py   = self.tracker.get_prediction(lookahead=LOOKAHEAD_TIME)
            norm_x   = max(-1.0, min(1.0, (px - 320.0) / 320.0))
            norm_y   = max(-1.0, min(1.0, (240.0 - py) / 240.0))
            try:
                self.sock.sendto(f"{norm_x:.3f},{norm_y:.3f}".encode(), (ip, port))
            except Exception:
                pass
            time.sleep(0.016)   # ~60 Hz

    def process(self, model):
        raw_frame = self.stream.read()

        if raw_frame is None:
            frame = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)
            msg   = "Connecting..." if not self.stream.connected else "No Signal"
            cv2.putText(frame, f"{self.name}: {msg}", (50, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            frame = cv2.resize(raw_frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
            now   = time.time()

            self.tracker.predict_only()

            if now - self.last_inference_time > INFERENCE_INTERVAL:
                results = model.predict(frame, conf=CONFIDENCE, verbose=False, imgsz=YOLO_SIZE)
                self.last_inference_time = now

                boxes = list(results[0].boxes) if results[0].boxes else []
                if TRACK_CLASS_ID is not None:
                    boxes = [b for b in boxes if int(b.cls[0]) == TRACK_CLASS_ID]

                if boxes:
                    best = max(boxes, key=lambda b: float(b.conf[0]))
                    x, y, w, h = best.xywh[0].cpu().numpy()
                    self.tracker.correct(x, y)
                    self.last_detection_time = now

                    x1, y1 = int(x - w / 2), int(y - h / 2)
                    x2, y2 = int(x + w / 2), int(y + h / 2)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{model.names[int(best.cls[0])]}  {float(best.conf[0]):.2f}"
                    cv2.putText(frame, label, (x1, max(y1 - 8, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if now - self.last_detection_time > PREDICTION_TIMEOUT:
                self.tracker.stop_prediction()

            # Draw tracker dots
            px, py = self.tracker.get_prediction(0)
            fx, fy = self.tracker.get_prediction(LOOKAHEAD_TIME)
            cv2.circle(frame, (int(px), int(py)), 6, (0, 255, 0), -1)   # current (green)
            cv2.circle(frame, (int(fx), int(fy)), 6, (0, 0, 255), -1)   # predicted (red)

            # Status overlay
            state_text  = "ACTIVE"   if self.is_active else "INACTIVE"
            state_color = (0, 255, 0) if self.is_active else (0, 0, 200)
            cv2.putText(frame, f"{self.name}  [{state_text}]", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

        # ── Button bar ────────────────────────────────────────────────────────
        bar = np.full((BUTTON_AREA_HEIGHT, DISPLAY_WIDTH, 3), (50, 50, 50), dtype=np.uint8)
        btn_color = (0, 130, 0) if self.is_active else (0, 0, 160)
        btn_label = "DEACTIVATE" if self.is_active else "ACTIVATE"
        cv2.rectangle(bar, (BTN_X, BTN_Y), (BTN_X + BTN_W, BTN_Y + BTN_H), btn_color, -1)
        ts = cv2.getTextSize(btn_label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        cv2.putText(bar, btn_label,
                    (BTN_X + (BTN_W - ts[0]) // 2, BTN_Y + (BTN_H + ts[1]) // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return np.vstack((frame, bar))

    def handle_click(self, x, y):
        if y >= DISPLAY_HEIGHT:
            ui_y = y - DISPLAY_HEIGHT
            if (BTN_X <= x <= BTN_X + BTN_W) and (BTN_Y <= ui_y <= BTN_Y + BTN_H):
                if self.is_active:
                    self._send_cmd("disabled")
                    self.is_active = False
                else:
                    self._send_cmd("active")
                    self.is_active = True

    def stop(self):
        self.running = False
        if self._owns_stream:
            self.stream.stop()


# ─── MOUSE CALLBACK ───────────────────────────────────────────────────────────
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        controllers = param
        col = x // DISPLAY_WIDTH
        if 0 <= col < len(controllers):
            controllers[col].handle_click(x % DISPLAY_WIDTH, y)


# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    print("=" * 54)
    print("  Distributed Camera Node Tracker  |  Simulation Mode")
    print("=" * 54)
    print(f"  Source  : {VIDEO_SOURCE}")
    print(f"  Model   : {YOLO_MODEL_PATH}")
    print(f"  Class   : {TRACK_CLASS_ID if TRACK_CLASS_ID is not None else 'any'}")
    print("=" * 54)
    print("  Also run esp32_simulator.py for Unity heading data.")
    print("=" * 54)
    print()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Loading YOLO model...")
    model = YOLO(YOLO_MODEL_PATH)

    # One shared stream – both virtual nodes see the same feed
    shared = VideoStream(VIDEO_SOURCE).start()
    time.sleep(1.0)  # let stream connect before processing

    controllers = [DroneController(cfg, sock, shared_stream=shared)
                   for cfg in SYSTEM_CONFIG]

    win = "Multi-Node Tracker  |  Q = quit"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, mouse_callback, controllers)

    try:
        while True:
            frames = [c.process(model) for c in controllers]
            cv2.imshow(win, np.hstack(frames))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down...")
        for c in controllers:
            c.stop()
        shared.stop()
        cv2.destroyAllWindows()
        sock.close()


if __name__ == "__main__":
    main()
