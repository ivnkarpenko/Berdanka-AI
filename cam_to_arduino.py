import time
import socket
import threading
import math

import cv2
import serial
from ultralytics import YOLO

# ================== SETTINGS ==================
ARDUINO_PORT = "COM3"
BAUD = 115200

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

CAM_INDEX = 0
W, H = 480, 320
WINDOW_NAME = "Berdanka-AI (ESC / Q to quit)"

MODEL_NAME = "yolo11n.pt"  # если не пойдет: "yolov8n.pt"

# roll compensation: можно выключить, если надо сравнить
USE_IMU_ROLL_COMP = True

# ================== SERIAL OPEN (robust) ==================
def open_serial(port: str, baud: int):
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.05
        ser.write_timeout = 0.05

        # flow control off
        ser.dsrdtr = False
        ser.rtscts = False
        ser.xonxoff = False

        ser.open()

        # avoid toggling lines too aggressively
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass

        # many boards reset on open
        time.sleep(2.0)

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        print(f"[SERIAL] Opened {port} @ {baud}")
        return ser
    except Exception as e:
        print(f"[SERIAL] Open failed: {e}")
        return None

ser = open_serial(ARDUINO_PORT, BAUD)

# ================== IMU SHARED STATE ==================
imu_roll = 0.0
imu_pitch = 0.0
imu_yaw = 0.0
imu_status = ""
imu_last_ms = 0
imu_lock = threading.Lock()

def parse_imu_line(line: str):
    """
    Expected:
      ANG:roll,pitch,yaw;S:All is well.
    S part is optional.
    """
    global imu_roll, imu_pitch, imu_yaw, imu_status, imu_last_ms

    if not line.startswith("ANG:"):
        return

    try:
        s_idx = line.find(";S:")
        ang_part = line[4:s_idx] if s_idx >= 0 else line[4:]
        st_part = line[s_idx + 3:] if s_idx >= 0 else ""

        vals = ang_part.split(",")
        if len(vals) >= 3:
            r = float(vals[0])
            p = float(vals[1])
            y = float(vals[2])
            with imu_lock:
                imu_roll, imu_pitch, imu_yaw = r, p, y
                imu_status = st_part.strip()
                imu_last_ms = int(time.time() * 1000)
    except Exception:
        pass

def serial_reader_thread():
    """
    Reads incoming IMU lines from Arduino on the same COM port.
    """
    if ser is None:
        return

    buf = ""
    while True:
        try:
            chunk = ser.read(256)
            if not chunk:
                time.sleep(0.005)
                continue

            buf += chunk.decode("utf-8", errors="ignore")
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip()
                if line:
                    parse_imu_line(line)
        except Exception:
            break

if ser is not None:
    threading.Thread(target=serial_reader_thread, daemon=True).start()

# ================== UDP ==================
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_to_processing(msg: str):
    try:
        udp.sendto(msg.encode("utf-8"), (UDP_IP, UDP_PORT))
    except Exception:
        pass

def send_to_arduino(msg: str):
    if ser is None:
        return
    try:
        ser.write(msg.encode("utf-8"))
    except Exception:
        pass

def send_xy(cx: int, cy: int, ok: bool, conf: float, roll: float, pitch: float, yaw: float, status: str):
    """
    Unified packet to Processing (UDP) and Arduino (Serial).
    Added IMU fields for debugging.
    """
    # XY:... plus IMU fields (optional)
    # Example:
    # XY:240,160;OK:1;C:0.83;R:1.2;P:-3.4;Y:120.0;S:All is well.
    safe_status = status.replace("\n", " ").replace("\r", " ")
    msg = (
        f"XY:{cx},{cy};OK:{1 if ok else 0};C:{conf:.2f}"
        f";R:{roll:.2f};P:{pitch:.2f};Y:{yaw:.2f};S:{safe_status}\n"
    )

    send_to_processing(msg)
    send_to_arduino(msg)

# ================== IMU COMPENSATION (ROLL) ==================
def correct_xy_by_roll(cx: int, cy: int, roll_deg: float) -> tuple[int, int]:
    """
    Stabilize target coords by compensating roll around the screen center.
    """
    dx = cx - W * 0.5
    dy = cy - H * 0.5

    r = math.radians(-roll_deg)  # compensate roll
    c = math.cos(r)
    s = math.sin(r)

    dx2 = dx * c - dy * s
    dy2 = dx * s + dy * c

    cx2 = int(W * 0.5 + dx2)
    cy2 = int(H * 0.5 + dy2)

    cx2 = max(0, min(W - 1, cx2))
    cy2 = max(0, min(H - 1, cy2))
    return cx2, cy2

# ================== YOLO + CAMERA ==================
model = YOLO(MODEL_NAME)

cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

def draw_cross(img, x, y, size=10, thickness=2):
    cv2.line(img, (x - size, y), (x + size, y), (0, 255, 0), thickness)
    cv2.line(img, (x, y - size), (x, y + size), (0, 255, 0), thickness)

def put_text(img, text, x, y, scale=0.5, thickness=1):
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (255, 255, 255), thickness, cv2.LINE_AA)

# ================== MAIN LOOP ==================
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        # no frame: send center with ok=0
        with imu_lock:
            r, p, y = imu_roll, imu_pitch, imu_yaw
            st = imu_status
        send_xy(W // 2, H // 2, ok=False, conf=0.0, roll=r, pitch=p, yaw=y, status=st)
        continue

    frame = cv2.resize(frame, (W, H))

    # YOLO
    results = model.predict(frame, verbose=False)[0]

    best = None
    best_conf = 0.0

    # pick best person (COCO class 0)
    if results.boxes is not None and len(results.boxes) > 0:
        for b in results.boxes:
            cls = int(b.cls[0])
            conf = float(b.conf[0])
            if cls == 0 and conf > best_conf:
                best = b
                best_conf = conf

    # read IMU snapshot
    with imu_lock:
        r, p, y = imu_roll, imu_pitch, imu_yaw
        st = imu_status
        last_ms = imu_last_ms

    imu_age = (int(time.time() * 1000) - last_ms) if last_ms else 10_000

    if best is None:
        cx_raw, cy_raw = W // 2, H // 2
        cx_send, cy_send = cx_raw, cy_raw
        ok = False
        conf = 0.0
    else:
        x1, y1, x2, y2 = map(int, best.xyxy[0].tolist())
        cx_raw = (x1 + x2) // 2
        cy_raw = (y1 + y2) // 2

        # compensate roll if enabled and IMU is fresh enough
        if USE_IMU_ROLL_COMP and imu_age < 5000:
            cx_send, cy_send = correct_xy_by_roll(cx_raw, cy_raw, r)
        else:
            cx_send, cy_send = cx_raw, cy_raw

        ok = True
        conf = best_conf

        # draw bbox + raw center
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        draw_cross(frame, cx_raw, cy_raw, size=10, thickness=2)

        # draw compensated center as small dot (for debug)
        if (cx_send, cy_send) != (cx_raw, cy_raw):
            cv2.circle(frame, (cx_send, cy_send), 3, (0, 255, 255), -1)

    # send packet
    send_xy(cx_send, cy_send, ok=ok, conf=conf, roll=r, pitch=p, yaw=y, status=st)

    # overlay info
    put_text(frame, f"person={'YES' if ok else 'NO'} conf={conf:.2f}", 8, 18)
    put_text(frame, f"raw:  cx={cx_raw} cy={cy_raw}", 8, 38)
    put_text(frame, f"send: cx={cx_send} cy={cy_send} (roll-comp={'ON' if USE_IMU_ROLL_COMP else 'OFF'})", 8, 58)

    put_text(frame, f"IMU: R={r:.1f} P={p:.1f} Y={y:.1f} age={imu_age}ms", 8, 78)
    if st:
        put_text(frame, f"S: {st[:60]}", 8, 98)

    put_text(frame, f"UDP {UDP_IP}:{UDP_PORT} | Serial {'OK' if ser else 'OFF'} {ARDUINO_PORT}", 8, 118)

    cv2.imshow(WINDOW_NAME, frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q'):
        break

# ================== CLEANUP ==================
cap.release()
try:
    if ser:
        ser.close()
except Exception:
    pass
udp.close()
cv2.destroyAllWindows()
