import argparse, json, math, re, socket, sys, time, threading, collections
from typing import Optional, Tuple, Dict
plt = None
import os, csv, queue, datetime as t


# ---------- UART line format ----------
LINE_RE = re.compile(
    r"^\s*(?:Received:\s*)?(?P<name>[^,]+)\s*,\s*"
    r"(?P<w>[+-]?\d*\.?\d+)\s*,\s*(?P<x>[+-]?\d*\.?\d+)\s*,\s*(?P<y>[+-]?\d*\.?\d+)\s*,\s*(?P<z>[+-]?\d*\.?\d+)"
)

S_HEALTHY_UP = "Healthy Upper Arm"
S_HEALTHY_LO = "Healthy Lower Arm"
S_EXO_UP     = "Exo Upper Arm"
S_EXO_LO     = "Exo Lower Arm"
S_JOINT_M2 = "Joint Motor 2 IMU"
SENSORS = [S_HEALTHY_UP, S_HEALTHY_LO, S_EXO_UP, S_EXO_LO]
ALIASES = {s.lower(): s for s in SENSORS}


# ---------- Math helpers ----------
def qnorm(w, x, y, z):
    n = (w*w + x*x + y*y + z*z) ** 0.5 or 1.0
    return (w/n, x/n, y/n, z/n)

def q_conj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    )

def unit(v):
    x, y, z = v
    n = (x*x + y*y + z*z) ** 0.5
    return (0.0, 0.0, 1.0) if n == 0 else (x/n, y/n, z/n)

def twist_deg_about_axis(q_rel, k=(0.0, 0.0, 1.0)):
    """
    Signed angle (deg) of relative quaternion q_rel around axis k.
    """
    w, x, y, z = q_rel
    s = k[0]*x + k[1]*y + k[2]*z
    return math.degrees(2.0 * math.atan2(s, w))

def wrap_deg(d):
    while d <= -180.0:
        d += 360.0
    while d > 180.0:
        d -= 360.0
    return d

def twist_deg_from_baseline(q0, q_now, k=(1.0, 0.0, 0.0)):
    """
    Angle (deg) of q_now relative to baseline q0 about axis k.
    Axis k is expressed in the baseline frame.
    Returns None if either quaternion is missing.
    """
    if not (q0 and q_now):
        return None
    q_rel = q_mul(q_conj(q0), q_now)
    return twist_deg_about_axis(q_rel, k)

class MetaWearBLEReader(threading.Thread):
    def __init__(self, mac: str, name="MetaWear"):
        super().__init__(daemon=True)
        self.mac = mac
        self.name = name
        self.q = None
        self.err = None
        self.stop_flag = False
        self.device = None

    def run(self):
        try:
            from mbientlab.metawear import MetaWear, libmetawear, parse_value
            from mbientlab.metawear.cbindings import SensorFusionMode, SensorFusionData, SensorFusionAccRange, SensorFusionGyroRange, FnVoid_VoidP_DataP
        except ImportError:
            self.err = "MetaWear library not installed. Try: pip3 install metawear"
            return

        def callback(ctx, data):
            val = parse_value(data)
            # val has w, x, y, z
            self.q = qnorm(val.w, val.x, val.y, val.z)

        try:
            self.device = MetaWear(self.mac)
            self.device.connect()
            print(f"[metawear] Connected to {self.name}: {self.mac}")

            self.callback = FnVoid_VoidP_DataP(callback)

            libmetawear.mbl_mw_sensor_fusion_set_mode(
                self.device.board,
                SensorFusionMode.IMU_PLUS
            )
            libmetawear.mbl_mw_sensor_fusion_set_acc_range(
                self.device.board,
                SensorFusionAccRange._8G
            )
            libmetawear.mbl_mw_sensor_fusion_set_gyro_range(
                self.device.board,
                SensorFusionGyroRange._2000DPS
            )
            libmetawear.mbl_mw_sensor_fusion_write_config(self.device.board)

            signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(
                self.device.board,
                SensorFusionData.QUATERNION
            )
            libmetawear.mbl_mw_datasignal_subscribe(signal, None, self.callback)

            libmetawear.mbl_mw_sensor_fusion_enable_data(
                self.device.board,
                SensorFusionData.QUATERNION
            )
            libmetawear.mbl_mw_sensor_fusion_start(self.device.board)

            while not self.stop_flag:
                time.sleep(0.05)

        except Exception as e:
            self.err = f"MetaWear BLE failed: {e}"

        finally:
            try:
                if self.device:
                    libmetawear.mbl_mw_sensor_fusion_stop(self.device.board)
                    libmetawear.mbl_mw_debug_disconnect(self.device.board)
            except:
                pass

    def stop(self):
        self.stop_flag = True


# ---------- PID ----------
class PID:
    def __init__(self, kp, ki, kd, dt, out_min=-90.0, out_max=90.0, i_min=-300.0, i_max=300.0, d_alpha=0.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = max(1e-3, dt)
        self.i = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0
        self.out_min, self.out_max = out_min, out_max
        self.i_min, self.i_max = i_min, i_max
        self.d_alpha = max(0.0, min(1.0, d_alpha))

    def reset(self):
        self.i = 0.0
        self.prev_err = 0.0
        self.d_filt = 0.0

    def step(self, err):
        self.i = max(self.i_min, min(self.i_max, self.i + err*self.dt))
        d_raw = (err - self.prev_err) / self.dt
        self.d_filt = self.d_alpha*d_raw + (1.0 - self.d_alpha)*self.d_filt
        u = self.kp*err + self.ki*self.i + self.kd*self.d_filt
        self.prev_err = err
        return max(self.out_min, min(self.out_max, u))


class MotionPredictor:
    def __init__(self, vel_alpha=0.35, acc_alpha=0.25, max_abs_vel=500.0, max_abs_acc=4000.0):
        self.prev_x = None
        self.prev_v = 0.0
        self.v = 0.0
        self.a = 0.0
        self.vel_alpha = max(0.0, min(1.0, vel_alpha))
        self.acc_alpha = max(0.0, min(1.0, acc_alpha))
        self.max_abs_vel = float(max_abs_vel)
        self.max_abs_acc = float(max_abs_acc)

    def reset(self):
        self.prev_x = None
        self.prev_v = 0.0
        self.v = 0.0
        self.a = 0.0

    def update(self, x, dt):
        dt = max(1e-4, float(dt))

        if self.prev_x is None:
            self.prev_x = float(x)
            self.prev_v = 0.0
            self.v = 0.0
            self.a = 0.0
            return self.v, self.a

        raw_v = (float(x) - self.prev_x) / dt
        raw_v = max(-self.max_abs_vel, min(self.max_abs_vel, raw_v))
        self.v = self.vel_alpha * raw_v + (1.0 - self.vel_alpha) * self.v

        raw_a = (self.v - self.prev_v) / dt
        raw_a = max(-self.max_abs_acc, min(self.max_abs_acc, raw_a))
        self.a = self.acc_alpha * raw_a + (1.0 - self.acc_alpha) * self.a

        self.prev_x = float(x)
        self.prev_v = self.v
        return self.v, self.a

    def predict(self, x, horizon_s):
        h = max(0.0, float(horizon_s))
        return float(x) + self.v * h + 0.5 * self.a * h * h


# ---------- UART reader ----------
class UARTReader(threading.Thread):
    def __init__(self, port: str, baud: int):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.stop_flag = False
        self.err: Optional[str] = None
        self.q: Dict[str, Tuple[float, float, float, float]] = {s: None for s in SENSORS}

    def run(self):
        try:
            import serial
        except ImportError:
            self.err = "pyserial not installed (pip3 install pyserial)"
            return

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.err = f"Serial open failed: {e}"
            return

        try:
            while not self.stop_flag:
                line = ser.readline().decode(errors="ignore").strip()
                m = LINE_RE.match(line)
                if not m:
                    continue

                name_raw = m.group("name").strip().lower()
                name = ALIASES.get(name_raw)
                if not name:
                    for s in SENSORS:
                        if name_raw.replace(" ", "") == s.lower().replace(" ", ""):
                            name = s
                            break
                    if not name:
                        continue

                try:
                    w = float(m.group("w"))
                    x = float(m.group("x"))
                    y = float(m.group("y"))
                    z = float(m.group("z"))
                except ValueError:
                    continue

                self.q[name] = qnorm(w, x, y, z)

        finally:
            try:
                ser.close()
            except:
                pass

    def stop(self):
        self.stop_flag = True


# ---------- Pi client ----------
class PiClient:
    def __init__(self, host: str, port: int, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.close()
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self.sock = s

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.sock = None

    def _send_no_wait(self, obj: dict) -> str:
        line = (json.dumps(obj) + "\n").encode()
        for _ in range(2):
            try:
                if not self.sock:
                    self.connect()
                self.sock.sendall(line)
                return "TX_ONLY"
            except Exception:
                self.close()
        return "ERR:no-connection"

    def servo_origin(self, motor_id: int, origin_mode: int = 1) -> str:
        obj = {
            "mode": "proto",
            "proto": "servo_set_origin",
            "payload": {"motor_id": int(motor_id), "origin_mode": int(origin_mode)}
        }
        return self._send_no_wait(obj)

    def servo_position(self, motor_id: int, deg: float) -> str:
        obj = {
            "mode": "proto",
            "proto": "servo_position",
            "payload": {"motor_id": int(motor_id), "position_deg": float(deg)}
        }
        return self._send_no_wait(obj)


# ---------- Angle computations ----------
def compute_elbows_deg(q: Dict[str, Tuple[float, float, float, float]], kH=(0, 0, 1), kE=(0, 0, 1)) -> Tuple[Optional[float], Optional[float]]:
    qHu, qHl, qEu, qEl = q[S_HEALTHY_UP], q[S_HEALTHY_LO], q[S_EXO_UP], q[S_EXO_LO]
    EH = EE = None
    if qHu and qHl:
        qrH = q_mul(q_conj(qHu), qHl)
        EH = twist_deg_about_axis(qrH, kH)
    if qEu and qEl:
        qrE = q_mul(q_conj(qEu), qEl)
        EE = twist_deg_about_axis(qrE, kE)
    return EH, EE

def axis_tuple(axis_name):
    if axis_name == "x":
        return (1.0, 0.0, 0.0)
    if axis_name == "y":
        return (0.0, 1.0, 0.0)
    return (0.0, 0.0, 1.0)


# ---------- Plot helper ----------
def _set_line_matched(line, xs, ys):
    nx, ny = len(xs), len(ys)
    n = min(nx, ny)
    if n <= 0:
        return
    line.set_data(list(xs)[-n:], list(ys)[-n:])


class RealtimePlotter(threading.Thread):
    """
    Separate plot thread. Three panels:
      1) Elbow
      2) Shoulder X (motor 2)
      3) Shoulder Z (motor 3)
    """
    def __init__(self,
                 xs,
                 ys_des1, ys_act1,
                 ys_des2, ys_act2,
                 ys_des3, ys_act3,
                 rate_hz=15.0):
        super().__init__(daemon=True)
        self.xs = xs
        self.ys_des1 = ys_des1
        self.ys_act1 = ys_act1
        self.ys_des2 = ys_des2
        self.ys_act2 = ys_act2
        self.ys_des3 = ys_des3
        self.ys_act3 = ys_act3
        self.rate_hz = max(1.0, float(rate_hz))
        self.stop_flag = False

        self.fig = None
        self.ax1 = None
        self.ax2 = None
        self.ax3 = None
        self.line_des1 = None
        self.line_act1 = None
        self.line_des2 = None
        self.line_act2 = None
        self.line_des3 = None
        self.line_act3 = None

    def run(self):
        if plt is None:
            return

        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

        self.ax1.set_title("Elbow angles (deg)")
        self.ax1.set_ylabel("deg")
        self.line_des1, = self.ax1.plot([], [], label="Healthy Elbow (desired)")
        self.line_act1, = self.ax1.plot([], [], label="Exo Elbow (actual)")
        self.ax1.set_ylim(-10, 100)
        self.ax1.legend()

        self.ax2.set_title("Shoulder X / Ab-Ad (Motor 2)")
        self.ax2.set_ylabel("deg")
        self.line_des2, = self.ax2.plot([], [], label="Healthy Upper X (desired)")
        self.line_act2, = self.ax2.plot([], [], label="Exo Upper X (actual)")
        self.ax2.set_ylim(-60, 90)
        self.ax2.legend()

        self.ax3.set_title("Shoulder Z / Parallel with Elbow (Motor 3)")
        self.ax3.set_xlabel("samples")
        self.ax3.set_ylabel("deg")
        self.line_des3, = self.ax3.plot([], [], label="Healthy Upper Z (desired)")
        self.line_act3, = self.ax3.plot([], [], label="Exo Upper Z (actual)")
        self.ax3.set_ylim(-90, 90)
        self.ax3.legend()

        while not self.stop_flag:
            try:
                if len(self.xs) > 0:
                    latest_sample = self.xs[-1]

                    _set_line_matched(self.line_des1, self.xs, self.ys_des1)
                    _set_line_matched(self.line_act1, self.xs, self.ys_act1)
                    self.ax1.set_xlim(max(0, latest_sample - 2000), latest_sample + 10)

                    _set_line_matched(self.line_des2, self.xs, self.ys_des2)
                    _set_line_matched(self.line_act2, self.xs, self.ys_act2)
                    self.ax2.set_xlim(max(0, latest_sample - 2000), latest_sample + 10)

                    _set_line_matched(self.line_des3, self.xs, self.ys_des3)
                    _set_line_matched(self.line_act3, self.xs, self.ys_act3)
                    self.ax3.set_xlim(max(0, latest_sample - 2000), latest_sample + 10)

                    self.fig.canvas.draw_idle()
                    self.fig.canvas.flush_events()
            except Exception:
                pass

            time.sleep(1.0 / self.rate_hz)

    def stop(self):
        self.stop_flag = True


class RealtimeLogger(threading.Thread):
    def __init__(self, csv_path: str, flush_every: int = 50):
        super().__init__(daemon=True)
        self.csv_path = csv_path
        self.flush_every = max(1, int(flush_every))
        self._q = queue.Queue()
        self._stop = threading.Event()
        self._header_written = False
        self._rows_since_flush = 0
        self._csv_file = None
        self._writer = None

    def run(self):
        try:
            os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
            self._csv_file = open(self.csv_path, "w", newline="")
            self._writer = None

            while not self._stop.is_set() or not self._q.empty():
                try:
                    row = self._q.get(timeout=0.1)
                except queue.Empty:
                    continue

                if row is None:
                    continue

                if not self._header_written:
                    self._writer = csv.DictWriter(self._csv_file, fieldnames=list(row.keys()))
                    self._writer.writeheader()
                    self._header_written = True

                self._writer.writerow(row)
                self._rows_since_flush += 1

                if self._rows_since_flush >= self.flush_every:
                    self._csv_file.flush()
                    self._rows_since_flush = 0
        finally:
            try:
                if self._csv_file:
                    self._csv_file.flush()
                    self._csv_file.close()
            except:
                pass

    def put(self, row: dict):
        try:
            self._q.put_nowait(row)
        except:
            pass

    def close(self):
        self._stop.set()
        self.join(timeout=2.0)
        try:
            import pandas as pd
            xlsx_path = os.path.splitext(self.csv_path)[0] + ".xlsx"
            df = pd.read_csv(self.csv_path)
            df.to_excel(xlsx_path, index=False)
            return xlsx_path
        except Exception:
            return None


# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Elbow + dual-shoulder PID control -> Pi CAN bridge (servo mode), with live plot & timing.")
    ap.add_argument("--serial", default="/dev/ttyTHS1", help="UART device for IMU data")
    ap.add_argument("--baud", type=int, default=115200, help="UART baud")
    ap.add_argument("--pi", default="10.100.161.178", help="Pi host/IP for CAN bridge")
    ap.add_argument("--pi-port", type=int, default=8008, help="Pi TCP port")
    ap.add_argument("--plot", type=int, choices=[0, 1], default=1, help="Enable live plotting (1=on, 0=off)")
    ap.add_argument("--joint2-mac", default=None, help="MAC address of new MetaWear IMU mounted between shoulder joints 1 and 2")
    ap.add_argument("--joint2-axis", choices=["x", "y", "z"], default="x", help="Axis used from new joint IMU for Motor 2 actual position")

    # Motor 1 (Elbow)
    ap.add_argument("--motor", type=int, default=1, help="Elbow motor ID (Motor 1)")
    ap.add_argument("--set-origin-mode", type=int, choices=[0, 1, 2], default=1, help="CubeMars origin mode for Motor 1 (1=permanent)")
    ap.add_argument("--rate", type=float, default=30.0, help="Control rate Hz")
    ap.add_argument("--calib-secs", type=float, default=1.0, help="Calibration seconds")
    ap.add_argument("--kp", type=float, default=1.25)
    ap.add_argument("--ki", type=float, default=1.0)
    ap.add_argument("--kd", type=float, default=0.05)
    ap.add_argument("--deadband", type=float, default=0.5, help="Elbow error deadband (deg)")
    ap.add_argument("--max-step", type=float, default=3.0, help="Elbow: max command change per tick (deg)")
    ap.add_argument("--motor-min", type=float, default=0.0)
    ap.add_argument("--motor-max", type=float, default=90.0)
    ap.add_argument("--invert-exo-for-pid", action="store_true", default=True, help="Invert Exo elbow sign for PID (enabled by default)")
    ap.add_argument("--auto-axis", action="store_true", help="Estimate elbow hinge axes during calib by small flex")

    # Motor 2 (Shoulder X)
    ap.add_argument("--motor2", type=int, default=2, help="Shoulder X motor ID (Motor 2)")
    ap.add_argument("--kp2", type=float, default=1.0)
    ap.add_argument("--ki2", type=float, default=0.6)
    ap.add_argument("--kd2", type=float, default=0.03)
    ap.add_argument("--deadband2", type=float, default=5, help="Shoulder X deadband (deg)")
    ap.add_argument("--max-step2", type=float, default=2.0, help="Shoulder X: max command change per tick (deg)")
    ap.add_argument("--motor2-min", type=float, default=0.0)
    ap.add_argument("--motor2-max", type=float, default=70.0)
    ap.add_argument("--invert-shoulder-for-pid", action="store_true", help="Invert shoulder X control sign if needed")
    ap.add_argument("--auto-axis-shoulder", action="store_true", help="Estimate shoulder X axis during calib by small ab/adduction")

    # Motor 3 (Shoulder Z, parallel to elbow axis)
    ap.add_argument("--motor3", type=int, default=3, help="Shoulder Z motor ID (Motor 3)")
    ap.add_argument("--kp3", type=float, default=1.0)
    ap.add_argument("--ki3", type=float, default=0.6)
    ap.add_argument("--kd3", type=float, default=0.03)
    ap.add_argument("--deadband3", type=float, default=3, help="Shoulder Z deadband (deg)")
    ap.add_argument("--max-step3", type=float, default=2.0, help="Shoulder Z: max command change per tick (deg)")
    ap.add_argument("--motor3-min", type=float, default=0.0)
    ap.add_argument("--motor3-max", type=float, default=70.0)
    ap.add_argument("--invert-shoulder2-for-pid", action="store_true", help="Invert shoulder Z control sign if needed")

    ap.add_argument("--weight", default="unlabeled", help="Label used only for output filenames")

    # Predictive control options
    ap.add_argument("--predictive", type=int, choices=[0, 1], default=1, help="Use one-step-ahead predictive desired angle")
    ap.add_argument("--vel-alpha", type=float, default=0.35, help="Low-pass filter alpha for velocity estimate")
    ap.add_argument("--acc-alpha", type=float, default=0.25, help="Low-pass filter alpha for acceleration estimate")
    ap.add_argument("--latency-scale", type=float, default=1.0, help="Multiplier on average measured runtime when forming prediction horizon")
    ap.add_argument("--pred-max-horizon", type=float, default=0.150, help="Clamp prediction horizon in seconds")

    args = ap.parse_args()
    

    global plt
    if args.plot:
        import matplotlib.pyplot as plt

    timestamp = t.datetime.now().strftime("%Y%m%d-%H%M%S")
    safe_weight = re.sub(r"[^A-Za-z0-9._-]+", "_", str(args.weight))
    out_dir = "pid_tests_3axis"
    base = f"{timestamp}_weight-{safe_weight}"
    csv_path = os.path.join(out_dir, f"{base}.csv")

    logger = RealtimeLogger(csv_path, flush_every=50)
    logger.start()
    print(f"[log] Writing CSV rows to {csv_path} (XLSX created on close if deps available)")

    reader = UARTReader(args.serial, args.baud)
    reader.start()
    t0 = time.time()
    while reader.err is None and all(reader.q[s] is None for s in SENSORS) and time.time() - t0 < 3.0:
        time.sleep(0.05)

    if reader.err:
        print(reader.err, file=sys.stderr)
        try:
            xlsx_path = logger.close()
            if xlsx_path:
                print(f"[log] Also wrote {xlsx_path}")
        except:
            pass
        return

    print(f"[uart] Streaming from {args.serial}@{args.baud}")

    joint2_reader = None
    if args.joint2_mac:
        joint2_reader = MetaWearBLEReader(args.joint2_mac, name="Joint Motor 2 IMU")
        joint2_reader.start()
    
        t_ble = time.time()
        while joint2_reader.err is None and joint2_reader.q is None and time.time() - t_ble < 6.0:
            time.sleep(0.05)
    
        if joint2_reader.err:
            print(joint2_reader.err, file=sys.stderr)
            return

        print("[metawear] Joint Motor 2 IMU streaming")

    # Calibration
    vH = [0.0, 0.0, 0.0]; nH = 0
    vE = [0.0, 0.0, 0.0]; nE = 0
    vS = [0.0, 0.0, 0.0]; nS = 0

    print(
        f"[calib] {args.calib_secs:.1f}s: zero elbows"
        + (" + flex lightly for elbow axis" if args.auto_axis else "")
        + (" + small shoulder X motion for shoulder axis" if args.auto_axis_shoulder else "")
    )

    t_end = time.time() + args.calib_secs

    qHu0 = reader.q[S_HEALTHY_UP]
    qEu0 = reader.q[S_EXO_UP]
    qJ20 = joint2_reader.q if joint2_reader else None
    kJ2 = axis_tuple(args.joint2_axis)

    while time.time() < t_end:
        EH, EE = compute_elbows_deg(reader.q)

        if args.auto_axis:
            qHu, qHl, qEu, qEl = reader.q[S_HEALTHY_UP], reader.q[S_HEALTHY_LO], reader.q[S_EXO_UP], reader.q[S_EXO_LO]
            if qHu and qHl:
                _, vx, vy, vz = q_mul(q_conj(qHu), qHl)
                vH[0] += vx; vH[1] += vy; vH[2] += vz; nH += 1
            if qEu and qEl:
                _, vx, vy, vz = q_mul(q_conj(qEu), qEl)
                vE[0] += vx; vE[1] += vy; vE[2] += vz; nE += 1

        if args.auto_axis_shoulder:
            qHu, qEu = reader.q[S_HEALTHY_UP], reader.q[S_EXO_UP]
            if qHu and qEu:
                _, vx, vy, vz = q_mul(q_conj(qEu), qHu)
                vS[0] += vx; vS[1] += vy; vS[2] += vz; nS += 1

        time.sleep(0.01)

    # Elbow axes from upper/lower
    kH = unit(vH) if args.auto_axis and nH > 0 else (0.0, 0.0, 1.0)
    kE = unit(vE) if args.auto_axis and nE > 0 else (0.0, 0.0, 1.0)

    # Shoulder motor 2 uses upper IMUs only, default X
    kS = unit(vS) if args.auto_axis_shoulder and nS > 0 else (1.0, 0.0, 0.0)

    # Shoulder motor 3 uses upper IMUs only, forced Z to match elbow axis
    kS2 = (0.0, 0.0, 1.0)

    print(f"[axis] Elbow Healthy k={tuple(round(x, 3) for x in kH)}")
    print(f"[axis] Elbow Exo    k={tuple(round(x, 3) for x in kE)}")
    print(f"[axis] Shoulder X  k={tuple(round(x, 3) for x in kS)}")
    print(f"[axis] Shoulder Z  k={tuple(round(x, 3) for x in kS2)}")

    EH0, EE0 = compute_elbows_deg(reader.q, kH, kE)
    EH0 = EH0 or 0.0
    EE0 = EE0 or 0.0

    qHu0 = qHu0 or reader.q[S_HEALTHY_UP]
    qEu0 = qEu0 or reader.q[S_EXO_UP]

    print(f"[zero] Healthy Elbow {EH0:+.2f}°, Exo Elbow {EE0:+.2f}° (upper-arm shoulder baselines captured)")

    pi = PiClient(args.pi, args.pi_port)

    print("[motor1] Setting origin on motor", args.motor)
    print("[motor1]", pi.servo_origin(args.motor, origin_mode=args.set_origin_mode))
    time.sleep(0.05)
    cmd_deg_1 = 0.0
    print("[motor1]", pi.servo_position(args.motor, cmd_deg_1))
    
    print("[motor2]", pi.servo_origin(args.motor2, origin_mode=args.set_origin_mode))
    time.sleep(0.05)
    cmd_deg_2 = 0.0
    print("[motor2] initial pos cmd:", cmd_deg_2)
    print("[motor2]", pi.servo_position(args.motor2, cmd_deg_2))
    
    print("[motor3]", pi.servo_origin(args.motor3, origin_mode=args.set_origin_mode))
    time.sleep(0.05)
    cmd_deg_3 = 0.0
    print("[motor3] initial pos cmd:", cmd_deg_3)
    print("[motor3]", pi.servo_position(args.motor3, cmd_deg_3))

    xs = collections.deque(maxlen=2000)
    ys_des1 = collections.deque(maxlen=2000)
    ys_act1 = collections.deque(maxlen=2000)
    ys_des2 = collections.deque(maxlen=2000)
    ys_act2 = collections.deque(maxlen=2000)
    ys_des3 = collections.deque(maxlen=2000)
    ys_act3 = collections.deque(maxlen=2000)

    plotter = None
    if args.plot:
        print("[plot] enabled")
        plotter = RealtimePlotter(
            xs,
            ys_des1, ys_act1,
            ys_des2, ys_act2,
            ys_des3, ys_act3,
            rate_hz=min(args.rate, 15.0)
        )
        plotter.start()
    else:
        print("[plot] disabled")

    dt = max(1e-3, 1.0 / args.rate)
    pid1 = PID(args.kp, args.ki, args.kd, dt, out_min=-30, out_max=30)
    pid2 = PID(args.kp2, args.ki2, args.kd2, dt, out_min=-25, out_max=25)
    pid3 = PID(args.kp3, args.ki3, args.kd3, dt, out_min=-25, out_max=25)

    elbow_predictor = MotionPredictor(
        vel_alpha=args.vel_alpha,
        acc_alpha=args.acc_alpha
    )
    shoulder_predictor = MotionPredictor(
        vel_alpha=args.vel_alpha,
        acc_alpha=args.acc_alpha
    )
    shoulder2_predictor = MotionPredictor(
        vel_alpha=args.vel_alpha,
        acc_alpha=args.acc_alpha
    )

    avg_exec_time = dt
    loop_count_for_avg = 0
    prev_loop_wall_time = None

    print(f"[loop] {args.rate:.1f} Hz")
    print(f"[loop] Elbow PID(Kp={args.kp}, Ki={args.ki}, Kd={args.kd}) limits [{args.motor_min}, {args.motor_max}]°")
    print(f"[loop] Shoulder X PID(Kp={args.kp2}, Ki={args.ki2}, Kd={args.kd2}) limits [{args.motor2_min}, {args.motor2_max}]°")
    print(f"[loop] Shoulder Z PID(Kp={args.kp3}, Ki={args.ki3}, Kd={args.kd3}) limits [{args.motor3_min}, {args.motor3_max}]°")

    sample = 0

    try:
        while True:
            loop_start = time.time()

            if prev_loop_wall_time is None:
                loop_dt = dt
            else:
                loop_dt = max(1e-3, loop_start - prev_loop_wall_time)
            prev_loop_wall_time = loop_start

            pred_horizon = min(
                args.pred_max_horizon,
                loop_dt + args.latency_scale * avg_exec_time
            )

            # ---------- Elbow (Motor 1) ----------
            EH, EE = compute_elbows_deg(reader.q, kH, kE)
            if EH is None or EE is None:
                time.sleep(0.005)
                continue

            healthy_deg = wrap_deg(EH - EH0)
            exo_deg = wrap_deg(EE - EE0)
            actual_for_pid_1 = -exo_deg if args.invert_exo_for_pid else exo_deg if False else (-exo_deg if args.invert_exo_for_pid else exo_deg)

            elbow_vel, elbow_acc = elbow_predictor.update(healthy_deg, loop_dt)
            desired1 = wrap_deg(elbow_predictor.predict(healthy_deg, pred_horizon)) if args.predictive else healthy_deg

            err1 = desired1 - actual_for_pid_1
            if abs(err1) < args.deadband:
                corr1 = 0.0
            else:
                pid1.dt = loop_dt
                corr1 = pid1.step(err1)

            corr1 = max(-args.max_step, min(args.max-step if False else args.max_step, corr1))
            cmd_deg_1 = max(args.motor_min, min(args.motor_max, cmd_deg_1 + corr1))
            resp1 = pi.servo_position(args.motor, -cmd_deg_1)

            # ---------- Shoulder X (Motor 2) ----------
            qHu = reader.q[S_HEALTHY_UP]
            
            healthy_abs = twist_deg_from_baseline(qHu0, qHu, kS)
            
            # New actual feedback source for Motor 2
            if joint2_reader and joint2_reader.q:
                exo_abs = twist_deg_from_baseline(qJ20, joint2_reader.q, kJ2)
            else:
                qEu = reader.q[S_EXO_UP]
                exo_abs = twist_deg_from_baseline(qEu0, qEu, kS)
            
            if healthy_abs is None or exo_abs is None:
                time.sleep(0.005)
                continue
            
            healthy_shoulder_now = -wrap_deg(healthy_abs)
            actual_for_pid_2 = wrap_deg(-exo_abs if args.invert_shoulder_for_pid else exo_abs)

            shoulder_vel, shoulder_acc = shoulder_predictor.update(healthy_shoulder_now, loop_dt)
            desired2 = wrap_deg(shoulder_predictor.predict(healthy_shoulder_now, pred_horizon)) if args.predictive else healthy_shoulder_now

            err2 = desired2 - actual_for_pid_2
            if abs(err2) < args.deadband2:
                corr2 = 0.0
            else:
                pid2.dt = loop_dt
                corr2 = pid2.step(err2)

            corr2 = max(-args.max_step2, min(args.max_step2, corr2))
            cmd_deg_2 = max(args.motor2_min, min(args.motor2_max, cmd_deg_2 + corr2))
            resp2 = pi.servo_position(args.motor2, cmd_deg_2)

            # ---------- Shoulder Z / parallel with elbow axis (Motor 3) ----------
            # Uses upper IMUs only; no lower IMUs involved
            healthy_abs_3 = -twist_deg_from_baseline(qHu0, qHu, kS2)
            exo_abs_3 = -twist_deg_from_baseline(qEu0, qEu, kS2)

            if healthy_abs_3 is None or exo_abs_3 is None:
                time.sleep(0.005)
                continue

            healthy_shoulder2_now = -wrap_deg(healthy_abs_3)
            actual_for_pid_3 = wrap_deg(-exo_abs_3 if args.invert_shoulder2_for_pid else exo_abs_3)

            shoulder2_vel, shoulder2_acc = shoulder2_predictor.update(healthy_shoulder2_now, loop_dt)
            desired3 = wrap_deg(shoulder2_predictor.predict(healthy_shoulder2_now, pred_horizon)) if args.predictive else healthy_shoulder2_now

            err3 = desired3 - actual_for_pid_3
            if abs(err3) < args.deadband3:
                corr3 = 0.0
            else:
                pid3.dt = loop_dt
                corr3 = pid3.step(err3)

            corr3 = max(-args.max_step3, min(args.max_step3, corr3))
            cmd_deg_3 = max(args.motor3_min, min(args.motor3_max, cmd_deg_3 + corr3))
            resp3 = pi.servo_position(args.motor3, cmd_deg_3)

            # ---------- Plot buffers ----------
            xs.append(sample)
            ys_des1.append(healthy_deg)
            ys_act1.append(actual_for_pid_1)
            ys_des2.append(healthy_shoulder_now)
            ys_act2.append(actual_for_pid_2)
            ys_des3.append(healthy_shoulder2_now)
            ys_act3.append(actual_for_pid_3)

            exec_time = time.time() - loop_start
            loop_count_for_avg += 1
            avg_exec_time += (exec_time - avg_exec_time) / loop_count_for_avg

            print(f"[t={sample:05d}] ELB:  now={healthy_deg:+6.1f}° pred={desired1:+6.1f}° "
                  f"act={actual_for_pid_1:+6.1f}° err={err1:+6.1f}° "
                  f"v={elbow_vel:+7.2f}°/s a={elbow_acc:+8.2f}°/s² "
                  f"corr={corr1:+5.2f}° cmd={cmd_deg_1:6.2f}° | {resp1}")

            print(f"           SHX: now={healthy_shoulder_now:+6.1f}° pred={desired2:+6.1f}° "
                  f"act={actual_for_pid_2:+6.1f}° err={err2:+6.1f}° "
                  f"v={shoulder_vel:+7.2f}°/s a={shoulder_acc:+8.2f}°/s² "
                  f"corr={corr2:+5.2f}° cmd={cmd_deg_2:6.2f}° | {resp2}")

            print(f"           SHZ: now={healthy_shoulder2_now:+6.1f}° pred={desired3:+6.1f}° "
                  f"act={actual_for_pid_3:+6.1f}° err={err3:+6.1f}° "
                  f"v={shoulder2_vel:+7.2f}°/s a={shoulder2_acc:+8.2f}°/s² "
                  f"corr={corr3:+5.2f}° cmd={cmd_deg_3:6.2f}° | {resp3}    "
                  f"dt={exec_time*1000:.1f} ms avg={avg_exec_time*1000:.1f} ms pred_h={pred_horizon*1000:.1f} ms")

            logger.put({
                "stamp_unix": time.time(),
                "sample": sample,

                # Elbow / Motor 1
                "elbow_measured_healthy_deg": healthy_deg,
                "elbow_predicted_desired_deg": desired1,
                "elbow_actual_deg_for_pid": actual_for_pid_1,
                "elbow_velocity_deg_s": elbow_vel,
                "elbow_acceleration_deg_s2": elbow_acc,
                "elbow_error_deg": err1,
                "elbow_corr_deg": corr1,
                "elbow_cmd_deg": cmd_deg_1,

                # Shoulder X / Motor 2
                "shoulder_x_measured_healthy_deg": healthy_shoulder_now,
                "shoulder_x_predicted_desired_deg": desired2,
                "shoulder_x_actual_deg_for_pid": actual_for_pid_2,
                "shoulder_x_velocity_deg_s": shoulder_vel,
                "shoulder_x_acceleration_deg_s2": shoulder_acc,
                "shoulder_x_error_deg": err2,
                "shoulder_x_corr_deg": corr2,
                "shoulder_x_cmd_deg": cmd_deg_2,

                # Shoulder Z / Motor 3
                "shoulder_z_measured_healthy_deg": healthy_shoulder2_now,
                "shoulder_z_predicted_desired_deg": desired3,
                "shoulder_z_actual_deg_for_pid": actual_for_pid_3,
                "shoulder_z_velocity_deg_s": shoulder2_vel,
                "shoulder_z_acceleration_deg_s2": shoulder2_acc,
                "shoulder_z_error_deg": err3,
                "shoulder_z_corr_deg": corr3,
                "shoulder_z_cmd_deg": cmd_deg_3,

                # Timing
                "loop_dt_ms": loop_dt * 1000.0,
                "exec_time_ms": exec_time * 1000.0,
                "avg_exec_time_ms": avg_exec_time * 1000.0,
                "prediction_horizon_ms": pred_horizon * 1000.0
            })

            sample += 1
            sleep_left = dt - (time.time() - loop_start)
            if sleep_left > 0:
                time.sleep(sleep_left)

    except KeyboardInterrupt:
        print("\n[ctrl] Ctrl-C -> return to 0° and exit")
        try:
            print("[motor1]", pi.servo_position(args.motor, 0.0))
        except:
            pass
        try:
            print("[motor2]", pi.servo_position(args.motor2, 0.0))
        except:
            pass
        try:
            print("[motor3]", pi.servo_position(args.motor3, 0.0))
        except:
            pass

        try:
            xlsx_path = logger.close()
            if xlsx_path:
                print(f"[log] Also wrote {xlsx_path}")
        except:
            pass

    finally:
        try:
            if plotter is not None:
                plotter.stop()
        except:
            pass

        reader.stop()

        try:
            pi.close()
        except:
            pass

        if args.plot and plt is not None:
            plt.ioff()
            try:
                plt.show(block=False)
            except:
                pass

        try:
            xlsx_path = logger.close()
            if xlsx_path:
                print(f"[log] Also wrote {xlsx_path}")
        except:
            pass

        try:
            if joint2_reader is not None:
                joint2_reader.stop()
        except:
            pass


if __name__ == "__main__":
    main()
