#!/usr/bin/env python3

import argparse
import json
import socket
import time


class PiClient:
    def __init__(self, host: str, port: int, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)

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

    def servo_current(self, motor_id: int, current_a: float) -> str:
        obj = {
            "mode": "proto",
            "proto": "servo_current",
            "payload": {
                "motor_id": int(motor_id),
                "current_a": float(current_a)
            }
        }
        return self._send_no_wait(obj)


def ramp_current(pi, motor_id, target_a, step_a, delay_s):
    current = 0.0
    direction = 1.0 if target_a >= 0 else -1.0
    step_a = abs(step_a) * direction

    while abs(current) < abs(target_a):
        current += step_a

        if abs(current) > abs(target_a):
            current = target_a

        resp = pi.servo_current(motor_id, current)
        print(f"[motor {motor_id}] current={current:+.3f} A | {resp}")
        time.sleep(delay_s)


def main():
    ap = argparse.ArgumentParser(description="Simple CubeMars current command test from Jetson through Pi bridge")

    ap.add_argument("--pi", default="10.100.161.178", help="Raspberry Pi CAN bridge IP")
    ap.add_argument("--pi-port", type=int, default=8008, help="Raspberry Pi CAN bridge TCP port")

    ap.add_argument("--elbow-motor", type=int, default=1, help="Elbow motor ID")
    ap.add_argument("--shoulder-motor", type=int, default=3, help="Shoulder motor inline with elbow axis")

    ap.add_argument("--current", type=float, default=0.25, help="Target test current in amps")
    ap.add_argument("--step", type=float, default=0.05, help="Current ramp step in amps")
    ap.add_argument("--hold", type=float, default=2.0, help="Hold time at target current")
    ap.add_argument("--delay", type=float, default=0.15, help="Delay between ramp steps")

    ap.add_argument("--test", choices=["elbow", "shoulder", "both"], default="both")

    args = ap.parse_args()

    pi = PiClient(args.pi, args.pi_port)

    motors = []
    if args.test in ["elbow", "both"]:
        motors.append(args.elbow_motor)
    if args.test in ["shoulder", "both"]:
        motors.append(args.shoulder_motor)

    print("[start] Current command test")
    print(f"[config] motors={motors}, target={args.current} A")

    try:
        for motor_id in motors:
            print(f"\n[test] Motor {motor_id}: ramping to {args.current:+.3f} A")
            ramp_current(pi, motor_id, args.current, args.step, args.delay)

            print(f"[test] Motor {motor_id}: holding for {args.hold:.1f}s")
            time.sleep(args.hold)

            print(f"[test] Motor {motor_id}: zero current")
            print(f"[motor {motor_id}]", pi.servo_current(motor_id, 0.0))
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[ctrl] Ctrl-C detected, zeroing motors")

    finally:
        for motor_id in motors:
            try:
                print(f"[safe-zero motor {motor_id}]", pi.servo_current(motor_id, 0.0))
            except:
                pass

        pi.close()
        print("[done]")


if __name__ == "__main__":
    main()
