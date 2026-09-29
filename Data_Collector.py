#!/usr/bin/env python3

import argparse
import csv
import json
import math
import socket
import time

# ============================================================
# PI CLIENT
# ============================================================

class PiClient:

    def __init__(self, host: str, port: int, timeout=2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.close()

        self.sock = socket.create_connection(
            (self.host, self.port),
            timeout=self.timeout
        )

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

    def servo_current(
        self,
        motor_id: int,
        current_a: float
    ) -> str:

        obj = {
            "mode": "proto",
            "proto": "servo_current",
            "payload": {
                "motor_id": int(motor_id),
                "current_a": float(current_a)
            }
        }

        return self._send_no_wait(obj)


# ============================================================
# CONFIGURATION
# ============================================================

PI_IP = "10.100.161.178"
PI_PORT = 8008

# Motor ID
MOTOR_ID = 3

# Maximum commanded current
MAX_CURRENT = 0.01

# Sensor receiver
UDP_PORT = 5005

# Controller update rate
CONTROL_RATE = 50.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE

# How long we will accept old sensor data
SENSOR_TIMEOUT = 0.25


# ============================================================
# QUATERNION → ARM X AXIS
# ============================================================

def quaternion_to_x_axis(q):

    i = q["i"]
    j = q["j"]
    k = q["k"]
    r = q["r"]

    x_world = 1.0 - 2.0 * (j * j + k * k)

    y_world = 2.0 * (i * j + r * k)

    z_world = 2.0 * (i * k - r * j)

    return x_world, y_world, z_world


# ============================================================
# ARM X AXIS → ELEVATION
# ============================================================

def quaternion_to_elevation(q):

    x_world, y_world, z_world = quaternion_to_x_axis(q)

    # Prevent numerical errors from producing
    # a value slightly outside [-1, 1].

    z_world = max(
        -1.0,
        min(1.0, z_world)
    )

    elevation_from_horizontal = math.degrees(
        math.asin(z_world)
    )

    # Definition:
    #
    # 0°   = arm straight down
    # 90°  = arm horizontal
    # 180° = arm straight up

    elevation = (
        elevation_from_horizontal + 90.0
    )

    elevation = max(
        0.0,
        min(180.0, elevation)
    )

    return elevation


# ============================================================
# CSV DATA STORAGE
# ============================================================

def create_data_record(
    timestamp,
    elevation,
    quaternion,
    pressure
):

    record = {
        "timestamp": timestamp,
        "elevation_deg": elevation,

        # IMU quaternion
        "imu_i": quaternion["i"],
        "imu_j": quaternion["j"],
        "imu_k": quaternion["k"],
        "imu_r": quaternion["r"]
    }

    # --------------------------------------------------------
    # Pressure sensors
    # --------------------------------------------------------
    #
    # Each pressure channel is stored as both voltage and
    # percentage of the maximum sensor voltage.
    #
    # Example:
    # pressure_ch0_voltage
    # pressure_ch0_percentage

    for channel, values in pressure.items():

        record[f"{channel}_voltage"] = values["voltage"]

        record[f"{channel}_percentage"] = values["percentage"]

    return record


def save_data_to_csv(
    filename,
    data
):

    if not data:
        print("[csv] No sensor data was collected.")
        return

    # --------------------------------------------------------
    # Collect every possible column
    # --------------------------------------------------------

    fieldnames = []

    for record in data:

        for key in record.keys():

            if key not in fieldnames:

                fieldnames.append(key)

    # --------------------------------------------------------
    # Write CSV
    # --------------------------------------------------------

    with open(
        filename,
        "w",
        newline=""
    ) as csv_file:

        writer = csv.DictWriter(
            csv_file,
            fieldnames=fieldnames
        )

        writer.writeheader()

        writer.writerows(data)

    print()
    print(
        f"[csv] Saved {len(data)} samples to: {filename}"
    )


# ============================================================
# DISPLAY
# ============================================================

def display_data(
    elevation,
    quaternion,
    pressure
):

    print("\033[2J\033[H", end="")

    print("=" * 60)
    print("             Sensor Data Collection")
    print("=" * 60)

    print()

    print("ARM ORIENTATION")
    print("-" * 60)

    print(
        f"Elevation:          {elevation:8.2f}°"
    )

    print()

    print("QUATERNION")
    print("-" * 60)

    print(
        f"i = {quaternion['i']:+.5f}"
    )

    print(
        f"j = {quaternion['j']:+.5f}"
    )

    print(
        f"k = {quaternion['k']:+.5f}"
    )

    print(
        f"r = {quaternion['r']:+.5f}"
    )

    print()

    print("PRESSURE")
    print("-" * 60)

    for channel, values in pressure.items():

        print(
            f"{channel.upper()}: "
            f"{values['voltage']:.3f} V   "
            f"{values['percentage']:6.2f}%"
        )

    print()

    print("=" * 60)


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=
        "BNO08X elevation-based CubeMars gravity controller"
    )

    parser.add_argument(
        "--pi",
        default=PI_IP,
        help="Raspberry Pi CAN bridge IP"
    )

    parser.add_argument(
        "--pi-port",
        type=int,
        default=PI_PORT,
        help="Raspberry Pi CAN bridge TCP port"
    )

    parser.add_argument(
        "--output",
        default="sensor_data.csv",
        help="Output CSV filename"
    )

    args = parser.parse_args()

    # --------------------------------------------------------
    # UDP sensor receiver
    # --------------------------------------------------------

    udp_socket = socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM
    )

    udp_socket.bind(
        ("0.0.0.0", UDP_PORT)
    )

    udp_socket.settimeout(1.0)


    # --------------------------------------------------------
    # Pi CAN bridge
    # --------------------------------------------------------

    pi = PiClient(
        args.pi,
        args.pi_port
    )


    # --------------------------------------------------------
    # Sensor data storage
    # --------------------------------------------------------
    #
    # Every valid sensor packet is stored here during the run.
    #
    # The data is not written to disk every loop. This avoids
    # unnecessary disk I/O while the controller is running.
    #

    sensor_data = []

    # Reference time for timestamps
    start_time = time.monotonic()


    # --------------------------------------------------------
    # Startup
    # --------------------------------------------------------

    print()
    print("=" * 60)
    print("       Pressure Sensor Data Collection")
    print("=" * 60)

    print()
    print(f"Pi CAN bridge: {args.pi}:{args.pi_port}")

    print()
    print(f"CSV output: {args.output}")

    print()
    print("Waiting for sensor data...")
    print()


    last_sensor_time = 0.0


    try:

        while True:

            loop_start = time.monotonic()


            # =================================================
            # RECEIVE SENSOR DATA
            # =================================================

            try:

                data, address = udp_socket.recvfrom(
                    4096
                )

                packet = json.loads(
                    data.decode("utf-8")
                )

                last_sensor_time = time.monotonic()


            except socket.timeout:

                print(
                    "\n[WARNING] Sensor timeout"
                )

                continue


            # =================================================
            # CHECK SENSOR AGE
            # =================================================

            sensor_age = (
                time.monotonic()
                -
                last_sensor_time
            )

            if sensor_age > SENSOR_TIMEOUT:

                continue


            # =================================================
            # EXTRACT SENSOR DATA
            # =================================================

            quaternion = packet["quaternion"]

            pressure = packet["pressure"]


            # =================================================
            # CALCULATE ELEVATION
            # =================================================

            elevation = quaternion_to_elevation(
                quaternion
            )


            # =================================================
            # STORE SENSOR DATA
            # =================================================

            timestamp = (
                time.monotonic()
                -
                start_time
            )

            record = create_data_record(
                timestamp,
                elevation,
                quaternion,
                pressure
            )

            sensor_data.append(
                record
            )


            # =================================================
            # DISPLAY
            # =================================================

            display_data(
                elevation,
                quaternion,
                pressure
            )


            # =================================================
            # CONTROL RATE
            # =================================================

            elapsed = (
                time.monotonic()
                -
                loop_start
            )

            sleep_time = (
                CONTROL_PERIOD
                -
                elapsed
            )

            if sleep_time > 0:

                time.sleep(
                    sleep_time
                )


    except KeyboardInterrupt:

        print()
        print("[ctrl] Ctrl-C detected")


    finally:

        # ----------------------------------------------------
        # ALWAYS ZERO MOTOR
        # ----------------------------------------------------

        pi.close()

        udp_socket.close()

        # ----------------------------------------------------
        # SAVE SENSOR DATA
        # ----------------------------------------------------

        save_data_to_csv(
            args.output,
            sensor_data
        )

        print("[done]")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()

