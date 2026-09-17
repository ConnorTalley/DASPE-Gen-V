#!/usr/bin/env python3

import argparse
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
MAX_CURRENT = 1

# Sensor receiver
UDP_PORT = 5005

# Controller update rate
CONTROL_RATE = 50.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE

# How long we will accept old sensor data
SENSOR_TIMEOUT = 0.25


# ============================================================
# CURRENT PROFILE
# ============================================================

# These are intentionally explicit.
#
# You can add additional points later.
#
#       Elevation       Current
#
#       0 degrees       0 A
#       90 degrees      4 A
#       180 degrees     2 A

CURRENT_PROFILE = [
    (0.0, 0.0),
    (90.0, MAX_CURRENT),
    (180.0, 0.5 * MAX_CURRENT),
]


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
# CURRENT PROFILE INTERPOLATION
# ============================================================

def get_current_from_elevation(elevation):

    # Below first point
    if elevation <= CURRENT_PROFILE[0][0]:
        return CURRENT_PROFILE[0][1]

    # Above last point
    if elevation >= CURRENT_PROFILE[-1][0]:
        return CURRENT_PROFILE[-1][1]

    # Find the surrounding points
    for index in range(
        len(CURRENT_PROFILE) - 1
    ):

        angle_1, current_1 = CURRENT_PROFILE[index]

        angle_2, current_2 = CURRENT_PROFILE[index + 1]

        if angle_1 <= elevation <= angle_2:

            fraction = (
                (elevation - angle_1)
                /
                (angle_2 - angle_1)
            )

            current = (
                current_1
                +
                fraction *
                (current_2 - current_1)
            )

            return current

    return 0.0


# ============================================================
# DISPLAY
# ============================================================

def display_data(
    elevation,
    commanded_current,
    quaternion,
    pressure,
    motor_response
):

    print("\033[2J\033[H", end="")

    print("=" * 60)
    print("             GRAVITY COMPENSATION")
    print("=" * 60)

    print()

    print("ARM ORIENTATION")
    print("-" * 60)

    print(
        f"Elevation:          {elevation:8.2f}°"
    )

    print(
        f"Commanded current:  {commanded_current:+8.3f} A"
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

    print("CURRENT PROFILE")
    print("-" * 60)

    for angle, current in CURRENT_PROFILE:

        print(
            f"{angle:6.1f}° → {current:+.3f} A"
        )

    print()

    print(
        f"Motor response: {motor_response}"
    )

    print("=" * 60)


# ============================================================
# ZERO MOTOR
# ============================================================

def zero_motor(pi):

    try:

        response = pi.servo_current(
            MOTOR_ID,
            0.0
        )

        print(
            f"[safe-zero] Motor {MOTOR_ID}: "
            f"{response}"
        )

    except Exception as error:

        print(
            f"[safe-zero] Error: {error}"
        )


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
        "--motor",
        type=int,
        default=MOTOR_ID,
        help="CubeMars motor ID"
    )

    parser.add_argument(
        "--max-current",
        type=float,
        default=MAX_CURRENT,
        help="Maximum current at 90 degrees"
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Calculate/display current without sending to motor"
    )

    args = parser.parse_args()


    # --------------------------------------------------------
    # Update global motor configuration
    # --------------------------------------------------------

    motor_id = args.motor
    max_current = abs(args.max_current)

    current_profile = [
        (0.0,0.4),
        (90.0, max_current),
        (180.0, 0.5 * max_current),
    ]


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
    # Startup
    # --------------------------------------------------------

    print()
    print("=" * 60)
    print("       ELEVATION GRAVITY CONTROLLER")
    print("=" * 60)

    print()
    print(f"Pi CAN bridge: {args.pi}:{args.pi_port}")
    print(f"Motor ID:      {MOTOR_ID}")
    print(f"Maximum:       {MAX_CURRENT:.3f} A")
    print(f"Dry run:       {args.dry_run}")

    print()
    print("Current profile:")

    for angle, current in CURRENT_PROFILE:

        print(
            f"  {angle:6.1f}° → {current:+.3f} A"
        )

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

                if not args.dry_run:
                    zero_motor(pi)

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

                if not args.dry_run:
                    zero_motor(pi)

                continue


            # =================================================
            # EXTRACT QUATERNION
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
            # CALCULATE CURRENT
            # =================================================

            commanded_current = (
                (get_current_from_elevation(
                    elevation
                ))
            )


            # =================================================
            # SEND CURRENT
            # =================================================

            if args.dry_run:

                motor_response = "DRY RUN"

            else:

                motor_response = (
                    pi.servo_current(
                        MOTOR_ID,
                        -commanded_current
                    )
                )


            # =================================================
            # DISPLAY
            # =================================================

            display_data(
                elevation,
                commanded_current,
                quaternion,
                pressure,
                motor_response
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

        if not args.dry_run:

            zero_motor(pi)

        pi.close()

        udp_socket.close()

        print("[done]")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
