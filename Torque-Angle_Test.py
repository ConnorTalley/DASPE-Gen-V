import argparse
import json
import math
import socket
import time
import numpy as np

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

# Maximum and minimum commanded current
MAX_CURRENT = 1.5
MIN_CURRENT = 0.5

# Current Step
MAX_CURRENT_STEP = 0.0025

# Sensor receiver
UDP_PORT = 5005

# How long we will accept old sensor data
SENSOR_TIMEOUT = 0.5

ELEVATIONS = []
CURRENTS = []

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
# GRAB ONLY MOST RECENT PACKET
# ============================================================

def get_latest_udp_packet(udp_socket):
    # Wait for at least one packet
    try:
        data, address = udp_socket.recvfrom(4096)
    except socket.timeout:
        return None

    latest_packet = (data, address)

    # Now grab everything else currently waiting
    udp_socket.setblocking(False)

    try:
        while True:
            data, address = udp_socket.recvfrom(4096)
            latest_packet = (data, address)
    except BlockingIOError:
        pass
    finally:
        udp_socket.setblocking(True)

    return latest_packet

# ============================================================
# DISPLAY
# ============================================================

def display_data(
    elevation,
    commanded_current,
    quaternion,
    motor_response,
    i
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

    print("Iteration")
    print("-" * 60)
    print(i)

    print()

    print(
        f"Motor response: {motor_response}"
    )

    print("=" * 60)

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
        "--dry-run",
        action="store_true",
        help="Calculate/display current without sending to motor"
    )

    args = parser.parse_args()


    # --------------------------------------------------------
    # Update global motor configuration
    # --------------------------------------------------------

    motor_id = args.motor

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
    print("       EXOSKELETON TORQUE TEST")
    print("=" * 60)

    print()
    print(f"Pi CAN bridge: {args.pi}:{args.pi_port}")
    print(f"Motor ID:      {MOTOR_ID}")
    print(f"Dry run:       {args.dry_run}")

    print()
    print("Waiting for sensor data...")
    print()


    last_sensor_time = 0.0


    try:

        i = 0

        while True:

            loop_start = time.monotonic()

            # =================================================
            # SEND CURRENT
            # =================================================

            commanded_current = (i * MAX_CURRENT_STEP) + MIN_CURRENT

            if commanded_current > MAX_CURRENT:
                print()
                print("Current Ramp Finished")
                break

            if args.dry_run:

                motor_response = "DRY RUN"

            else:

                motor_response = (
                    pi.servo_current(
                        motor_id,
                        -commanded_current
                    )
                )

            # =================================================
            # BRIEF PAUSE
            # =================================================
            
            time.sleep(0.1)

            # =================================================
            # RECEIVE SENSOR DATA
            # =================================================

            latest = get_latest_udp_packet(udp_socket)

            if latest is None:
                print("\n[WARNING] Sensor timeout")
                if not args.dry_run:
                    zero_motor(pi, motor_id)
                continue

            data, address = latest
            packet = json.loads(data.decode("utf-8"))

            # =================================================
            # EXTRACT QUATERNION
            # =================================================

            quaternion = packet["quaternion"]

            # =================================================
            # CALCULATE ELEVATION AND RECORD DATA
            # =================================================
            
            elevation = quaternion_to_elevation(
                quaternion
            )

            ELEVATIONS.append(elevation)
            CURRENTS.append(commanded_current)


            # =================================================
            # DISPLAY
            # =================================================

            display_data(
                elevation,
                commanded_current,
                quaternion,
                motor_response,
                i
            )

            i = i+1


    except KeyboardInterrupt:

        print()
        print("[ctrl] Ctrl-C detected")


    finally:

        # ----------------------------------------------------
        # ALWAYS ZERO MOTOR AND SAVE DATA
        # ----------------------------------------------------

        if not args.dry_run:

            zero_motor(pi)

        pi.close()

        udp_socket.close()

        print("[done]")

        current = np.array(CURRENTS)
        angle = np.array(ELEVATIONS)

        table = np.column_stack((current, angle))

        np.savetxt(
            "torque-angle_lookup_table.csv",
            table,
            delimiter=",",
            header="Current,Elevation",
            comments=""
        )


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()
