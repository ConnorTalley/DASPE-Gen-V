import socket
import json
import math
import time
import can


# ============================================================
# CONFIGURATION
# ============================================================

# ------------------------------------------------------------
# Network
# ------------------------------------------------------------

UDP_IP = "0.0.0.0"
UDP_PORT = 5005


# ------------------------------------------------------------
# CAN
# ------------------------------------------------------------

CAN_INTERFACE = "can0"

# CubeMars motor CAN ID
MOTOR_ID = 1


# ------------------------------------------------------------
# CURRENT LIMIT
# ------------------------------------------------------------

# IMPORTANT:
# Start VERY low for initial testing.
#
# Example:
#   0.5 A maximum
#
# Increase only after confirming direction and behavior.

MAX_CURRENT = 0.5


# ------------------------------------------------------------
# ELEVATION / CURRENT PROFILE
# ------------------------------------------------------------

# Format:
#
#     (elevation in degrees, current in amps)
#
# The controller linearly interpolates between these points.

CURRENT_PROFILE = [
    (0.0,   0.0),
    (90.0,  MAX_CURRENT),
    (180.0, 0.5 * MAX_CURRENT),
]


# ------------------------------------------------------------
# CONTROL RATE
# ------------------------------------------------------------

CONTROL_RATE = 100.0
CONTROL_PERIOD = 1.0 / CONTROL_RATE


# ============================================================
# CAN SETUP
# ============================================================

can_bus = can.interface.Bus(
    channel=CAN_INTERFACE,
    interface="socketcan"
)


# ============================================================
# UDP SETUP
# ============================================================

udp_socket = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

udp_socket.bind(
    (UDP_IP, UDP_PORT)
)

# Don't block forever waiting for sensor data.
udp_socket.settimeout(1.0)


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

    # Prevent floating-point errors from creating
    # values slightly outside [-1, 1].
    z_world = max(-1.0, min(1.0, z_world))

    elevation_from_horizontal = math.degrees(
        math.asin(z_world)
    )

    # Our desired definition:
    #
    # 0°   = arm straight down
    # 90°  = arm horizontal
    # 180° = arm straight up

    elevation = elevation_from_horizontal + 90.0

    # Keep within our expected range
    elevation = max(0.0, min(180.0, elevation))

    return elevation


# ============================================================
# CURRENT PROFILE
# ============================================================

def get_current_from_elevation(elevation):

    # Below first point
    if elevation <= CURRENT_PROFILE[0][0]:
        return CURRENT_PROFILE[0][1]

    # Above last point
    if elevation >= CURRENT_PROFILE[-1][0]:
        return CURRENT_PROFILE[-1][1]

    # Find the two points surrounding the current elevation
    for i in range(len(CURRENT_PROFILE) - 1):

        angle_1, current_1 = CURRENT_PROFILE[i]
        angle_2, current_2 = CURRENT_PROFILE[i + 1]

        if angle_1 <= elevation <= angle_2:

            # Linear interpolation
            fraction = (
                (elevation - angle_1)
                / (angle_2 - angle_1)
            )

            current = (
                current_1
                + fraction * (current_2 - current_1)
            )

            return current

    return 0.0


# ============================================================
# CUBEMARS CURRENT COMMAND
# ============================================================

def send_current(current):

    # CubeMars current command uses 0.001 A units.
    current_command = int(current * 1000.0)

    # Convert signed 32-bit integer to 4 bytes.
    current_bytes = current_command.to_bytes(
        4,
        byteorder="big",
        signed=True
    )

    # Current-loop control mode = 1
    #
    # Extended CAN ID:
    #
    # [control mode][motor ID]
    #
    # For motor ID 1:
    # 0x101

    can_id = (1 << 8) | MOTOR_ID

    message = can.Message(
        arbitration_id=can_id,
        is_extended_id=True,
        data=current_bytes
    )

    can_bus.send(message)


# ============================================================
# ZERO CURRENT
# ============================================================

def send_zero_current():

    try:
        send_current(0.0)
    except Exception:
        pass


# ============================================================
# DISPLAY
# ============================================================

def display_data(
    elevation,
    commanded_current,
    quaternion,
    pressure
):

    print("\033[2J\033[H", end="")

    print("================================================")
    print("        EXOSKELETON GRAVITY CONTROLLER")
    print("================================================")

    print()

    # --------------------------------------------------------
    # IMU
    # --------------------------------------------------------

    print("ARM ORIENTATION")
    print("------------------------------------------------")

    print(
        f"Elevation:          {elevation:8.2f} deg"
    )

    print(
        f"Commanded Current:   {commanded_current:8.3f} A"
    )

    print()

    print("Quaternion")
    print(
        f"  i: {quaternion['i']: .5f}"
    )

    print(
        f"  j: {quaternion['j']: .5f}"
    )

    print(
        f"  k: {quaternion['k']: .5f}"
    )

    print(
        f"  r: {quaternion['r']: .5f}"
    )

    # --------------------------------------------------------
    # PRESSURE
    # --------------------------------------------------------

    print()
    print("PRESSURE")
    print("------------------------------------------------")

    for channel, values in pressure.items():

        print(
            f"{channel.upper()}: "
            f"{values['voltage']:.3f} V  "
            f"{values['percentage']:6.2f}%"
        )

    print()
    print("Current profile:")
    print("  0°   → {:.3f} A".format(
        CURRENT_PROFILE[0][1]
    ))

    print("  90°  → {:.3f} A".format(
        MAX_CURRENT
    ))

    print("  180° → {:.3f} A".format(
        CURRENT_PROFILE[-1][1]
    ))

    print("================================================")


# ============================================================
# MAIN CONTROL LOOP
# ============================================================

print()
print("================================================")
print("      GRAVITY CONTROLLER STARTING")
print("================================================")
print()
print(f"CAN interface: {CAN_INTERFACE}")
print(f"Motor ID:      {MOTOR_ID}")
print(f"Maximum:       {MAX_CURRENT:.3f} A")
print()
print("Current profile:")

for angle, current in CURRENT_PROFILE:
    print(
        f"  {angle:6.1f}° → {current:.3f} A"
    )

print()
print("Waiting for Raspberry Pi...")
print()


last_packet_time = time.monotonic()


try:

    while True:

        loop_start = time.monotonic()

        # ----------------------------------------------------
        # RECEIVE SENSOR DATA
        # ----------------------------------------------------

        try:

            data, address = udp_socket.recvfrom(4096)

            packet = json.loads(
                data.decode("utf-8")
            )

            last_packet_time = time.monotonic()

        except socket.timeout:

            # No sensor data.
            # IMPORTANT: remove motor current.
            send_zero_current()

            print(
                "\nWARNING: No sensor data received."
            )

            continue


        # ----------------------------------------------------
        # EXTRACT DATA
        # ----------------------------------------------------

        quaternion = packet["quaternion"]

        pressure = packet["pressure"]


        # ----------------------------------------------------
        # CALCULATE ELEVATION
        # ----------------------------------------------------

        elevation = quaternion_to_elevation(
            quaternion
        )


        # ----------------------------------------------------
        # CALCULATE DESIRED CURRENT
        # ----------------------------------------------------

        commanded_current = (
            get_current_from_elevation(
                elevation
            )
        )


        # ----------------------------------------------------
        # SEND MOTOR CURRENT
        # ----------------------------------------------------

        send_current(
            commanded_current
        )


        # ----------------------------------------------------
        # DISPLAY
        # ----------------------------------------------------

        display_data(
            elevation,
            commanded_current,
            quaternion,
            pressure
        )


        # ----------------------------------------------------
        # CONTROL RATE
        # ----------------------------------------------------

        elapsed = (
            time.monotonic()
            - loop_start
        )

        sleep_time = (
            CONTROL_PERIOD
            - elapsed
        )

        if sleep_time > 0:
            time.sleep(sleep_time)


except KeyboardInterrupt:

    print()
    print("Stopping controller...")


finally:

    # ALWAYS remove motor current
    send_zero_current()

    can_bus.shutdown()

    udp_socket.close()

    print("Motor current set to zero.")
