import socket
import json
import math


# ============================================================
# CONFIGURATION
# ============================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5005


# ============================================================
# NETWORK
# ============================================================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

sock.bind((UDP_IP, UDP_PORT))


# ============================================================
# QUATERNION FUNCTIONS
# ============================================================

def quaternion_to_x_axis(q):
    """
    Determine where the IMU's +X axis is pointing
    in the BNO08X/world reference frame.

    Quaternion format:
        i = X
        j = Y
        k = Z
        r = W

    Returns:
        x_world, y_world, z_world
    """

    i = q["i"]
    j = q["j"]
    k = q["k"]
    r = q["r"]

    x_world = 1.0 - 2.0 * (j*j + k*k)

    y_world = 2.0 * (i*j + r*k)

    z_world = 2.0 * (i*k - r*j)

    return x_world, y_world, z_world


def quaternion_to_elevation(q):
    """
    Calculate arm elevation from the IMU's +X axis.

    IMPORTANT:
    This currently assumes the BNO08X world's Z axis
    corresponds to vertical.

    The zero/reference direction can be adjusted after
    testing the actual mounted sensor.
    """

    x_world, y_world, z_world = quaternion_to_x_axis(q)

    # Vertical component of arm axis
    vertical = max(-1.0, min(1.0, z_world))

    # Angle from horizontal
    elevation_from_horizontal = math.degrees(
        math.asin(vertical)
    )

    # Convert to elevation where:
    #   0°   = straight down
    #   90°  = horizontal
    #   180° = straight up
    #
    # This assumes +Z is UP.
    elevation = elevation_from_horizontal + 90.0

    return elevation, x_world, y_world, z_world


# ============================================================
# MAIN
# ============================================================

print("==============================================")
print("      EXOSKELETON IMU TEST")
print("==============================================")
print("Waiting for Raspberry Pi...")
print()


try:

    while True:

        data, address = sock.recvfrom(4096)

        packet = json.loads(
            data.decode("utf-8")
        )

        q = packet["quaternion"]

        elevation, xw, yw, zw = quaternion_to_elevation(q)

        # ----------------------------------------------------
        # DISPLAY
        # ----------------------------------------------------

        print("\033[2J\033[H", end="")

        print("==============================================")
        print("       EXOSKELETON SENSOR DATA")
        print("==============================================")

        print(f"Pi: {address[0]}")
        print()

        # ----------------------------------------------------
        # PRESSURE
        # ----------------------------------------------------

        print("PRESSURE")
        print("----------------------------------------------")

        for channel, values in packet["pressure"].items():

            print(
                f"{channel.upper()}: "
                f"{values['voltage']:.3f} V   "
                f"{values['percentage']:6.2f}%"
            )

        # ----------------------------------------------------
        # QUATERNION
        # ----------------------------------------------------

        print()
        print("QUATERNION")
        print("----------------------------------------------")

        print(
            f"i = {q['i']: .5f}"
        )

        print(
            f"j = {q['j']: .5f}"
        )

        print(
            f"k = {q['k']: .5f}"
        )

        print(
            f"r = {q['r']: .5f}"
        )

        # ----------------------------------------------------
        # ARM AXIS
        # ----------------------------------------------------

        print()
        print("ARM +X AXIS IN WORLD FRAME")
        print("----------------------------------------------")

        print(f"X = {xw: .4f}")
        print(f"Y = {yw: .4f}")
        print(f"Z = {zw: .4f}")

        # ----------------------------------------------------
        # ELEVATION
        # ----------------------------------------------------

        print()
        print("ARM ELEVATION")
        print("----------------------------------------------")

        print(
            f"{elevation:8.2f} degrees"
        )

        print("----------------------------------------------")


except KeyboardInterrupt:

    print("\nStopping receiver...")


finally:

    sock.close()
