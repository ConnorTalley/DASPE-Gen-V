import socket
import json
import math


# ============================================================
# CONFIGURATION
# ============================================================

UDP_IP = "0.0.0.0"
UDP_PORT = 5005


# ============================================================
# NETWORK SETUP
# ============================================================

sock = socket.socket(
    socket.AF_INET,
    socket.SOCK_DGRAM
)

sock.bind(
    (UDP_IP, UDP_PORT)
)

print("======================================")
print(" Jetson Orin Nano Sensor Receiver")
print("======================================")
print(f"Listening on UDP port {UDP_PORT}")
print("Waiting for Raspberry Pi...")
print()


# ============================================================
# QUATERNION → Z ROTATION
# ============================================================

def quaternion_to_z_rotation(q):
    """
    Convert quaternion to rotation around Z axis.

    BNO08X quaternion format:
        i = X
        j = Y
        k = Z
        r = W

    Returns:
        Z rotation in degrees
    """

    x = q["i"]
    y = q["j"]
    z = q["k"]
    w = q["r"]

    # Yaw / rotation about Z
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    )

    yaw_degrees = math.degrees(yaw)

    return yaw_degrees


# ============================================================
# MAIN RECEIVER LOOP
# ============================================================

try:

    while True:

        data, address = sock.recvfrom(4096)

        # Decode JSON
        packet = json.loads(
            data.decode("utf-8")
        )


        # ====================================================
        # PRESSURE DATA
        # ====================================================

        pressure = packet["pressure"]


        # ====================================================
        # QUATERNION
        # ====================================================

        quaternion = packet["quaternion"]


        # Calculate Z-axis rotation
        z_rotation = quaternion_to_z_rotation(
            quaternion
        )


        # ====================================================
        # TERMINAL DISPLAY
        # ====================================================

        print("\033[2J\033[H", end="")

        print("==============================================")
        print("       EXOSKELETON SENSOR DATA")
        print("==============================================")

        print(f"Source: {address[0]}")
        print()


        # ----------------------------------------------------
        # PRESSURE
        # ----------------------------------------------------

        print("PRESSURE SENSORS")
        print("----------------------------------------------")

        for channel, values in pressure.items():

            print(
                f"CH{channel}: "
                f"{values['voltage']:.3f} V   "
                f"{values['percentage']:6.2f}%"
            )


        # ----------------------------------------------------
        # IMU
        # ----------------------------------------------------

        print()
        print("BNO08X")
        print("----------------------------------------------")

        print(
            f"Quaternion: "
            f"i={quaternion['i']:.5f}  "
            f"j={quaternion['j']:.5f}  "
            f"k={quaternion['k']:.5f}  "
            f"r={quaternion['r']:.5f}"
        )

        print()
        print(
            f"Z Rotation: {z_rotation:8.2f}°"
        )

        print("----------------------------------------------")


except KeyboardInterrupt:

    print("\nStopping receiver...")


finally:

    sock.close()
