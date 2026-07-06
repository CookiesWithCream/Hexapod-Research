# hexapod_status_snapshot.py
# Purpose:
# - Print all Dynamixel AX motor status
# - Print raw position, degree offset from READY_POSE, load, voltage, temp, moving state
# - Capture current physical pose as a READY_POSE dictionary
#
# Example:
#   python hexapod_status_snapshot.py --port COM6 status
#   python hexapod_status_snapshot.py --port COM6 snapshot

from dynamixel_sdk import PortHandler, PacketHandler
import argparse
import time

# =========================
# USER CONFIG
# =========================

DEFAULT_PORT = "COM6"
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

MOTOR_IDS = list(range(1, 19))

RAW_MIN = 0
RAW_MAX = 1023
RAW_PER_DEG = 1023.0 / 300.0

ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43
ADDR_MOVING = 46

LEN_POSITION = 2
LEN_LOAD = 2
LEN_VOLTAGE = 1
LEN_TEMPERATURE = 1
LEN_MOVING = 1

TEMP_WARN_C = 50
TEMP_STOP_C = 58
LOAD_WARN = 450
LOAD_STOP = 700
VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5

READY_POSE = {
    1: 460,   # RL_hip
    2: 747,   # FL_hip
    3: 411,   # FR_femur
    4: 366,   # FL_femur
    5: 798,   # FR_tibia
    6: 796,   # FL_tibia
    7: 608,   # MR_hip
    8: 753,   # ML_hip
    9: 627,   # MR_femur
    10: 437,  # ML_femur
    11: 216,  # MR_tibia
    12: 787,  # ML_tibia
    13: 578,  # RR_hip
    14: 575,  # FR_hip
    15: 641,  # RR_femur
    16: 412,  # RL_femur
    17: 189,  # RR_tibia
    18: 817,  # RL_tibia
}

JOINT_NAMES = {
    1: "RL_hip",
    2: "FL_hip",
    3: "FR_femur",
    4: "FL_femur",
    5: "FR_tibia",
    6: "FL_tibia",
    7: "MR_hip",
    8: "ML_hip",
    9: "MR_femur",
    10: "ML_femur",
    11: "MR_tibia",
    12: "ML_tibia",
    13: "RR_hip",
    14: "FR_hip",
    15: "RR_femur",
    16: "RL_femur",
    17: "RR_tibia",
    18: "RL_tibia",
}


# =========================
# LOW LEVEL HELPERS
# =========================

def decode_ax_load(raw_load):
    """
    AX-series present load is a 10-bit magnitude with direction bit.
    This is not exact torque. Treat it as approximate motor stress/resistance.
    """
    if raw_load is None:
        return None

    magnitude = raw_load & 0x03FF
    direction = raw_load & 0x0400

    if direction:
        return -magnitude
    return magnitude


def read1(packet, port, dxl_id, addr):
    value, comm_result, error = packet.read1ByteTxRx(port, dxl_id, addr)
    if comm_result != 0 or error != 0:
        return None
    return value


def read2(packet, port, dxl_id, addr):
    value, comm_result, error = packet.read2ByteTxRx(port, dxl_id, addr)
    if comm_result != 0 or error != 0:
        return None
    return value


def get_motor_status(packet, port, dxl_id):
    pos = read2(packet, port, dxl_id, ADDR_PRESENT_POSITION)
    raw_load = read2(packet, port, dxl_id, ADDR_PRESENT_LOAD)
    voltage_raw = read1(packet, port, dxl_id, ADDR_PRESENT_VOLTAGE)
    temp = read1(packet, port, dxl_id, ADDR_PRESENT_TEMPERATURE)
    moving = read1(packet, port, dxl_id, ADDR_MOVING)

    load = decode_ax_load(raw_load)
    voltage = voltage_raw / 10.0 if voltage_raw is not None else None

    return {
        "id": dxl_id,
        "name": JOINT_NAMES.get(dxl_id, f"ID_{dxl_id}"),
        "pos": pos,
        "load": load,
        "voltage": voltage,
        "temp": temp,
        "moving": moving,
    }


def warning_text(status):
    warnings = []

    if status["pos"] is None:
        warnings.append("NO_RESPONSE")

    if status["voltage"] is not None:
        if status["voltage"] <= VOLT_STOP_V:
            warnings.append("VOLT_STOP")
        elif status["voltage"] <= VOLT_WARN_V:
            warnings.append("LOW_VOLTAGE")

    if status["temp"] is not None:
        if status["temp"] >= TEMP_STOP_C:
            warnings.append("TEMP_STOP")
        elif status["temp"] >= TEMP_WARN_C:
            warnings.append("TEMP_WARN")

    if status["load"] is not None:
        abs_load = abs(status["load"])
        if abs_load >= LOAD_STOP:
            warnings.append("LOAD_STOP")
        elif abs_load >= LOAD_WARN:
            warnings.append("LOAD_WARN")

    if not warnings:
        return "OK"

    return ",".join(warnings)


def connect(port_name):
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        raise RuntimeError(f"Failed to open port: {port_name}")

    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError(f"Failed to set baudrate: {BAUDRATE}")

    return port, packet


# =========================
# COMMANDS
# =========================

def print_status(port_name):
    port, packet = connect(port_name)

    print("\n===================================================")
    print(" HEXAPOD MOTOR STATUS / SNAPSHOT TOOL")
    print("===================================================")
    print(f"Port: {port_name}")
    print(f"Baud: {BAUDRATE}")
    print("Connected.\n")

    print(
        f"{'ID':>2} {'Joint':<10} {'Raw':>5} {'Ready':>6} "
        f"{'Delta':>6} {'DegFromReady':>12} {'Load':>7} "
        f"{'Volt':>6} {'Temp':>5} {'Moving':>6} {'Warnings'}"
    )
    print("-" * 112)

    connected = 0
    max_temp = None
    min_voltage = None
    max_abs_load = 0

    for dxl_id in MOTOR_IDS:
        s = get_motor_status(packet, port, dxl_id)
        warn = warning_text(s)

        raw = s["pos"]
        ready = READY_POSE.get(dxl_id)

        if raw is not None:
            connected += 1
            delta = raw - ready if ready is not None else None
            deg = delta / RAW_PER_DEG if delta is not None else None
        else:
            delta = None
            deg = None

        load = s["load"]
        voltage = s["voltage"]
        temp = s["temp"]
        moving = s["moving"]

        if temp is not None:
            max_temp = temp if max_temp is None else max(max_temp, temp)
        if voltage is not None:
            min_voltage = voltage if min_voltage is None else min(min_voltage, voltage)
        if load is not None:
            max_abs_load = max(max_abs_load, abs(load))

        print(
            f"{dxl_id:>2} {s['name']:<10} "
            f"{raw if raw is not None else '---':>5} "
            f"{ready if ready is not None else '---':>6} "
            f"{delta if delta is not None else '---':>6} "
            f"{deg if deg is not None else 0:>12.2f} "
            f"{load if load is not None else '---':>7} "
            f"{voltage if voltage is not None else 0:>6.1f} "
            f"{temp if temp is not None else '---':>5} "
            f"{moving if moving is not None else '---':>6} "
            f"{warn}"
        )

    print("\n===================================================")
    print(" SUMMARY")
    print("===================================================")
    print(f"Connected    : {connected}/18")
    print(f"Max temp     : {max_temp if max_temp is not None else 'N/A'} C")
    print(f"Min voltage  : {min_voltage if min_voltage is not None else 'N/A'} V")
    print(f"Max abs load : {max_abs_load}")
    print("===================================================\n")

    port.closePort()


def print_ready_pose_snapshot(port_name):
    port, packet = connect(port_name)

    print("\n===================================================")
    print(" CURRENT PHYSICAL READY POSE SNAPSHOT")
    print("===================================================")
    print("Copy this into your controller if this is the best physical standing pose.\n")
    print("READY_POSE = {")

    for dxl_id in MOTOR_IDS:
        pos = read2(packet, port, dxl_id, ADDR_PRESENT_POSITION)
        name = JOINT_NAMES.get(dxl_id, f"ID_{dxl_id}")
        if pos is None:
            print(f"    {dxl_id}: None,   # {name} NO_RESPONSE")
        else:
            print(f"    {dxl_id}: {pos},   # {name}")

    print("}")
    print("\n===================================================\n")

    port.closePort()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("command", choices=["status", "snapshot"])
    args = parser.parse_args()

    if args.command == "status":
        print_status(args.port)
    elif args.command == "snapshot":
        print_ready_pose_snapshot(args.port)


if __name__ == "__main__":
    main()