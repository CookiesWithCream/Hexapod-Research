# joint_limit_mechanical_retry_probe.py
# Purpose:
# - Find practical/mechanical joint range for Dynamixel AX-series motors.
# - Moves each selected motor step by step in the positive raw direction.
# - If the motor cannot follow the target, it retries 3 more times.
# - If it still cannot follow, it records the last successful position as the limit.
# - Then it returns to ready and repeats in the negative direction.
#
# Example:
#   python joint_limit_mechanical_retry_probe.py --port COM6 --ids 1 2 7 8 13 14
#   python joint_limit_mechanical_retry_probe.py --port COM6 --all
#
# IMPORTANT:
# - This script intentionally searches until the motor cannot continue.
# - Keep your hand near the power switch.
# - Support/lift the robot if needed.
# - This is for research/calibration, not normal movement.

from dynamixel_sdk import PortHandler, PacketHandler
import argparse
import csv
import time
from datetime import datetime

# =========================
# BASIC CONFIG
# =========================

DEFAULT_PORT = "COM6"
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

MOTOR_IDS = list(range(1, 19))

RAW_MIN = 0
RAW_MAX = 1023
RAW_EDGE_MARGIN = 5
RAW_PER_DEG = 1023.0 / 300.0

# AX-series Control Table
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43
ADDR_MOVING = 46

TORQUE_ON = 1

# Safety thresholds
TEMP_STOP_C = 58
VOLT_STOP_V = 9.5

# Higher because this is limit probing.
# Lower it if your robot jams too hard.
LOAD_STOP = 900

# Motion settings
DEFAULT_SPEED = 80
DEFAULT_TORQUE_LIMIT = 700
DEFAULT_STEP_RAW = 10
DEFAULT_SETTLE = 0.35
DEFAULT_RETRIES = 3

# If present position is farther than this from target,
# treat it as "motor did not follow".
FOLLOW_TOLERANCE_RAW = 25

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

def clamp(value, low, high):
    return max(low, min(high, value))


def raw_to_deg(raw_delta):
    return raw_delta / RAW_PER_DEG


def decode_ax_load(raw_load):
    if raw_load is None:
        return None

    magnitude = raw_load & 0x03FF
    direction = raw_load & 0x0400

    if direction:
        return -magnitude
    return magnitude


def connect(port_name):
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        raise RuntimeError(f"Failed to open port: {port_name}")

    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError(f"Failed to set baudrate: {BAUDRATE}")

    return port, packet


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


def write1(packet, port, dxl_id, addr, value):
    comm_result, error = packet.write1ByteTxRx(port, dxl_id, addr, value)
    return comm_result == 0 and error == 0


def write2(packet, port, dxl_id, addr, value):
    comm_result, error = packet.write2ByteTxRx(port, dxl_id, addr, value)
    return comm_result == 0 and error == 0


def get_status(packet, port, dxl_id):
    pos = read2(packet, port, dxl_id, ADDR_PRESENT_POSITION)
    raw_load = read2(packet, port, dxl_id, ADDR_PRESENT_LOAD)
    voltage_raw = read1(packet, port, dxl_id, ADDR_PRESENT_VOLTAGE)
    temp = read1(packet, port, dxl_id, ADDR_PRESENT_TEMPERATURE)
    moving = read1(packet, port, dxl_id, ADDR_MOVING)

    load = decode_ax_load(raw_load)
    voltage = voltage_raw / 10.0 if voltage_raw is not None else None

    return {
        "pos": pos,
        "load": load,
        "voltage": voltage,
        "temp": temp,
        "moving": moving,
    }


def configure_motor(packet, port, dxl_id, speed, torque_limit):
    write1(packet, port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ON)
    write2(packet, port, dxl_id, ADDR_MOVING_SPEED, speed)
    write2(packet, port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)


def move_motor(packet, port, dxl_id, target_raw, settle):
    target_raw = int(clamp(target_raw, RAW_MIN, RAW_MAX))
    write2(packet, port, dxl_id, ADDR_GOAL_POSITION, target_raw)
    time.sleep(settle)


def read_current_ready_positions(packet, port, ids):
    ready = {}

    print("\n===================================================")
    print(" CAPTURING LIVE READY POSITIONS")
    print("===================================================")

    for dxl_id in ids:
        pos = read2(packet, port, dxl_id, ADDR_PRESENT_POSITION)
        name = JOINT_NAMES.get(dxl_id, f"ID_{dxl_id}")

        if pos is None:
            print(f"ID {dxl_id:>2} {name:<10}: NO_RESPONSE")
        else:
            ready[dxl_id] = pos
            print(f"ID {dxl_id:>2} {name:<10}: ready raw = {pos}")

    print("===================================================\n")
    return ready


def check_failure_reason(status, target_raw):
    pos = status["pos"]
    load = status["load"]
    voltage = status["voltage"]
    temp = status["temp"]

    if pos is None:
        return "NO_RESPONSE"

    if voltage is not None and voltage <= VOLT_STOP_V:
        return "VOLT_STOP"

    if temp is not None and temp >= TEMP_STOP_C:
        return "TEMP_STOP"

    if load is not None and abs(load) >= LOAD_STOP:
        return "LOAD_STOP"

    if abs(pos - target_raw) > FOLLOW_TOLERANCE_RAW:
        return "POSITION_NOT_FOLLOWING"

    return None


def print_step(dxl_id, direction_name, target_raw, status, attempt_note=""):
    name = JOINT_NAMES.get(dxl_id, f"ID_{dxl_id}")
    pos = status["pos"]
    load = status["load"]
    voltage = status["voltage"]
    temp = status["temp"]

    print(
        f"{name:<10} {direction_name:<8} "
        f"target={target_raw:>4} "
        f"pos={pos if pos is not None else '---':>4} "
        f"load={load if load is not None else '---':>5} "
        f"volt={voltage if voltage is not None else 0:>4.1f} "
        f"temp={temp if temp is not None else '---':>3} "
        f"{attempt_note}"
    )


def try_target_with_retries(packet, port, dxl_id, target_raw, settle, retries):
    """
    Sends target once. If motor cannot follow, retries same target.
    Returns:
        success: bool
        reason: str
        final_status: dict
        attempts_used: int
    """
    for attempt in range(retries + 1):
        move_motor(packet, port, dxl_id, target_raw, settle)
        status = get_status(packet, port, dxl_id)
        reason = check_failure_reason(status, target_raw)

        if attempt == 0:
            note = ""
        else:
            note = f"retry {attempt}/{retries}"

        print_step(dxl_id, "TRY", target_raw, status, note)

        if reason is None:
            return True, "OK", status, attempt

        # Critical stops should not be retried.
        if reason in ["VOLT_STOP", "TEMP_STOP", "LOAD_STOP", "NO_RESPONSE"]:
            return False, reason, status, attempt

        # POSITION_NOT_FOLLOWING can be retried.

    return False, reason, status, retries


def return_to_ready(packet, port, dxl_id, ready_raw, settle):
    print(f"Returning ID {dxl_id} {JOINT_NAMES.get(dxl_id, '')} to ready raw {ready_raw}...")
    move_motor(packet, port, dxl_id, ready_raw, settle)
    time.sleep(settle)


# =========================
# PROBE LOGIC
# =========================

def probe_direction(packet, port, dxl_id, ready_raw, direction, step_raw, settle, retries):
    direction_name = "POSITIVE" if direction > 0 else "NEGATIVE"

    print("\n---------------------------------------------------")
    print(f"PROBING {JOINT_NAMES.get(dxl_id, f'ID_{dxl_id}')} ID {dxl_id} {direction_name}")
    print("---------------------------------------------------")

    last_good_raw = ready_raw
    last_good_deg = 0.0
    stop_reason = "RAW_EDGE"

    current_target = ready_raw

    while True:
        next_target = current_target + (direction * step_raw)

        if next_target <= RAW_MIN + RAW_EDGE_MARGIN:
            stop_reason = "RAW_EDGE_LOW"
            break

        if next_target >= RAW_MAX - RAW_EDGE_MARGIN:
            stop_reason = "RAW_EDGE_HIGH"
            break

        success, reason, status, attempts_used = try_target_with_retries(
            packet=packet,
            port=port,
            dxl_id=dxl_id,
            target_raw=next_target,
            settle=settle,
            retries=retries,
        )

        if success:
            current_target = next_target
            last_good_raw = next_target
            last_good_deg = raw_to_deg(last_good_raw - ready_raw)
        else:
            stop_reason = reason
            print(f"STOP {direction_name}: {reason}")
            print(f"Last successful raw: {last_good_raw}")
            print(f"Last successful deg from ready: {last_good_deg:+.2f}")
            break

    return {
        "best_raw": last_good_raw,
        "best_deg": last_good_deg,
        "stop_reason": stop_reason,
    }


def probe_motor(packet, port, dxl_id, ready_raw, step_raw, settle, retries, speed, torque_limit):
    name = JOINT_NAMES.get(dxl_id, f"ID_{dxl_id}")

    print("\n===================================================")
    print(f"MECHANICAL RETRY LIMIT PROBE: {name} ID {dxl_id}")
    print("===================================================")
    print(f"Ready raw      : {ready_raw}")
    print(f"Step raw       : {step_raw}")
    print(f"Approx step deg: {raw_to_deg(step_raw):.2f}")
    print(f"Retries        : {retries}")
    print("===================================================")

    configure_motor(packet, port, dxl_id, speed, torque_limit)

    return_to_ready(packet, port, dxl_id, ready_raw, settle)
    positive = probe_direction(packet, port, dxl_id, ready_raw, +1, step_raw, settle, retries)

    return_to_ready(packet, port, dxl_id, ready_raw, settle)
    negative = probe_direction(packet, port, dxl_id, ready_raw, -1, step_raw, settle, retries)

    return_to_ready(packet, port, dxl_id, ready_raw, settle)

    result = {
        "id": dxl_id,
        "joint": name,
        "ready_raw": ready_raw,
        "negative_deg": negative["best_deg"],
        "negative_raw": negative["best_raw"],
        "negative_stop": negative["stop_reason"],
        "positive_deg": positive["best_deg"],
        "positive_raw": positive["best_raw"],
        "positive_stop": positive["stop_reason"],
    }

    print("\nRESULT")
    print("---------------------------------------------------")
    print(f"Joint              : {name} ID {dxl_id}")
    print(f"Ready raw          : {ready_raw}")
    print(f"Safe negative deg  : {result['negative_deg']:+.2f}")
    print(f"Safe negative raw  : {result['negative_raw']}")
    print(f"Negative stop      : {result['negative_stop']}")
    print(f"Safe positive deg  : {result['positive_deg']:+.2f}")
    print(f"Safe positive raw  : {result['positive_raw']}")
    print(f"Positive stop      : {result['positive_stop']}")

    return result


def save_results_csv(results):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"mechanical_joint_limit_results_{timestamp}.csv"

    fields = [
        "id",
        "joint",
        "ready_raw",
        "negative_deg",
        "negative_raw",
        "negative_stop",
        "positive_deg",
        "positive_raw",
        "positive_stop",
    ]

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(results)

    return filename


def print_python_limit_dict(results):
    print("\n===================================================")
    print("PYTHON LIMIT DICTIONARY")
    print("===================================================")
    print("JOINT_LIMITS_DEG = {")
    for r in results:
        print(
            f"    {r['id']}: "
            f"({r['negative_deg']:.2f}, {r['positive_deg']:.2f}),"
            f"   # {r['joint']}"
        )
    print("}")
    print("===================================================\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--all", action="store_true", help="Probe all 18 motors")
    parser.add_argument("--ids", nargs="+", type=int, help="Probe selected motor IDs")
    parser.add_argument("--step-raw", type=int, default=DEFAULT_STEP_RAW)
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE)
    parser.add_argument("--retries", type=int, default=DEFAULT_RETRIES)
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED)
    parser.add_argument("--torque-limit", type=int, default=DEFAULT_TORQUE_LIMIT)
    args = parser.parse_args()

    if not args.all and not args.ids:
        raise SystemExit("Use --all or --ids, example: --ids 1 2 7 8 13 14")

    ids_to_probe = MOTOR_IDS if args.all else args.ids

    print("\n===================================================")
    print(" DANGER / RESEARCH MODE")
    print("===================================================")
    print("This script searches until the motor cannot follow.")
    print("It will retry a failed target 3 times by default.")
    print("Keep your hand near the power switch.")
    print("Support or lift the robot if needed.")
    print("Recommended first test: --ids 3 --step-raw 10")
    print("===================================================")
    input("Press ENTER to continue, or Ctrl+C to cancel...")

    port, packet = connect(args.port)

    results = []

    try:
        ready_positions = read_current_ready_positions(packet, port, ids_to_probe)

        for dxl_id in ids_to_probe:
            if dxl_id not in ready_positions:
                print(f"Skipping ID {dxl_id}: no ready position captured.")
                continue

            result = probe_motor(
                packet=packet,
                port=port,
                dxl_id=dxl_id,
                ready_raw=ready_positions[dxl_id],
                step_raw=args.step_raw,
                settle=args.settle,
                retries=args.retries,
                speed=args.speed,
                torque_limit=args.torque_limit,
            )

            results.append(result)

            print("\nCooling/settling pause...")
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        port.closePort()

    if results:
        csv_file = save_results_csv(results)
        print_python_limit_dict(results)
        print(f"Saved CSV: {csv_file}")


if __name__ == "__main__":
    main()