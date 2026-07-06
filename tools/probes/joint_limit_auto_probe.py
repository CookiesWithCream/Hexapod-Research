import sys
import time
import argparse
import csv
from datetime import datetime
from typing import Dict, Optional, Tuple, List

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("Missing library: dynamixel_sdk")
    print("Install using:")
    print("pip install dynamixel-sdk")
    sys.exit(1)

from hexapod_kinematics import (
    READY_POSE,
    LEG_JOINTS,
    JOINT_LIMITS,
    JOINT_DIRECTIONS,
    RAW_PER_DEG,
)


# ============================================================
# AUTOMATIC JOINT LIMIT PROBE
# ============================================================
#
# Purpose:
#   Hands-free safe-ish limit discovery.
#
# It does NOT violently brute force.
# It moves gradually and stops when:
#   - motor position stops following target
#   - load becomes too high
#   - temperature becomes too high
#   - raw position gets close to 0 or 1023
#   - software max probe angle is reached
#
# Run examples:
#
#   python joint_limit_auto_probe.py --joint MR_hip
#   python joint_limit_auto_probe.py --leg MR
#   python joint_limit_auto_probe.py --all
#
# Safer smaller movement:
#
#   python joint_limit_auto_probe.py --joint MR_hip --step 2 --max-angle 35
#
# Faster no pause:
#
#   python joint_limit_auto_probe.py --joint MR_hip --yes
#
# ============================================================


DEFAULT_PORT = "COM6"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TORQUE_ENABLE = 1
COMM_SUCCESS = 0

DEFAULT_SPEED = 35
DEFAULT_STEP_DEG = 2.0
DEFAULT_MAX_ANGLE = 45.0
DEFAULT_SETTLE_SECONDS = 0.18
DEFAULT_CENTER_WAIT = 0.65

# Safety thresholds
RAW_EDGE_STOP = 8
TEMP_WARN_C = 50
TEMP_STOP_C = 58

# AX load is not perfect torque, but useful for jam/stall warning.
# Raw load can be 0-1023 one direction, 1024-2047 other direction.
LOAD_WARN = 450
LOAD_STOP = 700

# Stall detection
MIN_RAW_MOVEMENT_PER_STEP = 1
STALL_COUNT_LIMIT = 3
FOLLOW_ERROR_RAW_LIMIT = 35


# ============================================================
# LEG 5 / 6 SPECIAL MOVEMENT SIGN
# ============================================================
#
# Console mapping:
#   1 = FL
#   2 = ML
#   3 = RL
#   4 = FR
#   5 = MR
#   6 = RR
#
# You discovered:
#   MR and RR femur/tibia are physically opposite.
#
# This script applies that same correction when probing logical
# positive/negative movement.
# ============================================================

LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1, "femur": 1, "tibia": 1},
    "ML": {"hip": 1, "femur": 1, "tibia": 1},
    "RL": {"hip": 1, "femur": 1, "tibia": 1},
    "FR": {"hip": 1, "femur": 1, "tibia": 1},

    # Legs 5 and 6 special reversed femur/tibia
    "MR": {"hip": 1, "femur": -1, "tibia": -1},
    "RR": {"hip": 1, "femur": -1, "tibia": -1},
}


SESSION_READY: Dict[int, int] = {}


JOINT_TEST_ORDER = [
    "FL_hip", "FL_femur", "FL_tibia",
    "ML_hip", "ML_femur", "ML_tibia",
    "RL_hip", "RL_femur", "RL_tibia",
    "FR_hip", "FR_femur", "FR_tibia",
    "MR_hip", "MR_femur", "MR_tibia",
    "RR_hip", "RR_femur", "RR_tibia",
]


# ============================================================
# MAP HELPERS
# ============================================================

def build_motor_to_joint_map() -> Dict[int, str]:
    result = {}
    for joint_name, info in JOINT_LIMITS.items():
        result[int(info["id"])] = joint_name
    return result


MOTOR_TO_JOINT = build_motor_to_joint_map()


def joint_to_leg_part(joint_name: str) -> Tuple[str, str]:
    for leg_name, parts in LEG_JOINTS.items():
        for part_name, jn in parts.items():
            if jn == joint_name:
                return leg_name, part_name
    return "?", "?"


# ============================================================
# BUS
# ============================================================

class DynamixelBus:
    def __init__(self, port_name: str):
        self.port_name = port_name
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def open(self) -> bool:
        print("\n===================================================")
        print(" CONNECTING")
        print("===================================================")
        print(f"Port: {self.port_name}")
        print(f"Baud: {BAUDRATE}")

        if not self.port_handler.openPort():
            print(f"FAILED: Cannot open {self.port_name}")
            print("Reminder: default is COM6. If failed, check Device Manager COM port.")
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            print(f"FAILED: Cannot set baudrate {BAUDRATE}")
            print("Reminder: check COM port / U2D2 / CM-530.")
            return False

        print("Connected.")
        return True

    def close(self):
        self.port_handler.closePort()
        print("Port closed.")

    def write1(self, motor_id: int, address: int, value: int) -> bool:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            motor_id,
            address,
            value,
        )

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        value = int(max(0, min(1023, value)))

        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler,
            motor_id,
            address,
            value,
        )

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def read1(self, motor_id: int, address: int) -> Optional[int]:
        value, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            motor_id,
            address,
        )

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def read2(self, motor_id: int, address: int) -> Optional[int]:
        value, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            motor_id,
            address,
        )

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def enable_torque(self, motor_id: int):
        self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_speed(self, motor_id: int, speed: int):
        self.write2(motor_id, ADDR_MOVING_SPEED, speed)

    def move_motor(self, motor_id: int, raw: int, speed: int):
        self.enable_torque(motor_id)
        self.set_speed(motor_id, speed)
        time.sleep(0.01)
        self.write2(motor_id, ADDR_GOAL_POSITION, raw)

    def move_many(self, targets: Dict[int, int], speed: int):
        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
            time.sleep(0.006)

        for motor_id, raw in targets.items():
            self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            time.sleep(0.006)


# ============================================================
# STATUS / FEEDBACK
# ============================================================

def decode_load_value(raw_load: Optional[int]) -> Optional[int]:
    if raw_load is None:
        return None

    if raw_load <= 1023:
        return raw_load

    return raw_load - 1024


def decode_load_text(raw_load: Optional[int]) -> str:
    if raw_load is None:
        return "----"

    if raw_load <= 1023:
        return f"+{raw_load}"

    return f"-{raw_load - 1024}"


def capture_current_pose(bus: DynamixelBus) -> Dict[int, int]:
    captured = {}

    print("\n===================================================")
    print(" CAPTURING CURRENT POSE AS SESSION_READY")
    print("===================================================")

    for motor_id in sorted(READY_POSE.keys()):
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)

        if pos is None:
            fallback = READY_POSE[motor_id]
            captured[motor_id] = fallback
            print(f"ID {motor_id:>2}: NO REPLY, fallback old ready {fallback}")
        else:
            captured[motor_id] = pos
            joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
            print(f"ID {motor_id:>2} {joint_name:<10}: {pos}")

    return captured


def get_motor_status(bus: DynamixelBus, motor_id: int) -> Dict[str, object]:
    joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
    leg_name, part_name = joint_to_leg_part(joint_name)

    raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
    speed = bus.read2(motor_id, ADDR_PRESENT_SPEED)
    load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
    volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
    temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

    session_ready = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    if raw is None:
        sess_deg = None
        delta_raw = None
    else:
        direction = JOINT_DIRECTIONS.get(joint_name, 1)
        sess_deg = ((raw - session_ready) / RAW_PER_DEG) * direction
        delta_raw = raw - session_ready

    load_value = decode_load_value(load_raw)

    warnings = []

    if raw is None:
        warnings.append("NO_REPLY")
    else:
        if raw <= RAW_EDGE_STOP:
            warnings.append("RAW_EDGE_LOW")
        if raw >= 1023 - RAW_EDGE_STOP:
            warnings.append("RAW_EDGE_HIGH")

    if load_value is not None:
        if load_value >= LOAD_STOP:
            warnings.append("LOAD_STOP")
        elif load_value >= LOAD_WARN:
            warnings.append("LOAD_WARN")

    if temp is not None:
        if temp >= TEMP_STOP_C:
            warnings.append("TEMP_STOP")
        elif temp >= TEMP_WARN_C:
            warnings.append("TEMP_WARN")

    return {
        "motor_id": motor_id,
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "raw": raw,
        "speed": speed,
        "session_deg": sess_deg,
        "delta_raw": delta_raw,
        "session_ready": session_ready,
        "load_raw": load_raw,
        "load_value": load_value,
        "load_text": decode_load_text(load_raw),
        "volt": volt,
        "temp": temp,
        "warnings": warnings,
    }


def print_status_table(bus: DynamixelBus):
    print("\n===================================================")
    print(" MOTOR STATUS")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'SessDeg':>8} {'Ready':>6} {'Delta':>6} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 115)

    connected = 0

    for motor_id in sorted(READY_POSE.keys()):
        s = get_motor_status(bus, motor_id)

        if s["raw"] is None:
            print(
                f"{motor_id:>2} {s['joint']:<10} {s['leg']:<2} {s['part']:<5} "
                f"{'----':>4} {'----':>8} {s['session_ready']:>6} {'----':>6} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1

        volt_text = "----" if s["volt"] is None else f"{s['volt'] / 10:.1f}"
        temp_text = "----" if s["temp"] is None else str(s["temp"])
        warn_text = "OK" if not s["warnings"] else ",".join(s["warnings"])

        print(
            f"{motor_id:>2} {s['joint']:<10} {s['leg']:<2} {s['part']:<5} "
            f"{s['raw']:>4} {s['session_deg']:>8.2f} {s['session_ready']:>6} {s['delta_raw']:>+6} "
            f"{s['load_text']:>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 115)
    print(f"Connected: {connected}/18")


# ============================================================
# RAW CONVERSION
# ============================================================

def clamp_raw_for_joint(joint_name: str, raw: int) -> int:
    info = JOINT_LIMITS[joint_name]

    min_raw = int(info["min_raw"])
    max_raw = int(info["max_raw"])

    raw = int(max(min_raw, min(max_raw, raw)))
    raw = int(max(0, min(1023, raw)))

    return raw


def logical_offset_to_raw(joint_name: str, logical_offset_deg: float) -> Tuple[int, int, float]:
    """
    logical_offset_deg:
        + means bridge logical positive
        - means bridge logical negative

    Applies:
        LEG_MOVEMENT_SIGN
        JOINT_DIRECTIONS
    """

    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    leg_name, part_name = joint_to_leg_part(joint_name)

    movement_sign = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)

    adjusted_offset = logical_offset_deg * movement_sign
    raw_unclamped = int(round(center + adjusted_offset * RAW_PER_DEG * joint_direction))
    raw_target = clamp_raw_for_joint(joint_name, raw_unclamped)

    return raw_target, raw_unclamped, adjusted_offset


def return_to_ready(bus: DynamixelBus, speed: int, wait: float):
    bus.move_many(dict(SESSION_READY), speed=speed)
    time.sleep(wait)


# ============================================================
# STOP LOGIC
# ============================================================

def should_stop_probe(
    status: Dict[str, object],
    target_raw: int,
    previous_raw: Optional[int],
    stall_count: int,
) -> Tuple[bool, str, int]:
    raw = status["raw"]
    load_value = status["load_value"]
    temp = status["temp"]
    warnings = status["warnings"]

    if raw is None:
        return True, "NO_REPLY", stall_count

    if raw <= RAW_EDGE_STOP:
        return True, "RAW_EDGE_LOW", stall_count

    if raw >= 1023 - RAW_EDGE_STOP:
        return True, "RAW_EDGE_HIGH", stall_count

    if temp is not None and temp >= TEMP_STOP_C:
        return True, "TEMP_STOP", stall_count

    if load_value is not None and load_value >= LOAD_STOP:
        return True, "LOAD_STOP", stall_count

    follow_error = abs(target_raw - raw)

    if follow_error >= FOLLOW_ERROR_RAW_LIMIT:
        return True, f"FOLLOW_ERROR_RAW_{follow_error}", stall_count

    if previous_raw is not None:
        movement = abs(raw - previous_raw)

        if movement <= MIN_RAW_MOVEMENT_PER_STEP:
            stall_count += 1
        else:
            stall_count = 0

        if stall_count >= STALL_COUNT_LIMIT:
            return True, f"STALL_NO_POSITION_UPDATE_{stall_count}_STEPS", stall_count

    return False, "CONTINUE", stall_count


# ============================================================
# PROBE LOGIC
# ============================================================

def probe_one_direction(
    bus: DynamixelBus,
    joint_name: str,
    direction_label: str,
    logical_sign: int,
    step_deg: float,
    max_angle: float,
    speed: int,
    settle: float,
) -> Dict[str, object]:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    leg_name, part_name = joint_to_leg_part(joint_name)

    print("\n===================================================")
    print(f" PROBING {joint_name} {direction_label}")
    print("===================================================")
    print(f"Motor ID       : {motor_id}")
    print(f"Leg/part       : {leg_name}/{part_name}")
    print(f"Session ready  : {SESSION_READY.get(motor_id)}")
    print(f"Step deg       : {step_deg}")
    print(f"Max angle      : {max_angle}")
    print(f"Speed          : {speed}")

    best_safe_logical = 0.0
    best_safe_raw = SESSION_READY.get(motor_id)
    stop_reason = "MAX_ANGLE_REACHED"

    previous_raw: Optional[int] = None
    stall_count = 0

    steps = int(max_angle / step_deg)

    for step_index in range(1, steps + 1):
        logical_offset = logical_sign * step_index * step_deg
        target_raw, raw_unclamped, adjusted_offset = logical_offset_to_raw(joint_name, logical_offset)

        print(
            f"Step {step_index:>2}/{steps} | "
            f"logical={logical_offset:+.2f}deg | "
            f"adjusted={adjusted_offset:+.2f}deg | "
            f"target={target_raw} | unclamped={raw_unclamped}"
        )

        bus.move_motor(motor_id, target_raw, speed=speed)
        time.sleep(settle)

        status = get_motor_status(bus, motor_id)

        raw = status["raw"]
        load_text = status["load_text"]
        load_value = status["load_value"]
        volt_text = "----" if status["volt"] is None else f"{status['volt'] / 10:.1f}"
        temp = status["temp"]
        warn_text = "OK" if not status["warnings"] else ",".join(status["warnings"])

        print(
            f"    feedback raw={raw} "
            f"sessDeg={status['session_deg']} "
            f"load={load_text} "
            f"volt={volt_text} "
            f"temp={temp} "
            f"warn={warn_text}"
        )

        stop, reason, stall_count = should_stop_probe(
            status=status,
            target_raw=target_raw,
            previous_raw=previous_raw,
            stall_count=stall_count,
        )

        if stop:
            stop_reason = reason
            print(f"    STOP: {stop_reason}")
            break

        best_safe_logical = logical_offset
        best_safe_raw = raw
        previous_raw = raw

    print(f"Best safe {direction_label}: {best_safe_logical:+.2f} deg, raw={best_safe_raw}")
    print("Returning to session ready...")
    return_to_ready(bus, speed=speed, wait=DEFAULT_CENTER_WAIT)

    final_status = get_motor_status(bus, motor_id)

    return {
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "motor_id": motor_id,
        "direction": direction_label,
        "best_safe_logical_deg": best_safe_logical,
        "best_safe_raw": best_safe_raw,
        "stop_reason": stop_reason,
        "session_ready_raw": SESSION_READY.get(motor_id),
        "final_raw_after_return": final_status["raw"],
        "final_delta_after_return": None if final_status["raw"] is None else final_status["raw"] - SESSION_READY.get(motor_id),
        "final_temp": final_status["temp"],
        "final_voltage": None if final_status["volt"] is None else final_status["volt"] / 10,
        "final_load": final_status["load_text"],
        "final_warnings": "OK" if not final_status["warnings"] else ",".join(final_status["warnings"]),
    }


def probe_joint(
    bus: DynamixelBus,
    joint_name: str,
    step_deg: float,
    max_angle: float,
    speed: int,
    settle: float,
) -> List[Dict[str, object]]:
    print("\n\n###################################################")
    print(f"JOINT LIMIT PROBE: {joint_name}")
    print("###################################################")

    return_to_ready(bus, speed=speed, wait=DEFAULT_CENTER_WAIT)

    positive = probe_one_direction(
        bus=bus,
        joint_name=joint_name,
        direction_label="POSITIVE",
        logical_sign=1,
        step_deg=step_deg,
        max_angle=max_angle,
        speed=speed,
        settle=settle,
    )

    time.sleep(DEFAULT_CENTER_WAIT)

    negative = probe_one_direction(
        bus=bus,
        joint_name=joint_name,
        direction_label="NEGATIVE",
        logical_sign=-1,
        step_deg=step_deg,
        max_angle=max_angle,
        speed=speed,
        settle=settle,
    )

    return [positive, negative]


# ============================================================
# TEST SELECTION
# ============================================================

def get_joints_for_args(args) -> List[str]:
    if args.joint:
        joint = args.joint.strip()
        if joint not in JOINT_LIMITS:
            print(f"Unknown joint: {joint}")
            sys.exit(1)
        return [joint]

    if args.leg:
        leg = args.leg.strip().upper()
        if leg not in LEG_JOINTS:
            print(f"Unknown leg: {leg}")
            sys.exit(1)

        return [
            LEG_JOINTS[leg]["hip"],
            LEG_JOINTS[leg]["femur"],
            LEG_JOINTS[leg]["tibia"],
        ]

    if args.all:
        return list(JOINT_TEST_ORDER)

    print("Choose --joint JOINT_NAME, --leg LEG_NAME, or --all")
    print("Examples:")
    print("  python joint_limit_auto_probe.py --joint MR_hip")
    print("  python joint_limit_auto_probe.py --leg MR")
    print("  python joint_limit_auto_probe.py --all")
    sys.exit(1)


# ============================================================
# OUTPUT
# ============================================================

def save_results_csv(results: List[Dict[str, object]], filename: str):
    if not results:
        return

    fieldnames = list(results[0].keys())

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in results:
            writer.writerow(row)

    print(f"CSV saved: {filename}")


def print_paste_summary(results: List[Dict[str, object]]):
    print("\n\n===================================================")
    print(" PASTE THIS SUMMARY TO CHATGPT")
    print("===================================================")

    grouped: Dict[str, Dict[str, Dict[str, object]]] = {}

    for r in results:
        grouped.setdefault(r["joint"], {})
        grouped[r["joint"]][r["direction"]] = r

    print("\nDISCOVERED LIMIT SUMMARY:")
    for joint, dirs in grouped.items():
        pos = dirs.get("POSITIVE")
        neg = dirs.get("NEGATIVE")

        pos_deg = None if pos is None else pos["best_safe_logical_deg"]
        neg_deg = None if neg is None else neg["best_safe_logical_deg"]

        pos_reason = None if pos is None else pos["stop_reason"]
        neg_reason = None if neg is None else neg["stop_reason"]

        print(
            f"{joint:<10} "
            f"NEG={neg_deg} reason={neg_reason} | "
            f"POS={pos_deg} reason={pos_reason}"
        )

    print("\nDETAILED:")
    for r in results:
        print(
            f"{r['joint']:<10} "
            f"{r['direction']:<8} "
            f"best={r['best_safe_logical_deg']:+.2f}deg "
            f"raw={r['best_safe_raw']} "
            f"stop={r['stop_reason']} "
            f"returnDelta={r['final_delta_after_return']} "
            f"temp={r['final_temp']} "
            f"volt={r['final_voltage']} "
            f"load={r['final_load']} "
            f"warn={r['final_warnings']}"
        )

    print("===================================================")


# ============================================================
# ARGS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(description="Automatic safe joint limit probe.")

    parser.add_argument("--port", default=DEFAULT_PORT, help="COM port. Default COM6.")
    parser.add_argument("--joint", default=None, help="Probe one joint, example MR_hip.")
    parser.add_argument("--leg", default=None, help="Probe one leg, example MR.")
    parser.add_argument("--all", action="store_true", help="Probe all 18 joints.")

    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG, help="Step degrees. Default 2.")
    parser.add_argument("--max-angle", type=float, default=DEFAULT_MAX_ANGLE, help="Max probe angle each side. Default 45.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Moving speed. Default 35.")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_SECONDS, help="Settle seconds after each step. Default 0.18.")

    parser.add_argument("--yes", action="store_true", help="Skip initial confirmation.")

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():
    global SESSION_READY

    args = parse_args()

    print("\n===================================================")
    print(" HEXAPOD AUTOMATIC JOINT LIMIT PROBE")
    print("===================================================")
    print(f"Port      : {args.port}")
    print(f"Step      : {args.step} deg")
    print(f"Max angle : {args.max_angle} deg each side")
    print(f"Speed     : {args.speed}")
    print(f"Settle    : {args.settle}s")
    print("===================================================")
    print("Safety stop conditions:")
    print(f"- raw <= {RAW_EDGE_STOP} or raw >= {1023 - RAW_EDGE_STOP}")
    print(f"- temp >= {TEMP_STOP_C}C")
    print(f"- load >= {LOAD_STOP}")
    print(f"- motor stops updating position")
    print(f"- follow error >= {FOLLOW_ERROR_RAW_LIMIT} raw")
    print("===================================================")
    print("Important:")
    print("- Put robot in good physical ready pose before starting.")
    print("- Support/lift robot if needed.")
    print("- This script runs automatically after start.")
    print("- MR and RR femur/tibia have reversed movement sign.")
    print("===================================================")

    if not args.yes:
        confirm = input("Type y to start automatic probe: ").strip().lower()
        if confirm != "y":
            print("Cancelled.")
            return

    bus = DynamixelBus(args.port)

    if not bus.open():
        return

    results: List[Dict[str, object]] = []

    try:
        SESSION_READY = capture_current_pose(bus)

        print_status_table(bus)

        joints = get_joints_for_args(args)

        print("\nSelected joints:")
        for j in joints:
            print(f"  - {j}")

        print("\nReturning to session ready before probe...")
        return_to_ready(bus, speed=args.speed, wait=DEFAULT_CENTER_WAIT)

        for joint in joints:
            joint_results = probe_joint(
                bus=bus,
                joint_name=joint,
                step_deg=args.step,
                max_angle=args.max_angle,
                speed=args.speed,
                settle=args.settle,
            )
            results.extend(joint_results)

        return_to_ready(bus, speed=args.speed, wait=DEFAULT_CENTER_WAIT)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_name = f"joint_limit_probe_{timestamp}.csv"

        save_results_csv(results, csv_name)
        print_paste_summary(results)

        print("\nDone. Paste the summary here.")

    finally:
        try:
            return_to_ready(bus, speed=args.speed, wait=DEFAULT_CENTER_WAIT)
        except Exception:
            pass

        bus.close()


if __name__ == "__main__":
    main()