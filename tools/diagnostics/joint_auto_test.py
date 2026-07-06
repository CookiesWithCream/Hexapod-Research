import sys
import time
import argparse
import csv
from datetime import datetime
from typing import Dict, Optional, List, Tuple

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
# AUTOMATED JOINT DISCOVERY / HARDWARE TEST SCRIPT
# ============================================================
#
# Purpose:
#   Run structured joint tests for the real hexapod.
#
# What it does:
#   1. Connect to COM6
#   2. Capture current physical pose as SESSION_READY
#   3. Print motor health/status
#   4. Test each joint one by one:
#       +angle
#       return ready
#       -angle
#       return ready
#   5. Read feedback after every movement
#   6. Ask you what you physically saw
#   7. Save results to CSV + TXT log
#
# Run:
#   python joint_auto_test.py
#
# Safer smaller test:
#   python joint_auto_test.py --step 5
#
# Test only one leg:
#   python joint_auto_test.py --leg MR
#
# Test only one joint:
#   python joint_auto_test.py --joint MR_femur
#
# Skip visual questions:
#   python joint_auto_test.py --no-observe
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

DEFAULT_SPEED = 50
DEFAULT_STEP_DEG = 8.0
DEFAULT_HOLD_SECONDS = 0.9
RETURN_HOLD_SECONDS = 0.45

TEMP_WARN_C = 55
TEMP_DANGER_C = 65
LOW_VOLTAGE_LIMIT = 100  # 10.0V
RAW_EDGE_WARN = 15


# ============================================================
# CURRENT CANDIDATE MOVEMENT SIGN
# ============================================================
# This is the candidate bridge movement sign.
#
# Based on your discovery:
#   MR and RR femur/tibia are physically reversed.
#
# This script tests with this candidate sign applied.
# If it is wrong, tell me from the observations.
# ============================================================

LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1, "femur": 1, "tibia": 1},
    "ML": {"hip": 1, "femur": 1, "tibia": 1},
    "RL": {"hip": 1, "femur": 1, "tibia": 1},
    "FR": {"hip": 1, "femur": 1, "tibia": 1},

    # Special reversed legs
    "MR": {"hip": 1, "femur": -1, "tibia": -1},
    "RR": {"hip": 1, "femur": -1, "tibia": -1},
}


SESSION_READY: Dict[int, int] = {}


# ============================================================
# JOINT ORDER
# ============================================================

JOINT_TEST_ORDER = [
    "FL_hip", "FL_femur", "FL_tibia",
    "ML_hip", "ML_femur", "ML_tibia",
    "RL_hip", "RL_femur", "RL_tibia",

    "FR_hip", "FR_femur", "FR_tibia",
    "MR_hip", "MR_femur", "MR_tibia",
    "RR_hip", "RR_femur", "RR_tibia",
]


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
        print(" CONNECTING TO HEXAPOD")
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
# STATUS HELPERS
# ============================================================

def decode_load(raw_load: Optional[int]) -> str:
    if raw_load is None:
        return "----"

    if raw_load <= 1023:
        return f"+{raw_load}"

    return f"-{raw_load - 1024}"


def capture_current_pose(bus: DynamixelBus) -> Dict[int, int]:
    captured = {}

    print("\n===================================================")
    print(" CAPTURING SESSION READY")
    print("===================================================")
    print("Manually set the robot pose before running the script.")
    print("This captured pose becomes 0 degrees for testing.")

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


def session_deg_for_joint(joint_name: str, raw: int) -> float:
    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center = SESSION_READY.get(motor_id, READY_POSE[motor_id])
    direction = JOINT_DIRECTIONS[joint_name]

    return ((raw - center) / RAW_PER_DEG) * direction


def get_motor_status(bus: DynamixelBus, motor_id: int) -> Dict[str, object]:
    joint_name = MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")
    leg_name, part_name = joint_to_leg_part(joint_name)

    pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
    speed = bus.read2(motor_id, ADDR_PRESENT_SPEED)
    load = bus.read2(motor_id, ADDR_PRESENT_LOAD)
    volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
    temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

    session_ready = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    if pos is None or joint_name == "UNKNOWN":
        deg = None
        delta_raw = None
    else:
        deg = session_deg_for_joint(joint_name, pos)
        delta_raw = pos - session_ready

    warnings = []

    if pos is None:
        warnings.append("NO_REPLY")
    else:
        if pos <= RAW_EDGE_WARN:
            warnings.append("RAW_NEAR_0")
        if pos >= 1023 - RAW_EDGE_WARN:
            warnings.append("RAW_NEAR_1023")

    if temp is not None:
        if temp >= TEMP_DANGER_C:
            warnings.append("TEMP_DANGER")
        elif temp >= TEMP_WARN_C:
            warnings.append("TEMP_WARN")

    if volt is not None and volt < LOW_VOLTAGE_LIMIT:
        warnings.append("LOW_VOLTAGE")

    return {
        "motor_id": motor_id,
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "raw": pos,
        "session_deg": deg,
        "session_ready": session_ready,
        "delta_raw": delta_raw,
        "speed": speed,
        "load_raw": load,
        "load_text": decode_load(load),
        "volt": volt,
        "temp": temp,
        "warnings": ",".join(warnings) if warnings else "OK",
    }


def print_motor_status_table(bus: DynamixelBus):
    print("\n===================================================")
    print(" MOTOR STATUS TABLE")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'SessDeg':>8} {'SessReady':>9} "
        f"{'Delta':>6} {'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 115)

    connected = 0
    missing = []

    for motor_id in sorted(READY_POSE.keys()):
        s = get_motor_status(bus, motor_id)

        if s["raw"] is None:
            missing.append(motor_id)
            print(
                f"{motor_id:>2} {s['joint']:<10} {s['leg']:<2} {s['part']:<5} "
                f"{'----':>4} {'----':>8} {s['session_ready']:>9} "
                f"{'----':>6} {'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1

        volt_text = "----" if s["volt"] is None else f"{s['volt'] / 10:.1f}"
        temp_text = "----" if s["temp"] is None else str(s["temp"])

        print(
            f"{motor_id:>2} {s['joint']:<10} {s['leg']:<2} {s['part']:<5} "
            f"{s['raw']:>4} {s['session_deg']:>8.2f} {s['session_ready']:>9} "
            f"{s['delta_raw']:>+6} {s['load_text']:>7} {volt_text:>5} {temp_text:>5} {s['warnings']}"
        )

    print("-" * 115)
    print(f"Connected: {connected}/18")

    if missing:
        print(f"Missing IDs: {missing}")


# ============================================================
# RAW / ANGLE CONVERSION
# ============================================================

def clamp_raw_for_joint(joint_name: str, raw: int) -> int:
    info = JOINT_LIMITS[joint_name]

    min_raw = int(info["min_raw"])
    max_raw = int(info["max_raw"])

    raw = int(max(min_raw, min(max_raw, raw)))
    raw = int(max(0, min(1023, raw)))

    return raw


def command_offset_to_raw(joint_name: str, command_offset_deg: float) -> Tuple[int, int, float]:
    """
    Converts logical bridge command offset into raw target.

    Applies:
      - candidate per-leg movement sign
      - joint direction
      - raw clamp

    Returns:
      raw_target, unclamped_raw, adjusted_offset
    """

    leg_name, part_name = joint_to_leg_part(joint_name)

    motor_id = int(JOINT_LIMITS[joint_name]["id"])
    center_raw = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    movement_sign = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS[joint_name]

    adjusted_offset = command_offset_deg * movement_sign
    unclamped_raw = int(round(center_raw + adjusted_offset * RAW_PER_DEG * joint_direction))
    raw_target = clamp_raw_for_joint(joint_name, unclamped_raw)

    return raw_target, unclamped_raw, adjusted_offset


def return_to_session_ready(bus: DynamixelBus, speed: int):
    bus.move_many(dict(SESSION_READY), speed=speed)
    time.sleep(RETURN_HOLD_SECONDS)


# ============================================================
# TEST SELECTION
# ============================================================

def get_joints_for_test(args) -> List[str]:
    if args.joint:
        joint_name = args.joint.strip()

        if joint_name not in JOINT_LIMITS:
            print(f"Unknown joint: {joint_name}")
            print("Available examples: FL_hip, MR_femur, RR_tibia")
            sys.exit(1)

        return [joint_name]

    if args.leg:
        leg_name = args.leg.strip().upper()

        if leg_name not in LEG_JOINTS:
            print(f"Unknown leg: {leg_name}")
            print("Available legs: FL, ML, RL, FR, MR, RR")
            sys.exit(1)

        return [
            LEG_JOINTS[leg_name]["hip"],
            LEG_JOINTS[leg_name]["femur"],
            LEG_JOINTS[leg_name]["tibia"],
        ]

    return list(JOINT_TEST_ORDER)


# ============================================================
# TEST RUNNER
# ============================================================

def ask_visual_observation(joint_name: str, command_label: str) -> str:
    print("\nWhat did you physically see?")
    print("Examples:")
    print("  ok")
    print("  wrong direction")
    print("  no movement")
    print("  stuck")
    print("  too much")
    print("  motor missing")
    print("  wire issue")
    print("  hit limit")
    print("  leg went up")
    print("  leg went down")
    print("  unsure")

    obs = input(f"Observation for {joint_name} {command_label}: ").strip()

    if obs == "":
        obs = "blank"

    return obs


def run_single_joint_command(
    bus: DynamixelBus,
    joint_name: str,
    command_deg: float,
    speed: int,
    hold_seconds: float,
    ask_observe: bool,
) -> Dict[str, object]:
    leg_name, part_name = joint_to_leg_part(joint_name)
    motor_id = int(JOINT_LIMITS[joint_name]["id"])

    before = get_motor_status(bus, motor_id)

    raw_target, unclamped_raw, adjusted_offset = command_offset_to_raw(joint_name, command_deg)

    command_label = f"{command_deg:+.2f}deg"

    print("\n===================================================")
    print(f" TEST {joint_name} | Motor ID {motor_id} | Command {command_label}")
    print("===================================================")
    print(f"Leg/part          : {leg_name} / {part_name}")
    print(f"Session ready raw : {SESSION_READY.get(motor_id)}")
    print(f"Candidate sign    : {LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)}")
    print(f"Joint direction   : {JOINT_DIRECTIONS[joint_name]}")
    print(f"Adjusted offset   : {adjusted_offset:+.2f} deg")
    print(f"Unclamped raw     : {unclamped_raw}")
    print(f"Target raw        : {raw_target}")
    print(f"Before raw        : {before['raw']}")
    print(f"Before temp       : {before['temp']}")
    print(f"Before volt       : {None if before['volt'] is None else before['volt'] / 10}")
    print("Moving...")

    bus.move_motor(motor_id, raw_target, speed=speed)
    time.sleep(hold_seconds)

    after = get_motor_status(bus, motor_id)

    actual_delta_raw = None
    actual_delta_deg = None

    if before["raw"] is not None and after["raw"] is not None:
        actual_delta_raw = int(after["raw"]) - int(before["raw"])

    if after["session_deg"] is not None:
        actual_delta_deg = float(after["session_deg"])

    print("\nResult feedback:")
    print(f"After raw         : {after['raw']}")
    print(f"After session deg : {actual_delta_deg}")
    print(f"Actual delta raw  : {actual_delta_raw}")
    print(f"Load              : {after['load_text']}")
    print(f"Voltage           : {None if after['volt'] is None else after['volt'] / 10}")
    print(f"Temperature       : {after['temp']}")
    print(f"Warnings          : {after['warnings']}")

    visual_observation = "not_asked"

    if ask_observe:
        visual_observation = ask_visual_observation(joint_name, command_label)

    print("Returning to session ready...")
    return_to_session_ready(bus, speed=speed)

    ready_after = get_motor_status(bus, motor_id)

    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "joint": joint_name,
        "leg": leg_name,
        "part": part_name,
        "motor_id": motor_id,
        "command_deg": command_deg,
        "candidate_leg_sign": LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1),
        "joint_direction": JOINT_DIRECTIONS[joint_name],
        "adjusted_offset_deg": adjusted_offset,
        "session_ready_raw": SESSION_READY.get(motor_id),
        "target_raw": raw_target,
        "unclamped_raw": unclamped_raw,
        "before_raw": before["raw"],
        "after_raw": after["raw"],
        "actual_delta_raw": actual_delta_raw,
        "after_session_deg": actual_delta_deg,
        "after_load": after["load_text"],
        "after_voltage": None if after["volt"] is None else after["volt"] / 10,
        "after_temp": after["temp"],
        "after_warnings": after["warnings"],
        "ready_after_raw": ready_after["raw"],
        "ready_after_delta_raw": None if ready_after["raw"] is None else int(ready_after["raw"]) - int(SESSION_READY.get(motor_id)),
        "visual_observation": visual_observation,
    }


def run_joint_tests(bus: DynamixelBus, joints: List[str], args) -> List[Dict[str, object]]:
    results = []

    total = len(joints)

    for i, joint_name in enumerate(joints, start=1):
        print("\n\n###################################################")
        print(f"JOINT {i}/{total}: {joint_name}")
        print("###################################################")

        leg_name, part_name = joint_to_leg_part(joint_name)
        motor_id = int(JOINT_LIMITS[joint_name]["id"])

        print(f"Leg/part : {leg_name}/{part_name}")
        print(f"Motor ID : {motor_id}")

        if not args.no_confirm_each:
            confirm = input("Press ENTER to test this joint, type skip to skip, x to stop: ").strip().lower()

            if confirm == "skip":
                print(f"Skipping {joint_name}")
                continue

            if confirm == "x":
                print("Stopping tests.")
                break

        # Positive test
        result_pos = run_single_joint_command(
            bus=bus,
            joint_name=joint_name,
            command_deg=abs(args.step),
            speed=args.speed,
            hold_seconds=args.hold,
            ask_observe=not args.no_observe,
        )
        results.append(result_pos)

        # Negative test
        result_neg = run_single_joint_command(
            bus=bus,
            joint_name=joint_name,
            command_deg=-abs(args.step),
            speed=args.speed,
            hold_seconds=args.hold,
            ask_observe=not args.no_observe,
        )
        results.append(result_neg)

    return results


# ============================================================
# OUTPUT
# ============================================================

def write_csv(results: List[Dict[str, object]], filename: str):
    if not results:
        print("No results to write.")
        return

    fieldnames = list(results[0].keys())

    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        for row in results:
            writer.writerow(row)

    print(f"CSV saved: {filename}")


def write_txt_summary(results: List[Dict[str, object]], filename: str):
    with open(filename, "w", encoding="utf-8") as f:
        f.write("HEXAPOD JOINT AUTO TEST SUMMARY\n")
        f.write("=" * 80 + "\n\n")

        f.write("Candidate LEG_MOVEMENT_SIGN:\n")
        for leg, parts in LEG_MOVEMENT_SIGN.items():
            f.write(f"{leg}: {parts}\n")

        f.write("\nResults:\n")
        f.write("=" * 80 + "\n")

        for r in results:
            f.write(
                f"{r['joint']:<10} "
                f"cmd={r['command_deg']:+.2f}deg "
                f"target_raw={r['target_raw']} "
                f"after_raw={r['after_raw']} "
                f"delta_raw={r['actual_delta_raw']} "
                f"temp={r['after_temp']} "
                f"volt={r['after_voltage']} "
                f"warnings={r['after_warnings']} "
                f"obs={r['visual_observation']}\n"
            )

    print(f"TXT summary saved: {filename}")


def print_paste_summary(results: List[Dict[str, object]]):
    print("\n\n===================================================")
    print(" PASTE THIS SUMMARY TO CHATGPT")
    print("===================================================")

    for r in results:
        print(
            f"{r['joint']:<10} "
            f"cmd={r['command_deg']:+.1f}deg "
            f"sign={r['candidate_leg_sign']:+} "
            f"target={r['target_raw']} "
            f"after={r['after_raw']} "
            f"dRaw={r['actual_delta_raw']} "
            f"readyBackDelta={r['ready_after_delta_raw']} "
            f"temp={r['after_temp']} "
            f"volt={r['after_voltage']} "
            f"warn={r['after_warnings']} "
            f"obs={r['visual_observation']}"
        )

    print("===================================================")


# ============================================================
# ARGS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Automated joint discovery and hardware feedback tester."
    )

    parser.add_argument("--port", default=DEFAULT_PORT, help="COM port. Default COM6.")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_DEG, help="Test angle in degrees. Default 8.")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED, help="Dynamixel speed. Default 50.")
    parser.add_argument("--hold", type=float, default=DEFAULT_HOLD_SECONDS, help="Hold time after movement. Default 0.9s.")

    parser.add_argument("--leg", default=None, help="Test only one leg: FL, ML, RL, FR, MR, RR.")
    parser.add_argument("--joint", default=None, help="Test only one joint: example MR_femur.")

    parser.add_argument("--no-observe", action="store_true", help="Do not ask physical observation questions.")
    parser.add_argument("--no-confirm-each", action="store_true", help="Do not pause before each joint.")

    return parser.parse_args()


# ============================================================
# MAIN
# ============================================================

def main():
    global SESSION_READY

    args = parse_args()

    print("\n===================================================")
    print(" HEXAPOD AUTOMATED JOINT TEST")
    print("===================================================")
    print(f"Port      : {args.port}")
    print(f"Step deg  : {args.step}")
    print(f"Speed     : {args.speed}")
    print(f"Hold      : {args.hold}s")
    print(f"Observe   : {not args.no_observe}")
    print("===================================================")
    print("Before running:")
    print("1. Put robot in the physical ready pose you want.")
    print("2. Support/lift robot if needed.")
    print("3. Watch the tested joint carefully.")
    print("===================================================")

    start_confirm = input("Type y to connect and capture current pose: ").strip().lower()

    if start_confirm != "y":
        print("Cancelled.")
        return

    bus = DynamixelBus(args.port)

    if not bus.open():
        return

    results: List[Dict[str, object]] = []

    try:
        SESSION_READY = capture_current_pose(bus)

        print_motor_status_table(bus)

        joints = get_joints_for_test(args)

        print("\nJoints selected for test:")
        for j in joints:
            print(f"  - {j}")

        confirm = input("\nType y to start joint testing: ").strip().lower()

        if confirm != "y":
            print("Cancelled before testing.")
            return

        return_to_session_ready(bus, speed=args.speed)

        results = run_joint_tests(bus, joints, args)

        return_to_session_ready(bus, speed=args.speed)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_name = f"joint_auto_test_results_{timestamp}.csv"
        txt_name = f"joint_auto_test_summary_{timestamp}.txt"

        write_csv(results, csv_name)
        write_txt_summary(results, txt_name)
        print_paste_summary(results)

        print("\nDone.")
        print("Paste the PASTE THIS SUMMARY section here.")
        print("Also tell me what you physically saw if the observation notes are short.")

    finally:
        try:
            return_to_session_ready(bus, speed=args.speed)
        except Exception:
            pass

        bus.close()


if __name__ == "__main__":
    main()