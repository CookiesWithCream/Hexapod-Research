# tools/calibration/origin7_fixed_pose_tuner.py

import sys
import time
from typing import Dict, Optional, Tuple

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("Missing library: dynamixel_sdk")
    print("Install using:")
    print("pip install dynamixel-sdk")
    sys.exit(1)


# ============================================================
# ORIGIN7 FIXED POSE TUNER
# ============================================================
#
# Purpose:
#   This script uses origin7 as a FIXED ABSOLUTE RAW POSE.
#   It does NOT calculate origin7 from offsets anymore.
#
# Workflow:
#   1. Run this script
#   2. Type: origin7
#   3. Type: zero
#   4. Tune with nudge / set / raw / id / leg
#   5. Type: save
#   6. Save READY_POSE output for calibration notes
#
# Commands:
#   h
#   p
#   origin7
#   zero
#   r
#   nudge JOINT DEG
#   set JOINT DEG
#   raw ID RAW
#   id ID DEG
#   leg LEG H F T
#   save
#   x
#
# Examples:
#   origin7
#   zero
#   nudge FL_femur -3
#   nudge MR_tibia 5
#   raw 11 530
#   id 11 -5
#   leg FL 0 -3 5
#   save
#
# ============================================================


# ============================================================
# DYNAMIXEL CONFIG
# ============================================================

DEFAULT_PORT = "COM6"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TORQUE_ENABLE = 1
COMM_SUCCESS = 0

READ_RETRIES = 3
READ_RETRY_DELAY = 0.04

RAW_PER_DEG = 1023.0 / 300.0

TEMP_WARN_C = 50
TEMP_STOP_C = 58

LOAD_WARN = 450
LOAD_STOP = 700

DEFAULT_SPEED = 20
NUDGE_SPEED = 18
ORIGIN_SPEED = 14


# ============================================================
# FIXED ORIGIN7 POSE
# ============================================================
#
# This is based on the actual MOTOR STATUS raw values you pasted.
# This pose is now absolute, not offset-based.
#
# ============================================================

ORIGIN7_FIXED_POSE = {
    1: 511,   # RL_hip
    2: 799,   # FL_hip
    3: 694,   # FR_femur
    4: 609,   # FL_femur
    5: 374,   # FR_tibia
    6: 413,   # FL_tibia
    7: 560,   # MR_hip
    8: 805,   # ML_hip
    9: 398,   # MR_femur
    10: 665,  # ML_femur
    11: 569,  # MR_tibia
    12: 429,  # ML_tibia
    13: 528,  # RR_hip
    14: 525,  # FR_hip
    15: 396,  # RR_femur
    16: 652,  # RL_femur
    17: 551,  # RR_tibia
    18: 430,  # RL_tibia
}


# ============================================================
# ROBOT MODEL
# ============================================================

LEG_JOINTS = {
    "FL": {"hip": "FL_hip", "femur": "FL_femur", "tibia": "FL_tibia"},
    "ML": {"hip": "ML_hip", "femur": "ML_femur", "tibia": "ML_tibia"},
    "RL": {"hip": "RL_hip", "femur": "RL_femur", "tibia": "RL_tibia"},
    "FR": {"hip": "FR_hip", "femur": "FR_femur", "tibia": "FR_tibia"},
    "MR": {"hip": "MR_hip", "femur": "MR_femur", "tibia": "MR_tibia"},
    "RR": {"hip": "RR_hip", "femur": "RR_femur", "tibia": "RR_tibia"},
}

JOINT_INFO = {
    "FL_hip": {"id": 2, "type": "hip"},
    "ML_hip": {"id": 8, "type": "hip"},
    "RL_hip": {"id": 1, "type": "hip"},
    "FR_hip": {"id": 14, "type": "hip"},
    "MR_hip": {"id": 7, "type": "hip"},
    "RR_hip": {"id": 13, "type": "hip"},

    "FL_femur": {"id": 4, "type": "femur"},
    "ML_femur": {"id": 10, "type": "femur"},
    "RL_femur": {"id": 16, "type": "femur"},
    "FR_femur": {"id": 3, "type": "femur"},
    "MR_femur": {"id": 9, "type": "femur"},
    "RR_femur": {"id": 15, "type": "femur"},

    "FL_tibia": {"id": 6, "type": "tibia"},
    "ML_tibia": {"id": 12, "type": "tibia"},
    "RL_tibia": {"id": 18, "type": "tibia"},
    "FR_tibia": {"id": 5, "type": "tibia"},
    "MR_tibia": {"id": 11, "type": "tibia"},
    "RR_tibia": {"id": 17, "type": "tibia"},
}

MOTOR_TO_JOINT = {info["id"]: name for name, info in JOINT_INFO.items()}
ALL_MOTOR_IDS = sorted(ORIGIN7_FIXED_POSE.keys())


# ============================================================
# MOVEMENT SIGN MODEL
# ============================================================
#
# This keeps degree nudges intuitive based on your existing scripts.
# MR and RR femur/tibia are reversed.
#
# ============================================================

LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1, "femur": 1, "tibia": 1},
    "ML": {"hip": 1, "femur": 1, "tibia": 1},
    "RL": {"hip": 1, "femur": 1, "tibia": 1},
    "FR": {"hip": 1, "femur": 1, "tibia": 1},
    "MR": {"hip": 1, "femur": -1, "tibia": -1},
    "RR": {"hip": 1, "femur": -1, "tibia": -1},
}

JOINT_DIRECTIONS = {joint: 1 for joint in JOINT_INFO.keys()}


# Runtime pose memories
TUNE_ZERO: Dict[int, int] = dict(ORIGIN7_FIXED_POSE)
ACTIVE_GOALS: Dict[int, int] = dict(ORIGIN7_FIXED_POSE)


# ============================================================
# MODEL HELPERS
# ============================================================

def joint_to_motor_id(joint_name: str) -> int:
    return int(JOINT_INFO[joint_name]["id"])


def motor_id_to_joint(motor_id: int) -> str:
    return MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")


def joint_to_leg_part(joint_name: str) -> Tuple[str, str]:
    for leg_name, parts in LEG_JOINTS.items():
        for part_name, candidate in parts.items():
            if candidate == joint_name:
                return leg_name, part_name
    return "?", "?"


def leg_part_to_joint(leg_name: str, part_name: str) -> str:
    return LEG_JOINTS[leg_name][part_name]


def logical_deg_to_raw_delta(joint_name: str, deg: float) -> int:
    leg, part = joint_to_leg_part(joint_name)
    movement_sign = LEG_MOVEMENT_SIGN.get(leg, {}).get(part, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)
    return int(round(deg * RAW_PER_DEG * movement_sign * joint_direction))


def raw_delta_to_logical_deg(joint_name: str, raw_delta: int) -> float:
    leg, part = joint_to_leg_part(joint_name)
    movement_sign = LEG_MOVEMENT_SIGN.get(leg, {}).get(part, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)
    sign = movement_sign * joint_direction

    if sign == 0:
        sign = 1

    return raw_delta / RAW_PER_DEG / sign


def clamp_raw(raw: int) -> int:
    return int(max(0, min(1023, raw)))


# ============================================================
# DYNAMIXEL BUS
# ============================================================

class DynamixelBus:
    def __init__(self, port_name: str = DEFAULT_PORT):
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
            return False

        if not self.port_handler.setBaudRate(BAUDRATE):
            print(f"FAILED: Cannot set baudrate {BAUDRATE}")
            return False

        print("Connected.")
        return True

    def close(self):
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        print("Port closed.")

    def write1(self, motor_id: int, address: int, value: int) -> bool:
        try:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, address, int(value)
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE1 EXCEPTION: {type(e).__name__}: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        value = clamp_raw(value)

        try:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, address, value
            )
        except Exception as e:
            print(f"[ID {motor_id}] WRITE2 EXCEPTION: {type(e).__name__}: {e}")
            return False

        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False

        if error != 0:
            print(f"[ID {motor_id}] PACKET ERROR: {self.packet_handler.getRxPacketError(error)}")
            return False

        return True

    def read1_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, address
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, address
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def read1(self, motor_id: int, address: int) -> Optional[int]:
        for _ in range(READ_RETRIES):
            value = self.read1_once(motor_id, address)
            if value is not None:
                return value
            time.sleep(READ_RETRY_DELAY)
        return None

    def read2(self, motor_id: int, address: int) -> Optional[int]:
        for _ in range(READ_RETRIES):
            value = self.read2_once(motor_id, address)
            if value is not None:
                return value
            time.sleep(READ_RETRY_DELAY)
        return None

    def enable_torque(self, motor_id: int):
        self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_speed(self, motor_id: int, speed: int):
        self.write2(motor_id, ADDR_MOVING_SPEED, speed)

    def move_motor(self, motor_id: int, raw: int, speed: int = NUDGE_SPEED):
        raw = clamp_raw(raw)
        self.enable_torque(motor_id)
        self.set_speed(motor_id, speed)
        time.sleep(0.01)
        self.write2(motor_id, ADDR_GOAL_POSITION, raw)

    def move_many(self, targets: Dict[int, int], speed: int = DEFAULT_SPEED):
        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
            time.sleep(0.006)

        for motor_id, raw in targets.items():
            raw = clamp_raw(raw)
            ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            if not ok:
                time.sleep(0.02)
                self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            time.sleep(0.006)


# ============================================================
# STATUS / CAPTURE
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


def capture_pose(bus: DynamixelBus) -> Dict[int, int]:
    pose = {}

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        if pos is None:
            pose[motor_id] = ACTIVE_GOALS.get(motor_id, ORIGIN7_FIXED_POSE[motor_id])
        else:
            pose[motor_id] = pos

    return pose


def max_temperature(bus: DynamixelBus) -> int:
    max_temp = 0
    for motor_id in ALL_MOTOR_IDS:
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
        if temp is not None:
            max_temp = max(max_temp, int(temp))
    return max_temp


def pre_motion_check(bus: DynamixelBus) -> bool:
    temp = max_temperature(bus)

    if temp >= TEMP_STOP_C:
        print(f"\n[SAFETY STOP] Max temperature is {temp}C. Movement blocked.")
        return False

    if temp >= TEMP_WARN_C:
        print(f"\n[WARNING] Max temperature is {temp}C. Movement allowed, but take breaks.")

    return True


def print_status(bus: DynamixelBus):
    print("\n===================================================")
    print(" MOTOR STATUS / TUNING STATUS")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'DegZero':>8} {'Zero':>5} {'Goal':>5} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 120)

    connected = 0
    max_temp = 0

    for motor_id in ALL_MOTOR_IDS:
        joint = motor_id_to_joint(motor_id)
        leg, part = joint_to_leg_part(joint)

        raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
        volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

        if temp is not None:
            max_temp = max(max_temp, int(temp))

        zero = TUNE_ZERO.get(motor_id, ORIGIN7_FIXED_POSE[motor_id])
        goal = ACTIVE_GOALS.get(motor_id, zero)

        warnings = []

        if raw is None:
            warnings.append("NO_REPLY")
            print(
                f"{motor_id:>2} {joint:<10} {leg:<2} {part:<5} "
                f"{'----':>4} {'----':>8} {zero:>5} {goal:>5} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1

        deg_zero = raw_delta_to_logical_deg(joint, raw - zero)

        load_value = decode_load_value(load_raw)
        if load_value is not None:
            if abs(load_value) >= LOAD_STOP:
                warnings.append("LOAD_STOP")
            elif abs(load_value) >= LOAD_WARN:
                warnings.append("LOAD_WARN")

        if temp is not None:
            if temp >= TEMP_STOP_C:
                warnings.append("TEMP_STOP")
            elif temp >= TEMP_WARN_C:
                warnings.append("TEMP_WARN")

        if volt is not None and volt <= 105:
            warnings.append("LOW_VOLTAGE")

        volt_text = "----" if volt is None else f"{volt / 10:.1f}"
        temp_text = "----" if temp is None else str(temp)
        warn_text = "OK" if not warnings else ",".join(warnings)

        print(
            f"{motor_id:>2} {joint:<10} {leg:<2} {part:<5} "
            f"{raw:>4} {deg_zero:>8.2f} {zero:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")

    if max_temp >= TEMP_WARN_C:
        print(f"WARNING: Max motor temperature is {max_temp}C. Let robot cool.")


def print_ready_pose(bus: DynamixelBus):
    pose = capture_pose(bus)

    print("\n===================================================")
    print(" COPY THIS READY_POSE INTO THE GAIT SCRIPT")
    print("===================================================")
    print("READY_POSE = {")

    for motor_id in sorted(pose.keys()):
        joint = motor_id_to_joint(motor_id)
        print(f"    {motor_id}: {pose[motor_id]},   # {joint}")

    print("}")
    print("===================================================")

    print("\nDEGREES FROM TUNE_ZERO:")
    for motor_id in sorted(pose.keys()):
        joint = motor_id_to_joint(motor_id)
        zero = TUNE_ZERO.get(motor_id, ORIGIN7_FIXED_POSE[motor_id])
        deg = raw_delta_to_logical_deg(joint, pose[motor_id] - zero)
        print(f"{motor_id:>2} {joint:<10} {deg:+8.2f} deg")


# ============================================================
# TUNING ACTIONS
# ============================================================

def action_origin7(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print("\n===================================================")
    print(" MOVING TO FIXED ORIGIN7")
    print("===================================================")
    print("This is absolute raw position origin7, not offset-based.")

    ACTIVE_GOALS = dict(ORIGIN7_FIXED_POSE)
    bus.move_many(ACTIVE_GOALS, speed=ORIGIN_SPEED)
    time.sleep(1.5)
    print_status(bus)


def action_zero(bus: DynamixelBus):
    global TUNE_ZERO, ACTIVE_GOALS

    TUNE_ZERO = capture_pose(bus)
    ACTIVE_GOALS = dict(TUNE_ZERO)

    print("\n===================================================")
    print(" NEW TUNING ZERO CAPTURED")
    print("===================================================")
    print("Current pose is now 0 degrees for set/nudge tuning.")
    print_status(bus)


def action_return_to_tune_zero(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print("\nACTION: RETURN TO TUNE_ZERO")
    ACTIVE_GOALS = dict(TUNE_ZERO)
    bus.move_many(ACTIVE_GOALS, speed=DEFAULT_SPEED)
    time.sleep(1.0)
    print_status(bus)


def nudge_joint(bus: DynamixelBus, joint_name: str, deg: float):
    global ACTIVE_GOALS

    if joint_name not in JOINT_INFO:
        print(f"Unknown joint: {joint_name}")
        return

    if not pre_motion_check(bus):
        return

    motor_id = joint_to_motor_id(joint_name)
    current_goal = ACTIVE_GOALS.get(motor_id)

    if current_goal is None:
        current_goal = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        if current_goal is None:
            current_goal = ORIGIN7_FIXED_POSE[motor_id]

    new_goal = clamp_raw(current_goal + logical_deg_to_raw_delta(joint_name, deg))
    ACTIVE_GOALS[motor_id] = new_goal

    print(f"\nNUDGE {joint_name} ID {motor_id}: {deg:+.2f} deg -> raw {new_goal}")
    bus.move_motor(motor_id, new_goal, speed=NUDGE_SPEED)
    time.sleep(0.4)
    print_status(bus)


def set_joint_from_zero(bus: DynamixelBus, joint_name: str, deg: float):
    global ACTIVE_GOALS

    if joint_name not in JOINT_INFO:
        print(f"Unknown joint: {joint_name}")
        return

    if not pre_motion_check(bus):
        return

    motor_id = joint_to_motor_id(joint_name)
    zero = TUNE_ZERO.get(motor_id, ORIGIN7_FIXED_POSE[motor_id])
    new_goal = clamp_raw(zero + logical_deg_to_raw_delta(joint_name, deg))
    ACTIVE_GOALS[motor_id] = new_goal

    print(f"\nSET {joint_name} ID {motor_id}: {deg:+.2f} deg from TUNE_ZERO -> raw {new_goal}")
    bus.move_motor(motor_id, new_goal, speed=NUDGE_SPEED)
    time.sleep(0.4)
    print_status(bus)


def set_raw_motor(bus: DynamixelBus, motor_id: int, raw: int):
    global ACTIVE_GOALS

    if motor_id not in ORIGIN7_FIXED_POSE:
        print(f"Unknown motor ID: {motor_id}")
        return

    if not pre_motion_check(bus):
        return

    raw = clamp_raw(raw)
    ACTIVE_GOALS[motor_id] = raw

    joint = motor_id_to_joint(motor_id)
    print(f"\nRAW SET ID {motor_id} {joint}: raw {raw}")
    bus.move_motor(motor_id, raw, speed=NUDGE_SPEED)
    time.sleep(0.4)
    print_status(bus)


def nudge_id(bus: DynamixelBus, motor_id: int, deg: float):
    joint = motor_id_to_joint(motor_id)

    if joint == "UNKNOWN":
        print(f"Unknown motor ID: {motor_id}")
        return

    nudge_joint(bus, joint, deg)


def set_leg_from_zero(bus: DynamixelBus, leg: str, hip_deg: float, femur_deg: float, tibia_deg: float):
    if leg not in LEG_JOINTS:
        print(f"Unknown leg: {leg}")
        return

    set_joint_from_zero(bus, leg_part_to_joint(leg, "hip"), hip_deg)
    set_joint_from_zero(bus, leg_part_to_joint(leg, "femur"), femur_deg)
    set_joint_from_zero(bus, leg_part_to_joint(leg, "tibia"), tibia_deg)


# ============================================================
# HELP / MAIN
# ============================================================

def print_help():
    print("\n===================================================")
    print(" FIXED ORIGIN7 POSE TUNER COMMANDS")
    print("===================================================")
    print("p")
    print("  Print current motor status, raw, degrees from zero, load, voltage, temp.")
    print("")
    print("origin7")
    print("  Move robot to fixed absolute origin7 pose.")
    print("")
    print("zero")
    print("  Capture current pose as tuning zero.")
    print("")
    print("r")
    print("  Return to tuning zero.")
    print("")
    print("nudge JOINT DEG")
    print("  Move one joint by degree amount from current target.")
    print("  Example: nudge FL_femur -3")
    print("")
    print("set JOINT DEG")
    print("  Set one joint to degree offset from tuning zero.")
    print("  Example: set FL_femur -5")
    print("")
    print("raw ID RAW")
    print("  Send exact raw value to motor.")
    print("  Example: raw 11 530")
    print("")
    print("id ID DEG")
    print("  Nudge motor ID by degrees.")
    print("  Example: id 11 -5")
    print("")
    print("leg LEG H F T")
    print("  Set leg joints from tuning zero.")
    print("  Example: leg FL 0 -3 5")
    print("")
    print("save")
    print("  Print READY_POSE dictionary and degrees from tuning zero.")
    print("")
    print("x")
    print("  Exit.")
    print("===================================================")


def main():
    global TUNE_ZERO, ACTIVE_GOALS

    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        print_status(bus)
        print_help()

        print("\nRecommended workflow:")
        print("1. origin7")
        print("2. zero")
        print("3. nudge / set / raw / id until pose is good")
        print("4. save")
        print("5. Save READY_POSE output for calibration notes")

        while True:
            cmd = input("\nTuner command [h help]: ").strip()

            if not cmd:
                continue

            parts = cmd.split()
            cmd_lower = parts[0].lower()

            try:
                if cmd_lower == "x":
                    print("Exiting.")
                    break

                elif cmd_lower == "h":
                    print_help()

                elif cmd_lower == "p":
                    print_status(bus)

                elif cmd_lower == "origin7":
                    action_origin7(bus)

                elif cmd_lower == "zero":
                    action_zero(bus)

                elif cmd_lower == "r":
                    action_return_to_tune_zero(bus)

                elif cmd_lower == "save":
                    print_ready_pose(bus)

                elif cmd_lower == "nudge":
                    if len(parts) != 3:
                        print("Usage: nudge JOINT DEG")
                        continue
                    nudge_joint(bus, parts[1], float(parts[2]))

                elif cmd_lower == "set":
                    if len(parts) != 3:
                        print("Usage: set JOINT DEG")
                        continue
                    set_joint_from_zero(bus, parts[1], float(parts[2]))

                elif cmd_lower == "raw":
                    if len(parts) != 3:
                        print("Usage: raw ID RAW")
                        continue
                    set_raw_motor(bus, int(parts[1]), int(parts[2]))

                elif cmd_lower == "id":
                    if len(parts) != 3:
                        print("Usage: id ID DEG")
                        continue
                    nudge_id(bus, int(parts[1]), float(parts[2]))

                elif cmd_lower == "leg":
                    if len(parts) != 5:
                        print("Usage: leg LEG H F T")
                        continue
                    set_leg_from_zero(
                        bus,
                        parts[1].upper(),
                        float(parts[2]),
                        float(parts[3]),
                        float(parts[4]),
                    )

                else:
                    print(f"Unknown command: {cmd}")
                    print("Type h for help.")

            except ValueError:
                print("Invalid number format.")
            except KeyboardInterrupt:
                print("\nKeyboard interrupt.")
                break

    finally:
        bus.close()


if __name__ == "__main__":
    main()