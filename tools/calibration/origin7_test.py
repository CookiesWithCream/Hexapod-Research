# tools/calibration/fixed_pose_tuner_from_status.py

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
# FIXED POSE TUNER FROM MOTOR STATUS
# ============================================================
#
# This script starts by moving the hexapod to the exact raw motor
# positions from the MOTOR STATUS table you pasted.
#
# Then you can tune individual joints/motors while torque is on.
#
# Workflow:
#   1. Run script
#   2. fixed
#   3. zero
#   4. tune using nudge / set / id / raw / leg
#   5. save
#   6. save READY_POSE output for calibration notes
#
# Commands:
#   h                   = help
#   p                   = print status
#   fixed               = move to the pasted fixed pose
#   zero                = capture current pose as tuning zero
#   r                   = return to tuning zero
#   save                = print current pose as READY_POSE dictionary
#   nudge JOINT DEG     = nudge one named joint from current target
#   set JOINT DEG       = set one named joint offset from tuning zero
#   id ID DEG           = nudge motor ID by degree amount
#   raw ID RAW          = set motor ID exact raw value
#   leg LEG H F T       = set one leg offsets from tuning zero
#   x                   = exit
#
# Examples:
#   fixed
#   zero
#   nudge FL_femur -3
#   nudge MR_tibia 5
#   set RR_femur -2
#   id 11 -5
#   raw 11 530
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

POSE_SPEED = 14
NUDGE_SPEED = 18
RETURN_SPEED = 24


# ============================================================
# FIXED STARTING POSE FROM YOUR MOTOR STATUS TABLE
# ============================================================
# Raw values copied from the MOTOR STATUS you pasted.

FIXED_START_POSE = {
    1: 511,   # RL_hip
    2: 799,   # FL_hip
    3: 530,   # FR_femur
    4: 493,   # FL_femur
    5: 652,   # FR_tibia
    6: 682,   # FL_tibia
    7: 559,   # MR_hip
    8: 805,   # ML_hip
    9: 513,   # MR_femur
    10: 537,  # ML_femur
    11: 453,  # MR_tibia
    12: 702,  # ML_tibia
    13: 527,  # RR_hip
    14: 525,  # FR_hip
    15: 562,  # RR_femur
    16: 526,  # RL_femur
    17: 266,  # RR_tibia
    18: 710,  # RL_tibia
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

MOTOR_TO_JOINT = {info["id"]: joint for joint, info in JOINT_INFO.items()}
ALL_MOTOR_IDS = sorted(FIXED_START_POSE.keys())

# Same sign model used in your previous scripts.
# MR and RR femur/tibia are reversed.
LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1, "femur": 1, "tibia": 1},
    "ML": {"hip": 1, "femur": 1, "tibia": 1},
    "RL": {"hip": 1, "femur": 1, "tibia": 1},
    "FR": {"hip": 1, "femur": 1, "tibia": 1},
    "MR": {"hip": 1, "femur": -1, "tibia": -1},
    "RR": {"hip": 1, "femur": -1, "tibia": -1},
}

JOINT_DIRECTIONS = {joint: 1 for joint in JOINT_INFO.keys()}


# Runtime memories
TUNE_ZERO: Dict[int, int] = dict(FIXED_START_POSE)
ACTIVE_GOALS: Dict[int, int] = dict(FIXED_START_POSE)


# ============================================================
# HELPERS
# ============================================================

def motor_id_to_joint(motor_id: int) -> str:
    return MOTOR_TO_JOINT.get(motor_id, "UNKNOWN")


def joint_to_motor_id(joint_name: str) -> int:
    return int(JOINT_INFO[joint_name]["id"])


def joint_to_leg_part(joint_name: str) -> Tuple[str, str]:
    for leg_name, parts in LEG_JOINTS.items():
        for part_name, candidate in parts.items():
            if candidate == joint_name:
                return leg_name, part_name
    return "?", "?"


def leg_part_to_joint(leg_name: str, part_name: str) -> str:
    return LEG_JOINTS[leg_name][part_name]


def clamp_raw(raw: int) -> int:
    return int(max(0, min(1023, raw)))


def logical_deg_to_raw_delta(joint_name: str, deg: float) -> int:
    leg_name, part_name = joint_to_leg_part(joint_name)
    movement_sign = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)
    return int(round(deg * RAW_PER_DEG * movement_sign * joint_direction))


def raw_delta_to_logical_deg(joint_name: str, raw_delta: int) -> float:
    leg_name, part_name = joint_to_leg_part(joint_name)
    movement_sign = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)
    sign = movement_sign * joint_direction
    if sign == 0:
        sign = 1
    return raw_delta / RAW_PER_DEG / sign


def raw_from_tune_zero_offset(joint_name: str, logical_deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    center_raw = TUNE_ZERO.get(motor_id, FIXED_START_POSE[motor_id])
    return clamp_raw(center_raw + logical_deg_to_raw_delta(joint_name, logical_deg))


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

    def move_motor(self, motor_id: int, raw: int, speed: int):
        self.enable_torque(motor_id)
        self.set_speed(motor_id, speed)
        time.sleep(0.008)
        ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)
        if not ok:
            time.sleep(0.025)
            self.write2(motor_id, ADDR_GOAL_POSITION, raw)

    def move_many(self, targets: Dict[int, int], speed: int):
        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
            time.sleep(0.008)

        for motor_id, raw in targets.items():
            ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            if not ok:
                time.sleep(0.025)
                self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            time.sleep(0.008)


# ============================================================
# STATUS / CAPTURE
# ============================================================

def capture_current_pose(bus: DynamixelBus, label: str = "CURRENT POSE") -> Dict[int, int]:
    captured = {}

    print("\n===================================================")
    print(f" CAPTURING {label}")
    print("===================================================")

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        joint_name = motor_id_to_joint(motor_id)

        if pos is None:
            fallback = ACTIVE_GOALS.get(motor_id, FIXED_START_POSE[motor_id])
            captured[motor_id] = fallback
            print(f"ID {motor_id:>2} {joint_name:<10}: NO REPLY, fallback {fallback}")
        else:
            captured[motor_id] = pos
            print(f"ID {motor_id:>2} {joint_name:<10}: {pos}")

    return captured


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
    print(" MOTOR STATUS / FIXED POSE TUNER")
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
        joint_name = motor_id_to_joint(motor_id)
        leg_name, part_name = joint_to_leg_part(joint_name)

        raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
        volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

        if temp is not None:
            max_temp = max(max_temp, int(temp))

        zero = TUNE_ZERO.get(motor_id, FIXED_START_POSE[motor_id])
        goal = ACTIVE_GOALS.get(motor_id, zero)

        warnings = []

        if raw is None:
            print(
                f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
                f"{'----':>4} {'----':>8} {zero:>5} {goal:>5} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1
        zero_deg = raw_delta_to_logical_deg(joint_name, raw - zero)

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
            f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
            f"{raw:>4} {zero_deg:>8.2f} {zero:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")

    if max_temp >= TEMP_WARN_C:
        print(f"WARNING: Max motor temperature is {max_temp}C. Let robot cool.")


def print_ready_pose(bus: DynamixelBus):
    pose = {}

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        if pos is None:
            pos = ACTIVE_GOALS.get(motor_id, TUNE_ZERO.get(motor_id, FIXED_START_POSE[motor_id]))
        pose[motor_id] = pos

    print("\n===================================================")
    print(" COPY THIS READY_POSE / FINAL FIXED POSE")
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
        zero = TUNE_ZERO.get(motor_id, FIXED_START_POSE[motor_id])
        deg = raw_delta_to_logical_deg(joint, pose[motor_id] - zero)
        print(f"{motor_id:>2} {joint:<10} {deg:+8.2f} deg")


# ============================================================
# ACTIONS
# ============================================================

def action_fixed_pose(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print("\n===================================================")
    print(" MOVING TO FIXED START POSE")
    print("===================================================")
    print("This pose is copied from your pasted MOTOR STATUS raw values.")

    ACTIVE_GOALS = dict(FIXED_START_POSE)
    bus.move_many(ACTIVE_GOALS, speed=POSE_SPEED)
    time.sleep(1.8)
    print_status(bus)


def action_zero(bus: DynamixelBus):
    global TUNE_ZERO, ACTIVE_GOALS

    TUNE_ZERO = capture_current_pose(bus, label="TUNING ZERO")
    ACTIVE_GOALS = dict(TUNE_ZERO)
    print("\nNew TUNE_ZERO captured. set/nudge now tune from this pose.")
    print_status(bus)


def action_return_zero(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print("\nACTION: RETURN TO TUNE_ZERO")
    ACTIVE_GOALS = dict(TUNE_ZERO)
    bus.move_many(dict(TUNE_ZERO), speed=RETURN_SPEED)
    time.sleep(0.8)
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
            current_goal = TUNE_ZERO.get(motor_id, FIXED_START_POSE[motor_id])

    new_goal = clamp_raw(current_goal + logical_deg_to_raw_delta(joint_name, deg))
    ACTIVE_GOALS[motor_id] = new_goal

    print(f"\nNUDGE {joint_name} ID {motor_id}: {deg:+.2f} deg -> raw {new_goal}")
    bus.move_motor(motor_id, new_goal, speed=NUDGE_SPEED)
    time.sleep(0.35)
    print_status(bus)


def set_joint_from_zero(bus: DynamixelBus, joint_name: str, deg: float):
    global ACTIVE_GOALS

    if joint_name not in JOINT_INFO:
        print(f"Unknown joint: {joint_name}")
        return

    if not pre_motion_check(bus):
        return

    motor_id = joint_to_motor_id(joint_name)
    new_goal = raw_from_tune_zero_offset(joint_name, deg)
    ACTIVE_GOALS[motor_id] = new_goal

    print(f"\nSET {joint_name} ID {motor_id}: {deg:+.2f} deg from TUNE_ZERO -> raw {new_goal}")
    bus.move_motor(motor_id, new_goal, speed=NUDGE_SPEED)
    time.sleep(0.35)
    print_status(bus)


def set_raw_motor(bus: DynamixelBus, motor_id: int, raw: int):
    global ACTIVE_GOALS

    if motor_id not in FIXED_START_POSE:
        print(f"Unknown motor ID: {motor_id}")
        return

    if not pre_motion_check(bus):
        return

    raw = clamp_raw(raw)
    ACTIVE_GOALS[motor_id] = raw
    joint = motor_id_to_joint(motor_id)

    print(f"\nRAW SET ID {motor_id} {joint}: raw {raw}")
    bus.move_motor(motor_id, raw, speed=NUDGE_SPEED)
    time.sleep(0.35)
    print_status(bus)


def nudge_id(bus: DynamixelBus, motor_id: int, deg: float):
    joint_name = motor_id_to_joint(motor_id)

    if joint_name == "UNKNOWN":
        print(f"Unknown motor ID: {motor_id}")
        return

    nudge_joint(bus, joint_name, deg)


def set_leg_from_zero(bus: DynamixelBus, leg_name: str, hip_deg: float, femur_deg: float, tibia_deg: float):
    global ACTIVE_GOALS

    if leg_name not in LEG_JOINTS:
        print(f"Unknown leg: {leg_name}")
        return

    if not pre_motion_check(bus):
        return

    offsets = {
        leg_part_to_joint(leg_name, "hip"): hip_deg,
        leg_part_to_joint(leg_name, "femur"): femur_deg,
        leg_part_to_joint(leg_name, "tibia"): tibia_deg,
    }

    targets = {}
    for joint_name, deg in offsets.items():
        motor_id = joint_to_motor_id(joint_name)
        targets[motor_id] = raw_from_tune_zero_offset(joint_name, deg)

    ACTIVE_GOALS.update(targets)

    print(f"\nSET LEG {leg_name} FROM TUNE_ZERO")
    print(f"hip={hip_deg:+.2f}, femur={femur_deg:+.2f}, tibia={tibia_deg:+.2f}")
    bus.move_many(targets, speed=NUDGE_SPEED)
    time.sleep(0.6)
    print_status(bus)


# ============================================================
# HELP / MAIN
# ============================================================

def print_help():
    print("\n===================================================")
    print(" FIXED START POSE + INDIVIDUAL TUNER")
    print("===================================================")
    print("p                   = print motor status")
    print("fixed               = move to pasted fixed start pose")
    print("zero                = capture current pose as TUNE_ZERO")
    print("r                   = return to TUNE_ZERO")
    print("save                = print current pose as READY_POSE dictionary")
    print("x                   = exit")
    print("---------------------------------------------------")
    print("nudge JOINT DEG     = nudge one joint from current target")
    print("set JOINT DEG       = set one joint offset from TUNE_ZERO")
    print("id ID DEG           = nudge motor ID by degree amount")
    print("raw ID RAW          = set motor ID exact raw value")
    print("leg LEG H F T       = set one leg offsets from TUNE_ZERO")
    print("---------------------------------------------------")
    print("Examples:")
    print("  fixed")
    print("  zero")
    print("  nudge FL_femur -3")
    print("  nudge MR_tibia 5")
    print("  set RR_femur -2")
    print("  id 11 -5")
    print("  raw 11 530")
    print("  leg FL 0 -3 5")
    print("  save")
    print("===================================================")


def main():
    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        print_status(bus)
        print_help()

        print("\nRecommended workflow:")
        print("1. fixed")
        print("2. zero")
        print("3. tune using nudge/set/id/raw/leg")
        print("4. save")
        print("5. paste READY_POSE here")

        while True:
            raw_cmd = input("\nCommand [h help]: ").strip()

            if raw_cmd == "":
                continue

            parts = raw_cmd.split()
            cmd = parts[0].lower()

            try:
                if cmd == "x":
                    print("Exiting.")
                    break
                elif cmd == "h":
                    print_help()
                elif cmd == "p":
                    print_status(bus)
                elif cmd == "fixed":
                    action_fixed_pose(bus)
                elif cmd == "zero":
                    action_zero(bus)
                elif cmd == "r":
                    action_return_zero(bus)
                elif cmd == "save":
                    print_ready_pose(bus)
                elif cmd == "nudge":
                    if len(parts) != 3:
                        print("Usage: nudge JOINT DEG")
                        continue
                    nudge_joint(bus, parts[1], float(parts[2]))
                elif cmd == "set":
                    if len(parts) != 3:
                        print("Usage: set JOINT DEG")
                        continue
                    set_joint_from_zero(bus, parts[1], float(parts[2]))
                elif cmd == "id":
                    if len(parts) != 3:
                        print("Usage: id ID DEG")
                        continue
                    nudge_id(bus, int(parts[1]), float(parts[2]))
                elif cmd == "raw":
                    if len(parts) != 3:
                        print("Usage: raw ID RAW")
                        continue
                    set_raw_motor(bus, int(parts[1]), int(parts[2]))
                elif cmd == "leg":
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
                    print(f"Unknown command: {raw_cmd}")
                    print("Type h for help.")

            except ValueError:
                print("Invalid number format.")
            except KeyboardInterrupt:
                print("\nKeyboard interrupt.")
                break

    finally:
        try:
            action_return_zero(bus)
        except Exception:
            pass
        bus.close()


if __name__ == "__main__":
    main()
