# tools/calibration/lower_stance.py
#
# HEXAPOD LOWER STANCE TEST - REVISED DEEP STANCE VERSION
#
# Based on latest test:
#   - low5 was tested on stool, so do not over-trust it.
#   - low6 onward was tested on ground, so those results matter more.
#   - low12 looked like the best deep candidate:
#       minVolt around 11.0V
#       maxLoad around 448
#       noReply False
#       Status OK
#   - low13 still worked, but FR_tibia showed LOAD_WARN.
#
# This version:
#   - Keeps positive tibia direction because positive tibia tucks inward correctly.
#   - Reduces tibia inward amount from low11 onward.
#   - Adds optional low14 and low15.
#   - Does NOT auto-return to old tall ready pose.
#
# Recommended test:
#   health
#   low10
#   hold 5
#   health
#   low11
#   hold 5
#   health
#   low12
#   hold 10
#   health
#   low13
#   hold 5
#   health
#   save
#   x
#
# Stop if:
#   maxLoad >= 450 warning repeatedly
#   maxLoad >= 700 danger
#   minVolt <= 10.8V repeatedly
#   minVolt <= 9.5V danger
#   temp >= 50C
#   body touches ground
#   legs bind
#   feet slip badly
#   any NO_REPLY

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

RAW_PER_DEG = 1023.0 / 300.0

READ_RETRIES = 3
READ_RETRY_DELAY = 0.04

MOVE_SPEED = 10

TEMP_WARN_C = 50
TEMP_STOP_C = 58

LOAD_WARN = 450
LOAD_STOP = 700

VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5
VOLT_DANGER_V = 9.2


# ============================================================
# ORIGINAL READY POSE
# ============================================================
# Old/tall ready reference.
# Lower poses are calculated from this reference.
# This script will NOT automatically return to this old tall pose.

ORIGINAL_READY_POSE = {
    1: 511,   # RL_hip
    2: 798,   # FL_hip
    3: 531,   # FR_femur
    4: 494,   # FL_femur
    5: 606,   # FR_tibia
    6: 596,   # FL_tibia
    7: 557,   # MR_hip
    8: 804,   # ML_hip
    9: 510,   # MR_femur
    10: 539,  # ML_femur
    11: 405,  # MR_tibia
    12: 613,  # ML_tibia
    13: 527,  # RR_hip
    14: 524,  # FR_hip
    15: 534,  # RR_femur
    16: 529,  # RL_femur
    17: 368,  # RR_tibia
    18: 628,  # RL_tibia
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
    "RL_hip": {"id": 1, "type": "hip"},
    "FL_hip": {"id": 2, "type": "hip"},
    "FR_femur": {"id": 3, "type": "femur"},
    "FL_femur": {"id": 4, "type": "femur"},
    "FR_tibia": {"id": 5, "type": "tibia"},
    "FL_tibia": {"id": 6, "type": "tibia"},
    "MR_hip": {"id": 7, "type": "hip"},
    "ML_hip": {"id": 8, "type": "hip"},
    "MR_femur": {"id": 9, "type": "femur"},
    "ML_femur": {"id": 10, "type": "femur"},
    "MR_tibia": {"id": 11, "type": "tibia"},
    "ML_tibia": {"id": 12, "type": "tibia"},
    "RR_hip": {"id": 13, "type": "hip"},
    "FR_hip": {"id": 14, "type": "hip"},
    "RR_femur": {"id": 15, "type": "femur"},
    "RL_femur": {"id": 16, "type": "femur"},
    "RR_tibia": {"id": 17, "type": "tibia"},
    "RL_tibia": {"id": 18, "type": "tibia"},
}

MOTOR_TO_JOINT = {info["id"]: joint for joint, info in JOINT_INFO.items()}
ALL_MOTOR_IDS = sorted(ORIGINAL_READY_POSE.keys())
ALL_LEGS = ["FL", "ML", "RL", "FR", "MR", "RR"]


# ============================================================
# MOVEMENT SIGN MODEL
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

# Hip inward direction.
# If feet spread outward instead of inward, flip these signs.
HIP_INWARD_SIGN = {
    "FL": -1,
    "ML": -1,
    "RL": -1,
    "FR": 1,
    "MR": 1,
    "RR": 1,
}


# ============================================================
# HEIGHT LEVELS - REVISED AFTER GROUND TEST
# ============================================================
#
# Important:
#   low5 was on stool.
#   low6 onward was on ground.
#
# Finding:
#   low12 looked like the best deep stance candidate:
#       minVolt around 11.0V
#       maxLoad around 448
#       noReply False
#       status OK
#
# Adjustment:
#   From low11 onward, reduce tibia inward tuck slightly.
#   Keep femur and hip going lower.
#
# Positive tibia = inward tuck on this robot.

HEIGHT_LEVELS = {
    # Early lower range
    "base": {"hip": 0.0, "femur": -7.0,  "tibia": -5.5},
    "low1": {"hip": 0.0, "femur": -9.0,  "tibia": -7.0},
    "low2": {"hip": 0.0, "femur": -11.0, "tibia": -8.5},
    "low3": {"hip": 0.0, "femur": -13.0, "tibia": -10.0},
    "low4": {"hip": 0.0, "femur": -15.0, "tibia": -11.5},

    # Ground-test range
    "low5": {"hip": 1.5,  "femur": -17.0, "tibia": 14.0},
    "low6": {"hip": 3.0,  "femur": -19.0, "tibia": 20.0},
    "low7": {"hip": 4.5,  "femur": -21.0, "tibia": 26.0},
    "low8": {"hip": 6.0,  "femur": -23.0, "tibia": 32.0},
    "low9": {"hip": 7.5,  "femur": -25.0, "tibia": 38.0},
    "low10": {"hip": 9.0, "femur": -27.0, "tibia": 44.0},

    # Revised deeper levels:
    # Old:
    #   low11 +50, low12 +56, low13 +62
    # New:
    #   low11 +46, low12 +50, low13 +54
    "low11": {"hip": 10.5, "femur": -29.0, "tibia": 46.0},
    "low12": {"hip": 12.0, "femur": -31.0, "tibia": 50.0},
    "low13": {"hip": 13.5, "femur": -33.0, "tibia": 54.0},

    # Optional deeper tests.
    # Use only if low11-low13 remain mechanically clear.
    "low14": {"hip": 15.0, "femur": -35.0, "tibia": 56.0},
    "low15": {"hip": 16.5, "femur": -37.0, "tibia": 58.0},
}


# ============================================================
# RUNTIME STATE
# ============================================================

CURRENT_POSE_NAME = "unknown"
ACTIVE_GOALS: Dict[int, int] = dict(ORIGINAL_READY_POSE)


# ============================================================
# HELPERS
# ============================================================

def clamp_raw(raw: int) -> int:
    return int(max(0, min(1023, raw)))


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


def raw_from_original_offset(joint_name: str, deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    base = ORIGINAL_READY_POSE[motor_id]
    return clamp_raw(base + logical_deg_to_raw_delta(joint_name, deg))


def raw_hip_inward_from_original(leg_name: str, hip_deg: float) -> int:
    hip_joint = leg_part_to_joint(leg_name, "hip")
    hip_id = joint_to_motor_id(hip_joint)

    raw_delta = int(round(
        hip_deg * RAW_PER_DEG * HIP_INWARD_SIGN.get(leg_name, 1)
    ))

    return clamp_raw(ORIGINAL_READY_POSE[hip_id] + raw_delta)


def build_height_pose_targets(level_name: str) -> Dict[int, int]:
    if level_name not in HEIGHT_LEVELS:
        raise ValueError(f"Unknown height level: {level_name}")

    hip_deg = HEIGHT_LEVELS[level_name].get("hip", 0.0)
    femur_deg = HEIGHT_LEVELS[level_name]["femur"]
    tibia_deg = HEIGHT_LEVELS[level_name]["tibia"]

    targets = dict(ORIGINAL_READY_POSE)

    for leg in ALL_LEGS:
        hip_joint = leg_part_to_joint(leg, "hip")
        femur_joint = leg_part_to_joint(leg, "femur")
        tibia_joint = leg_part_to_joint(leg, "tibia")

        hip_id = joint_to_motor_id(hip_joint)
        femur_id = joint_to_motor_id(femur_joint)
        tibia_id = joint_to_motor_id(tibia_joint)

        targets[hip_id] = raw_hip_inward_from_original(leg, hip_deg)
        targets[femur_id] = raw_from_original_offset(femur_joint, femur_deg)
        targets[tibia_id] = raw_from_original_offset(tibia_joint, tibia_deg)

    return targets


# ============================================================
# DYNAMIXEL BUS
# ============================================================

class DynamixelBus:
    def __init__(self, port_name: str = DEFAULT_PORT):
        self.port_name = port_name
        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

    def open(self) -> bool:
        print()
        print("===================================================")
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
                self.port_handler,
                motor_id,
                address,
                int(value),
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
                self.port_handler,
                motor_id,
                address,
                value,
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
                self.port_handler,
                motor_id,
                address,
            )
        except Exception:
            return None

        if result != COMM_SUCCESS or error != 0:
            return None

        return value

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                motor_id,
                address,
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

    def move_many(self, targets: Dict[int, int], speed: int):
        global ACTIVE_GOALS

        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
            time.sleep(0.008)

        for motor_id, raw in targets.items():
            raw = clamp_raw(raw)

            ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)

            if not ok:
                time.sleep(0.04)
                self.write2(motor_id, ADDR_GOAL_POSITION, raw)

            ACTIVE_GOALS[motor_id] = raw
            time.sleep(0.01)


# ============================================================
# HEALTH / STATUS
# ============================================================

def read_bus_health(bus: DynamixelBus) -> Tuple[int, float, int, bool, int]:
    max_temp = 0
    min_volt = 99.0
    max_abs_load = 0
    any_no_reply = False
    connected = 0

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
        volt_raw = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)

        if pos is None:
            any_no_reply = True
        else:
            connected += 1

        if temp is not None:
            max_temp = max(max_temp, int(temp))

        if volt_raw is not None:
            min_volt = min(min_volt, volt_raw / 10.0)

        load_value = decode_load_value(load_raw)

        if load_value is not None:
            max_abs_load = max(max_abs_load, abs(load_value))

    return max_temp, min_volt, max_abs_load, any_no_reply, connected


def health_status(
    max_temp: int,
    min_volt: float,
    max_abs_load: int,
    any_no_reply: bool,
) -> str:
    if any_no_reply:
        return "NO_REPLY"

    if min_volt <= VOLT_DANGER_V:
        return "DANGER_VOLT"

    if min_volt <= VOLT_STOP_V:
        return "VOLT_STOP"

    if max_abs_load >= LOAD_STOP:
        return "LOAD_STOP"

    if max_temp >= TEMP_STOP_C:
        return "TEMP_STOP"

    if min_volt <= VOLT_WARN_V or max_abs_load >= LOAD_WARN or max_temp >= TEMP_WARN_C:
        return "WARN"

    return "OK"


def print_health(bus: DynamixelBus, label: str = "HEALTH"):
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)
    status = health_status(max_temp, min_volt, max_abs_load, any_no_reply)

    print()
    print("===================================================")
    print(f" {label}")
    print("===================================================")
    print(f"Current pose : {CURRENT_POSE_NAME}")
    print(f"Connected    : {connected}/18")
    print(f"Max temp     : {max_temp} C")
    print(f"Min voltage  : {min_volt:.1f} V")
    print(f"Max abs load : {max_abs_load}")
    print(f"No reply     : {any_no_reply}")
    print(f"Status       : {status}")

    if min_volt <= VOLT_DANGER_V:
        print("DANGER: voltage near/below 9V. Stop testing and power cycle if motors stop replying.")

    if max_abs_load >= LOAD_STOP:
        print("DANGER: load exceeded LOAD_STOP. Pose is too stressful.")

    if any_no_reply:
        print("DANGER: at least one motor stopped replying.")


def pre_motion_check(bus: DynamixelBus) -> bool:
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)
    status = health_status(max_temp, min_volt, max_abs_load, any_no_reply)

    if status in ["NO_REPLY", "DANGER_VOLT", "VOLT_STOP", "LOAD_STOP", "TEMP_STOP"]:
        print()
        print(f"[SAFETY STOP] Movement blocked. Status={status}")
        print(
            f"connected={connected}/18, "
            f"minVolt={min_volt:.1f}V, "
            f"maxLoad={max_abs_load}, "
            f"maxTemp={max_temp}C"
        )
        print("Use force_base only if physically supporting the robot.")
        return False

    if status == "WARN":
        print()
        print("[WARNING] Movement allowed but status is WARN.")
        print(
            f"connected={connected}/18, "
            f"minVolt={min_volt:.1f}V, "
            f"maxLoad={max_abs_load}, "
            f"maxTemp={max_temp}C"
        )

    return True


def print_status(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" MOTOR STATUS / REVISED DEEP STANCE TEST")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'DegOrig':>8} {'Orig':>5} {'Goal':>5} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} Warnings"
    )
    print("-" * 120)

    connected = 0
    max_temp = 0
    min_volt = 99.0
    max_abs_load = 0

    for motor_id in ALL_MOTOR_IDS:
        joint_name = motor_id_to_joint(motor_id)
        leg_name, part_name = joint_to_leg_part(joint_name)

        raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
        volt = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

        orig = ORIGINAL_READY_POSE[motor_id]
        goal = ACTIVE_GOALS.get(motor_id, orig)

        warnings = []

        if raw is None:
            print(
                f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
                f"{'----':>4} {'----':>8} {orig:>5} {goal:>5} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1
        deg = raw_delta_to_logical_deg(joint_name, raw - orig)

        load_value = decode_load_value(load_raw)

        if load_value is not None:
            max_abs_load = max(max_abs_load, abs(load_value))

            if abs(load_value) >= LOAD_STOP:
                warnings.append("LOAD_STOP")
            elif abs(load_value) >= LOAD_WARN:
                warnings.append("LOAD_WARN")

        if temp is not None:
            max_temp = max(max_temp, int(temp))

            if temp >= TEMP_STOP_C:
                warnings.append("TEMP_STOP")
            elif temp >= TEMP_WARN_C:
                warnings.append("TEMP_WARN")

        if volt is not None:
            v = volt / 10.0
            min_volt = min(min_volt, v)

            if v <= VOLT_STOP_V:
                warnings.append("VOLT_STOP")
            elif v <= VOLT_WARN_V:
                warnings.append("LOW_VOLTAGE")

        volt_text = "----" if volt is None else f"{volt / 10:.1f}"
        temp_text = "----" if temp is None else str(temp)
        warn_text = "OK" if not warnings else ",".join(warnings)

        print(
            f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
            f"{raw:>4} {deg:>8.2f} {orig:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")
    print(f"Health: maxTemp={max_temp}C, minVolt={min_volt:.1f}V, maxAbsLoad={max_abs_load}")
    print(f"Current pose: {CURRENT_POSE_NAME}")


# ============================================================
# ACTIONS
# ============================================================

def move_to_level(bus: DynamixelBus, level_name: str, use_safety_check: bool = True):
    global ACTIVE_GOALS, CURRENT_POSE_NAME

    if level_name not in HEIGHT_LEVELS:
        print(f"Unknown level: {level_name}")
        return

    if use_safety_check:
        if not pre_motion_check(bus):
            return

    hip_deg = HEIGHT_LEVELS[level_name].get("hip", 0.0)
    femur_deg = HEIGHT_LEVELS[level_name]["femur"]
    tibia_deg = HEIGHT_LEVELS[level_name]["tibia"]

    print()
    print("===================================================")
    print(f" ACTION: MOVE TO {level_name.upper()}")
    print("===================================================")
    print(f"Hip inward offset from original ready: {hip_deg:+.2f} deg")
    print(f"Femur offset from original ready    : {femur_deg:+.2f} deg")
    print(f"Tibia offset from original ready    : {tibia_deg:+.2f} deg")
    print("Hips move inward based on HIP_INWARD_SIGN.")
    print("Positive tibia from low5 onward should tuck inward.")
    print("===================================================")

    print_health(bus, f"BEFORE {level_name.upper()}")

    targets = build_height_pose_targets(level_name)

    print()
    print("Targets:")
    for motor_id, raw in sorted(targets.items()):
        print(f"  ID {motor_id:>2} {motor_id_to_joint(motor_id):<10} raw={raw}")

    ACTIVE_GOALS = dict(targets)
    CURRENT_POSE_NAME = level_name

    bus.move_many(targets, speed=MOVE_SPEED)
    time.sleep(1.0)

    print_status(bus)
    print_health(bus, f"AFTER {level_name.upper()}")


def hold_health(bus: DynamixelBus, seconds: float, label: str):
    seconds = max(0.0, float(seconds))
    start = time.time()
    next_print = start

    print()
    print("===================================================")
    print(f" HOLDING: {label} for {seconds:.1f}s")
    print("===================================================")

    while time.time() - start < seconds:
        now = time.time()

        if now >= next_print:
            elapsed = now - start
            max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)
            status = health_status(max_temp, min_volt, max_abs_load, any_no_reply)

            print(
                f"[{label}] t={elapsed:>5.1f}s | "
                f"pose={CURRENT_POSE_NAME} | "
                f"connected={connected}/18 | "
                f"minVolt={min_volt:.1f}V | "
                f"maxLoad={max_abs_load} | "
                f"maxTemp={max_temp}C | "
                f"noReply={any_no_reply} | "
                f"status={status}"
            )

            if status in ["NO_REPLY", "DANGER_VOLT", "VOLT_STOP", "LOAD_STOP", "TEMP_STOP"]:
                print("DANGER condition detected during hold. Stop test.")
                break

            next_print = now + 0.5

        time.sleep(0.05)

    print_health(bus, f"AFTER HOLD: {label}")


def print_current_pose_as_ready(bus: DynamixelBus):
    pose = {}

    print()
    print("===================================================")
    print(" COPY THIS AS NEW READY_POSE")
    print("===================================================")

    for motor_id in ALL_MOTOR_IDS:
        raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)

        if raw is None:
            raw = ACTIVE_GOALS.get(motor_id, ORIGINAL_READY_POSE[motor_id])

        pose[motor_id] = raw

    print("READY_POSE = {")
    for motor_id in sorted(pose.keys()):
        print(f"    {motor_id}: {pose[motor_id]},   # {motor_id_to_joint(motor_id)}")
    print("}")

    print("===================================================")
    print(f"Saved pose name: {CURRENT_POSE_NAME}")
    print("===================================================")


# ============================================================
# HELP / MAIN
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" HEXAPOD LOWER STANCE TEST - REVISED DEEP STANCE")
    print("===================================================")
    print("p             = print full motor status")
    print("health        = compact health summary")
    print("base          = move to lower base stance")
    print("low1          = move to lower level 1")
    print("low2          = move to lower level 2")
    print("low3          = move to lower level 3")
    print("low4          = move to lower level 4")
    print("low5          = move to lower level 5")
    print("low6          = move to lower level 6")
    print("low7          = move to lower level 7")
    print("low8          = move to lower level 8")
    print("low9          = move to lower level 9")
    print("low10         = move to lower level 10")
    print("low11         = move to lower level 11")
    print("low12         = move to lower level 12")
    print("low13         = move to lower level 13")
    print("low14         = move to lower level 14")
    print("low15         = move to lower level 15")
    print("hold 10       = hold current pose for 10 seconds")
    print("save          = print current pose as READY_POSE")
    print("force_base    = force move to base without safety check")
    print("x             = exit without moving")
    print("---------------------------------------------------")
    print("Height levels from original tall ready:")
    for name, offsets in HEIGHT_LEVELS.items():
        print(
            f"  {name:<5} "
            f"hip={offsets.get('hip', 0.0):+5.1f} deg, "
            f"femur={offsets['femur']:+6.1f} deg, "
            f"tibia={offsets['tibia']:+6.1f} deg"
        )
    print("---------------------------------------------------")
    print("Recommended test:")
    print("  health")
    print("  low10")
    print("  hold 5")
    print("  health")
    print("  low11")
    print("  hold 5")
    print("  health")
    print("  low12")
    print("  hold 10")
    print("  health")
    print("  low13")
    print("  hold 5")
    print("  health")
    print("  save")
    print("  x")
    print("---------------------------------------------------")
    print("Stop if:")
    print("  maxLoad >= 450 warning repeatedly")
    print("  maxLoad >= 700 danger")
    print("  minVolt <= 10.8 warning repeatedly")
    print("  minVolt <= 9.5 danger")
    print("  temp >= 50C warning")
    print("  body touches ground")
    print("  legs bind mechanically")
    print("  feet slip badly")
    print("  any NO_REPLY")
    print("---------------------------------------------------")
    print("If legs spread OUT instead of moving inward, flip HIP_INWARD_SIGN.")
    print("If tibias still go outward, we need per-leg tibia signs.")
    print("===================================================")


def main():
    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        print_status(bus)
        print_health(bus, "START HEALTH")
        print_help()

        while True:
            try:
                raw_cmd = input("\nHeightTest command [h help]: ").strip()
            except KeyboardInterrupt:
                print()
                print("KeyboardInterrupt detected. Exiting without moving.")
                break

            if not raw_cmd:
                continue

            parts = raw_cmd.split()
            cmd = parts[0].lower()

            try:
                if cmd == "x":
                    print("Exit requested. No auto-return to old tall ready.")
                    break

                elif cmd == "h":
                    print_help()

                elif cmd == "p":
                    print_status(bus)

                elif cmd == "health":
                    print_health(bus, "MANUAL HEALTH CHECK")

                elif cmd in HEIGHT_LEVELS:
                    move_to_level(bus, cmd, use_safety_check=True)

                elif cmd == "force_base":
                    print()
                    print("FORCE_BASE: moving to base without safety check.")
                    print("Physically support the robot before using this.")
                    move_to_level(bus, "base", use_safety_check=False)

                elif cmd == "hold":
                    if len(parts) != 2:
                        print("Usage: hold 10")
                        continue

                    hold_health(bus, float(parts[1]), "MANUAL HOLD")

                elif cmd == "save":
                    print_current_pose_as_ready(bus)

                else:
                    print(f"Unknown command: {raw_cmd}")
                    print("Type h for help.")

            except ValueError:
                print("Invalid number format.")

            except KeyboardInterrupt:
                print()
                print("KeyboardInterrupt detected during command. Exiting without moving.")
                break

    finally:
        bus.close()


if __name__ == "__main__":
    main()