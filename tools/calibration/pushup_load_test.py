# tools/calibration/pushup_load_test.py
#
# HEXAPOD 6-LEG PUSH-UP LOAD / VOLTAGE TEST
#
# Purpose:
#   Test whether the robot can safely hold body-height changes while standing on all 6 legs.
#
# This does NOT lift tripod A/B.
# It keeps all 6 legs on the ground and moves femur/tibia of every leg together.
#
# Commands:
#   p             = print full motor status
#   health        = print compact health summary
#   ready         = move to READY_POSE
#   zero          = capture current pose as SESSION_READY
#   low           = move body to LOW_POSE
#   up            = return to SESSION_READY
#   cycle         = ready/up -> low -> up with health checks
#   hold 5        = hold current pose for 5 seconds while printing health
#   force_up      = return to SESSION_READY without safety check
#   x             = clean exit
#
# Notes:
#   - If "low" makes the robot go UP instead, flip BODY_LOW_FEMUR_DEG and BODY_LOW_TIBIA_DEG signs.
#   - Start with very small movement.
#   - Watch min voltage, max load, and temperature.
#   - If voltage drops near/below 9V or motors give NO_REPLY, stop and power cycle.
#
# Based on your current motor map / READY_POSE from calibrate_lift.py.

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

READY_SPEED = 22
PUSHUP_SPEED = 12

TEMP_WARN_C = 50
TEMP_STOP_C = 58

LOAD_WARN = 450
LOAD_STOP = 700

VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5

# Hard danger zone for your own observation.
# If it goes near 9V, expect Dynamixel brownout / no reply risk.
VOLT_DANGER_V = 9.2


# ============================================================
# READY POSE
# ============================================================

READY_POSE = {
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
ALL_MOTOR_IDS = sorted(READY_POSE.keys())
ALL_LEGS = ["FL", "ML", "RL", "FR", "MR", "RR"]

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


# ============================================================
# PUSH-UP TEST SETTINGS
# ============================================================
# Start small.
#
# If "low" makes the body go UP instead of DOWN:
#   change both signs, for example:
#       BODY_LOW_FEMUR_DEG = +5.0
#       BODY_LOW_TIBIA_DEG = +4.0
#
# If voltage/load spikes too much:
#   reduce to -3 / -2 or even -2 / -1.

BODY_LOW_FEMUR_DEG = -5.0
BODY_LOW_TIBIA_DEG = -4.0

LOW_HOLD_SECONDS = 3.0
UP_HOLD_SECONDS = 3.0


# ============================================================
# RUNTIME STATE
# ============================================================

SESSION_READY: Dict[int, int] = dict(READY_POSE)
ACTIVE_GOALS: Dict[int, int] = dict(READY_POSE)


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


def raw_from_session_offset(joint_name: str, deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    base = SESSION_READY.get(motor_id, READY_POSE[motor_id])
    return clamp_raw(base + logical_deg_to_raw_delta(joint_name, deg))


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
# STATUS / HEALTH
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


def print_health(bus: DynamixelBus, label: str = "HEALTH"):
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)

    status = "OK"

    if any_no_reply:
        status = "NO_REPLY"
    elif min_volt <= VOLT_DANGER_V:
        status = "DANGER_VOLT"
    elif min_volt <= VOLT_STOP_V:
        status = "VOLT_STOP"
    elif max_abs_load >= LOAD_STOP:
        status = "LOAD_STOP"
    elif max_temp >= TEMP_STOP_C:
        status = "TEMP_STOP"
    elif min_volt <= VOLT_WARN_V or max_abs_load >= LOAD_WARN or max_temp >= TEMP_WARN_C:
        status = "WARN"

    print()
    print("===================================================")
    print(f" {label}")
    print("===================================================")
    print(f"Connected   : {connected}/18")
    print(f"Max temp    : {max_temp} C")
    print(f"Min voltage : {min_volt:.1f} V")
    print(f"Max abs load: {max_abs_load}")
    print(f"No reply    : {any_no_reply}")
    print(f"Status      : {status}")

    if min_volt <= VOLT_DANGER_V:
        print("WARNING: Voltage is near/below 9V danger zone. Stop testing and power cycle if motors stop replying.")

    if max_abs_load >= LOAD_STOP:
        print("WARNING: Motor load exceeded LOAD_STOP threshold. Pose is too stressful.")

    if any_no_reply:
        print("WARNING: At least one motor stopped replying. Do not continue movement tests until recovered.")


def pre_motion_check(bus: DynamixelBus) -> bool:
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)

    if any_no_reply:
        print()
        print("[SAFETY STOP] At least one motor gave NO_REPLY. Movement blocked.")
        print("Use force_up only if you are trying to recover while physically supporting the robot.")
        return False

    if max_temp >= TEMP_STOP_C:
        print()
        print(f"[SAFETY STOP] Max temperature is {max_temp}C. Movement blocked.")
        return False

    if min_volt <= VOLT_STOP_V:
        print()
        print(f"[SAFETY STOP] Minimum voltage is {min_volt:.1f}V. Movement blocked.")
        print("Voltage is too low. Power cycle / recharge / improve power supply.")
        return False

    if max_abs_load >= LOAD_STOP:
        print()
        print(f"[SAFETY STOP] Max absolute load is {max_abs_load}. Movement blocked.")
        return False

    if max_temp >= TEMP_WARN_C:
        print()
        print(f"[WARNING] Max temperature is {max_temp}C. Let motors cool soon.")

    if min_volt <= VOLT_WARN_V:
        print()
        print(f"[WARNING] Minimum voltage is {min_volt:.1f}V. Battery/power is sagging.")

    if max_abs_load >= LOAD_WARN:
        print()
        print(f"[WARNING] Max absolute load is {max_abs_load}. Reduce movement or support robot.")

    return True


def capture_current_pose(bus: DynamixelBus, label: str) -> Dict[int, int]:
    captured = {}

    print()
    print("===================================================")
    print(f" CAPTURING {label}")
    print("===================================================")

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        joint_name = motor_id_to_joint(motor_id)

        if pos is None:
            fallback = ACTIVE_GOALS.get(motor_id, READY_POSE[motor_id])
            captured[motor_id] = fallback
            print(f"ID {motor_id:>2} {joint_name:<10}: NO REPLY, fallback {fallback}")
        else:
            captured[motor_id] = pos
            print(f"ID {motor_id:>2} {joint_name:<10}: {pos}")

    return captured


def print_status(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" MOTOR STATUS / PUSH-UP LOAD TEST")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'Deg':>8} {'Ready':>5} {'Goal':>5} "
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

        if temp is not None:
            max_temp = max(max_temp, int(temp))

        if volt is not None:
            min_volt = min(min_volt, volt / 10.0)

        ready = SESSION_READY.get(motor_id, READY_POSE[motor_id])
        goal = ACTIVE_GOALS.get(motor_id, ready)

        warnings = []

        if raw is None:
            print(
                f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
                f"{'----':>4} {'----':>8} {ready:>5} {goal:>5} "
                f"{'----':>7} {'----':>5} {'----':>5} NO_REPLY"
            )
            continue

        connected += 1
        deg = raw_delta_to_logical_deg(joint_name, raw - ready)

        load_value = decode_load_value(load_raw)

        if load_value is not None:
            max_abs_load = max(max_abs_load, abs(load_value))

            if abs(load_value) >= LOAD_STOP:
                warnings.append("LOAD_STOP")
            elif abs(load_value) >= LOAD_WARN:
                warnings.append("LOAD_WARN")

        if temp is not None:
            if temp >= TEMP_STOP_C:
                warnings.append("TEMP_STOP")
            elif temp >= TEMP_WARN_C:
                warnings.append("TEMP_WARN")

        if volt is not None:
            v = volt / 10.0

            if v <= VOLT_STOP_V:
                warnings.append("VOLT_STOP")
            elif v <= VOLT_WARN_V:
                warnings.append("LOW_VOLTAGE")

        volt_text = "----" if volt is None else f"{volt / 10:.1f}"
        temp_text = "----" if temp is None else str(temp)
        warn_text = "OK" if not warnings else ",".join(warnings)

        print(
            f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
            f"{raw:>4} {deg:>8.2f} {ready:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")
    print(f"Health: maxTemp={max_temp}C, minVolt={min_volt:.1f}V, maxAbsLoad={max_abs_load}")


# ============================================================
# TARGET BUILDERS
# ============================================================

def build_low_pose_targets() -> Dict[int, int]:
    targets = {}

    for leg in ALL_LEGS:
        femur_joint = leg_part_to_joint(leg, "femur")
        tibia_joint = leg_part_to_joint(leg, "tibia")

        femur_id = joint_to_motor_id(femur_joint)
        tibia_id = joint_to_motor_id(tibia_joint)

        targets[femur_id] = raw_from_session_offset(femur_joint, BODY_LOW_FEMUR_DEG)
        targets[tibia_id] = raw_from_session_offset(tibia_joint, BODY_LOW_TIBIA_DEG)

        # Hips are intentionally unchanged for push-up test.

    return targets


# ============================================================
# ACTIONS
# ============================================================

def action_ready(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print()
    print("ACTION: READY_POSE")

    ACTIVE_GOALS = dict(READY_POSE)
    bus.move_many(dict(READY_POSE), speed=READY_SPEED)
    time.sleep(1.0)

    print_status(bus)
    print_health(bus, "AFTER READY")


def action_zero(bus: DynamixelBus):
    global SESSION_READY, ACTIVE_GOALS

    SESSION_READY = capture_current_pose(bus, "SESSION_READY / ZERO")
    ACTIVE_GOALS = dict(SESSION_READY)

    print()
    print("New SESSION_READY captured.")
    print_status(bus)
    print_health(bus, "AFTER ZERO")


def action_up(bus: DynamixelBus, use_safety_check: bool = True):
    global ACTIVE_GOALS

    if use_safety_check:
        if not pre_motion_check(bus):
            return

    print()
    print("ACTION: BODY UP / RETURN TO SESSION_READY")

    ACTIVE_GOALS = dict(SESSION_READY)
    bus.move_many(dict(SESSION_READY), speed=READY_SPEED)
    time.sleep(1.0)

    print_status(bus)
    print_health(bus, "AFTER UP")


def action_low(bus: DynamixelBus):
    global ACTIVE_GOALS

    if not pre_motion_check(bus):
        return

    print()
    print("===================================================")
    print(" ACTION: BODY LOW / 6-LEG PUSH-UP TEST")
    print("===================================================")
    print(f"BODY_LOW_FEMUR_DEG = {BODY_LOW_FEMUR_DEG:+.2f}")
    print(f"BODY_LOW_TIBIA_DEG = {BODY_LOW_TIBIA_DEG:+.2f}")
    print("All hips unchanged.")
    print("All 6 femurs/tibias move together.")
    print("===================================================")

    print_health(bus, "BEFORE LOW")

    targets = build_low_pose_targets()

    print()
    print("Targets:")
    for motor_id, raw in sorted(targets.items()):
        print(f"  ID {motor_id:>2} {motor_id_to_joint(motor_id):<10} raw={raw}")

    ACTIVE_GOALS.update(targets)
    bus.move_many(targets, speed=PUSHUP_SPEED)
    time.sleep(1.0)

    print_status(bus)
    print_health(bus, "AFTER LOW")


def action_cycle(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" PUSH-UP CYCLE TEST")
    print("===================================================")
    print("Sequence:")
    print("  health before")
    print("  low")
    print(f"  hold low for {LOW_HOLD_SECONDS:.1f}s")
    print("  up")
    print(f"  hold up for {UP_HOLD_SECONDS:.1f}s")
    print("===================================================")

    print_health(bus, "CYCLE START HEALTH")

    action_low(bus)
    hold_health(bus, LOW_HOLD_SECONDS, "LOW HOLD")

    action_up(bus)
    hold_health(bus, UP_HOLD_SECONDS, "UP HOLD")

    print_health(bus, "CYCLE END HEALTH")


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
            print(
                f"[{label}] t={elapsed:>5.1f}s | "
                f"connected={connected}/18 | "
                f"minVolt={min_volt:.1f}V | "
                f"maxLoad={max_abs_load} | "
                f"maxTemp={max_temp}C | "
                f"noReply={any_no_reply}"
            )

            if any_no_reply:
                print("NO_REPLY detected during hold. Stop test and recover.")
                break

            if min_volt <= VOLT_DANGER_V:
                print("DANGER: voltage near/below 9V. Stop test.")
                break

            if max_abs_load >= LOAD_STOP:
                print("DANGER: load exceeded LOAD_STOP. Stop test.")
                break

            next_print = now + 0.5

        time.sleep(0.05)

    print_health(bus, f"AFTER HOLD: {label}")


# ============================================================
# HELP / MAIN
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" HEXAPOD 6-LEG PUSH-UP LOAD TEST")
    print("===================================================")
    print("p             = print full motor status")
    print("health        = print compact health summary")
    print("ready         = move to READY_POSE")
    print("zero          = capture current pose as SESSION_READY")
    print("low           = lower body using all 6 legs")
    print("up            = return to SESSION_READY")
    print("cycle         = low -> hold -> up -> hold")
    print("hold 5        = hold current pose for 5 seconds with health logs")
    print("force_up      = force return to SESSION_READY without safety check")
    print("x             = clean exit")
    print("---------------------------------------------------")
    print("Current low pose offsets:")
    print(f"  BODY_LOW_FEMUR_DEG = {BODY_LOW_FEMUR_DEG:+.2f}")
    print(f"  BODY_LOW_TIBIA_DEG = {BODY_LOW_TIBIA_DEG:+.2f}")
    print("---------------------------------------------------")
    print("Recommended workflow:")
    print("  ready")
    print("  zero")
    print("  health")
    print("  low")
    print("  hold 5")
    print("  up")
    print("  health")
    print("  x")
    print("---------------------------------------------------")
    print("If low moves body upward instead of downward:")
    print("  edit BODY_LOW_FEMUR_DEG and BODY_LOW_TIBIA_DEG signs.")
    print("===================================================")


def main():
    global SESSION_READY, ACTIVE_GOALS

    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    clean_exit = False
    emergency_interrupt = False

    try:
        print()
        print("Moving to READY_POSE first...")
        action_ready(bus)

        SESSION_READY = capture_current_pose(bus, "SESSION_READY / ZERO")
        ACTIVE_GOALS = dict(SESSION_READY)

        print_help()

        while True:
            try:
                raw_cmd = input("\nPushUpTest command [h help]: ").strip()
            except KeyboardInterrupt:
                emergency_interrupt = True
                print()
                print("KeyboardInterrupt detected.")
                print("Emergency stop behavior: NOT auto-returning.")
                break

            if not raw_cmd:
                continue

            parts = raw_cmd.split()
            cmd = parts[0].lower()

            try:
                if cmd == "x":
                    clean_exit = True
                    print("Clean exit requested.")
                    break

                elif cmd == "h":
                    print_help()

                elif cmd == "p":
                    print_status(bus)

                elif cmd == "health":
                    print_health(bus, "MANUAL HEALTH CHECK")

                elif cmd == "ready":
                    action_ready(bus)

                elif cmd == "zero":
                    action_zero(bus)

                elif cmd == "low":
                    action_low(bus)

                elif cmd == "up":
                    action_up(bus)

                elif cmd == "force_up":
                    print()
                    print("FORCE_UP: returning to SESSION_READY without safety check.")
                    print("Physically support the robot before using this.")
                    action_up(bus, use_safety_check=False)

                elif cmd == "cycle":
                    action_cycle(bus)

                elif cmd == "hold":
                    if len(parts) != 2:
                        print("Usage: hold 5")
                        continue

                    hold_health(bus, float(parts[1]), "MANUAL HOLD")

                else:
                    print(f"Unknown command: {raw_cmd}")
                    print("Type h for help.")

            except ValueError:
                print("Invalid number format.")

            except KeyboardInterrupt:
                emergency_interrupt = True
                print()
                print("KeyboardInterrupt detected during command.")
                print("Emergency stop behavior: NOT auto-returning.")
                break

    finally:
        if clean_exit:
            try:
                print()
                print("Clean exit: returning to SESSION_READY before closing port.")
                action_up(bus, use_safety_check=False)
            except Exception as e:
                print(f"Could not return during clean exit: {type(e).__name__}: {e}")

        elif emergency_interrupt:
            print()
            print("Emergency interrupt exit.")
            print("No auto-return attempted.")
            print("If the robot is in a bad pose, support it, rerun script, then type force_up.")

        else:
            try:
                print()
                print("Normal ending: returning to SESSION_READY.")
                action_up(bus, use_safety_check=False)
            except Exception as e:
                print(f"Could not return during normal exit: {type(e).__name__}: {e}")

        bus.close()


if __name__ == "__main__":
    main()