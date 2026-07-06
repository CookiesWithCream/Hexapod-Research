# tools/calibration/calibrate_lift.py
#
# HEXAPOD LIFT A/B TIBIA CALIBRATION TOOL - FIXED VERSION
#
# Includes:
#   - ready
#   - zero
#   - liftA
#   - liftB
#   - leg FL / leg ML / etc.
#   - +5 / -5 raw tuning for selected tibia
#   - set 620 raw tuning for selected tibia
#   - nudge JOINT DEG
#   - id ID DEG
#   - print tuned liftA/liftB tibia coordinates
#
# Added:
#   - Automatic tuned LiftA tibia override:
#       FL_tibia ID 6  = 521
#       MR_tibia ID 11 = 477
#       RL_tibia ID 18 = 686
#
# Important:
#   Use:
#       leg ML
#       +5
#
#   Or:
#       nudge ML_tibia 5
#
#   Do NOT use:
#       leg ML_tibia 5

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
LIFT_SPEED = 16
TUNE_SPEED = 10
NUDGE_SPEED = 14

TEMP_WARN_C = 50
TEMP_STOP_C = 58

LOAD_WARN = 450
LOAD_STOP = 700

VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5


# ============================================================
# CURRENT DEFAULT STANCE
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

TRIPOD_A = ["FL", "MR", "RL"]
TRIPOD_B = ["FR", "ML", "RR"]
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
# DEFAULT LIFT SETTINGS
# ============================================================

LIFT_FEMUR_DEG = 20.0
LIFT_TIBIA_DEG = 18.0

SUPPORT_FEMUR_DEG = -10.0
SUPPORT_TIBIA_DEG = -8.0


# ============================================================
# MANUALLY TUNED LIFTA TIBIA RAW POSITIONS
# ============================================================
# Captured from your correct LiftA pose.
# These override the calculated +18 degree tibia lift for tripod A.

USE_TUNED_LIFTA_TIBIA = True

TUNED_LIFTA_TIBIA_RAW = {
    "FL": 521,   # ID 6  FL_tibia
    "MR": 477,   # ID 11 MR_tibia
    "RL": 686,   # ID 18 RL_tibia
}


# ============================================================
# RUNTIME STATE
# ============================================================

SESSION_READY: Dict[int, int] = dict(READY_POSE)
ACTIVE_GOALS: Dict[int, int] = dict(READY_POSE)

CURRENT_MODE = "READY"
CURRENT_SELECTED_LEG: Optional[str] = None

LIFTA_TIBIA_RAW: Dict[str, int] = {}
LIFTB_TIBIA_RAW: Dict[str, int] = {}


# ============================================================
# BASIC HELPERS
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


def tibia_id_for_leg(leg_name: str) -> int:
    tibia_joint = leg_part_to_joint(leg_name, "tibia")
    return joint_to_motor_id(tibia_joint)


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

    def move_one(self, motor_id: int, raw: int, speed: int):
        global ACTIVE_GOALS

        raw = clamp_raw(raw)

        self.enable_torque(motor_id)
        self.set_speed(motor_id, speed)
        time.sleep(0.01)

        ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)

        if not ok:
            time.sleep(0.04)
            self.write2(motor_id, ADDR_GOAL_POSITION, raw)

        ACTIVE_GOALS[motor_id] = raw

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
# STATUS / SAFETY
# ============================================================

def read_bus_health(bus: DynamixelBus) -> Tuple[int, float, int, bool]:
    max_temp = 0
    min_volt = 99.0
    max_abs_load = 0
    any_no_reply = False

    for motor_id in ALL_MOTOR_IDS:
        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
        volt_raw = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)

        if pos is None:
            any_no_reply = True

        if temp is not None:
            max_temp = max(max_temp, int(temp))

        if volt_raw is not None:
            min_volt = min(min_volt, volt_raw / 10.0)

        load_value = decode_load_value(load_raw)

        if load_value is not None:
            max_abs_load = max(max_abs_load, abs(load_value))

    return max_temp, min_volt, max_abs_load, any_no_reply


def pre_motion_check(bus: DynamixelBus) -> bool:
    max_temp, min_volt, max_abs_load, any_no_reply = read_bus_health(bus)

    if any_no_reply:
        print()
        print("[SAFETY STOP] At least one motor gave NO_REPLY. Movement blocked.")
        return False

    if max_temp >= TEMP_STOP_C:
        print()
        print(f"[SAFETY STOP] Max temperature is {max_temp}C. Movement blocked.")
        return False

    if min_volt <= VOLT_STOP_V:
        print()
        print(f"[SAFETY STOP] Minimum voltage is {min_volt:.1f}V. Movement blocked.")
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
    print(" MOTOR STATUS / LIFT TIBIA TUNER FIXED")
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

        selected_mark = ""

        if CURRENT_SELECTED_LEG is not None:
            selected_tibia_id = tibia_id_for_leg(CURRENT_SELECTED_LEG)

            if motor_id == selected_tibia_id:
                selected_mark = "  <== SELECTED"

        print(
            f"{motor_id:>2} {joint_name:<10} {leg_name:<2} {part_name:<5} "
            f"{raw:>4} {deg:>8.2f} {ready:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
            f"{selected_mark}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")
    print(f"Health: maxTemp={max_temp}C, minVolt={min_volt:.1f}V, maxAbsLoad={max_abs_load}")
    print(f"Current mode: {CURRENT_MODE}")
    print(f"Selected leg: {CURRENT_SELECTED_LEG}")


# ============================================================
# TARGET BUILDERS
# ============================================================

def build_joint_target(joint_name: str, deg: float) -> Tuple[int, int]:
    motor_id = joint_to_motor_id(joint_name)
    raw = raw_from_session_offset(joint_name, deg)
    return motor_id, raw


def build_leg_targets(
    leg: str,
    hip_deg: float = 0.0,
    femur_deg: float = 0.0,
    tibia_deg: float = 0.0,
) -> Dict[int, int]:
    targets = {}

    hip_joint = leg_part_to_joint(leg, "hip")
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")

    motor_id, raw = build_joint_target(hip_joint, hip_deg)
    targets[motor_id] = raw

    motor_id, raw = build_joint_target(femur_joint, femur_deg)
    targets[motor_id] = raw

    motor_id, raw = build_joint_target(tibia_joint, tibia_deg)
    targets[motor_id] = raw

    return targets


# ============================================================
# TUNED OUTPUT MEMORY
# ============================================================

def save_current_tibia_goals_from_mode():
    if CURRENT_MODE == "LIFTA":
        for leg in TRIPOD_A:
            motor_id = tibia_id_for_leg(leg)
            LIFTA_TIBIA_RAW[leg] = ACTIVE_GOALS.get(
                motor_id,
                SESSION_READY[motor_id],
            )

    elif CURRENT_MODE == "LIFTB":
        for leg in TRIPOD_B:
            motor_id = tibia_id_for_leg(leg)
            LIFTB_TIBIA_RAW[leg] = ACTIVE_GOALS.get(
                motor_id,
                SESSION_READY[motor_id],
            )


# ============================================================
# MAIN ACTIONS
# ============================================================

def action_ready(bus: DynamixelBus):
    global ACTIVE_GOALS, CURRENT_MODE, CURRENT_SELECTED_LEG

    if not pre_motion_check(bus):
        return

    print()
    print("ACTION: READY_POSE")

    ACTIVE_GOALS = dict(READY_POSE)
    CURRENT_MODE = "READY"
    CURRENT_SELECTED_LEG = None

    bus.move_many(dict(READY_POSE), speed=READY_SPEED)
    time.sleep(1.0)
    print_status(bus)


def action_zero(bus: DynamixelBus):
    global SESSION_READY, ACTIVE_GOALS

    SESSION_READY = capture_current_pose(bus, "SESSION_READY / ZERO")
    ACTIVE_GOALS = dict(SESSION_READY)

    print()
    print("New SESSION_READY captured.")
    print_status(bus)


def action_return_session(bus: DynamixelBus, use_safety_check: bool = True):
    global ACTIVE_GOALS, CURRENT_MODE, CURRENT_SELECTED_LEG

    if use_safety_check:
        if not pre_motion_check(bus):
            return

    print()
    print("ACTION: RETURN TO SESSION_READY")

    ACTIVE_GOALS = dict(SESSION_READY)
    CURRENT_MODE = "READY"
    CURRENT_SELECTED_LEG = None

    bus.move_many(dict(SESSION_READY), speed=READY_SPEED)
    time.sleep(0.9)
    print_status(bus)


def action_lift_tripod(bus: DynamixelBus, tripod_name: str):
    global CURRENT_MODE, CURRENT_SELECTED_LEG

    if not pre_motion_check(bus):
        return

    tripod_name = tripod_name.upper()

    if tripod_name == "A":
        lift_legs = TRIPOD_A
        support_legs = TRIPOD_B
        CURRENT_MODE = "LIFTA"
        CURRENT_SELECTED_LEG = "FL"

    elif tripod_name == "B":
        lift_legs = TRIPOD_B
        support_legs = TRIPOD_A
        CURRENT_MODE = "LIFTB"
        CURRENT_SELECTED_LEG = "FR"

    else:
        print("Unknown tripod. Use A or B.")
        return

    targets = {}

    # ========================================================
    # LIFTED TRIPOD
    # ========================================================
    for leg in lift_legs:
        leg_targets = build_leg_targets(
            leg,
            hip_deg=0.0,
            femur_deg=LIFT_FEMUR_DEG,
            tibia_deg=LIFT_TIBIA_DEG,
        )

        # ----------------------------------------------------
        # Override LiftA tibia with manually tuned raw values.
        # This only changes lifted Tripod A tibias:
        #   FL_tibia, MR_tibia, RL_tibia
        # ----------------------------------------------------
        if tripod_name == "A" and USE_TUNED_LIFTA_TIBIA:
            if leg in TUNED_LIFTA_TIBIA_RAW:
                tibia_id = tibia_id_for_leg(leg)
                leg_targets[tibia_id] = TUNED_LIFTA_TIBIA_RAW[leg]

        targets.update(leg_targets)

    # ========================================================
    # SUPPORT TRIPOD
    # ========================================================
    for leg in support_legs:
        targets.update(
            build_leg_targets(
                leg,
                hip_deg=0.0,
                femur_deg=SUPPORT_FEMUR_DEG,
                tibia_deg=SUPPORT_TIBIA_DEG,
            )
        )

    print()
    print("===================================================")
    print(f" ACTION: LIFT TRIPOD {tripod_name} / TIBIA TUNE MODE")
    print("===================================================")
    print(f"Lift legs    : {lift_legs}")
    print(f"Support legs : {support_legs}")
    print(f"Lift femur   : {LIFT_FEMUR_DEG:+.1f} deg")
    print(f"Lift tibia   : {LIFT_TIBIA_DEG:+.1f} deg")

    if tripod_name == "A" and USE_TUNED_LIFTA_TIBIA:
        print("LiftA tibia  : USING MANUAL RAW OVERRIDE")
        for leg in TRIPOD_A:
            tibia_id = tibia_id_for_leg(leg)
            raw = TUNED_LIFTA_TIBIA_RAW[leg]
            print(f"  {leg}_tibia ID {tibia_id}: raw {raw}")

    print(f"Support femur: {SUPPORT_FEMUR_DEG:+.1f} deg")
    print(f"Support tibia: {SUPPORT_TIBIA_DEG:+.1f} deg")
    print()
    print("Tune examples:")
    print("  leg FL")
    print("  +5")
    print("  -5")
    print("  set 620")
    print("  nudge FL_tibia 3")
    print("  id 6 -2")
    print("  print")
    print("===================================================")

    bus.move_many(targets, speed=LIFT_SPEED)
    time.sleep(0.8)

    save_current_tibia_goals_from_mode()
    print_status(bus)


# ============================================================
# TUNING FUNCTIONS
# ============================================================

def select_leg(leg_name: str):
    global CURRENT_SELECTED_LEG

    leg_name = leg_name.upper()

    if leg_name not in ALL_LEGS:
        print(f"Unknown leg: {leg_name}")
        print(f"Valid legs: {ALL_LEGS}")
        print("Use leg names only, for example: leg ML")
        print("Do NOT use: leg ML_tibia 5")
        print("For joint tuning, use: nudge ML_tibia 5")
        return

    if CURRENT_MODE == "LIFTA" and leg_name not in TRIPOD_A:
        print()
        print(f"Warning: You are in LIFTA mode.")
        print(f"Tripod A lifted legs are: {TRIPOD_A}")
        print("Normally tune only the lifted tripod tibias.")

    if CURRENT_MODE == "LIFTB" and leg_name not in TRIPOD_B:
        print()
        print(f"Warning: You are in LIFTB mode.")
        print(f"Tripod B lifted legs are: {TRIPOD_B}")
        print("Normally tune only the lifted tripod tibias.")

    CURRENT_SELECTED_LEG = leg_name

    motor_id = tibia_id_for_leg(leg_name)
    joint = motor_id_to_joint(motor_id)
    goal = ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id])

    print()
    print(f"Selected leg: {leg_name}")
    print(f"Tibia motor : ID {motor_id} / {joint}")
    print(f"Current goal: {goal}")


def tune_selected_tibia_by_raw(bus: DynamixelBus, delta_raw: int):
    if CURRENT_SELECTED_LEG is None:
        print("No selected leg. Use: leg FL")
        return

    if not pre_motion_check(bus):
        return

    motor_id = tibia_id_for_leg(CURRENT_SELECTED_LEG)
    old_raw = ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id])
    new_raw = clamp_raw(old_raw + delta_raw)

    print()
    print(
        f"TUNE {CURRENT_SELECTED_LEG} tibia "
        f"ID {motor_id}: {old_raw} -> {new_raw} "
        f"delta {delta_raw:+d} raw"
    )

    bus.move_one(motor_id, new_raw, speed=TUNE_SPEED)
    time.sleep(0.25)

    save_current_tibia_goals_from_mode()
    print_selected_tibia_status(bus)


def set_selected_tibia_raw(bus: DynamixelBus, raw: int):
    if CURRENT_SELECTED_LEG is None:
        print("No selected leg. Use: leg FL")
        return

    if not pre_motion_check(bus):
        return

    motor_id = tibia_id_for_leg(CURRENT_SELECTED_LEG)
    raw = clamp_raw(raw)

    old_raw = ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id])

    print()
    print(
        f"SET {CURRENT_SELECTED_LEG} tibia "
        f"ID {motor_id}: {old_raw} -> {raw}"
    )

    bus.move_one(motor_id, raw, speed=TUNE_SPEED)
    time.sleep(0.25)

    save_current_tibia_goals_from_mode()
    print_selected_tibia_status(bus)


def nudge_joint(bus: DynamixelBus, joint_name: str, deg: float):
    global ACTIVE_GOALS

    if joint_name not in JOINT_INFO:
        print(f"Unknown joint: {joint_name}")
        print("Examples:")
        print("  nudge FL_tibia 3")
        print("  nudge ML_tibia -2")
        print("  nudge RR_femur 1")
        return

    if not pre_motion_check(bus):
        return

    motor_id = joint_to_motor_id(joint_name)

    current_goal = ACTIVE_GOALS.get(motor_id)

    if current_goal is None:
        current_goal = bus.read2(motor_id, ADDR_PRESENT_POSITION)

        if current_goal is None:
            current_goal = SESSION_READY.get(motor_id, READY_POSE[motor_id])

    raw_delta = logical_deg_to_raw_delta(joint_name, deg)
    new_goal = clamp_raw(current_goal + raw_delta)

    ACTIVE_GOALS[motor_id] = new_goal

    print()
    print(
        f"NUDGE {joint_name} ID {motor_id}: "
        f"{deg:+.2f} deg | raw {current_goal} -> {new_goal}"
    )

    bus.move_one(motor_id, new_goal, speed=NUDGE_SPEED)
    time.sleep(0.30)

    save_current_tibia_goals_from_mode()
    print_status(bus)


def nudge_id(bus: DynamixelBus, motor_id: int, deg: float):
    joint_name = motor_id_to_joint(motor_id)

    if joint_name == "UNKNOWN":
        print(f"Unknown motor ID: {motor_id}")
        return

    nudge_joint(bus, joint_name, deg)


def print_selected_tibia_status(bus: DynamixelBus):
    if CURRENT_SELECTED_LEG is None:
        print("No selected leg.")
        return

    motor_id = tibia_id_for_leg(CURRENT_SELECTED_LEG)
    joint_name = motor_id_to_joint(motor_id)

    raw = bus.read2(motor_id, ADDR_PRESENT_POSITION)
    goal = ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id])
    ready = SESSION_READY[motor_id]

    if raw is None:
        raw_text = "NO_REPLY"
        deg_text = "----"
    else:
        raw_text = str(raw)
        deg = raw_delta_to_logical_deg(joint_name, raw - ready)
        deg_text = f"{deg:+.2f}"

    print()
    print("---------------------------------------------------")
    print(f"Selected leg : {CURRENT_SELECTED_LEG}")
    print(f"Tibia joint  : {joint_name}")
    print(f"Motor ID     : {motor_id}")
    print(f"Ready raw    : {ready}")
    print(f"Goal raw     : {goal}")
    print(f"Present raw  : {raw_text}")
    print(f"Deg from zero: {deg_text}")
    print("---------------------------------------------------")


# ============================================================
# OUTPUT PRINTING
# ============================================================

def print_tuned_output():
    print()
    print("===================================================")
    print(" COPY THIS OUTPUT")
    print("===================================================")

    if CURRENT_MODE == "LIFTA":
        save_current_tibia_goals_from_mode()

        print("# Tuned LiftA tibia raw positions")
        print("LIFTA_TIBIA_RAW = {")
        for leg in TRIPOD_A:
            motor_id = tibia_id_for_leg(leg)
            raw = LIFTA_TIBIA_RAW.get(
                leg,
                ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id]),
            )
            print(f'    "{leg}": {raw},   # ID {motor_id} {motor_id_to_joint(motor_id)}')
        print("}")

        print()
        print("# LiftA full tibia motor raw dictionary")
        print("LIFTA_TIBIA_MOTOR_RAW = {")
        for leg in TRIPOD_A:
            motor_id = tibia_id_for_leg(leg)
            raw = LIFTA_TIBIA_RAW.get(
                leg,
                ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id]),
            )
            print(f"    {motor_id}: {raw},   # {leg}_tibia")
        print("}")

    elif CURRENT_MODE == "LIFTB":
        save_current_tibia_goals_from_mode()

        print("# Tuned LiftB tibia raw positions")
        print("LIFTB_TIBIA_RAW = {")
        for leg in TRIPOD_B:
            motor_id = tibia_id_for_leg(leg)
            raw = LIFTB_TIBIA_RAW.get(
                leg,
                ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id]),
            )
            print(f'    "{leg}": {raw},   # ID {motor_id} {motor_id_to_joint(motor_id)}')
        print("}")

        print()
        print("# LiftB full tibia motor raw dictionary")
        print("LIFTB_TIBIA_MOTOR_RAW = {")
        for leg in TRIPOD_B:
            motor_id = tibia_id_for_leg(leg)
            raw = LIFTB_TIBIA_RAW.get(
                leg,
                ACTIVE_GOALS.get(motor_id, SESSION_READY[motor_id]),
            )
            print(f"    {motor_id}: {raw},   # {leg}_tibia")
        print("}")

    else:
        print("You are not in liftA or liftB mode.")
        print("Use liftA or liftB first, tune tibias, then type print.")

    print("===================================================")


# ============================================================
# HELP
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" HEXAPOD LIFT TIBIA TUNER - FIXED")
    print("===================================================")
    print("p                   = print full motor status")
    print("ready               = move to default READY_POSE")
    print("zero                = capture current pose as SESSION_READY")
    print("r                   = return to SESSION_READY")
    print("liftA               = go to tripod A lift position")
    print("liftB               = go to tripod B lift position")
    print("leg FL              = select FL tibia")
    print("leg MR              = select MR tibia")
    print("leg RL              = select RL tibia")
    print("leg FR              = select FR tibia")
    print("leg ML              = select ML tibia")
    print("leg RR              = select RR tibia")
    print("+5                  = increase selected tibia by +5 raw")
    print("-5                  = decrease selected tibia by -5 raw")
    print("+10                 = increase selected tibia by +10 raw")
    print("-10                 = decrease selected tibia by -10 raw")
    print("set 620             = set selected tibia raw position directly")
    print("nudge JOINT DEG     = nudge any joint by degree amount")
    print("id ID DEG           = nudge motor ID by degree amount")
    print("show                = show selected tibia status")
    print("print               = print tuned liftA/liftB tibia coordinates")
    print("x                   = clean exit and return to SESSION_READY")
    print("---------------------------------------------------")
    print("Important command difference:")
    print("  leg ML             CORRECT: selects ML leg tibia")
    print("  leg ML_tibia 5     WRONG")
    print("  nudge ML_tibia 5   CORRECT: nudges ML_tibia by +5 degrees")
    print("  id 12 5            CORRECT: nudges motor ID 12 by +5 degrees")
    print("---------------------------------------------------")
    print("LiftA tibia IDs:")
    print("  FL_tibia = ID 6")
    print("  MR_tibia = ID 11")
    print("  RL_tibia = ID 18")
    print()
    print("LiftB tibia IDs:")
    print("  FR_tibia = ID 5")
    print("  ML_tibia = ID 12")
    print("  RR_tibia = ID 17")
    print("---------------------------------------------------")
    print("Tuned LiftA auto override:")
    print(f"  Enabled: {USE_TUNED_LIFTA_TIBIA}")
    print("  FL_tibia ID 6  = 521")
    print("  MR_tibia ID 11 = 477")
    print("  RL_tibia ID 18 = 686")
    print("---------------------------------------------------")
    print("Recommended LiftA test:")
    print("  ready")
    print("  zero")
    print("  liftA")
    print("  print")
    print("  r")
    print()
    print("Recommended LiftB workflow:")
    print("  liftB")
    print("  nudge FR_tibia 3")
    print("  nudge ML_tibia -2")
    print("  nudge RR_tibia 4")
    print("  print")
    print("  r")
    print("---------------------------------------------------")
    print("Raw selected-leg tuning:")
    print("  leg ML")
    print("  +5")
    print("  -5")
    print("  set 640")
    print()
    print("Degree motor-ID tuning:")
    print("  id 5 2")
    print("  id 12 -2")
    print("  id 17 3")
    print("===================================================")


# ============================================================
# MAIN
# ============================================================

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
                raw_cmd = input("\nTibiaTuner command [h help]: ").strip()
            except KeyboardInterrupt:
                emergency_interrupt = True
                print()
                print("KeyboardInterrupt detected.")
                print("Emergency stop behavior: NOT auto-returning to SESSION_READY.")
                print("Use Ctrl+C only when you intentionally want to stop immediately.")
                break

            if not raw_cmd:
                continue

            parts = raw_cmd.split()
            cmd = parts[0].lower()

            try:
                if cmd == "x":
                    print("Clean exit requested.")
                    clean_exit = True
                    break

                elif cmd == "h":
                    print_help()

                elif cmd == "p":
                    print_status(bus)

                elif cmd == "ready":
                    action_ready(bus)

                elif cmd == "zero":
                    action_zero(bus)

                elif cmd == "r":
                    action_return_session(bus)

                elif cmd == "lifta":
                    action_lift_tripod(bus, "A")

                elif cmd == "liftb":
                    action_lift_tripod(bus, "B")

                elif cmd == "leg":
                    if len(parts) != 2:
                        print("Usage: leg FL")
                        print("Example: leg ML")
                        print("Do NOT use: leg ML_tibia 5")
                        print("For degree tuning use: nudge ML_tibia 5")
                        continue

                    select_leg(parts[1])

                elif cmd in ["show", "s"]:
                    print_selected_tibia_status(bus)

                elif cmd == "set":
                    if len(parts) != 2:
                        print("Usage: set 620")
                        continue

                    set_selected_tibia_raw(bus, int(parts[1]))

                elif cmd == "nudge":
                    if len(parts) != 3:
                        print("Usage: nudge JOINT DEG")
                        print("Example: nudge ML_tibia 5")
                        continue

                    nudge_joint(bus, parts[1], float(parts[2]))

                elif cmd == "id":
                    if len(parts) != 3:
                        print("Usage: id ID DEG")
                        print("Example: id 12 5")
                        continue

                    nudge_id(bus, int(parts[1]), float(parts[2]))

                elif cmd == "print":
                    print_tuned_output()

                elif cmd.startswith("+") or cmd.startswith("-"):
                    delta = int(cmd)
                    tune_selected_tibia_by_raw(bus, delta)

                else:
                    print(f"Unknown command: {raw_cmd}")
                    print("Type h for help.")

            except ValueError:
                print("Invalid number format.")

            except KeyboardInterrupt:
                emergency_interrupt = True
                print()
                print("KeyboardInterrupt detected during command.")
                print("Emergency stop behavior: NOT auto-returning to SESSION_READY.")
                break

    finally:
        if clean_exit:
            try:
                print()
                print("Clean exit: returning to SESSION_READY before closing port.")
                action_return_session(bus, use_safety_check=False)
            except KeyboardInterrupt:
                print()
                print("Interrupted during clean return. Closing port now.")
            except Exception as e:
                print()
                print(f"Could not return to SESSION_READY during exit: {type(e).__name__}: {e}")

        elif emergency_interrupt:
            print()
            print("Emergency interrupt exit.")
            print("No auto-return was attempted to avoid another Ctrl+C traceback.")
            print("If the robot is still in a lifted pose, run the script again and type:")
            print("  ready")
            print("or:")
            print("  r")

        else:
            try:
                print()
                print("Normal script ending: returning to SESSION_READY.")
                action_return_session(bus, use_safety_check=False)
            except Exception as e:
                print()
                print(f"Could not return to SESSION_READY: {type(e).__name__}: {e}")

        bus.close()


if __name__ == "__main__":
    main()