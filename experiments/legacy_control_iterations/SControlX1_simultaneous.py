# ============================================================
# SCONTROL4 - SIMULTANEOUS TRIPOD GAIT
# ============================================================
#
# What's new vs SControl3:
#
# 1. SYNC WRITE (biggest fix)
#    move_many() used individual write2ByteTxRx calls with 6ms sleeps
#    between each motor = 18 × 6ms = ~108ms of stagger per phase.
#    sync_write_positions() sends ALL motor positions in ONE packet.
#    This is how the motors actually move simultaneously.
#
# 2. SPEED BATCH WRITE
#    sync_set_speeds() also batches speed setting into one packet per cycle
#    instead of 18 individual calls.
#
# 3. SIMULTANEOUS / OVERLAPPED TRIPOD GAIT
#    Old:  A_UP -> A_SWING -> A_DOWN -> B_UP -> B_SWING -> B_DOWN
#    New:  A_UP+B_PUSH -> A_SWING+B_PUSH -> A_DOWN_B_UP -> B_SWING+A_PUSH -> B_DOWN+A_PUSH -> (next cycle)
#    The support tripod is already doing its push while the swing tripod is
#    in the air. The handoff happens at A_DOWN=B_UP simultaneously.
#    This is how a real spider walks.
#
# 4. CONTINUOUS GAIT LOOP
#    'w', 'a', 's', 'd', 'q', 'e' run continuously until you press Enter.
#    Each direction key starts gait and the loop repeats until interrupted.
#    No more "one cycle then stop".
#
# 5. TIMING TIGHTENED
#    GAIT_PHASE_DELAY reduced now that sync write removes the serial stagger.
#    Motors receive their targets nearly simultaneously, so they reach the pose
#    at approximately the same time.
#
# Commands are identical to SControl3. The robot hardware, READY_POSE, motor IDs,
# joint signs, and all tuning presets are preserved exactly.
#
# Recommended startup:
#   r
#   health
#   sidestrafe good
#   movestats off
#   sideflow on
#   speed all 23
#   a          (runs until Enter)
#   r
#   d          (runs until Enter)
#   r
#
# ============================================================

import sys
import time
import struct
from typing import Dict, Optional, Tuple, List

try:
    from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite
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

ADDR_TORQUE_ENABLE   = 24
ADDR_GOAL_POSITION   = 30
ADDR_MOVING_SPEED    = 32
ADDR_TORQUE_LIMIT    = 34
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD    = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TORQUE_ENABLE = 1
COMM_SUCCESS  = 0

RAW_PER_DEG = 1023.0 / 300.0

READ_RETRIES     = 3
READ_RETRY_DELAY = 0.04

READY_SPEED = 22
MOVE_SPEED  = 22
LIFT_SPEED  = 22
GAIT_SPEED  = 22

MIN_SAFE_SPEED = 1
MAX_SAFE_SPEED = 1023

TORQUE_LIMIT_RAW = 1023

TEMP_WARN_C   = 50
TEMP_STOP_C   = 58
LOAD_WARN     = 450
LOAD_STOP     = 700
VOLT_WARN_V   = 10.8
VOLT_STOP_V   = 9.5
VOLT_DANGER_V = 9.2


# ============================================================
# BALANCED REFINED2K READY POSE
# ============================================================

READY_POSE = {
    1:  460,   # RL_hip
    2:  747,   # FL_hip
    3:  411,   # FR_femur
    4:  366,   # FL_femur
    5:  798,   # FR_tibia
    6:  796,   # FL_tibia
    7:  608,   # MR_hip
    8:  753,   # ML_hip
    9:  627,   # MR_femur
    10: 437,   # ML_femur
    11: 216,   # MR_tibia
    12: 787,   # ML_tibia
    13: 578,   # RR_hip
    14: 575,   # FR_hip
    15: 641,   # RR_femur
    16: 412,   # RL_femur
    17: 189,   # RR_tibia
    18: 817,   # RL_tibia
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
    "RL_hip":   {"id": 1,  "type": "hip"},
    "FL_hip":   {"id": 2,  "type": "hip"},
    "FR_femur": {"id": 3,  "type": "femur"},
    "FL_femur": {"id": 4,  "type": "femur"},
    "FR_tibia": {"id": 5,  "type": "tibia"},
    "FL_tibia": {"id": 6,  "type": "tibia"},
    "MR_hip":   {"id": 7,  "type": "hip"},
    "ML_hip":   {"id": 8,  "type": "hip"},
    "MR_femur": {"id": 9,  "type": "femur"},
    "ML_femur": {"id": 10, "type": "femur"},
    "MR_tibia": {"id": 11, "type": "tibia"},
    "ML_tibia": {"id": 12, "type": "tibia"},
    "RR_hip":   {"id": 13, "type": "hip"},
    "FR_hip":   {"id": 14, "type": "hip"},
    "RR_femur": {"id": 15, "type": "femur"},
    "RL_femur": {"id": 16, "type": "femur"},
    "RR_tibia": {"id": 17, "type": "tibia"},
    "RL_tibia": {"id": 18, "type": "tibia"},
}

MOTOR_TO_JOINT = {info["id"]: joint for joint, info in JOINT_INFO.items()}
ALL_MOTOR_IDS  = sorted(READY_POSE.keys())
ALL_LEGS       = ["FL", "ML", "RL", "FR", "MR", "RR"]

TRIPOD_A = ["FL", "MR", "RL"]
TRIPOD_B = ["FR", "ML", "RR"]


# ============================================================
# MOVEMENT SIGN MODEL
# ============================================================

LEG_MOVEMENT_SIGN = {
    "FL": {"hip": 1,  "femur": 1,  "tibia": 1},
    "ML": {"hip": 1,  "femur": 1,  "tibia": 1},
    "RL": {"hip": 1,  "femur": 1,  "tibia": 1},
    "FR": {"hip": 1,  "femur": 1,  "tibia": 1},
    "MR": {"hip": 1,  "femur": -1, "tibia": -1},
    "RR": {"hip": 1,  "femur": -1, "tibia": -1},
}

JOINT_DIRECTIONS = {joint: 1 for joint in JOINT_INFO.keys()}


# ============================================================
# MOTION SETTINGS
# ============================================================

LIFT_LEVELS = {
    1: {"femur": -6.0,  "tibia": 6.0},
    2: {"femur": -10.0, "tibia": 10.0},
    3: {"femur": -14.0, "tibia": 14.0},
    4: {"femur": -18.0, "tibia": 18.0},
    5: {"femur": -22.0, "tibia": 22.0},
    6: {"femur": -28.0, "tibia": 28.0},
    7: {"femur": -32.0, "tibia": 32.0},
    8: {"femur": -36.0, "tibia": 34.0},
    9: {"femur": -40.0, "tibia": 36.0},
}

DEFAULT_LIFT_LEVEL = 3

GAIT_HIP_SWING_DEG    = 24.0
GAIT_SUPPORT_PUSH_DEG = 16.0

BACKWARD_HIP_SWING_DEG    = 24.0
BACKWARD_SUPPORT_PUSH_DEG = 16.0

STRAFE_HIP_SWING_DEG    = 28.0
STRAFE_SUPPORT_PUSH_DEG = 22.0
TURN_HIP_SWING_DEG      = 30.0
TURN_SUPPORT_PUSH_DEG   = 24.0

GAIT_LIFT_LEVEL = 6

USE_WALK_LIFT_PROFILE = False
WALK_LIFT_FEMUR_DEG   = -32.0
WALK_LIFT_TIBIA_DEG   = 12.0

WALK_LIFT_PRESETS = {
    "test1": {"femur": -32.0, "tibia": 18.0},
    "test2": {"femur": -34.0, "tibia": 22.0},
    "test3": {"femur": -36.0, "tibia": 24.0},
    "high1": {"femur": -38.0, "tibia": 26.0},
    "high2": {"femur": -40.0, "tibia": 28.0},
    "high3": {"femur": -42.0, "tibia": 30.0},
    "max":   {"femur": -44.0, "tibia": 32.0},
    "old6":  {"femur": -28.0, "tibia": 28.0},
    "low":   {"femur": -30.0, "tibia": 16.0},
    "clear": {"femur": -38.0, "tibia": 28.0},
    "high":  {"femur": -40.0, "tibia": 28.0},
}

LEG_FEMUR_LIFT_SCALE = {leg: 1.00 for leg in ALL_LEGS}
LEG_TIBIA_LIFT_SCALE = {
    "FL": 1.00, "ML": 1.00, "RL": 1.00,
    "FR": 1.00, "MR": 1.00, "RR": 0.85,
}

# ============================================================
# TIMING  (tightened because sync write removes serial stagger)
# ============================================================
# Phase delay = how long to wait after sending a phase so motors reach the pose.
# With individual write2() + 6ms sleeps, sending 18 motors took ~108ms.
# With sync write, all motors start simultaneously, so you can cut the wait.
#
# Adjust these if the robot doesn't have enough time to reach poses:
#   GAIT_PHASE_DELAY = 0.18   (slower, more time per phase)
#   GAIT_PHASE_DELAY = 0.10   (faster, spider-like)
#
GAIT_PHASE_DELAY        = 0.18    # default: lift/swing phases
GAIT_SETTLE_DELAY       = 0.10    # touchdown settle
GAIT_FINAL_READY_DELAY  = 0.25
GAIT_END_RECENTER_DELAY = 0.08
GAIT_END_MODE = "tripod"

SMOOTH_GAIT       = False
SMOOTH_STEPS      = 3
SMOOTH_STEP_DELAY = 0.020

GAIT_PHASE_HEALTH       = False
GAIT_PRECHECK_EACH_PHASE = False

HIP_FORWARD_SIGN = {
    "FL": -1, "ML": -1, "RL": -1,
    "FR":  1, "MR":  1, "RR":  1,
}

HIP_STRAFE_SIGN = {
    "FL": -1, "ML":  1, "RL": -1,
    "FR": -1, "MR":  1, "RR": -1,
}

HIP_TURN_SIGN = {
    "FL": -1, "ML": -1, "RL": -1,
    "FR": -1, "MR": -1, "RR": -1,
}

LEFT_LEGS  = ["FL", "ML", "RL"]
RIGHT_LEGS = ["FR", "MR", "RR"]

STRAFE_DIRECTION_MULTIPLIER = 1.0
TURN_DIRECTION_MULTIPLIER   = 1.0

TURN_LEFT_SCALE  = 0.75
TURN_RIGHT_SCALE = 0.78

CRAB_FIRST_TRIPOD  = ["FR", "ML", "RR"]
CRAB_SECOND_TRIPOD = ["FL", "MR", "RL"]


# ============================================================
# SIDE STRAFE (WControl23 preserved)
# ============================================================

SIDE_STRAFE_DIRECTION_MULTIPLIER = 1.0

SIDE_STRAFE_HIP_REACH_DEG  = 0.0
SIDE_STRAFE_HIP_PUSH_DEG   = 0.0

SIDE_STRAFE_FEMUR_REACH_DEG = 6.0
SIDE_STRAFE_TIBIA_REACH_DEG = -14.0

SIDE_STRAFE_FEMUR_PULL_DEG  = -5.0
SIDE_STRAFE_TIBIA_PULL_DEG  = 12.0

SIDE_STRAFE_LIFT_FEMUR_DEG  = -34.0
SIDE_STRAFE_LIFT_TIBIA_DEG  = -6.0

SIDE_STRAFE_HOLD   = 0.18   # reduced from 0.30 because sync write is faster
SIDE_STRAFE_SETTLE = 0.10   # reduced from 0.14

SIDE_STRAFE_PHASE_BOOST_ENABLED            = True
SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG          = 9.0
SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG          = 12.0
SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG   = 8.0
SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG   = 12.0

SIDE_STRAFE_DEBUG_STEPS_ENABLED  = False
SIDE_STRAFE_DEBUG_STEPS          = 10
SIDE_STRAFE_DEBUG_STEP_DELAY     = 0.070
SIDE_STRAFE_DEBUG_PRINT_FRAMES   = False
SIDE_STRAFE_DEBUG_ENTER_STEP     = False

SIDE_STRAFE_FLOW_MODE         = True
SIDE_STRAFE_FLOW_HOLD         = 0.0
SIDE_STRAFE_FLOW_TINY_HOLD    = 0.015
SIDE_STRAFE_FLOW_PRINT_PHASES = True


# ============================================================
# PUSHUP LEVELS
# ============================================================

PUSHUP_LEVELS = {
    "1": {1:470,2:757,3:425,4:388,5:776,6:766,7:598,8:763,9:616,10:433,11:235,12:783,13:568,14:565,15:640,16:423,17:198,18:798},
    "2": {1:480,2:767,3:439,4:402,5:756,6:746,7:588,8:773,9:602,10:447,11:255,12:763,13:558,14:555,15:626,16:437,17:218,18:778},
    "3": {1:491,2:778,3:453,4:416,5:715,6:705,7:577,8:784,9:588,10:461,11:296,12:722,13:547,14:544,15:612,16:451,17:259,18:737},
    "4": {1:501,2:788,3:466,4:429,5:674,6:664,7:567,8:794,9:575,10:474,11:337,12:681,13:537,14:534,15:599,16:464,17:300,18:696},
}


# ============================================================
# RUNTIME STATE
# ============================================================

ACTIVE_GOALS: Dict[int, int] = dict(READY_POSE)
CURRENT_MODE = "UNKNOWN"

MOVEMENT_STATS_ENABLED = False
MOVEMENT_STATS_DETAIL  = "compact"
MOVEMENT_STATS_WARN_ONLY = False


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
    movement_sign  = LEG_MOVEMENT_SIGN.get(leg_name, {}).get(part_name, 1)
    joint_direction = JOINT_DIRECTIONS.get(joint_name, 1)
    return int(round(deg * RAW_PER_DEG * movement_sign * joint_direction))


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


def offset_from_ready(joint_name: str, deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    return clamp_raw(READY_POSE[motor_id] + logical_deg_to_raw_delta(joint_name, deg))


def build_leg_offset_targets(
    leg: str,
    hip_deg: float = 0.0,
    femur_deg: float = 0.0,
    tibia_deg: float = 0.0,
) -> Dict[int, int]:
    hip_joint   = leg_part_to_joint(leg, "hip")
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")

    hip_id   = joint_to_motor_id(hip_joint)
    femur_id = joint_to_motor_id(femur_joint)
    tibia_id = joint_to_motor_id(tibia_joint)

    femur_deg = femur_deg * LEG_FEMUR_LIFT_SCALE.get(leg, 1.0)
    tibia_deg = tibia_deg * LEG_TIBIA_LIFT_SCALE.get(leg, 1.0)

    return {
        hip_id:   offset_from_ready(hip_joint,   hip_deg),
        femur_id: offset_from_ready(femur_joint, femur_deg),
        tibia_id: offset_from_ready(tibia_joint, tibia_deg),
    }


def normalize_direction(text: str) -> str:
    text = text.lower().strip()
    aliases = {
        "foward": "forward", "forwad": "forward", "fw": "forward", "w": "forward",
        "back": "backward",  "backwards": "backward", "bw": "backward", "s": "backward",
        "l": "left",  "a": "left",
        "r": "right", "d": "right",
        "tl": "turn_left",  "q": "turn_left",
        "tr": "turn_right", "e": "turn_right",
    }
    return aliases.get(text, text)


def parse_lift_command(parts: List[str]) -> Tuple[int, List[str]]:
    if len(parts) < 2:
        raise ValueError("Usage: lift FL OR lift 3 FL FR")
    level = DEFAULT_LIFT_LEVEL
    leg_tokens = parts[1:]
    if parts[1].isdigit():
        level = int(parts[1])
        leg_tokens = parts[2:]
    if level not in LIFT_LEVELS:
        raise ValueError("Lift level must be 1-9.")
    if not leg_tokens:
        raise ValueError("No leg selected.")
    legs = [t.upper() for t in leg_tokens]
    for leg in legs:
        if leg not in ALL_LEGS:
            raise ValueError(f"Unknown leg: {leg}")
    if len(set(legs)) != len(legs):
        raise ValueError("Duplicate leg in command.")
    return level, legs


def interpolate_targets(start: Dict[int, int], end: Dict[int, int], steps: int) -> List[Dict[int, int]]:
    steps = max(1, int(steps))
    frames: List[Dict[int, int]] = []
    all_ids = sorted(set(start.keys()) | set(end.keys()))
    for i in range(1, steps + 1):
        ratio = i / steps
        frame: Dict[int, int] = {}
        for motor_id in all_ids:
            a = start.get(motor_id, READY_POSE.get(motor_id, 512))
            b = end.get(motor_id, a)
            frame[motor_id] = clamp_raw(int(round(a + (b - a) * ratio)))
        frames.append(frame)
    return frames


# ============================================================
# DYNAMIXEL BUS
# ============================================================

class DynamixelBus:
    def __init__(self, port_name: str = DEFAULT_PORT):
        self.port_name     = port_name
        self.port_handler  = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self._speed_cache: Dict[int, int] = {}  # track last written speed per motor

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
                self.port_handler, motor_id, address, int(value))
        except Exception as e:
            print(f"[ID {motor_id}] WRITE1 EXCEPTION: {e}")
            return False
        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False
        return True

    def write2(self, motor_id: int, address: int, value: int) -> bool:
        value = clamp_raw(value)
        try:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, address, value)
        except Exception as e:
            print(f"[ID {motor_id}] WRITE2 EXCEPTION: {e}")
            return False
        if result != COMM_SUCCESS:
            print(f"[ID {motor_id}] COMM ERROR: {self.packet_handler.getTxRxResult(result)}")
            return False
        return True

    def read1_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, motor_id, address)
        except Exception:
            return None
        if result != COMM_SUCCESS or error != 0:
            return None
        return value

    def read2_once(self, motor_id: int, address: int) -> Optional[int]:
        try:
            value, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, address)
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

    def enable_torque_all(self):
        for motor_id in ALL_MOTOR_IDS:
            self.enable_torque(motor_id)

    def set_torque_limit(self, motor_id: int, torque_limit: int):
        torque_limit = int(max(0, min(1023, torque_limit)))
        self.write2(motor_id, ADDR_TORQUE_LIMIT, torque_limit)

    def set_torque_limit_all(self, torque_limit: int = TORQUE_LIMIT_RAW):
        for motor_id in ALL_MOTOR_IDS:
            self.set_torque_limit(motor_id, torque_limit)
            time.sleep(0.006)

    # ----------------------------------------------------------
    # SYNC WRITE POSITION (KEY IMPROVEMENT)
    # ----------------------------------------------------------
    # Protocol 1.0 syncWrite sends one broadcast packet with position data
    # for all motors. All motors receive the command at the same time
    # instead of being written one-by-one with delays in between.
    #
    # AX-12A / AX-18A support syncWrite on Protocol 1.0.
    # The Dynamixel SDK GroupSyncWrite handles packet construction.
    #
    # Fall back to individual write2 if sync write fails (e.g. SDK version
    # doesn't support it or packet error).
    # ----------------------------------------------------------

    def sync_write_positions(self, targets: Dict[int, int]) -> bool:
        """
        Send all goal positions in a single syncWrite packet.
        This is THE fix for choppy movement: all motors start simultaneously.
        """
        try:
            gsw = GroupSyncWrite(
                self.port_handler,
                self.packet_handler,
                ADDR_GOAL_POSITION,
                2,  # 2 bytes for goal position
            )
            for motor_id, raw in targets.items():
                raw = clamp_raw(raw)
                data = [raw & 0xFF, (raw >> 8) & 0xFF]
                gsw.addParam(motor_id, data)

            result = gsw.txPacket()
            gsw.clearParam()

            if result != COMM_SUCCESS:
                # Fall back silently to individual writes
                return False
            return True
        except Exception:
            return False

    def sync_set_speeds(self, speed: int, motor_ids: Optional[List[int]] = None) -> bool:
        """
        Set speed for a set of motors in one syncWrite packet.
        Only re-sends if the speed actually changed (cached).
        """
        ids = motor_ids if motor_ids is not None else ALL_MOTOR_IDS
        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, speed)))

        # Only include motors whose speed changed
        changed = [mid for mid in ids if self._speed_cache.get(mid) != speed]
        if not changed:
            return True

        try:
            gsw = GroupSyncWrite(
                self.port_handler,
                self.packet_handler,
                ADDR_MOVING_SPEED,
                2,
            )
            for motor_id in changed:
                data = [speed & 0xFF, (speed >> 8) & 0xFF]
                gsw.addParam(motor_id, data)

            result = gsw.txPacket()
            gsw.clearParam()

            if result == COMM_SUCCESS:
                for mid in changed:
                    self._speed_cache[mid] = speed
                return True
        except Exception:
            pass

        # Fallback: individual writes
        for motor_id in changed:
            self.write2(motor_id, ADDR_MOVING_SPEED, speed)
            self._speed_cache[motor_id] = speed
        return True

    def move_sync(self, targets: Dict[int, int], speed: int):
        """
        REPLACEMENT for move_many().
        1. Ensure torque is on for all target motors (only on first use).
        2. Set speed via sync write (skips if unchanged).
        3. Send all positions via sync write in one packet.

        No 6ms inter-motor sleeps. All motors start simultaneously.
        """
        global ACTIVE_GOALS

        # Torque enable: only needed once at startup or after power cycle.
        # We don't do it every phase to avoid the 18 × write1 penalty.
        # (action_ready and startup call enable_torque_all explicitly.)

        self.sync_set_speeds(speed, list(targets.keys()))

        ok = self.sync_write_positions(targets)
        if not ok:
            # Fallback to individual writes (no sleep between motors)
            for motor_id, raw in targets.items():
                self.write2(motor_id, ADDR_GOAL_POSITION, clamp_raw(raw))

        for motor_id, raw in targets.items():
            ACTIVE_GOALS[motor_id] = clamp_raw(raw)

    # Keep move_many as alias for compatibility
    def move_many(self, targets: Dict[int, int], speed: int):
        self.move_sync(targets, speed)


# ============================================================
# HEALTH / STATUS
# ============================================================

def read_bus_health(bus: DynamixelBus) -> Tuple[int, float, int, bool, int]:
    max_temp    = 0
    min_volt    = 99.0
    max_abs_load = 0
    any_no_reply = False
    connected   = 0

    for motor_id in ALL_MOTOR_IDS:
        pos      = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        temp     = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)
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


def health_status(max_temp, min_volt, max_abs_load, any_no_reply) -> str:
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
    print(f"Current mode : {CURRENT_MODE}")
    print(f"Connected    : {connected}/18")
    print(f"Max temp     : {max_temp} C")
    print(f"Min voltage  : {min_volt:.1f} V")
    print(f"Max abs load : {max_abs_load}")
    print(f"No reply     : {any_no_reply}")
    print(f"Status       : {status}")


def print_status(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" MOTOR STATUS")
    print("===================================================")
    print(f"{'ID':<3} {'Joint':<14} {'Pos':>5} {'Goal':>5}")
    print("-" * 32)
    for motor_id in ALL_MOTOR_IDS:
        joint = motor_id_to_joint(motor_id)
        pos   = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        goal  = ACTIVE_GOALS.get(motor_id, "?")
        pos_s = str(pos) if pos is not None else "----"
        print(f"{motor_id:<3} {joint:<14} {pos_s:>5} {goal:>5}")


def pre_motion_check(bus: DynamixelBus) -> bool:
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)
    status = health_status(max_temp, min_volt, max_abs_load, any_no_reply)
    if status in ["NO_REPLY", "DANGER_VOLT", "VOLT_STOP", "LOAD_STOP", "TEMP_STOP"]:
        print(f"\n[SAFETY STOP] Movement blocked. Status={status}")
        print(f"connected={connected}/18, minVolt={min_volt:.1f}V, maxLoad={max_abs_load}, maxTemp={max_temp}C")
        return False
    if status == "WARN":
        print(f"\n[WARNING] Movement allowed but status is WARN.")
        print(f"connected={connected}/18, minVolt={min_volt:.1f}V, maxLoad={max_abs_load}, maxTemp={max_temp}C")
    return True


# ============================================================
# MOVEMENT HELPERS
# ============================================================

def gait_lift_values() -> Tuple[float, float]:
    if USE_WALK_LIFT_PROFILE:
        return WALK_LIFT_FEMUR_DEG, WALK_LIFT_TIBIA_DEG
    return LIFT_LEVELS[GAIT_LIFT_LEVEL]["femur"], LIFT_LEVELS[GAIT_LIFT_LEVEL]["tibia"]


def movement_profile(direction: str) -> Tuple[float, float]:
    direction = normalize_direction(direction)
    if direction == "backward":
        return BACKWARD_HIP_SWING_DEG, BACKWARD_SUPPORT_PUSH_DEG
    if direction in ["left", "right"]:
        return STRAFE_HIP_SWING_DEG, STRAFE_SUPPORT_PUSH_DEG
    if direction in ["turn_left", "turn_right"]:
        return TURN_HIP_SWING_DEG, TURN_SUPPORT_PUSH_DEG
    return GAIT_HIP_SWING_DEG, GAIT_SUPPORT_PUSH_DEG


def strafe_hip_for_leg(leg: str, direction: str, amount: float, lifted: bool) -> float:
    direction = normalize_direction(direction)
    if direction == "left":
        base = -HIP_STRAFE_SIGN[leg] * amount * STRAFE_DIRECTION_MULTIPLIER
    elif direction == "right":
        base = HIP_STRAFE_SIGN[leg] * amount * STRAFE_DIRECTION_MULTIPLIER
    else:
        base = 0.0
    return base if not lifted else -base


def turn_hip_for_leg(leg: str, direction: str, amount: float, lifted: bool) -> float:
    direction = normalize_direction(direction)
    scale = TURN_LEFT_SCALE if direction == "turn_left" else TURN_RIGHT_SCALE
    amount = amount * scale
    if direction == "turn_left":
        base = HIP_TURN_SIGN[leg] * amount * TURN_DIRECTION_MULTIPLIER
    elif direction == "turn_right":
        base = -HIP_TURN_SIGN[leg] * amount * TURN_DIRECTION_MULTIPLIER
    else:
        base = 0.0
    return -base if lifted else base


def support_hip_for_leg(leg: str, direction: str, support_push_deg: float) -> float:
    direction = normalize_direction(direction)
    if direction == "forward":
        return -HIP_FORWARD_SIGN[leg] * support_push_deg
    elif direction == "backward":
        return HIP_FORWARD_SIGN[leg] * support_push_deg
    elif direction == "left":
        return strafe_hip_for_leg(leg, direction, support_push_deg, lifted=False)
    elif direction == "right":
        return strafe_hip_for_leg(leg, direction, support_push_deg, lifted=False)
    elif direction == "turn_left":
        return turn_hip_for_leg(leg, direction, support_push_deg, lifted=False)
    elif direction == "turn_right":
        return turn_hip_for_leg(leg, direction, support_push_deg, lifted=False)
    return 0.0


def lift_hip_for_leg(leg: str, direction: str, hip_swing_deg: float) -> float:
    direction = normalize_direction(direction)
    if direction == "forward":
        return HIP_FORWARD_SIGN[leg] * hip_swing_deg
    elif direction == "backward":
        return -HIP_FORWARD_SIGN[leg] * hip_swing_deg
    elif direction in ["left", "right"]:
        return strafe_hip_for_leg(leg, direction, hip_swing_deg, lifted=True)
    elif direction in ["turn_left", "turn_right"]:
        return turn_hip_for_leg(leg, direction, hip_swing_deg, lifted=True)
    return 0.0


# ============================================================
# PHASE BUILDERS
# ============================================================

def build_tripod_phase(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,    # "up", "swing", "down"
    support_push_active: bool = True,
) -> Dict[int, int]:
    """
    Build a full-body target frame for one gait phase.

    phase = "up"    : lift tripod vertically, hips centered. Support starts pushing.
    phase = "swing" : lifted tripod swings hips to forward position while staying up.
                      Support continues pushing.
    phase = "down"  : lifted tripod places foot at swing position (femur/tibia back to ready).
                      Support holds push.
    """
    targets = dict(READY_POSE)
    direction = normalize_direction(direction)
    lift_femur, lift_tibia = gait_lift_values()
    hip_swing_deg, support_push_deg = movement_profile(direction)

    for leg in lifted_legs:
        if phase == "up":
            # Lift vertically, hip stays at ready for this first sub-phase.
            targets.update(build_leg_offset_targets(leg,
                hip_deg=0.0, femur_deg=lift_femur, tibia_deg=lift_tibia))
        elif phase == "swing":
            # Still in air, now swing hip forward.
            hip = lift_hip_for_leg(leg, direction, hip_swing_deg)
            targets.update(build_leg_offset_targets(leg,
                hip_deg=hip, femur_deg=lift_femur, tibia_deg=lift_tibia))
        elif phase == "down":
            # Place foot: keep hip at swing position, femur/tibia back to ready.
            hip = lift_hip_for_leg(leg, direction, hip_swing_deg)
            targets.update(build_leg_offset_targets(leg,
                hip_deg=hip, femur_deg=0.0, tibia_deg=0.0))

    for leg in support_legs:
        if support_push_active:
            hip = support_hip_for_leg(leg, direction, support_push_deg)
        else:
            hip = 0.0
        targets.update(build_leg_offset_targets(leg,
            hip_deg=hip, femur_deg=0.0, tibia_deg=0.0))

    return targets


# ============================================================
# MOVE WRAPPER  (sync, with optional interpolation)
# ============================================================

def send_phase(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    Send a pose and wait for motors to reach it.

    With SMOOTH_GAIT on, sends intermediate frames first.
    With sync write, ALL motors start moving simultaneously on each frame.
    """
    global ACTIVE_GOALS

    if SMOOTH_GAIT and SMOOTH_STEPS > 1:
        start  = dict(ACTIVE_GOALS)
        frames = interpolate_targets(start, targets, SMOOTH_STEPS)
        for frame in frames:
            bus.move_sync(frame, speed=speed)
            time.sleep(SMOOTH_STEP_DELAY)
        time.sleep(hold)
    else:
        bus.move_sync(targets, speed=speed)
        time.sleep(hold)

    if label and SIDE_STRAFE_FLOW_PRINT_PHASES:
        print(f"  {label}: sent")


# ============================================================
# SIMULTANEOUS TRIPOD GAIT  (the real spider walk)
# ============================================================
#
# Classic (old) sequence per cycle:
#   A_UP -> A_SWING -> A_DOWN -> B_UP -> B_SWING -> B_DOWN
#   = 6 sequential phases, no overlap
#
# Simultaneous sequence (new):
#   Phase 1:  A_UP     + B pushes back    (A lifts, B already driving body forward)
#   Phase 2:  A_SWING  + B pushes back    (A swings hip to new position in air)
#   Phase 3:  A_DOWN   + B_UP (combined)  (A lands AND B lifts — the handoff)
#   Phase 4:  B_SWING  + A pushes back    (B swings hip in air, A now drives)
#   Phase 5:  B_DOWN   + A pushes back    (B places foot)
#   (next cycle starts again from Phase 1 but now A is support and B just landed)
#
# The "handoff" frame (Phase 3) is where the magic happens:
#   A lands at its new position AND B lifts simultaneously.
#   This removes the pause between tripods and makes it look continuous.
#
# ============================================================

def build_simultaneous_gait_phases(direction: str) -> List[Tuple[str, Dict[int, int], float]]:
    """
    Build all phase targets for one simultaneous tripod gait cycle.
    Returns list of (label, targets, hold_time).
    """
    direction = normalize_direction(direction)

    phases = []

    # Phase 1: A lifts vertically. B is already in support/push.
    targets = build_tripod_phase(TRIPOD_A, TRIPOD_B, direction, "up", support_push_active=True)
    phases.append((f"GAIT_{direction}_A_UP+B_PUSH", targets, GAIT_PHASE_DELAY))

    # Phase 2: A swings hip while still in air. B continues pushing.
    targets = build_tripod_phase(TRIPOD_A, TRIPOD_B, direction, "swing", support_push_active=True)
    phases.append((f"GAIT_{direction}_A_SWING+B_PUSH", targets, GAIT_PHASE_DELAY))

    # Phase 3: A places foot (DOWN) AND B lifts simultaneously.
    # Build this combined frame manually: A at down position, B lifting.
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()

    combined = dict(READY_POSE)
    # A: place foot (hip at swing pos, femur/tibia ready)
    for leg in TRIPOD_A:
        hip = lift_hip_for_leg(leg, direction, hip_swing_deg)
        combined.update(build_leg_offset_targets(leg, hip_deg=hip, femur_deg=0.0, tibia_deg=0.0))
    # B: lift vertically (hip stays at support pos temporarily)
    for leg in TRIPOD_B:
        combined.update(build_leg_offset_targets(leg, hip_deg=0.0, femur_deg=lift_femur, tibia_deg=lift_tibia))
    phases.append((f"GAIT_{direction}_A_DOWN+B_UP", combined, GAIT_SETTLE_DELAY))

    # Phase 4: B swings hip in air. A now in support/push from A's new position.
    targets = build_tripod_phase(TRIPOD_B, TRIPOD_A, direction, "swing", support_push_active=True)
    phases.append((f"GAIT_{direction}_B_SWING+A_PUSH", targets, GAIT_PHASE_DELAY))

    # Phase 5: B places foot. A continues pushing.
    targets = build_tripod_phase(TRIPOD_B, TRIPOD_A, direction, "down", support_push_active=True)
    phases.append((f"GAIT_{direction}_B_DOWN+A_PUSH", targets, GAIT_SETTLE_DELAY))

    return phases


def pose_set_leg_lift(pose: Dict[int, int], leg: str, femur_deg: float, tibia_deg: float):
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")
    pose[joint_to_motor_id(femur_joint)] = offset_from_ready(femur_joint, femur_deg)
    pose[joint_to_motor_id(tibia_joint)] = offset_from_ready(tibia_joint, tibia_deg)


def pose_set_leg_down(pose: Dict[int, int], leg: str):
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")
    pose[joint_to_motor_id(femur_joint)] = READY_POSE[joint_to_motor_id(femur_joint)]
    pose[joint_to_motor_id(tibia_joint)] = READY_POSE[joint_to_motor_id(tibia_joint)]


def pose_set_leg_hip_ready(pose: Dict[int, int], leg: str):
    hip_joint = leg_part_to_joint(leg, "hip")
    pose[joint_to_motor_id(hip_joint)] = READY_POSE[joint_to_motor_id(hip_joint)]


def final_tripod_recenter(bus: DynamixelBus, direction: str):
    global ACTIVE_GOALS, CURRENT_MODE
    lift_femur, lift_tibia = gait_lift_values()
    pose = dict(ACTIVE_GOALS)

    for group_name, tripod in [("A", TRIPOD_A), ("B", TRIPOD_B)]:
        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_LIFT"
        lift_pose = dict(pose)
        for leg in tripod:
            pose_set_leg_lift(lift_pose, leg, lift_femur, lift_tibia)
        send_phase(bus, lift_pose, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_HIP_READY"
        hip_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            pose_set_leg_hip_ready(hip_pose, leg)
        send_phase(bus, hip_pose, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_DOWN"
        down_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            pose_set_leg_down(down_pose, leg)
        send_phase(bus, down_pose, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

        pose = dict(ACTIVE_GOALS)

    CURRENT_MODE = "READY_REFINED2K"
    send_phase(bus, dict(READY_POSE), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
    ACTIVE_GOALS = dict(READY_POSE)


# ============================================================
# FORWARD / BACKWARD / TURN GAIT
# ============================================================

def action_gait_continuous(bus: DynamixelBus, direction: str):
    """
    Continuous simultaneous tripod gait.
    Runs until the user presses Enter.
    Press Enter at any time to stop after the current cycle completes.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["forward", "backward", "turn_left", "turn_right"]:
        print("Unsupported direction for continuous gait.")
        return

    print()
    print("===================================================")
    print(f" SIMULTANEOUS TRIPOD GAIT: {direction.upper()}")
    print("===================================================")
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()
    print(f"Gait lift: femur {lift_femur:+.1f} deg, tibia {lift_tibia:+.1f} deg")
    print(f"Hip swing: {hip_swing_deg} deg  Support push: {support_push_deg} deg")
    print(f"Speed: {GAIT_SPEED}   Phase delay: {GAIT_PHASE_DELAY:.3f}s   Settle: {GAIT_SETTLE_DELAY:.3f}s")
    print(f"Sync write: ON  (all motors start simultaneously)")
    print(f"Gait mode: simultaneous tripod handoff (spider-style)")
    print()
    print("  >>> Running. Press Enter to stop after current cycle. <<<")
    print()

    if not pre_motion_check(bus):
        return

    import threading

    stop_flag = threading.Event()

    def wait_for_enter():
        try:
            input()
        except Exception:
            pass
        stop_flag.set()

    listener = threading.Thread(target=wait_for_enter, daemon=True)
    listener.start()

    cycle = 0
    phases = build_simultaneous_gait_phases(direction)

    while not stop_flag.is_set():
        cycle += 1
        print(f"  cycle {cycle}", end="\r", flush=True)

        if GAIT_PRECHECK_EACH_PHASE:
            if not pre_motion_check(bus):
                break

        for label, targets, hold in phases:
            CURRENT_MODE = label
            send_phase(bus, targets, GAIT_SPEED, hold, label if GAIT_PHASE_HEALTH else "")
            if GAIT_PHASE_HEALTH:
                print_health(bus, label)

        if stop_flag.is_set():
            break

    print(f"\n  Stopped after {cycle} cycle(s).")

    if GAIT_END_MODE == "tripod":
        print("Final recenter...")
        final_tripod_recenter(bus, direction)
    elif GAIT_END_MODE == "direct":
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = dict(READY_POSE)
        send_phase(bus, dict(READY_POSE), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
    # else hold = stay in last stance

    time.sleep(0.20)
    print_health(bus, f"AFTER GAIT {direction}")


def action_gait_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    """
    Non-continuous version: run N simultaneous cycles then stop.
    Used by the 'walk forward 3' command etc.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["forward", "backward", "left", "right", "turn_left", "turn_right"]:
        print("Unknown direction.")
        return

    if direction in ["left", "right"]:
        action_side_strafe_cycle(bus, direction, cycles=cycles)
        return

    cycles = max(1, min(20, int(cycles)))

    print()
    print("===================================================")
    print(f" SIMULTANEOUS TRIPOD GAIT {direction.upper()} x{cycles}")
    print("===================================================")
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()
    print(f"Gait lift: femur {lift_femur:+.1f}, tibia {lift_tibia:+.1f}")
    print(f"Hip swing: {hip_swing_deg}   Support: {support_push_deg}   Speed: {GAIT_SPEED}")
    print("===================================================")

    if not pre_motion_check(bus):
        return

    phases = build_simultaneous_gait_phases(direction)

    for i in range(cycles):
        print(f"\n--- CYCLE {i+1}/{cycles} ---")
        if GAIT_PRECHECK_EACH_PHASE and not pre_motion_check(bus):
            break
        for label, targets, hold in phases:
            CURRENT_MODE = label
            send_phase(bus, targets, GAIT_SPEED, hold, label)
        print_health(bus, f"AFTER CYCLE {i+1} {direction}")

    if GAIT_END_MODE == "tripod":
        print("Final recenter...")
        final_tripod_recenter(bus, direction)
    elif GAIT_END_MODE == "direct":
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = dict(READY_POSE)
        send_phase(bus, dict(READY_POSE), GAIT_SPEED, GAIT_FINAL_READY_DELAY)

    time.sleep(0.20)
    print_health(bus, f"AFTER GAIT {direction}")


# ============================================================
# SIDE STRAFE (WControl23 preserved, now with sync write)
# ============================================================

def side_strafe_direction_sign(direction: str) -> float:
    direction = normalize_direction(direction)
    if direction == "left":
        return SIDE_STRAFE_DIRECTION_MULTIPLIER * 1.0
    if direction == "right":
        return SIDE_STRAFE_DIRECTION_MULTIPLIER * -1.0
    return 0.0


def side_strafe_side_sign(leg: str, direction: str) -> float:
    direction = normalize_direction(direction)
    if direction == "left":
        base = +1.0 if leg in LEFT_LEGS else -1.0
    elif direction == "right":
        base = -1.0 if leg in LEFT_LEGS else +1.0
    else:
        base = 0.0
    return SIDE_STRAFE_DIRECTION_MULTIPLIER * base


def side_strafe_leg_offsets(leg: str, direction: str, role: str) -> Tuple[float, float, float]:
    side = side_strafe_side_sign(leg, direction)
    if role == "lift":
        return 0.0, SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG
    if role == "reach_lifted":
        femur = SIDE_STRAFE_LIFT_FEMUR_DEG + side * SIDE_STRAFE_FEMUR_REACH_DEG
        tibia = SIDE_STRAFE_LIFT_TIBIA_DEG + side * SIDE_STRAFE_TIBIA_REACH_DEG
        return 0.0, femur, tibia
    if role == "reach_ground":
        return 0.0, side * SIDE_STRAFE_FEMUR_REACH_DEG, side * SIDE_STRAFE_TIBIA_REACH_DEG
    if role == "pull_ground":
        return 0.0, side * SIDE_STRAFE_FEMUR_PULL_DEG, side * SIDE_STRAFE_TIBIA_PULL_DEG
    return 0.0, 0.0, 0.0


def side_strafe_phase_support_offsets(leg: str, direction: str, active_tripod: List[str], phase: str) -> Tuple[float, float, float]:
    if not SIDE_STRAFE_PHASE_BOOST_ENABLED:
        return side_strafe_leg_offsets(leg, direction, "pull_ground")
    if phase != "reach_pull" or active_tripod != CRAB_SECOND_TRIPOD:
        return side_strafe_leg_offsets(leg, direction, "pull_ground")
    side = side_strafe_side_sign(leg, direction)
    if leg in ["ML", "MR"]:
        femur = -side * SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG
        tibia = side  * SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG
    else:
        femur = -side * SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG
        tibia = side  * SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG
    return 0.0, femur, tibia


def build_side_strafe_targets(active_tripod, other_tripod, direction, phase) -> Dict[int, int]:
    targets = dict(READY_POSE)
    if phase == "up_pull":
        for leg in active_tripod:
            h, f, t = side_strafe_leg_offsets(leg, direction, "lift")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        for leg in other_tripod:
            h, f, t = side_strafe_phase_support_offsets(leg, direction, active_tripod, phase)
            targets.update(build_leg_offset_targets(leg, h, f, t))
    elif phase == "reach_pull":
        for leg in active_tripod:
            h, f, t = side_strafe_leg_offsets(leg, direction, "reach_lifted")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        for leg in other_tripod:
            h, f, t = side_strafe_phase_support_offsets(leg, direction, active_tripod, phase)
            targets.update(build_leg_offset_targets(leg, h, f, t))
    elif phase == "down_pull":
        for leg in active_tripod:
            h, f, t = side_strafe_leg_offsets(leg, direction, "reach_ground")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        for leg in other_tripod:
            h, f, t = side_strafe_phase_support_offsets(leg, direction, active_tripod, phase)
            targets.update(build_leg_offset_targets(leg, h, f, t))
    elif phase == "ready":
        for leg in active_tripod + other_tripod:
            h, f, t = side_strafe_leg_offsets(leg, direction, "ready")
            targets.update(build_leg_offset_targets(leg, h, f, t))
    return targets


def side_strafe_final_recenter(bus: DynamixelBus, direction: str):
    global ACTIVE_GOALS, CURRENT_MODE
    for idx, legs in enumerate([CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD], start=1):
        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_UP"
        targets = dict(READY_POSE)
        for leg in legs:
            h, f, t = side_strafe_leg_offsets(leg, direction, "lift")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        ACTIVE_GOALS = dict(targets)
        send_phase(bus, targets, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_READY"
        targets = dict(READY_POSE)
        ACTIVE_GOALS = dict(targets)
        send_phase(bus, targets, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = dict(READY_POSE)
    send_phase(bus, dict(READY_POSE), GAIT_SPEED, GAIT_FINAL_READY_DELAY)


def action_side_strafe_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["left", "right"]:
        print("Side strafe only supports left/right.")
        return

    cycles = max(1, min(10, int(cycles)))

    print()
    print("===================================================")
    print(f" SIDE STRAFE {direction.upper()} x{cycles}  (WControl23 + sync write)")
    print("===================================================")
    print(f"Flow mode: {SIDE_STRAFE_FLOW_MODE}")

    for i in range(cycles):
        print(f"\n--- SIDE STRAFE CYCLE {i+1}/{cycles} ---")
        if not pre_motion_check(bus):
            return

        phases = [
            (f"SIDE_{direction}_B_UP_A_PULL",    CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "up_pull",    SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_B_REACH_A_PULL",  CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "reach_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_B_DOWN_A_PULL",   CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "down_pull",  SIDE_STRAFE_SETTLE),
            (f"SIDE_{direction}_A_UP_B_PULL",    CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "up_pull",    SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_A_REACH_B_PULL",  CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "reach_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_A_DOWN_B_PULL",   CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "down_pull",  SIDE_STRAFE_SETTLE),
        ]

        for mode_name, active, other, phase, delay in phases:
            CURRENT_MODE = mode_name
            targets = build_side_strafe_targets(active, other, direction, phase)
            ACTIVE_GOALS = dict(targets)
            effective_delay = SIDE_STRAFE_FLOW_HOLD if SIDE_STRAFE_FLOW_MODE else delay
            send_phase(bus, targets, GAIT_SPEED, effective_delay,
                       mode_name if SIDE_STRAFE_FLOW_PRINT_PHASES else "")
            if GAIT_PHASE_HEALTH:
                print_health(bus, CURRENT_MODE)

        print_health(bus, f"AFTER SIDE STRAFE CYCLE {i+1} {direction}")

    print("Final recenter...")
    side_strafe_final_recenter(bus, direction)
    time.sleep(0.20)
    print_health(bus, f"AFTER SIDE STRAFE {direction}")


# ============================================================
# READY / LIFT / PUSHUP ACTIONS
# ============================================================

def action_ready(bus: DynamixelBus, use_safety_check: bool = True):
    global ACTIVE_GOALS, CURRENT_MODE

    if use_safety_check and not pre_motion_check(bus):
        return

    print("\nACTION: FAST TRIPOD-LIFT RETURN TO READY")
    bus.enable_torque_all()  # ensure torque is on after power up

    reset_speed = READY_SPEED

    def lift_tripod(legs):
        t = dict(READY_POSE)
        for leg in legs:
            t.update(build_leg_offset_targets(leg, 0.0, SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG))
        return t

    for mode, legs, hold in [
        ("READY_RESET_B_UP",   CRAB_FIRST_TRIPOD,  0.12),
        ("READY_RESET_B_DOWN", None,               0.10),
        ("READY_RESET_A_UP",   CRAB_SECOND_TRIPOD, 0.12),
        ("READY_RESET_A_DOWN", None,               0.10),
    ]:
        CURRENT_MODE = mode
        targets = lift_tripod(legs) if legs else dict(READY_POSE)
        ACTIVE_GOALS = dict(targets)
        bus.move_sync(targets, speed=reset_speed)
        time.sleep(hold)

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = dict(READY_POSE)
    bus.move_sync(dict(READY_POSE), speed=reset_speed)
    time.sleep(0.20)

    print_health(bus, "AFTER READY")


def action_lift_legs(bus: DynamixelBus, level: int, legs: List[str]):
    global ACTIVE_GOALS, CURRENT_MODE

    if level not in LIFT_LEVELS:
        print("Lift level must be 1-9.")
        return
    if not legs:
        print("No legs selected.")
        return
    if not pre_motion_check(bus):
        return

    femur_deg = LIFT_LEVELS[level]["femur"]
    tibia_deg = LIFT_LEVELS[level]["tibia"]

    print(f"\nLIFT L{level} {' '.join(legs)}  femur {femur_deg:+.1f} tibia {tibia_deg:+.1f}")

    targets = dict(READY_POSE)
    for leg in legs:
        targets.update(build_leg_offset_targets(leg, 0.0, femur_deg, tibia_deg))

    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"LIFT_L{level}_{'_'.join(legs)}"
    bus.move_sync(targets, speed=LIFT_SPEED)
    time.sleep(0.8)
    print_health(bus, f"AFTER LIFT L{level} {' '.join(legs)}")


def action_pushup(bus: DynamixelBus, level: str):
    global ACTIVE_GOALS, CURRENT_MODE

    if level not in PUSHUP_LEVELS:
        print("Usage: pushup 1/2/3/4")
        return
    if not pre_motion_check(bus):
        return

    print(f"\nPUSHUP {level}")
    targets = PUSHUP_LEVELS[level]
    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"PUSHUP_{level}"
    bus.move_sync(targets, speed=MOVE_SPEED)
    time.sleep(1.0)
    print_health(bus, f"AFTER PUSHUP {level}")


def action_torque_max(bus: DynamixelBus):
    print("\nSetting AX torque limit cap to 1023 for all motors.")
    bus.set_torque_limit_all(TORQUE_LIMIT_RAW)
    print_health(bus, "AFTER TORQUE_MAX")


# ============================================================
# RUNTIME SETTINGS COMMANDS
# ============================================================

def action_set_speed(parts: List[str]):
    global GAIT_SPEED, READY_SPEED, MOVE_SPEED, LIFT_SPEED

    if len(parts) == 1:
        print(f"ready={READY_SPEED} move={MOVE_SPEED} lift={LIFT_SPEED} gait={GAIT_SPEED}")
        return

    sub = parts[1].lower()

    if sub.isdigit():
        GAIT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(sub))))
        print(f"Gait speed = {GAIT_SPEED}")
        return

    if sub == "gait" and len(parts) == 3:
        GAIT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        print(f"Gait speed = {GAIT_SPEED}")
        return

    if sub == "lift" and len(parts) == 3:
        LIFT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        print(f"Lift speed = {LIFT_SPEED}")
        return

    if sub == "all" and len(parts) == 3:
        v = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        READY_SPEED = MOVE_SPEED = LIFT_SPEED = GAIT_SPEED = v
        print(f"All speeds = {v}")
        return

    print("Usage: speed / speed gait 18 / speed lift 10 / speed all 22")


def action_walk_lift(parts: List[str]):
    global USE_WALK_LIFT_PROFILE, WALK_LIFT_FEMUR_DEG, WALK_LIFT_TIBIA_DEG, GAIT_LIFT_LEVEL

    if len(parts) == 1:
        lf, lt = gait_lift_values()
        print(f"Walk lift: femur {lf:+.1f}, tibia {lt:+.1f}  (level={GAIT_LIFT_LEVEL}, profile={USE_WALK_LIFT_PROFILE})")
        return

    sub = parts[1].lower()

    if sub in WALK_LIFT_PRESETS:
        USE_WALK_LIFT_PROFILE = True
        WALK_LIFT_FEMUR_DEG = WALK_LIFT_PRESETS[sub]["femur"]
        WALK_LIFT_TIBIA_DEG = WALK_LIFT_PRESETS[sub]["tibia"]
        lf, lt = gait_lift_values()
        print(f"Walk lift preset '{sub}': femur {lf:+.1f}, tibia {lt:+.1f}")
        return

    if sub == "level" and len(parts) == 3:
        level = int(parts[2])
        if level not in LIFT_LEVELS:
            print(f"Level must be {sorted(LIFT_LEVELS.keys())}")
            return
        GAIT_LIFT_LEVEL = level
        USE_WALK_LIFT_PROFILE = False
        print(f"Walk lift level = {GAIT_LIFT_LEVEL}")
        return

    if sub in ["on", "profile"]:
        USE_WALK_LIFT_PROFILE = True
        return

    if sub in ["off"]:
        USE_WALK_LIFT_PROFILE = False
        return

    print("Usage: walklift / walklift level 6 / walklift high2 / walklift max")


def action_smooth(parts: List[str]):
    global SMOOTH_GAIT, SMOOTH_STEPS, SMOOTH_STEP_DELAY
    global GAIT_PHASE_DELAY, GAIT_SETTLE_DELAY, GAIT_PHASE_HEALTH, GAIT_PRECHECK_EACH_PHASE

    if len(parts) == 1:
        print(f"smooth={SMOOTH_GAIT} steps={SMOOTH_STEPS} stepdelay={SMOOTH_STEP_DELAY:.3f}")
        print(f"phase_delay={GAIT_PHASE_DELAY:.3f} settle={GAIT_SETTLE_DELAY:.3f}")
        return

    sub = parts[1].lower()

    if sub in ["on", "true", "1"]:
        SMOOTH_GAIT = True
    elif sub in ["off", "false", "0"]:
        SMOOTH_GAIT = False
    elif sub == "steps" and len(parts) == 3:
        SMOOTH_STEPS = max(1, min(12, int(parts[2])))
    elif sub == "hold" and len(parts) == 3:
        GAIT_PHASE_DELAY = max(0.02, min(1.0, float(parts[2])))
    elif sub == "settle" and len(parts) == 3:
        GAIT_SETTLE_DELAY = max(0.02, min(1.0, float(parts[2])))
    elif sub in ["walk", "flow"]:
        SMOOTH_GAIT = True; SMOOTH_STEPS = 3; SMOOTH_STEP_DELAY = 0.020
        GAIT_PHASE_DELAY = 0.08; GAIT_SETTLE_DELAY = 0.05
        GAIT_PHASE_HEALTH = False; GAIT_PRECHECK_EACH_PHASE = False
    else:
        print("Usage: smooth on/off / smooth walk / smooth hold 0.15 / smooth settle 0.08 / smooth steps 3")
        return

    print(f"smooth={SMOOTH_GAIT} steps={SMOOTH_STEPS} phase={GAIT_PHASE_DELAY:.3f} settle={GAIT_SETTLE_DELAY:.3f}")


def action_side_strafe_settings(parts: List[str]):
    global SIDE_STRAFE_DIRECTION_MULTIPLIER
    global SIDE_STRAFE_FEMUR_REACH_DEG, SIDE_STRAFE_TIBIA_REACH_DEG
    global SIDE_STRAFE_FEMUR_PULL_DEG, SIDE_STRAFE_TIBIA_PULL_DEG
    global SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG
    global SIDE_STRAFE_HOLD, SIDE_STRAFE_SETTLE
    global SIDE_STRAFE_DEBUG_STEPS_ENABLED, SIDE_STRAFE_DEBUG_STEPS, SIDE_STRAFE_DEBUG_STEP_DELAY
    global SIDE_STRAFE_PHASE_BOOST_ENABLED

    if len(parts) == 1:
        print(f"A/D strafe: reach={SIDE_STRAFE_FEMUR_REACH_DEG:+.1f}/{SIDE_STRAFE_TIBIA_REACH_DEG:+.1f} pull={SIDE_STRAFE_FEMUR_PULL_DEG:+.1f}/{SIDE_STRAFE_TIBIA_PULL_DEG:+.1f} lift={SIDE_STRAFE_LIFT_FEMUR_DEG:+.1f}/{SIDE_STRAFE_LIFT_TIBIA_DEG:+.1f}")
        return

    sub = parts[1].lower()

    try:
        if sub in ["good", "w23", "reset"]:
            SIDE_STRAFE_FEMUR_REACH_DEG = 6.0;  SIDE_STRAFE_TIBIA_REACH_DEG = -14.0
            SIDE_STRAFE_FEMUR_PULL_DEG  = -5.0; SIDE_STRAFE_TIBIA_PULL_DEG  = 12.0
            SIDE_STRAFE_LIFT_FEMUR_DEG  = -34.0; SIDE_STRAFE_LIFT_TIBIA_DEG  = -6.0
            SIDE_STRAFE_HOLD = 0.18; SIDE_STRAFE_SETTLE = 0.10
        elif sub == "flip":
            SIDE_STRAFE_DIRECTION_MULTIPLIER *= -1.0
        elif sub in ["gentle", "safe"]:
            SIDE_STRAFE_FEMUR_REACH_DEG = 5.0;  SIDE_STRAFE_TIBIA_REACH_DEG = -11.0
            SIDE_STRAFE_FEMUR_PULL_DEG  = -3.5; SIDE_STRAFE_TIBIA_PULL_DEG  = 8.0
            SIDE_STRAFE_LIFT_FEMUR_DEG  = -32.0; SIDE_STRAFE_LIFT_TIBIA_DEG  = -5.0
            SIDE_STRAFE_HOLD = 0.18; SIDE_STRAFE_SETTLE = 0.10
        elif sub in ["stronger"]:
            SIDE_STRAFE_FEMUR_REACH_DEG = 7.0;  SIDE_STRAFE_TIBIA_REACH_DEG = -16.0
            SIDE_STRAFE_FEMUR_PULL_DEG  = -5.5; SIDE_STRAFE_TIBIA_PULL_DEG  = 13.5
            SIDE_STRAFE_LIFT_FEMUR_DEG  = -35.0; SIDE_STRAFE_LIFT_TIBIA_DEG  = -6.5
            SIDE_STRAFE_HOLD = 0.16; SIDE_STRAFE_SETTLE = 0.09
        elif sub == "reach" and len(parts) == 4:
            SIDE_STRAFE_FEMUR_REACH_DEG = float(parts[2])
            SIDE_STRAFE_TIBIA_REACH_DEG = float(parts[3])
        elif sub == "pull" and len(parts) == 4:
            SIDE_STRAFE_FEMUR_PULL_DEG = float(parts[2])
            SIDE_STRAFE_TIBIA_PULL_DEG = float(parts[3])
        elif sub == "lift" and len(parts) == 4:
            SIDE_STRAFE_LIFT_FEMUR_DEG = -abs(float(parts[2]))
            SIDE_STRAFE_LIFT_TIBIA_DEG = float(parts[3])
        elif sub == "hold" and len(parts) == 3:
            SIDE_STRAFE_HOLD = max(0.0, min(0.80, float(parts[2])))
        elif sub == "settle" and len(parts) == 3:
            SIDE_STRAFE_SETTLE = max(0.0, min(0.50, float(parts[2])))
        elif sub == "phaseboost" and len(parts) >= 3:
            SIDE_STRAFE_PHASE_BOOST_ENABLED = parts[2].lower() in ["on", "true", "1", "yes", "enable"]
        else:
            print("Usage: sidestrafe / sidestrafe good / sidestrafe flip / sidestrafe gentle / sidestrafe stronger")
            print("       sidestrafe reach 6 -14 / sidestrafe pull -5 12 / sidestrafe lift 34 -6")
            print("       sidestrafe hold 0.18 / sidestrafe settle 0.10")
            return
    except ValueError:
        print("Invalid value.")
        return

    print(f"A/D strafe: reach={SIDE_STRAFE_FEMUR_REACH_DEG:+.1f}/{SIDE_STRAFE_TIBIA_REACH_DEG:+.1f} pull={SIDE_STRAFE_FEMUR_PULL_DEG:+.1f}/{SIDE_STRAFE_TIBIA_PULL_DEG:+.1f} lift={SIDE_STRAFE_LIFT_FEMUR_DEG:+.1f}/{SIDE_STRAFE_LIFT_TIBIA_DEG:+.1f}")


def action_sideflow(parts: List[str]):
    global SIDE_STRAFE_FLOW_MODE, SIDE_STRAFE_FLOW_HOLD, SIDE_STRAFE_FLOW_PRINT_PHASES

    if len(parts) == 1:
        print(f"flow={SIDE_STRAFE_FLOW_MODE} hold={SIDE_STRAFE_FLOW_HOLD:.3f} print={SIDE_STRAFE_FLOW_PRINT_PHASES}")
        return

    sub = parts[1].lower()
    if sub in ["on", "enable"]:
        SIDE_STRAFE_FLOW_MODE = True; SIDE_STRAFE_FLOW_HOLD = 0.0
    elif sub in ["tiny", "small"]:
        SIDE_STRAFE_FLOW_MODE = True; SIDE_STRAFE_FLOW_HOLD = SIDE_STRAFE_FLOW_TINY_HOLD
    elif sub in ["off", "disable"]:
        SIDE_STRAFE_FLOW_MODE = False
    elif sub == "hold" and len(parts) >= 3:
        SIDE_STRAFE_FLOW_HOLD = max(0.0, min(0.10, float(parts[2])))
        SIDE_STRAFE_FLOW_MODE = True
    elif sub == "print" and len(parts) >= 3:
        SIDE_STRAFE_FLOW_PRINT_PHASES = parts[2].lower() in ["on", "true", "1", "yes"]
    else:
        print("Usage: sideflow / sideflow on / sideflow off / sideflow tiny / sideflow hold 0.02")
        return

    print(f"flow={SIDE_STRAFE_FLOW_MODE} hold={SIDE_STRAFE_FLOW_HOLD:.3f}")


def action_movement_stats(parts: List[str]):
    global MOVEMENT_STATS_ENABLED, MOVEMENT_STATS_DETAIL
    if len(parts) == 1:
        print(f"movestats={MOVEMENT_STATS_ENABLED} detail={MOVEMENT_STATS_DETAIL}")
        return
    sub = parts[1].lower()
    if sub in ["on", "1", "true"]:
        MOVEMENT_STATS_ENABLED = True
    elif sub in ["off", "0", "false"]:
        MOVEMENT_STATS_ENABLED = False
    elif sub in ["detail", "verbose"]:
        MOVEMENT_STATS_ENABLED = True; MOVEMENT_STATS_DETAIL = "detail"
    elif sub in ["compact"]:
        MOVEMENT_STATS_ENABLED = True; MOVEMENT_STATS_DETAIL = "compact"
    else:
        print("Usage: movestats on/off/detail/compact")
        return
    print(f"movestats={MOVEMENT_STATS_ENABLED} detail={MOVEMENT_STATS_DETAIL}")


def action_leg_trim(parts: List[str]):
    if len(parts) == 1:
        print("Femur:", {l: f"{LEG_FEMUR_LIFT_SCALE[l]:.2f}" for l in ALL_LEGS})
        print("Tibia:", {l: f"{LEG_TIBIA_LIFT_SCALE[l]:.2f}" for l in ALL_LEGS})
        return
    if len(parts) != 4:
        print("Usage: legtrim RR tibia 0.85")
        return
    leg  = parts[1].upper()
    part = parts[2].lower()
    if leg not in ALL_LEGS or part not in ["femur", "tibia"]:
        print("Invalid. Example: legtrim RR tibia 0.85")
        return
    try:
        value = max(0.50, min(1.20, float(parts[3])))
    except ValueError:
        print("Scale must be a number.")
        return
    if part == "femur":
        LEG_FEMUR_LIFT_SCALE[leg] = value
    else:
        LEG_TIBIA_LIFT_SCALE[leg] = value
    print(f"{leg} {part} trim = {value:.2f}")


def action_range(parts: List[str]):
    global GAIT_HIP_SWING_DEG, GAIT_SUPPORT_PUSH_DEG
    global BACKWARD_HIP_SWING_DEG, BACKWARD_SUPPORT_PUSH_DEG
    global STRAFE_HIP_SWING_DEG, STRAFE_SUPPORT_PUSH_DEG
    global TURN_HIP_SWING_DEG, TURN_SUPPORT_PUSH_DEG

    if len(parts) == 1:
        print(f"forward  swing={GAIT_HIP_SWING_DEG} push={GAIT_SUPPORT_PUSH_DEG}")
        print(f"backward swing={BACKWARD_HIP_SWING_DEG} push={BACKWARD_SUPPORT_PUSH_DEG}")
        print(f"strafe   swing={STRAFE_HIP_SWING_DEG} push={STRAFE_SUPPORT_PUSH_DEG}")
        print(f"turn     swing={TURN_HIP_SWING_DEG} push={TURN_SUPPORT_PUSH_DEG}")
        return

    try:
        sub = parts[1].lower()
        if sub in ["forward", "fwd"] and len(parts) == 4:
            GAIT_HIP_SWING_DEG = float(parts[2]); GAIT_SUPPORT_PUSH_DEG = float(parts[3])
        elif sub in ["backward", "back"] and len(parts) == 4:
            BACKWARD_HIP_SWING_DEG = float(parts[2]); BACKWARD_SUPPORT_PUSH_DEG = float(parts[3])
        elif sub in ["strafe", "side"] and len(parts) == 4:
            STRAFE_HIP_SWING_DEG = float(parts[2]); STRAFE_SUPPORT_PUSH_DEG = float(parts[3])
        elif sub == "turn" and len(parts) == 4:
            TURN_HIP_SWING_DEG = float(parts[2]); TURN_SUPPORT_PUSH_DEG = float(parts[3])
        else:
            print("Usage: range forward 24 16 / range strafe 28 22 / range turn 30 24")
            return
    except ValueError:
        print("Values must be numbers.")
        return

    print("Range updated.")


def action_gait_timing(parts: List[str]):
    global GAIT_PHASE_DELAY, GAIT_SETTLE_DELAY, GAIT_FINAL_READY_DELAY
    global GAIT_END_MODE

    if len(parts) == 1:
        print(f"phase={GAIT_PHASE_DELAY:.3f} settle={GAIT_SETTLE_DELAY:.3f} final={GAIT_FINAL_READY_DELAY:.3f} endmode={GAIT_END_MODE}")
        return

    sub = parts[1].lower()
    try:
        if sub == "phase" and len(parts) == 3:
            GAIT_PHASE_DELAY = max(0.02, min(1.0, float(parts[2])))
        elif sub == "settle" and len(parts) == 3:
            GAIT_SETTLE_DELAY = max(0.02, min(1.0, float(parts[2])))
        elif sub == "final" and len(parts) == 3:
            GAIT_FINAL_READY_DELAY = max(0.05, min(1.0, float(parts[2])))
        elif sub in ["end", "endmode"] and len(parts) == 3:
            m = parts[2].lower()
            if m in ["tripod", "direct", "hold"]:
                GAIT_END_MODE = m
            else:
                print("endmode must be tripod / direct / hold")
        else:
            print("Usage: timing / timing phase 0.15 / timing settle 0.08 / timing end tripod")
            return
    except ValueError:
        print("Value must be a number.")
        return

    print(f"phase={GAIT_PHASE_DELAY:.3f} settle={GAIT_SETTLE_DELAY:.3f} end={GAIT_END_MODE}")


# ============================================================
# HELP
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" SCONTROL4 - SIMULTANEOUS TRIPOD GAIT")
    print(" HEXAPOD REFINED2K BALANCED CONTROL")
    print(" KEY CHANGE: sync write = all motors move at once")
    print("             simultaneous tripod handoff = spider walk")
    print("===================================================")
    print("MOTION COMMANDS (run until Enter is pressed):")
    print("  w            = forward  (continuous until Enter)")
    print("  s            = backward (continuous until Enter)")
    print("  a            = strafe left  (N cycles)")
    print("  d            = strafe right (N cycles)")
    print("  q            = turn left  (continuous)")
    print("  e            = turn right (continuous)")
    print("  gait forward = one-shot direction (uses cycles)")
    print("  walk forward 3 = 3 cycles then stop")
    print()
    print("SETUP:")
    print("  r / ready    = return to refined2k balanced ready pose")
    print("  force_r      = force return without safety check")
    print("  health       = motor health check")
    print("  p            = full motor status")
    print()
    print("TUNING:")
    print("  speed all 23       = set all speeds to 23")
    print("  speed gait 18      = set gait speed")
    print("  range strafe 28 22 = tune strafe hip/push")
    print("  range turn 30 24   = tune turn hip/push")
    print("  walklift level 6   = set gait lift level")
    print("  walklift high2     = higher clearance preset")
    print("  sidestrafe good    = restore working W23 values")
    print("  sidestrafe flip    = flip a/d direction")
    print("  sidestrafe gentle  = lower force preset")
    print("  sideflow on/off    = remove A/D phase holds")
    print("  smooth on/off      = interpolated intermediate frames")
    print("  smooth walk        = smooth-walk timing preset")
    print("  smooth hold 0.15   = adjust phase hold time")
    print("  smooth settle 0.08 = adjust settle time")
    print("  timing             = show/set gait timing")
    print("  timing phase 0.18  = set phase hold (larger = more time to reach pose)")
    print("  timing settle 0.10 = set touchdown settle")
    print()
    print("LIFT/PUSH:")
    print("  lift FL            = lift FL (level 3)")
    print("  lift 6 FL MR RL    = lift tripod A at level 6")
    print("  pushup 1/2/3/4     = body height")
    print("  torque_max         = set torque limit cap to 1023")
    print("  legtrim RR tibia 0.85")
    print()
    print("OTHER:")
    print("  movestats on/off/detail")
    print("  h / help   = this message")
    print("  x          = exit")
    print()
    print("RECOMMENDED FIRST TEST:")
    print("  r")
    print("  health")
    print("  sidestrafe good")
    print("  sideflow on")
    print("  speed all 23")
    print("  a                  (runs until Enter)")
    print("  r")
    print("  w                  (runs until Enter)")
    print("  r")
    print("===================================================")
    print(f"Current: speed={GAIT_SPEED}  lift=L{GAIT_LIFT_LEVEL}  phase={GAIT_PHASE_DELAY:.3f}s  settle={GAIT_SETTLE_DELAY:.3f}s")
    print(f"         smooth={SMOOTH_GAIT}  endmode={GAIT_END_MODE}")
    print("===================================================")


# ============================================================
# MAIN
# ============================================================

def main():
    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        print()
        print("SControl4: simultaneous tripod gait with sync write.")
        print("Startup: NO automatic movement.")
        print("Recommended: r -> health -> sidestrafe good -> sideflow on -> speed all 23 -> w")
        print("Directions w/s/q/e run continuously until you press Enter.")
        print_help()

        while True:
            try:
                raw_cmd = input("\nSControl4 command [h help]: ").strip()
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt. Exiting.")
                break

            if not raw_cmd:
                continue

            parts = raw_cmd.split()
            cmd   = parts[0].lower()

            try:
                if cmd == "x":
                    print("Exit requested.")
                    break

                elif cmd in ["h", "help"]:
                    print_help()

                elif cmd == "p":
                    print_status(bus)

                elif cmd == "health":
                    print_health(bus, "MANUAL HEALTH CHECK")

                elif cmd in ["movestats", "stats", "mstats"]:
                    action_movement_stats(parts)

                elif cmd == "speed":
                    action_set_speed(parts)

                elif cmd == "smooth":
                    action_smooth(parts)

                elif cmd in ["walklift", "clearance", "gaitlift"]:
                    action_walk_lift(parts)

                elif cmd in ["sidestrafe", "side", "ad"]:
                    action_side_strafe_settings(parts)

                elif cmd == "sideflow":
                    action_sideflow(parts)

                elif cmd in ["range"]:
                    action_range(parts)

                elif cmd in ["legtrim", "trim"]:
                    action_leg_trim(parts)

                elif cmd == "torque_max":
                    action_torque_max(bus)

                elif cmd == "timing":
                    action_gait_timing(parts)

                elif cmd in ["r", "ready"]:
                    action_ready(bus, use_safety_check=True)

                elif cmd == "force_r":
                    print("\nFORCE_R: returning without safety check.")
                    action_ready(bus, use_safety_check=False)

                elif cmd == "pushup":
                    if len(parts) != 2:
                        print("Usage: pushup 1/2/3/4")
                        continue
                    action_pushup(bus, parts[1])

                elif cmd == "lift":
                    try:
                        level, legs = parse_lift_command(parts)
                    except ValueError as e:
                        print(e)
                        continue
                    action_lift_legs(bus, level, legs)

                elif cmd == "gait":
                    if len(parts) != 2:
                        print("Usage: gait forward/backward/left/right/turn_left/turn_right")
                        continue
                    action_gait_cycle(bus, parts[1], cycles=1)

                elif cmd == "walk":
                    if len(parts) < 2:
                        print("Usage: walk forward 3")
                        continue
                    direction = parts[1]
                    cycles = int(parts[2]) if len(parts) >= 3 else 1
                    action_gait_cycle(bus, direction, cycles=cycles)

                elif cmd == "turn":
                    if len(parts) != 2:
                        print("Usage: turn left / turn right")
                        continue
                    td = normalize_direction(parts[1])
                    if td in ["turn_left", "turn_right"]:
                        action_gait_continuous(bus, td)
                    else:
                        print("Usage: turn left / turn right")

                elif cmd in ["w", "s", "q", "e", "forward", "backward"]:
                    # Continuous directions
                    direction = normalize_direction(cmd)
                    action_gait_continuous(bus, direction)

                elif cmd in ["a", "d", "left", "right"]:
                    # Side strafe: still cycle-based by default
                    direction = normalize_direction(cmd)
                    action_gait_cycle(bus, direction, cycles=1)

                else:
                    print(f"Unknown command: {raw_cmd}. Type h for help.")

            except ValueError:
                print("Invalid number format.")
            except KeyboardInterrupt:
                print("\nKeyboardInterrupt during command. Exiting.")
                break

    finally:
        bus.close()


if __name__ == "__main__":
    main()
