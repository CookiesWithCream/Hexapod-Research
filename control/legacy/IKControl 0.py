# ============================================================
# SCONTROLX2 - SEMI-OVERLAP TRIPOD GAIT + V5 NATIVE FOOT-SPACE IK TEST
# ============================================================
#
# What's new vs SControlX1:
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
# 3. SEMI-OVERLAP TRIPOD GAIT (safer than SControlX1 full handoff)
#    SControlX1 tried: A_DOWN+B_UP in the same phase.
#    On the real robot this can collapse into tiny wiggles because the next
#    cycle snaps hips between prebuilt READY-based targets.
#
#    SControlX2 uses a safer 6-phase sequence:
#      A_UP+B_PUSH -> A_SWING+B_PUSH -> A_DOWN+B_HOLD
#      B_UP+A_PUSH -> B_SWING+A_PUSH -> B_DOWN+A_HOLD
#
#    The support tripod still pushes while the swing tripod moves, but the
#    dangerous simultaneous touchdown/lift handoff is removed for stability.
#
# 4. CONTINUOUS GAIT LOOP
#    'w', 'a', 's', 'd', 'q', 'e' run continuously until you press Enter.
#    Each direction key starts gait and the loop repeats until interrupted.
#    No more "one cycle then stop".
#
# 5. TIMING KEPT TUNABLE
#    Sync write is preserved, but real AX motors under hexapod load may need
#    longer holds than simulation. Suggested test tuning:
#      speed all 25
#      walklift clear
#      smooth on
#      smooth steps 5
#      smooth hold 0.22
#      smooth settle 0.14
#
# Commands are identical to SControl3. The robot hardware, READY_POSE, motor IDs,
# joint signs, and all tuning presets are preserved exactly.
#
# Recommended forward/backward startup test:
#   r
#   health
#   speed all 25
#   walklift clear
#   smooth on
#   smooth steps 5
#   smooth hold 0.22
#   smooth settle 0.14
#   w          (runs until Enter)
#   r
#   s          (runs until Enter)
#   r
#
# Your usual side-strafe startup is still preserved:
#   r
#   health
#   sidestrafe good
#   movestats off
#   sideflow on
#   speed all 25
#   a
#   r
#   d
#   r
#
# ============================================================


# ============================================================
# SCONTROLX2 FULLSTEP NOTE
# ============================================================
# Built after real-world test feedback:
#   - SControlX2 was much better than X1.
#   - But step height/reach still looked smaller than the original SControl3.
# Main fix here:
#   - Keep sync-write and semi-overlap gait.
#   - Restore SControl3-like phase timing so AX motors can finish each lift/reach.
#   - Add smooth presets: smooth fullstep / smooth smoothfull.
# Recommended first test:
#   r
#   health
#   speed all 25
#   walklift clear
#   smooth fullstep
#   w
#   r
#   s
#   r
# If movement is complete but too jerky:
#   smooth smoothfull
# ============================================================

import sys
import time
import struct
import math
import threading
import io
import contextlib
from typing import Dict, Optional, Tuple, List

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

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

DEFAULT_PORT = None  # auto-detect/select serial port instead of hardcoded COM6
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
# BALANCED REFINED2K READY POSE - OFFSET-NORMALIZED TEST
# ============================================================

# The original READY_POSE is kept for traceability.
# This test file uses the user's latest calibration decision:
#   - HIPS use the direct measured calibrated centers (520 / 575 / pending AX-18A).
#   - FEMURS and TIBIAS keep the offset-normalized standing READY values.
#
# Meaning:
#   Hip READY is the calibrated spread/center pose found during hip testing.
#   Femur/tibia READY is still a standing pose derived from the old ready shape,
#   corrected by the measured physical centers so comparable joints share cleaner offsets.
OLD_READY_POSE_BEFORE_OFFSET_NORMALIZATION = {
    1:  460,  # RL_hip
    2:  747,  # FL_hip
    3:  411,  # FR_femur
    4:  366,  # FL_femur
    5:  798,  # FR_tibia
    6:  796,  # FL_tibia
    7:  608,  # MR_hip
    8:  753,  # ML_hip
    9:  627,  # MR_femur
    10:  437,  # ML_femur
    11:  216,  # MR_tibia
    12:  787,  # ML_tibia
    13:  578,  # RR_hip
    14:  575,  # FR_hip
    15:  641,  # RR_femur
    16:  412,  # RL_femur
    17:  189,  # RR_tibia
    18:  817,  # RL_tibia
}

READY_POSE = {
    # HIPS: latest direct calibrated center values from hip diagnostics.
    # All hips center at 520 except ID 7, which visually aligns at 575
    # because of the temporary horn/plate/screw mechanical offset.
    1:  520,  # RL_hip | calibrated hip center
    2:  520,  # FL_hip AX-18A | calibrated hip center
    7:  575,  # MR_hip | temporary calibrated center due plate/screw issue
    8:  520,  # ML_hip AX-18A | calibrated hip center
    13: 520,  # RR_hip | calibrated hip center
    14: 520,  # FR_hip | calibrated hip center

    # FEMURS: offset-normalized standing READY values from measured femur centers.
    3:  414,  # FR_femur
    4:  351,  # FL_femur | physical offset from 453 center
    9:  618,  # MR_femur | inverted direction
    10: 414,  # ML_femur
    15: 618,  # RR_femur | inverted direction
    16: 414,  # RL_femur

    # TIBIAS: offset-normalized standing READY values from measured tibia centers.
    5:  810,  # FR_tibia
    6:  810,  # FL_tibia
    11: 223,  # MR_tibia | inverted direction
    12: 810,  # ML_tibia
    17: 221,  # RR_tibia | inverted direction
    18: 810,  # RL_tibia
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
    1:  {"femur": -6.0,  "tibia": 6.0},
    2:  {"femur": -10.0, "tibia": 10.0},
    3:  {"femur": -14.0, "tibia": 14.0},
    4:  {"femur": -18.0, "tibia": 18.0},
    5:  {"femur": -22.0, "tibia": 22.0},
    6:  {"femur": -28.0, "tibia": 28.0},
    7:  {"femur": -32.0, "tibia": 32.0},
    8:  {"femur": -36.0, "tibia": 34.0},
    9:  {"femur": -40.0, "tibia": 36.0},
    10: {"femur": -44.0, "tibia": 38.0},
    11: {"femur": -48.0, "tibia": 40.0},
    12: {"femur": -52.0, "tibia": 42.0},
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
    "high4": {"femur": -46.0, "tibia": 36.0},
    "max":   {"femur": -52.0, "tibia": 42.0},
    "max12": {"femur": -52.0, "tibia": 42.0},
    "old6":  {"femur": -28.0, "tibia": 28.0},
    "low":   {"femur": -30.0, "tibia": 16.0},
    "clear": {"femur": -38.0, "tibia": 28.0},
    "high":  {"femur": -40.0, "tibia": 28.0},
}

LEG_FEMUR_LIFT_SCALE = {leg: 1.00 for leg in ALL_LEGS}
# Small real-hardware compensation: MR and RL were observed lifting lower than the
# other legs during gait, so give their femur/tibia lift command slightly more
# magnitude. RR keeps its previous conservative tibia scale.
LEG_FEMUR_LIFT_SCALE.update({"MR": 1.10, "RL": 1.10})
LEG_TIBIA_LIFT_SCALE = {
    "FL": 1.00, "ML": 1.00, "RL": 1.10,
    "FR": 1.00, "MR": 1.10, "RR": 0.85,
}

# ============================================================
# TIMING  (FULLSTEP COMPENSATION)
#
# SControl3 looked bigger because move_many() wrote motors one-by-one:
# speed writes + position writes + 6ms sleeps created extra real dwell time.
# SControlX1/X2 sync-write removed that serial delay, so the same gait could
# look smaller because the next phase interrupted before motors finished.
#
# This FULLSTEP version restores SControl3-like physical phase time while
# keeping sync-write simultaneous motor starts.
# ============================================================
# Phase delay = how long to wait after sending a phase so motors reach the pose.
# With individual write2() + 6ms sleeps, sending 18 motors took ~108ms.
# With sync write, all motors start simultaneously, so you can cut the wait.
#
# Adjust these if the robot doesn't have enough time to reach poses:
#   GAIT_PHASE_DELAY = 0.18   (slower, more time per phase)
#   GAIT_PHASE_DELAY = 0.10   (faster, spider-like)
#
GAIT_PHASE_DELAY        = 0.30    # fullstep: restored SControl3 physical dwell time
GAIT_SETTLE_DELAY       = 0.14    # fullstep: restored SControl3 touchdown settle
GAIT_FINAL_READY_DELAY  = 0.35
GAIT_END_RECENTER_DELAY = 0.10
GAIT_END_MODE = "tripod"

SMOOTH_GAIT       = False
SMOOTH_STEPS      = 3
SMOOTH_STEP_DELAY = 0.025

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

SIDE_STRAFE_HOLD   = 0.30   # restored from SControl3 working A/D strafe
SIDE_STRAFE_SETTLE = 0.14   # restored from SControl3 working A/D strafe

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

PUSHUP_LEVELS = PUSHUP_LEVELS = {
    "1": {1:530,2:530,3:428,4:373,5:788,6:780,7:565,8:530,9:607,10:410,11:242,12:806,13:510,14:510,15:617,16:425,17:230,18:791},
    "2": {1:540,2:540,3:442,4:387,5:768,6:760,7:555,8:540,9:593,10:424,11:262,12:786,13:500,14:500,15:603,16:439,17:250,18:771},
    "3": {1:551,2:551,3:456,4:401,5:727,6:719,7:544,8:551,9:579,10:438,11:303,12:745,13:489,14:489,15:589,16:453,17:291,18:730},
    "4": {1:561,2:561,3:469,4:414,5:686,6:678,7:534,8:561,9:566,10:451,11:344,12:704,13:479,14:479,15:576,16:466,17:332,18:689},
}


# ============================================================
# RUNTIME STATE
# ============================================================

ACTIVE_GOALS: Dict[int, int] = dict(READY_POSE)
CURRENT_MODE = "UNKNOWN"

# Persistent body-height level for web/controller mode.
# -7 = lower body / liftall-like, 0 = original READY_POSE, +7 = higher body.
BODY_HEIGHT_LEVEL = 0

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


# ============================================================
# PERSISTENT BODY HEIGHT LEVEL
# ============================================================
# Body height level is a persistent offset applied to READY pose and gait base.
#   0  = original READY_POSE
#  +7  = highest / more obstacle-clearance posture
#  -7  = lowest / closer-to-ground posture
#
# R2/LT in the controller raises this value.
# L2/LT in the controller lowers this value.
# Circle/B returns to ready using the CURRENT body level.
# Cross/A resets the body level to 0.
# ============================================================

BODY_HEIGHT_LEVEL = 0
BODY_HEIGHT_MIN = -7
BODY_HEIGHT_MAX = 7
BODY_HEIGHT_FEMUR_STEP_DEG = 4.5
BODY_HEIGHT_TIBIA_STEP_DEG = -4.5

# Smooth body-height transition:
# Instead of jumping one full body level at once, each level is split into smaller frames.
# Example: level -3 to -4 becomes -3.1, -3.2, ... -4.0.
BODY_HEIGHT_SMOOTH_ENABLED = True
BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL = 10
BODY_HEIGHT_SMOOTH_STEP_DELAY = 0.045


def clamp_body_height_level(level: int) -> int:
    return int(max(BODY_HEIGHT_MIN, min(BODY_HEIGHT_MAX, int(level))))


def clamp_body_height_value(level: float) -> float:
    return float(max(BODY_HEIGHT_MIN, min(BODY_HEIGHT_MAX, float(level))))


def level_ready_pose(level: Optional[int] = None) -> Dict[int, int]:
    """
    READY_POSE adjusted by persistent body-height level.

    Positive level:
      raises body / longer stance direction.

    Negative level:
      lowers body / liftall-like direction.
      At -7 this is close to old liftall level 7 depth.
    """
    if level is None:
        level = BODY_HEIGHT_LEVEL
    level = clamp_body_height_value(level)

    pose = dict(READY_POSE)
    femur_deg = level * BODY_HEIGHT_FEMUR_STEP_DEG
    tibia_deg = level * BODY_HEIGHT_TIBIA_STEP_DEG

    if abs(level) < 1e-9:
        return pose

    for leg in ALL_LEGS:
        femur_joint = leg_part_to_joint(leg, "femur")
        tibia_joint = leg_part_to_joint(leg, "tibia")
        femur_id = joint_to_motor_id(femur_joint)
        tibia_id = joint_to_motor_id(tibia_joint)

        pose[femur_id] = clamp_raw(READY_POSE[femur_id] + logical_deg_to_raw_delta(femur_joint, femur_deg))
        pose[tibia_id] = clamp_raw(READY_POSE[tibia_id] + logical_deg_to_raw_delta(tibia_joint, tibia_deg))

    return pose


def body_height_degrees(level: Optional[float] = None) -> Tuple[float, float]:
    if level is None:
        level = BODY_HEIGHT_LEVEL
    level = clamp_body_height_value(level)
    return level * BODY_HEIGHT_FEMUR_STEP_DEG, level * BODY_HEIGHT_TIBIA_STEP_DEG


def offset_from_ready(joint_name: str, deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    base_pose = level_ready_pose()
    return clamp_raw(base_pose[motor_id] + logical_deg_to_raw_delta(joint_name, deg))


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
        raise ValueError(f"Lift level must be {min(LIFT_LEVELS)}-{max(LIFT_LEVELS)}.")
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
# CALIBRATION OFFSET NORMALIZATION REPORT
# ============================================================
# These are NOT the standing READY pose. They are the measured visual-center
# raw positions from the diagnostic scripts while the robot was laid down.
# They are used only to calculate each joint's offset from its physical zero.
CALIBRATION_CENTER_RAW = {
    # hips measured; all hips center at 520 except ID 7 temporary mechanical offset
    1: 520,   # RL_hip measured center
    2: 520,   # FL_hip AX-18A measured center
    7: 575,   # MR_hip temporary center due horn/plate issue
    8: 520,   # ML_hip AX-18A measured center
    13: 520,  # RR_hip measured center
    14: 520,  # FR_hip measured center

    # femurs measured visual centers
    3: 516,   # FR_femur
    4: 453,   # FL_femur physical offset
    9: 516,   # MR_femur inverted direction
    10: 516,  # ML_femur
    15: 516,  # RR_femur inverted direction
    16: 516,  # RL_femur

    # tibias measured visual centers
    5: 366,   # FR_tibia
    6: 366,   # FL_tibia
    11: 667,  # MR_tibia inverted direction observed center
    12: 366,  # ML_tibia
    17: 665,  # RR_tibia inverted direction observed center
    18: 366,  # RL_tibia
}

CALIBRATION_COMPARE_SIGN = {
    # hips: direct physical center comparison; all calibrated centers should be 0 offset
    1: 1, 2: 1, 7: 1, 8: 1, 13: 1, 14: 1,
    # femurs: IDs 9 and 15 are mounted inverted
    3: 1, 4: 1, 9: -1, 10: 1, 15: -1, 16: 1,
    # tibias: IDs 11 and 17 are mounted inverted
    5: 1, 6: 1, 11: -1, 12: 1, 17: -1, 18: 1,
}

CALIBRATION_GROUPS = {
    "hip_all_direct_centers": [1, 2, 7, 8, 13, 14],
    "femur_all_signed": [3, 4, 9, 10, 15, 16],
    "tibia_all_signed": [5, 6, 11, 12, 17, 18],
}


def calibration_physical_offset(motor_id: int, pose: Optional[Dict[int, int]] = None) -> Optional[int]:
    """Signed offset from measured physical center. Comparable across inverted/mirrored joints."""
    if pose is None:
        pose = READY_POSE
    if motor_id not in CALIBRATION_CENTER_RAW:
        return None
    sign = CALIBRATION_COMPARE_SIGN.get(motor_id, 1)
    return int(sign * (pose[motor_id] - CALIBRATION_CENTER_RAW[motor_id]))


def print_calibration_normalization_report():
    print()
    print("===================================================")
    print(" CALIBRATION OFFSET NORMALIZATION - TEST VERSION")
    print("===================================================")
    print("This is NOT the flat/star pose as READY.")
    print("It uses the old standing READY pose, then normalizes each joint")
    print("by its measured physical center from the diagnostic scripts.")
    print("Hip IDs 1,2,8,13,14 use direct 520 centers; ID 7 uses temporary 575 center.")
    print()
    print(f"{'ID':>2} {'Joint':<10} {'Center':>6} {'OldReady':>8} {'NewReady':>8} {'Shift':>7} {'OldOff':>7} {'NewOff':>7}")
    print("-" * 78)
    for motor_id in ALL_MOTOR_IDS:
        joint = motor_id_to_joint(motor_id)
        center = CALIBRATION_CENTER_RAW.get(motor_id)
        old = OLD_READY_POSE_BEFORE_OFFSET_NORMALIZATION.get(motor_id)
        new = READY_POSE.get(motor_id)
        shift = new - old
        old_off = calibration_physical_offset(motor_id, OLD_READY_POSE_BEFORE_OFFSET_NORMALIZATION)
        new_off = calibration_physical_offset(motor_id, READY_POSE)
        center_s = str(center) if center is not None else "pending"
        old_off_s = str(old_off) if old_off is not None else "pending"
        new_off_s = str(new_off) if new_off is not None else "pending"
        print(f"{motor_id:>2} {joint:<10} {center_s:>6} {old:>8} {new:>8} {shift:>+7} {old_off_s:>7} {new_off_s:>7}")

    print()
    print("Group averages used:")
    for name, ids in CALIBRATION_GROUPS.items():
        vals = [calibration_physical_offset(mid, READY_POSE) for mid in ids]
        vals = [v for v in vals if v is not None]
        avg = sum(vals) / len(vals) if vals else 0.0
        print(f"  {name:<18}: {ids} -> average signed offset {avg:+.1f} raw")
    print("===================================================")



# ============================================================
# SERIAL PORT DETECTION / SELECTION
# ============================================================

def detected_serial_ports() -> List[Tuple[str, str, str]]:
    """
    Return detected serial ports as (device, description, hwid).
    Works on Windows COM ports and Linux/Raspberry Pi /dev/tty* ports.
    """
    if list_ports is None:
        return []

    ports = []
    for p in list_ports.comports():
        device = str(getattr(p, "device", "") or "")
        description = str(getattr(p, "description", "") or "")
        hwid = str(getattr(p, "hwid", "") or "")
        if device:
            ports.append((device, description, hwid))

    # Put common USB serial adapters first, but keep everything available.
    def score(item):
        dev, desc, hwid = item
        combined = f"{dev} {desc} {hwid}".lower()
        preferred = any(k in combined for k in [
            "usb", "u2d2", "ftdi", "ch340", "cp210", "acm", "serial"
        ])
        return (0 if preferred else 1, dev)

    return sorted(ports, key=score)


def print_serial_ports(ports: List[Tuple[str, str, str]]):
    print()
    print("===================================================")
    print(" SERIAL PORT DETECTION")
    print("===================================================")

    if not ports:
        print("No serial ports detected automatically.")
        print("On Ubuntu/Raspberry Pi, try checking:")
        print("  ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
        print("Common ports are /dev/ttyUSB0 or /dev/ttyACM0.")
        return

    print("Detected serial ports:")
    for i, (device, description, hwid) in enumerate(ports, start=1):
        print(f"  {i}) {device:<18} {description}")
        if hwid:
            print(f"     HWID: {hwid}")


def choose_serial_port() -> str:
    """
    Let user choose a port by menu number, exact port name, or auto.
    This replaces the old hardcoded COM6 behavior.
    """
    ports = detected_serial_ports()
    print_serial_ports(ports)

    print()
    print("Choose serial port:")
    print("  - Type menu number, e.g. 1")
    print("  - Type exact port, e.g. COM6, /dev/ttyUSB0, /dev/ttyACM0")
    print("  - Type auto or press Enter to use the first detected port")
    print("  - Type rescan to scan again")

    while True:
        choice = input("Port choice [auto]: ").strip()

        if choice == "" or choice.lower() == "auto":
            if ports:
                selected = ports[0][0]
                print(f"Auto-selected port: {selected}")
                return selected
            manual = input("No ports found. Enter port manually: ").strip()
            if manual:
                return manual
            continue

        if choice.lower() in ["scan", "rescan", "refresh"]:
            ports = detected_serial_ports()
            print_serial_ports(ports)
            continue

        if choice.isdigit():
            index = int(choice) - 1
            if 0 <= index < len(ports):
                selected = ports[index][0]
                print(f"Selected port: {selected}")
                return selected
            print("Invalid menu number. Try again.")
            continue

        print(f"Selected manual port: {choice}")
        return choice

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
        try:
            if not self.port_handler.openPort():
                print(f"FAILED: Cannot open {self.port_name}")
                return False
            if not self.port_handler.setBaudRate(BAUDRATE):
                print(f"FAILED: Cannot set baudrate {BAUDRATE}")
                return False
        except Exception as e:
            print(f"FAILED: Cannot open {self.port_name}")
            print(f"Reason: {e}")
            print("Tip: On Ubuntu/Raspberry Pi, use /dev/ttyUSB0 or /dev/ttyACM0, not COM6.")
            print("If you get permission denied, run: sudo usermod -a -G dialout $USER")
            print("Then reboot the Pi/Ubuntu system.")
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

    def invalidate_runtime_cache(self):
        """Forget Python-side motor RAM assumptions after CM530/Dynamixel power reset."""
        try:
            self._speed_cache.clear()
        except Exception:
            self._speed_cache = {}

    def rearm_after_power_cycle(self, speed: Optional[int] = None, motor_ids: Optional[List[int]] = None, reason: str = "") -> bool:
        """
        Re-send volatile Dynamixel RAM settings that can be lost when CM530/motor
        power is restarted while the Raspberry Pi Python process stays alive.

        This intentionally clears the speed cache first, because after a power
        cycle Python may still remember speed=25 even though the physical motors
        reverted to default/full-speed RAM state.
        """
        ids = list(motor_ids) if motor_ids is not None else list(ALL_MOTOR_IDS)
        self.invalidate_runtime_cache()

        ok = True
        for motor_id in ids:
            ok = bool(self.write1(motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)) and ok
            time.sleep(0.002)

        for motor_id in ids:
            ok = bool(self.write2(motor_id, ADDR_TORQUE_LIMIT, TORQUE_LIMIT_RAW)) and ok
            time.sleep(0.002)

        if speed is not None:
            ok = bool(self.sync_set_speeds(speed, ids)) and ok

        if reason:
            print(f"[REARM] {reason}: torque on, torque limit {TORQUE_LIMIT_RAW}, speed {speed if speed is not None else 'unchanged'}")
        return ok

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

        Safety fix: always re-send speed instead of trusting _speed_cache.
        CM530/motor power reset clears Dynamixel RAM moving-speed registers, but
        the Raspberry Pi process and Python cache remain alive. If we skip this
        write after a reset, the next goal-position packet can move at full/default
        motor speed.
        """
        ids = list(motor_ids) if motor_ids is not None else list(ALL_MOTOR_IDS)
        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, speed)))

        # Always include motors. Cache is updated only after a successful write.
        changed = list(ids)

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
        2. Set speed via sync write (always re-sent for CM530 reset safety).
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

    def move_many_legacy(self, targets: Dict[int, int], speed: int):
        """
        SControl3-style per-motor gait sender.

        This intentionally does NOT use sync write. The original script sent
        speed/position commands motor-by-motor with small sleeps, which gave
        AX motors extra real time to complete the large lift/swing. On the
        physical robot this produced a bigger visible step than pure sync-write.

        Use this only for forward/back/turn gait. Side strafe can still use
        sync-write through move_sync().
        """
        global ACTIVE_GOALS

        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, speed)))

        for motor_id in targets:
            self.enable_torque(motor_id)
            self.write2(motor_id, ADDR_MOVING_SPEED, speed)
            self._speed_cache[motor_id] = speed
            time.sleep(0.006)

        for motor_id, raw in targets.items():
            raw = clamp_raw(raw)
            ok = self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            if not ok:
                time.sleep(0.03)
                self.write2(motor_id, ADDR_GOAL_POSITION, raw)
            ACTIVE_GOALS[motor_id] = raw
            time.sleep(0.006)


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


def motor_row_warnings(
    pos: Optional[int],
    load_value: Optional[int],
    volt: Optional[float],
    temp: Optional[int],
) -> str:
    """Return compact per-motor warning text for the full status table."""
    warnings: List[str] = []

    if pos is None:
        warnings.append("NO_REPLY")
    if volt is None:
        warnings.append("NO_VOLT")
    elif volt <= VOLT_DANGER_V:
        warnings.append("DANGER_VOLT")
    elif volt <= VOLT_STOP_V:
        warnings.append("VOLT_STOP")
    elif volt <= VOLT_WARN_V:
        warnings.append("LOW_VOLT")

    if temp is None:
        warnings.append("NO_TEMP")
    elif temp >= TEMP_STOP_C:
        warnings.append("TEMP_STOP")
    elif temp >= TEMP_WARN_C:
        warnings.append("TEMP_WARN")

    if load_value is None:
        warnings.append("NO_LOAD")
    elif abs(load_value) >= LOAD_STOP:
        warnings.append("LOAD_STOP")
    elif abs(load_value) >= LOAD_WARN:
        warnings.append("LOAD_WARN")

    return ",".join(warnings) if warnings else "OK"


def print_status(bus: DynamixelBus):
    """
    Full motor diagnostic table.

    This restores the older tuner-style output while keeping the same command
    name (`p`) and without changing any gait/control logic.

    Web UI note:
      Every time `p` is executed, the same motor rows are cached into
      WEB_LAST_MOTOR_STATUS so the browser can show a stable status board
      without relying on the scrolling terminal log.
    """
    global WEB_LAST_MOTOR_STATUS

    print()
    print("===================================================")
    print(" MOTOR STATUS / FULL DIAGNOSTICS")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<3} {'Part':<5} "
        f"{'Raw':>5} {'DegZero':>8} {'Zero':>5} {'Goal':>5} "
        f"{'Load':>7} {'Volt':>5} {'Temp':>5} {'Warnings'}"
    )
    print("-" * 104)

    rows: List[Dict[str, object]] = []
    warn_count = 0

    for motor_id in ALL_MOTOR_IDS:
        joint = motor_id_to_joint(motor_id)
        leg, part = joint_to_leg_part(joint)
        zero = READY_POSE.get(motor_id, 512)
        goal = ACTIVE_GOALS.get(motor_id, "?")

        pos = bus.read2(motor_id, ADDR_PRESENT_POSITION)
        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
        volt_raw = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

        load_value = decode_load_value(load_raw)
        load_s = decode_load_text(load_raw)

        if pos is None:
            raw_s = "----"
            deg_s = "----"
        else:
            raw_s = str(pos)
            deg_zero = (pos - zero) / RAW_PER_DEG
            deg_s = f"{deg_zero:+.2f}"

        volt = (volt_raw / 10.0) if volt_raw is not None else None
        volt_s = f"{volt:.1f}" if volt is not None else "----"
        temp_s = str(temp) if temp is not None else "----"
        warn_s = motor_row_warnings(pos, load_value, volt, temp)

        if warn_s != "OK":
            warn_count += 1

        rows.append({
            "id": motor_id,
            "joint": joint,
            "leg": leg,
            "part": part,
            "raw": raw_s,
            "deg_zero": deg_s,
            "zero": zero,
            "goal": str(goal),
            "load": load_s,
            "load_value": load_value,
            "volt": volt_s,
            "temp": temp_s,
            "warnings": warn_s,
            "ok": warn_s == "OK",
        })

        print(
            f"{motor_id:>2} {joint:<10} {leg:<3} {part:<5} "
            f"{raw_s:>5} {deg_s:>8} {zero:>5} {str(goal):>5} "
            f"{load_s:>7} {volt_s:>5} {temp_s:>5} {warn_s}"
        )

    try:
        WEB_LAST_MOTOR_STATUS = {
            "time": time.strftime("%H:%M:%S"),
            "rows": rows,
            "warn_count": warn_count,
            "summary": "OK - all motor rows normal" if warn_count == 0 else f"WARN - {warn_count} motor row(s) need attention",
        }
    except Exception:
        # Terminal mode should never fail just because the web status cache is unavailable.
        pass

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

    # Re-apply volatile RAM settings before the first movement after a CM530/motor
    # restart. This keeps web/terminal W/A/S/D safe even if the Raspberry Pi script
    # was never restarted.
    try:
        bus.rearm_after_power_cycle(GAIT_SPEED, reason="pre-motion hardware sync")
    except Exception as e:
        print(f"[REARM WARNING] {type(e).__name__}: {e}")
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
# EXPERIMENTAL INVERSE KINEMATICS LAYER
# ============================================================
# This layer keeps your existing READY_POSE, motor IDs, body-height system,
# web UI, controller, safety checks, and old fixed gait intact.
#
# How it works:
#   1) READY_POSE is treated as the calibrated standing reference.
#   2) Each leg has an assumed default foot coordinate in centimetres.
#   3) A movement command changes that foot coordinate, e.g. +5 cm forward
#      and +3 cm lift.
#   4) IK solves the hip/femur/tibia angles needed to reach that coordinate.
#   5) The angle difference from the READY foot coordinate becomes the servo
#      offset sent through your existing build_leg_offset_targets() function.
#
# Important:
#   This is a first real-hardware IK test layer, not a final calibrated IK map.
#   If a leg moves in the wrong direction, tune IK_OUTPUT_SIGN below instead of
#   rewriting the IK math.
# ============================================================

IK_ENABLED = False

# Your measured leg segment lengths.
IK_COXA_CM  = 5.1
IK_FEMUR_CM = 9.2
IK_TIBIA_CM = 13.2

# Conservative first-test movement values.
# After signs are confirmed, increase STEP from 3.0 toward 5.0 cm.
IK_STEP_CM         = 7.0
IK_SUPPORT_PUSH_CM = 3.8
IK_LIFT_CM         = 5.0
IK_TURN_DEG        = 7.0

# Assumed standing foot positions relative to the body, in centimetres.
# x = front/back, y = left/right, z = vertical. z is negative because the foot
# is below the body. These do not need to be perfect for the first test because
# the IK is used relatively from READY_POSE.
IK_DEFAULT_FEET_CM = {
    "FL": {"x":  6.0, "y":  9.0, "z": -10.0},
    "ML": {"x":  0.0, "y": 11.0, "z": -10.0},
    "RL": {"x": -6.0, "y":  9.0, "z": -10.0},
    "FR": {"x":  6.0, "y": -9.0, "z": -10.0},
    "MR": {"x":  0.0, "y":-11.0, "z": -10.0},
    "RR": {"x": -6.0, "y": -9.0, "z": -10.0},
}

# Output correction signs applied after the IK angle difference is calculated.
# Keep this table easy to edit during real-hardware testing.
IK_OUTPUT_SIGN = {
    # Important lift fix:
    # The geometric IK convention returns +femur when the target foot goes upward,
    # but your proven fixed gait uses NEGATIVE logical femur degrees for lift
    # e.g. walklift clear = femur -38, tibia +28.
    # Therefore femur is inverted here so IK lift follows the same logical
    # direction as your current working gait.
    "FL": {"hip": -1.0, "femur": -1.0, "tibia": 1.0},
    "ML": {"hip": -1.0, "femur": -1.0, "tibia": 1.0},
    "RL": {"hip": -1.0, "femur": -1.0, "tibia": 1.0},
    "FR": {"hip":  1.0, "femur": -1.0, "tibia": 1.0},
    "MR": {"hip":  1.0, "femur": -1.0, "tibia": 1.0},
    "RR": {"hip":  1.0, "femur": -1.0, "tibia": 1.0},
}

# Extra safety clamp for IK-generated logical offsets before raw conversion.
IK_MAX_HIP_DELTA_DEG   = 42.0
IK_MAX_FEMUR_DELTA_DEG = 44.0
IK_MAX_TIBIA_DELTA_DEG = 44.0


def clamp_float(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


def safe_acos(value: float) -> float:
    return math.acos(clamp_float(value, -1.0, 1.0))


def copy_foot(foot: Dict[str, float]) -> Dict[str, float]:
    return {"x": float(foot["x"]), "y": float(foot["y"]), "z": float(foot["z"])}


def ik_solve_absolute_angles(foot: Dict[str, float]) -> Tuple[float, float, float]:
    """
    Return approximate absolute (hip, femur, tibia) angles for one 3DOF leg.

    This is a standard coxa/femur/tibia geometric IK solve:
      - hip is solved from top view using x and outward y distance
      - femur/tibia are solved from the 2D side triangle

    Since your real servo zero positions differ per joint, the final gait uses
    only angle DIFFERENCES from the default READY foot point.
    """
    x = float(foot["x"])
    y = float(foot["y"])
    z = float(foot["z"])

    outward = max(0.1, abs(y))
    hip_deg = math.degrees(math.atan2(x, outward))

    horizontal = math.hypot(x, outward)
    leg_plane = max(0.1, horizontal - IK_COXA_CM)
    distance = math.hypot(leg_plane, z)

    # Keep the requested target inside the mathematical reachable range.
    min_reach = abs(IK_FEMUR_CM - IK_TIBIA_CM) + 0.05
    max_reach = (IK_FEMUR_CM + IK_TIBIA_CM) - 0.05
    distance = clamp_float(distance, min_reach, max_reach)

    base_angle = math.atan2(z, leg_plane)
    femur_inner = safe_acos(
        (IK_FEMUR_CM ** 2 + distance ** 2 - IK_TIBIA_CM ** 2) /
        (2.0 * IK_FEMUR_CM * distance)
    )
    femur_deg = math.degrees(base_angle + femur_inner)

    knee_inner = safe_acos(
        (IK_FEMUR_CM ** 2 + IK_TIBIA_CM ** 2 - distance ** 2) /
        (2.0 * IK_FEMUR_CM * IK_TIBIA_CM)
    )
    # Convert internal triangle angle to a more servo-like bend amount.
    tibia_deg = 180.0 - math.degrees(knee_inner)

    return hip_deg, femur_deg, tibia_deg


def ik_relative_leg_degrees(leg: str, target_foot: Dict[str, float]) -> Tuple[float, float, float]:
    base_foot = IK_DEFAULT_FEET_CM[leg]
    base_h, base_f, base_t = ik_solve_absolute_angles(base_foot)
    tgt_h, tgt_f, tgt_t = ik_solve_absolute_angles(target_foot)

    sign = IK_OUTPUT_SIGN.get(leg, {"hip": 1.0, "femur": 1.0, "tibia": 1.0})
    hip_delta = (tgt_h - base_h) * sign.get("hip", 1.0)
    femur_delta = (tgt_f - base_f) * sign.get("femur", 1.0)
    tibia_delta = (tgt_t - base_t) * sign.get("tibia", 1.0)

    hip_delta = clamp_float(hip_delta, -IK_MAX_HIP_DELTA_DEG, IK_MAX_HIP_DELTA_DEG)
    femur_delta = clamp_float(femur_delta, -IK_MAX_FEMUR_DELTA_DEG, IK_MAX_FEMUR_DELTA_DEG)
    tibia_delta = clamp_float(tibia_delta, -IK_MAX_TIBIA_DELTA_DEG, IK_MAX_TIBIA_DELTA_DEG)

    return hip_delta, femur_delta, tibia_delta


def build_leg_ik_targets(leg: str, foot_target: Dict[str, float]) -> Dict[int, int]:
    hip_deg, femur_deg, tibia_deg = ik_relative_leg_degrees(leg, foot_target)
    return build_leg_offset_targets(leg, hip_deg, femur_deg, tibia_deg)


def ik_direction_step(direction: str) -> Tuple[float, float]:
    direction = normalize_direction(direction)
    if direction == "forward":
        return IK_STEP_CM, 0.0
    if direction == "backward":
        return -IK_STEP_CM, 0.0
    if direction == "left":
        return 0.0, IK_STEP_CM
    if direction == "right":
        return 0.0, -IK_STEP_CM
    return 0.0, 0.0


def ik_support_step(direction: str) -> Tuple[float, float]:
    dx, dy = ik_direction_step(direction)
    # Grounded/support legs move opposite relative to the body.
    if IK_STEP_CM == 0:
        return 0.0, 0.0
    scale = IK_SUPPORT_PUSH_CM / IK_STEP_CM
    return -dx * scale, -dy * scale


def rotate_xy(x: float, y: float, deg: float) -> Tuple[float, float]:
    rad = math.radians(deg)
    c = math.cos(rad)
    s = math.sin(rad)
    return x * c - y * s, x * s + y * c


def ik_target_for_leg(leg: str, direction: str, role: str) -> Dict[str, float]:
    """
    role:
      lifted_up      = lift vertically from READY
      lifted_swing   = lift + move foot toward desired step
      lifted_down    = foot at step target but back on ground
      support_push   = grounded leg pushes opposite direction
      ready          = READY foot coordinate
    """
    direction = normalize_direction(direction)
    foot = copy_foot(IK_DEFAULT_FEET_CM[leg])

    if direction in ["turn_left", "turn_right"]:
        turn = IK_TURN_DEG if direction == "turn_left" else -IK_TURN_DEG
        if role in ["lifted_swing", "lifted_down"]:
            foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], turn)
        elif role == "support_push":
            foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], -turn * 0.65)
    else:
        if role in ["lifted_swing", "lifted_down"]:
            dx, dy = ik_direction_step(direction)
            foot["x"] += dx
            foot["y"] += dy
        elif role == "support_push":
            dx, dy = ik_support_step(direction)
            foot["x"] += dx
            foot["y"] += dy

    if role in ["lifted_up", "lifted_swing"]:
        foot["z"] += IK_LIFT_CM

    return foot


def build_tripod_phase_ik(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,
    support_push_active: bool = True,
) -> Dict[int, int]:
    targets = level_ready_pose()
    direction = normalize_direction(direction)

    for leg in lifted_legs:
        if phase == "up":
            role = "lifted_up"
        elif phase == "swing":
            role = "lifted_swing"
        elif phase == "down":
            role = "lifted_down"
        else:
            role = "ready"
        targets.update(build_leg_ik_targets(leg, ik_target_for_leg(leg, direction, role)))

    for leg in support_legs:
        role = "support_push" if support_push_active else "ready"
        targets.update(build_leg_ik_targets(leg, ik_target_for_leg(leg, direction, role)))

    return targets


def build_side_strafe_targets_ik(active_tripod, other_tripod, direction, phase) -> Dict[int, int]:
    targets = level_ready_pose()
    direction = normalize_direction(direction)

    for leg in active_tripod:
        if phase == "up_pull":
            role = "lifted_up"
        elif phase == "reach_pull":
            role = "lifted_swing"
        elif phase == "down_pull":
            role = "lifted_down"
        else:
            role = "ready"
        targets.update(build_leg_ik_targets(leg, ik_target_for_leg(leg, direction, role)))

    for leg in other_tripod:
        role = "support_push" if phase in ["up_pull", "reach_pull", "down_pull"] else "ready"
        targets.update(build_leg_ik_targets(leg, ik_target_for_leg(leg, direction, role)))

    return targets


def print_ik_status():
    print()
    print("===================================================")
    print(" EXPERIMENTAL IK STATUS")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step / support cm : step={IK_STEP_CM:.1f}, support={IK_SUPPORT_PUSH_CM:.1f}, lift={IK_LIFT_CM:.1f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f}")
    print("Commands:")
    print("  ik on / ik off")
    print("  ik step 7.0")
    print("  ik support 2.6")
    print("  ik lift 5.0")
    print("  ik turn 7")
    print("  ik preview FR forward")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_ENABLED, IK_STEP_CM, IK_SUPPORT_PUSH_CM, IK_LIFT_CM, IK_TURN_DEG

    if len(parts) == 1:
        print_ik_status()
        return

    sub = parts[1].lower()
    if sub in ["on", "true", "1", "enable", "enabled"]:
        IK_ENABLED = True
        print("Experimental IK gait ON. Bigger-step preset loaded: ik step 7.0 ; ik lift 5.0")
    elif sub in ["off", "false", "0", "disable", "disabled"]:
        IK_ENABLED = False
        print("Experimental IK gait OFF. Reverted to original fixed-degree gait builders.")
    elif sub == "step" and len(parts) >= 3:
        IK_STEP_CM = clamp_float(float(parts[2]), 0.5, 7.0)
        print(f"IK step = {IK_STEP_CM:.2f} cm")
    elif sub == "support" and len(parts) >= 3:
        IK_SUPPORT_PUSH_CM = clamp_float(float(parts[2]), 0.0, 5.0)
        print(f"IK support push = {IK_SUPPORT_PUSH_CM:.2f} cm")
    elif sub == "lift" and len(parts) >= 3:
        IK_LIFT_CM = clamp_float(float(parts[2]), 0.5, 6.0)
        print(f"IK lift = {IK_LIFT_CM:.2f} cm")
    elif sub == "turn" and len(parts) >= 3:
        IK_TURN_DEG = clamp_float(float(parts[2]), 1.0, 15.0)
        print(f"IK turn = {IK_TURN_DEG:.2f} deg")
    elif sub == "preview" and len(parts) >= 4:
        leg = parts[2].upper()
        direction = normalize_direction(parts[3])
        if leg not in ALL_LEGS:
            print(f"Unknown leg: {leg}")
            return
        for role in ["ready", "lifted_up", "lifted_swing", "lifted_down", "support_push"]:
            foot = ik_target_for_leg(leg, direction, role)
            h, f, t = ik_relative_leg_degrees(leg, foot)
            print(f"{leg} {direction:<10} {role:<13} foot={foot}  deg: hip={h:+.2f}, femur={f:+.2f}, tibia={t:+.2f}")
    else:
        print("Usage: ik on/off | ik step 7.0 | ik support 2.6 | ik lift 5.0 | ik turn 7 | ik preview FR forward")

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
    if IK_ENABLED:
        return build_tripod_phase_ik(lifted_legs, support_legs, direction, phase, support_push_active)

    targets = level_ready_pose()
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


def send_phase_fullswing(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    Forward/back/turn gait sender that restores the original SControl3 physical
    swing size by using legacy per-motor sending instead of sync write.

    Why: sync write is technically cleaner, but on this AX hexapod it made the
    real stride look only around 3-4/10 of the original. The legacy sender adds
    the same tiny motor-to-motor pacing that the original movement script had.
    """
    global ACTIVE_GOALS

    if SMOOTH_GAIT and SMOOTH_STEPS > 1:
        start = dict(ACTIVE_GOALS)
        frames = interpolate_targets(start, targets, SMOOTH_STEPS)
        for frame in frames:
            bus.move_many_legacy(frame, speed=speed)
            ACTIVE_GOALS = dict(frame)
            time.sleep(SMOOTH_STEP_DELAY)
        time.sleep(hold)
    else:
        bus.move_many_legacy(targets, speed=speed)
        ACTIVE_GOALS = dict(targets)
        time.sleep(hold)

    if label and SIDE_STRAFE_FLOW_PRINT_PHASES:
        print(f"  {label}: sent")




def send_phase_side_legacy(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    A/D side-strafe sender restored from the SControl3 behavior.

    Reason:
      The physical robot's working A/D strafe depended on the same per-motor
      pacing as the original gait. Pure sync write made the side strafe too
      abrupt/small/broken, similar to the earlier tiny forward/back wiggle.

    This uses the legacy sender for A/D phases only, while preserving the
    current working forward/backward/turn behavior.
    """
    global ACTIVE_GOALS

    # For A/D, keep the SControl3/WControl71 feel: no extra flow hold when
    # sideflow is on, but still use legacy motor-to-motor pacing.
    if SIDE_STRAFE_DEBUG_STEPS_ENABLED:
        start = dict(ACTIVE_GOALS)
        frames = interpolate_targets(start, targets, SIDE_STRAFE_DEBUG_STEPS)
        for frame in frames:
            bus.move_many_legacy(frame, speed=speed)
            ACTIVE_GOALS = dict(frame)
            time.sleep(SIDE_STRAFE_DEBUG_STEP_DELAY)
        time.sleep(min(hold, 0.06))
    else:
        bus.move_many_legacy(targets, speed=speed)
        ACTIVE_GOALS = dict(targets)
        time.sleep(hold)

    if label and SIDE_STRAFE_FLOW_PRINT_PHASES:
        print(f"  {label}: sent")

# ============================================================
# SCONTROLX2 SEMI-OVERLAP TRIPOD GAIT
# ============================================================
#
# Why this exists:
#   SControlX1 used a fully simultaneous handoff:
#     A_DOWN + B_UP in one phase.
#
#   On the real robot, that looked like tiny wiggles because the next cycle
#   reused READY-based target frames and the legs did not get enough time to
#   complete a useful step before the next tripod started changing state.
#
# SControlX2 safer sequence:
#   Phase 1: A_UP      + B_SUPPORT_PUSH
#   Phase 2: A_SWING   + B_SUPPORT_PUSH
#   Phase 3: A_DOWN    + B_SUPPORT_HOLD/PUSH
#   Phase 4: B_UP      + A_SUPPORT_PUSH
#   Phase 5: B_SWING   + A_SUPPORT_PUSH
#   Phase 6: B_DOWN    + A_SUPPORT_HOLD/PUSH
#
# This is not full spider-style handoff yet. It is a stable stepping bridge:
# the support tripod still drives the body while the swing tripod moves, but
# touchdown and next-tripod lift are separated.
#
# ============================================================

def build_simultaneous_gait_phases(direction: str) -> List[Tuple[str, Dict[int, int], float]]:
    """
    Build one SControlX2 semi-overlap tripod gait cycle.

    Kept name build_simultaneous_gait_phases() for compatibility with the
    rest of the old code, but this version intentionally removes the risky
    A_DOWN+B_UP same-frame handoff from SControlX1.
    """
    direction = normalize_direction(direction)

    phases: List[Tuple[str, Dict[int, int], float]] = []

    # 1) Tripod A lifts. Tripod B stays grounded and starts/supports push.
    targets = build_tripod_phase(
        TRIPOD_A, TRIPOD_B, direction, "up", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_A_UP+B_PUSH", targets, GAIT_PHASE_DELAY))

    # 2) Tripod A swings while still lifted. Tripod B continues push.
    targets = build_tripod_phase(
        TRIPOD_A, TRIPOD_B, direction, "swing", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_A_SWING+B_PUSH", targets, GAIT_PHASE_DELAY))

    # 3) Tripod A lands first. Tripod B does NOT lift yet.
    # This separated touchdown prevents the SControlX1 tiny-wiggle handoff.
    targets = build_tripod_phase(
        TRIPOD_A, TRIPOD_B, direction, "down", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_A_DOWN+B_HOLD", targets, GAIT_SETTLE_DELAY))

    # 4) Now Tripod B lifts. Tripod A becomes the grounded support/push tripod.
    targets = build_tripod_phase(
        TRIPOD_B, TRIPOD_A, direction, "up", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_B_UP+A_PUSH", targets, GAIT_PHASE_DELAY))

    # 5) Tripod B swings while lifted. Tripod A continues push.
    targets = build_tripod_phase(
        TRIPOD_B, TRIPOD_A, direction, "swing", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_B_SWING+A_PUSH", targets, GAIT_PHASE_DELAY))

    # 6) Tripod B lands. Tripod A remains grounded/supporting.
    targets = build_tripod_phase(
        TRIPOD_B, TRIPOD_A, direction, "down", support_push_active=True
    )
    phases.append((f"GAIT_{direction}_B_DOWN+A_HOLD", targets, GAIT_SETTLE_DELAY))

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


# Stronger airborne recenter for IK mode.
# The normal gait recenter was originally written for fixed joint-space gait.
# In IK mode, the last grounded pose can have femur/tibia offsets from the IK
# target. If the hip is returned to READY while the foot is not clearly airborne,
# the foot may scrape/drag across the floor. This patch lifts each tripod from
# its CURRENT pose first, recenters hip while airborne, then lowers to READY.
IK_RECENTER_EXTRA_FEMUR_DEG = -34.0
IK_RECENTER_EXTRA_TIBIA_DEG = 26.0
IK_RECENTER_LIFT_HOLD = 0.26
IK_RECENTER_HIP_HOLD = 0.24
IK_RECENTER_DOWN_HOLD = 0.18


def pose_add_leg_lift_from_current(pose: Dict[int, int], leg: str, femur_deg: float, tibia_deg: float):
    """Add lift relative to the current pose, instead of overwriting from READY."""
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")
    femur_id = joint_to_motor_id(femur_joint)
    tibia_id = joint_to_motor_id(tibia_joint)

    pose[femur_id] = clamp_raw(pose.get(femur_id, ACTIVE_GOALS.get(femur_id, READY_POSE[femur_id])) + logical_deg_to_raw_delta(femur_joint, femur_deg))
    pose[tibia_id] = clamp_raw(pose.get(tibia_id, ACTIVE_GOALS.get(tibia_id, READY_POSE[tibia_id])) + logical_deg_to_raw_delta(tibia_joint, tibia_deg))


def pose_set_leg_ready_all(pose: Dict[int, int], leg: str):
    """Set hip/femur/tibia of one leg to the current body-height READY pose."""
    base = level_ready_pose()
    for part in ["hip", "femur", "tibia"]:
        joint = leg_part_to_joint(leg, part)
        mid = joint_to_motor_id(joint)
        pose[mid] = base[mid]


def final_tripod_recenter_ik(bus: DynamixelBus, direction: str):
    """
    IK-specific final recenter:
      1) lift tripod from its CURRENT stance
      2) return hip to READY while foot is still lifted
      3) lower all joints to READY

    This prevents the final reset from dragging feet across the floor.
    """
    global ACTIVE_GOALS, CURRENT_MODE
    pose = dict(ACTIVE_GOALS)
    base_ready = level_ready_pose()

    for group_name, tripod in [("A", TRIPOD_A), ("B", TRIPOD_B)]:
        CURRENT_MODE = f"GAIT_{direction}_END_IK_RECENTER_{group_name}_LIFT_CURRENT"
        lift_pose = dict(pose)
        for leg in tripod:
            pose_add_leg_lift_from_current(
                lift_pose,
                leg,
                IK_RECENTER_EXTRA_FEMUR_DEG,
                IK_RECENTER_EXTRA_TIBIA_DEG,
            )
        send_phase_fullswing(bus, lift_pose, GAIT_SPEED, IK_RECENTER_LIFT_HOLD, CURRENT_MODE)

        CURRENT_MODE = f"GAIT_{direction}_END_IK_RECENTER_{group_name}_HIP_READY_AIRBORNE"
        hip_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            hip_joint = leg_part_to_joint(leg, "hip")
            hip_id = joint_to_motor_id(hip_joint)
            hip_pose[hip_id] = base_ready[hip_id]
        send_phase_fullswing(bus, hip_pose, GAIT_SPEED, IK_RECENTER_HIP_HOLD, CURRENT_MODE)

        CURRENT_MODE = f"GAIT_{direction}_END_IK_RECENTER_{group_name}_DOWN_READY"
        down_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            pose_set_leg_ready_all(down_pose, leg)
        send_phase_fullswing(bus, down_pose, GAIT_SPEED, IK_RECENTER_DOWN_HOLD, CURRENT_MODE)

        pose = dict(ACTIVE_GOALS)

    CURRENT_MODE = "READY_REFINED2K"
    send_phase(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
    ACTIVE_GOALS = level_ready_pose()


def final_tripod_recenter(bus: DynamixelBus, direction: str):
    global ACTIVE_GOALS, CURRENT_MODE

    if IK_ENABLED:
        print("Final IK recenter: lift current tripod -> hip ready while airborne -> down")
        final_tripod_recenter_ik(bus, direction)
        return

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
    send_phase(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
    ACTIVE_GOALS = level_ready_pose()


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
    print(f" SCONTROLX2 ORIGINAL-SWING LEGACY GAIT: {direction.upper()}")
    print("===================================================")
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()
    print(f"Gait lift: femur {lift_femur:+.1f} deg, tibia {lift_tibia:+.1f} deg")
    print(f"Hip swing: {hip_swing_deg} deg  Support push: {support_push_deg} deg")
    print(f"Speed: {GAIT_SPEED}   Phase delay: {GAIT_PHASE_DELAY:.3f}s   Settle: {GAIT_SETTLE_DELAY:.3f}s")
    print(f"Gait sender: LEGACY per-motor pacing for larger physical swing")
    print(f"Gait mode: SControlX2 original-swing legacy sender for full physical step")
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

    while not stop_flag.is_set():
        cycle += 1
        print(f"  cycle {cycle}", end="\r", flush=True)

        if GAIT_PRECHECK_EACH_PHASE:
            if not pre_motion_check(bus):
                break

        # Rebuild phases each cycle so tuning changes and gait state stay fresh.
        phases = build_simultaneous_gait_phases(direction)

        for label, targets, hold in phases:
            CURRENT_MODE = label
            send_phase_fullswing(bus, targets, GAIT_SPEED, hold, label if GAIT_PHASE_HEALTH else "")
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
        ACTIVE_GOALS = level_ready_pose()
        send_phase(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
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
    print(f" SCONTROLX2 ORIGINAL-SWING LEGACY GAIT {direction.upper()} x{cycles}")
    print("===================================================")
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()
    print(f"Gait lift: femur {lift_femur:+.1f}, tibia {lift_tibia:+.1f}")
    print(f"Hip swing: {hip_swing_deg}   Support: {support_push_deg}   Speed: {GAIT_SPEED}")
    print("===================================================")

    if not pre_motion_check(bus):
        return

    for i in range(cycles):
        print(f"\n--- CYCLE {i+1}/{cycles} ---")
        if GAIT_PRECHECK_EACH_PHASE and not pre_motion_check(bus):
            break
        # Rebuild each cycle instead of reusing old prebuilt frames.
        phases = build_simultaneous_gait_phases(direction)
        for label, targets, hold in phases:
            CURRENT_MODE = label
            send_phase_fullswing(bus, targets, GAIT_SPEED, hold, label)
        print_health(bus, f"AFTER CYCLE {i+1} {direction}")

    if GAIT_END_MODE == "tripod":
        print("Final recenter...")
        final_tripod_recenter(bus, direction)
    elif GAIT_END_MODE == "direct":
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = level_ready_pose()
        send_phase(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)

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
    if IK_ENABLED:
        return build_side_strafe_targets_ik(active_tripod, other_tripod, direction, phase)

    targets = level_ready_pose()
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
    """
    Restored SControl3-style A/D recenter.
    Lift one tripod, return it to READY, then lift the other tripod.
    Uses legacy pacing so feet do not snap/drag.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    for idx, legs in enumerate([CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD], start=1):
        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_UP"
        targets = level_ready_pose()
        for leg in legs:
            h, f, t = side_strafe_leg_offsets(leg, direction, "lift")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        send_phase_side_legacy(bus, targets, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_READY"
        targets = level_ready_pose()
        send_phase_side_legacy(bus, targets, GAIT_SPEED, GAIT_END_RECENTER_DELAY, CURRENT_MODE)

    CURRENT_MODE = "READY_REFINED2K"
    send_phase_side_legacy(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)
    ACTIVE_GOALS = level_ready_pose()


def _run_side_strafe_cycle_body(bus: DynamixelBus, direction: str, cycle_label: str = "") -> bool:
    """
    One restored SControl3/WControl23 A/D strafe cycle.
    Returns False if safety check blocks movement.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    if not pre_motion_check(bus):
        return False

    phases = [
        (f"SIDE_{direction}_B_UP_A_PULL",     CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "up_pull",    SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_B_REACH_A_PULL",  CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "reach_pull", SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_B_DOWN_A_PULL",   CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "down_pull",  SIDE_STRAFE_SETTLE),
        (f"SIDE_{direction}_A_UP_B_PULL",     CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "up_pull",    SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_A_REACH_B_PULL",  CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "reach_pull", SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_A_DOWN_B_PULL",   CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "down_pull",  SIDE_STRAFE_SETTLE),
    ]

    for mode_name, active, other, phase, delay in phases:
        CURRENT_MODE = mode_name
        targets = build_side_strafe_targets(active, other, direction, phase)
        effective_delay = SIDE_STRAFE_FLOW_HOLD if SIDE_STRAFE_FLOW_MODE else delay
        send_phase_side_legacy(
            bus,
            targets,
            GAIT_SPEED,
            effective_delay,
            mode_name if SIDE_STRAFE_FLOW_PRINT_PHASES else "",
        )
        if GAIT_PHASE_HEALTH:
            print_health(bus, CURRENT_MODE)

    return True


def action_side_strafe_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    """
    Restored SControl3 working A/D side strafe.
    Uses legacy per-motor pacing like the fixed forward/backward swing.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["left", "right"]:
        print("Side strafe only supports left/right.")
        return

    cycles = max(1, min(10, int(cycles)))

    print()
    print("===================================================")
    print(f" RESTORED SCONTROL3 A/D SIDE STRAFE: {direction.upper()} x{cycles}")
    print("===================================================")
    print("Mode: WControl23 no-hip lift-out + planted-pull strafe")
    print("Sender: legacy per-motor pacing, same idea as original-swing forward/back")
    print(f"Tripod B first: {CRAB_FIRST_TRIPOD}")
    print(f"Tripod A second: {CRAB_SECOND_TRIPOD}")
    print(f"Flow mode: {SIDE_STRAFE_FLOW_MODE}, hold={SIDE_STRAFE_FLOW_HOLD:.3f}")
    print("===================================================")

    for i in range(cycles):
        print(f"\n--- SIDE STRAFE CYCLE {i + 1}/{cycles} ---")
        ok = _run_side_strafe_cycle_body(bus, direction, f"{i+1}/{cycles}")
        if not ok:
            return
        print_health(bus, f"AFTER SIDE STRAFE CYCLE {i + 1} {direction}")

    print("Final recenter...")
    side_strafe_final_recenter(bus, direction)
    time.sleep(0.20)
    print_health(bus, f"AFTER SIDE STRAFE {direction}")


def action_side_strafe_continuous(bus: DynamixelBus, direction: str):
    """
    A/D continuous mode, matching W/S behavior:
    press A or D once, it keeps side-strafing until Enter is pressed.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["left", "right"]:
        print("Side strafe only supports left/right.")
        return

    print()
    print("===================================================")
    print(f" CONTINUOUS RESTORED A/D SIDE STRAFE: {direction.upper()}")
    print("===================================================")
    print("Press Enter to stop after the current full side-strafe cycle.")
    print("Uses SControl3 working A/D strafe + legacy pacing.")
    print("===================================================")

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
    while not stop_flag.is_set():
        cycle += 1
        print(f"  side cycle {cycle}", end="\r", flush=True)
        ok = _run_side_strafe_cycle_body(bus, direction, str(cycle))
        if not ok:
            break

    print(f"\n  Stopped after {cycle} side-strafe cycle(s).")
    print("Final recenter...")
    side_strafe_final_recenter(bus, direction)
    time.sleep(0.20)
    print_health(bus, f"AFTER CONTINUOUS SIDE STRAFE {direction}")


# ============================================================
# READY / LIFT / PUSHUP ACTIONS
# ============================================================

def action_ready(bus: DynamixelBus, use_safety_check: bool = True, print_after_health: bool = True):
    global ACTIVE_GOALS, CURRENT_MODE

    if use_safety_check and not pre_motion_check(bus):
        return

    print("\nACTION: FAST TRIPOD-LIFT RETURN TO READY")
    try:
        bus.rearm_after_power_cycle(READY_SPEED, reason="ready/recover")
    except Exception as e:
        print(f"[REARM WARNING] {type(e).__name__}: {e}")
    bus.enable_torque_all()  # ensure torque is on after power up

    reset_speed = READY_SPEED

    def lift_tripod(legs):
        t = level_ready_pose()
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
        targets = lift_tripod(legs) if legs else level_ready_pose()
        ACTIVE_GOALS = dict(targets)
        bus.move_sync(targets, speed=reset_speed)
        time.sleep(hold)

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = level_ready_pose()
    bus.move_sync(level_ready_pose(), speed=reset_speed)
    time.sleep(0.20)

    if print_after_health:
        print_health(bus, "AFTER READY")


def action_lift_legs(bus: DynamixelBus, level: int, legs: List[str]):
    global ACTIVE_GOALS, CURRENT_MODE

    if level not in LIFT_LEVELS:
        print(f"Lift level must be {min(LIFT_LEVELS)}-{max(LIFT_LEVELS)}.")
        return
    if not legs:
        print("No legs selected.")
        return
    if not pre_motion_check(bus):
        return

    femur_deg = LIFT_LEVELS[level]["femur"]
    tibia_deg = LIFT_LEVELS[level]["tibia"]

    print(f"\nLIFT L{level} {' '.join(legs)}  femur {femur_deg:+.1f} tibia {tibia_deg:+.1f}")

    targets = level_ready_pose()
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


def action_pushup_quick(bus: DynamixelBus, level: str):
    """Quick web/controller pushup action for progressive hold-lower."""
    global ACTIVE_GOALS, CURRENT_MODE

    level = str(level)
    if level not in PUSHUP_LEVELS:
        web_log("Usage: pushquick 1/2/3/4")
        return False

    web_log(f"QUICK PUSHUP {level}")
    targets = PUSHUP_LEVELS[level]
    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"PUSHUP_{level}"
    bus.move_sync(targets, speed=MOVE_SPEED)
    time.sleep(0.18)
    return True


def action_lift_all_quick(bus: DynamixelBus, level: int = 7):
    """Quick web/controller lift-all action. Use r/ready to recover."""
    global ACTIVE_GOALS, CURRENT_MODE

    try:
        level = int(level)
    except Exception:
        web_log(f"Usage: liftall {min(LIFT_LEVELS)}-{max(LIFT_LEVELS)}")
        return False

    if level not in LIFT_LEVELS:
        web_log(f"Liftall level must be {min(LIFT_LEVELS)}-{max(LIFT_LEVELS)}.")
        return False

    femur_deg = LIFT_LEVELS[level]["femur"]
    tibia_deg = LIFT_LEVELS[level]["tibia"]
    web_log(f"QUICK LIFTALL L{level}: femur {femur_deg:+.1f}, tibia {tibia_deg:+.1f}")

    targets = level_ready_pose()
    for leg in ALL_LEGS:
        targets.update(build_leg_offset_targets(leg, 0.0, femur_deg, tibia_deg))

    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"LIFTALL_L{level}"
    bus.move_sync(targets, speed=LIFT_SPEED)
    time.sleep(0.18)
    return True



def action_body_level_set(bus: DynamixelBus, level: int, move_to_pose: bool = True):
    """
    Set persistent body-height level and optionally move to ready at that level.

    v8 smooth change:
    - The stored level is still integer -7..+7.
    - The physical motor movement is interpolated through fractional levels.
    - This removes the frame-by-frame jump between body levels.
    """
    global BODY_HEIGHT_LEVEL, ACTIVE_GOALS, CURRENT_MODE

    try:
        target_level = clamp_body_height_level(int(level))
    except Exception:
        web_log("Usage: bodylevel -7..7")
        return False

    old_level = int(BODY_HEIGHT_LEVEL)
    femur_deg, tibia_deg = body_height_degrees(target_level)

    web_log(
        f"BODY LEVEL {old_level:+d} -> {target_level:+d} | "
        f"femur {femur_deg:+.1f} deg, tibia {tibia_deg:+.1f} deg | "
        f"smooth={'on' if BODY_HEIGHT_SMOOTH_ENABLED else 'off'}"
    )

    if move_to_pose:
        diff = target_level - old_level

        if BODY_HEIGHT_SMOOTH_ENABLED and diff != 0:
            steps = max(1, int(abs(diff) * BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL))

            for i in range(1, steps + 1):
                level_now = old_level + (diff * i / steps)
                targets = level_ready_pose(level_now)
                ACTIVE_GOALS = dict(targets)
                CURRENT_MODE = f"BODY_LEVEL_SMOOTH_{level_now:+.2f}"
                bus.move_sync(targets, speed=READY_SPEED)
                time.sleep(BODY_HEIGHT_SMOOTH_STEP_DELAY)

        else:
            targets = level_ready_pose(target_level)
            ACTIVE_GOALS = dict(targets)
            CURRENT_MODE = f"BODY_LEVEL_{target_level:+d}_READY"
            bus.move_sync(targets, speed=READY_SPEED)
            time.sleep(0.14)

    BODY_HEIGHT_LEVEL = target_level
    CURRENT_MODE = f"BODY_LEVEL_{BODY_HEIGHT_LEVEL:+d}_READY"
    return True


def action_body_level_delta(bus: DynamixelBus, delta: int):
    """Adjust persistent body-height level by delta and move to that new ready level."""
    global BODY_HEIGHT_LEVEL
    try:
        delta = int(delta)
    except Exception:
        web_log("Usage: bodydelta -1 / +1")
        return False
    return action_body_level_set(bus, BODY_HEIGHT_LEVEL + delta, True)


def action_body_level_reset(bus: DynamixelBus):
    """Reset persistent body-height level to original default 0."""
    return action_body_level_set(bus, 0, True)


def action_torque_max(bus: DynamixelBus):
    print("\nSetting AX torque limit cap to 1023 for all motors.")
    bus.set_torque_limit_all(TORQUE_LIMIT_RAW)
    print_health(bus, "AFTER TORQUE_MAX")


# ============================================================
# RUNTIME SETTINGS COMMANDS
# ============================================================

def action_set_speed(parts: List[str], bus: Optional[DynamixelBus] = None):
    global GAIT_SPEED, READY_SPEED, MOVE_SPEED, LIFT_SPEED

    def apply_hw_speed(v: int):
        if bus is None:
            return
        try:
            bus.rearm_after_power_cycle(v, reason="speed command")
        except Exception as e:
            print(f"[REARM WARNING] {type(e).__name__}: {e}")

    if len(parts) == 1:
        print(f"ready={READY_SPEED} move={MOVE_SPEED} lift={LIFT_SPEED} gait={GAIT_SPEED}")
        return

    sub = parts[1].lower()

    if sub.isdigit():
        GAIT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(sub))))
        apply_hw_speed(GAIT_SPEED)
        print(f"Gait speed = {GAIT_SPEED}")
        return

    if sub == "gait" and len(parts) == 3:
        GAIT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        apply_hw_speed(GAIT_SPEED)
        print(f"Gait speed = {GAIT_SPEED}")
        return

    if sub == "lift" and len(parts) == 3:
        LIFT_SPEED = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        apply_hw_speed(LIFT_SPEED)
        print(f"Lift speed = {LIFT_SPEED}")
        return

    if sub == "all" and len(parts) == 3:
        v = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, int(parts[2]))))
        READY_SPEED = MOVE_SPEED = LIFT_SPEED = GAIT_SPEED = v
        apply_hw_speed(v)
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

    print(f"Usage: walklift / walklift level 6..{max(LIFT_LEVELS)} / walklift high2 / walklift max")


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
        # Fast flow preset. Use only after the robot clearly completes full steps.
        SMOOTH_GAIT = True; SMOOTH_STEPS = 3; SMOOTH_STEP_DELAY = 0.020
        GAIT_PHASE_DELAY = 0.16; GAIT_SETTLE_DELAY = 0.10
        GAIT_PHASE_HEALTH = False; GAIT_PRECHECK_EACH_PHASE = False
    elif sub in ["fullstep", "original", "s3"]:
        # Best first test when sync-write makes the gait look too small.
        # This restores SControl3-like physical dwell time.
        SMOOTH_GAIT = False; SMOOTH_STEPS = 3; SMOOTH_STEP_DELAY = 0.025
        GAIT_PHASE_DELAY = 0.30; GAIT_SETTLE_DELAY = 0.14
        GAIT_PHASE_HEALTH = False; GAIT_PRECHECK_EACH_PHASE = False
    elif sub in ["smoothfull", "fullsmooth"]:
        # Smoother version of fullstep: bigger movement, less jerk.
        SMOOTH_GAIT = True; SMOOTH_STEPS = 5; SMOOTH_STEP_DELAY = 0.025
        GAIT_PHASE_DELAY = 0.26; GAIT_SETTLE_DELAY = 0.14
        GAIT_PHASE_HEALTH = False; GAIT_PRECHECK_EACH_PHASE = False
    else:
        print("Usage: smooth on/off / smooth walk / smooth fullstep / smooth smoothfull / smooth hold 0.15 / smooth settle 0.08 / smooth steps 3")
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
# WEB UI / COMMAND SERVER LAYER
# ============================================================
# Existing gait/motor functions stay intact. This layer exposes them through
# a local web dashboard. Only this Python process owns the Dynamixel port.


def action_latency_profile(parts: List[str]):
    """Controller/web latency timing presets.
    normal = original safer timing.
    fast = shorter phase/recenter waits for more responsive hold-release testing.
    """
    global GAIT_PHASE_DELAY, GAIT_SETTLE_DELAY, GAIT_END_RECENTER_DELAY, GAIT_FINAL_READY_DELAY

    mode = parts[1].lower() if len(parts) >= 2 else "show"
    if mode in ["show", "status"]:
        print(f"phase={GAIT_PHASE_DELAY:.2f}, settle={GAIT_SETTLE_DELAY:.2f}, recenter={GAIT_END_RECENTER_DELAY:.2f}, final={GAIT_FINAL_READY_DELAY:.2f}")
        return

    if mode in ["fast", "low", "controller"]:
        GAIT_PHASE_DELAY = 0.20
        GAIT_SETTLE_DELAY = 0.08
        GAIT_END_RECENTER_DELAY = 0.06
        GAIT_FINAL_READY_DELAY = 0.16
        print("Latency profile: FAST / controller")
        print("phase=0.20, settle=0.08, recenter=0.06, final=0.16")
        print("If feet drag or movement becomes too sharp, use: latency normal")
        return

    if mode in ["normal", "safe", "default"]:
        GAIT_PHASE_DELAY = 0.30
        GAIT_SETTLE_DELAY = 0.14
        GAIT_END_RECENTER_DELAY = 0.10
        GAIT_FINAL_READY_DELAY = 0.35
        print("Latency profile: NORMAL / original safer timing")
        print("phase=0.30, settle=0.14, recenter=0.10, final=0.35")
        return

    print("Usage: latency show|fast|normal")

WEB_LOG_LINES: List[str] = []
WEB_LOG_LIMIT = 350
WEB_BUS: Optional[DynamixelBus] = None
WEB_MOTION_THREAD: Optional[threading.Thread] = None
WEB_MOTION_STOP = threading.Event()
WEB_BUSY_LOCK = threading.RLock()
WEB_CURRENT_MOTION = "idle"
# Open-day responsiveness: keep the full READY return, but do not do
# an automatic 18-motor health poll before accepting the next web command.
WEB_POST_MOTION_HEALTH = False
WEB_POST_MOTION_IDLE_DELAY = 0.02
WEB_LAST_HEALTH = {"status": "NOT_READ", "connected": None, "max_temp": None, "min_volt": None, "max_abs_load": None, "no_reply": None}
WEB_LAST_MOTOR_STATUS = {
    "time": None,
    "rows": [],
    "warn_count": None,
    "summary": "No motor status board yet. Press the Motor Status Board button or run p.",
}


def apply_web_startup_defaults():
    global READY_SPEED, MOVE_SPEED, LIFT_SPEED, GAIT_SPEED
    global MOVEMENT_STATS_ENABLED
    global SIDE_STRAFE_FEMUR_REACH_DEG, SIDE_STRAFE_TIBIA_REACH_DEG
    global SIDE_STRAFE_FEMUR_PULL_DEG, SIDE_STRAFE_TIBIA_PULL_DEG
    global SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG
    global SIDE_STRAFE_HOLD, SIDE_STRAFE_SETTLE
    global SIDE_STRAFE_FLOW_MODE, SIDE_STRAFE_FLOW_HOLD
    SIDE_STRAFE_FEMUR_REACH_DEG = 6.0; SIDE_STRAFE_TIBIA_REACH_DEG = -14.0
    SIDE_STRAFE_FEMUR_PULL_DEG = -5.0; SIDE_STRAFE_TIBIA_PULL_DEG = 12.0
    SIDE_STRAFE_LIFT_FEMUR_DEG = -34.0; SIDE_STRAFE_LIFT_TIBIA_DEG = -6.0
    SIDE_STRAFE_HOLD = 0.18; SIDE_STRAFE_SETTLE = 0.10
    MOVEMENT_STATS_ENABLED = False
    SIDE_STRAFE_FLOW_MODE = True; SIDE_STRAFE_FLOW_HOLD = 0.0
    READY_SPEED = MOVE_SPEED = LIFT_SPEED = GAIT_SPEED = 25


def is_sidestrafe_good_preset() -> bool:
    return (abs(SIDE_STRAFE_FEMUR_REACH_DEG - 6.0) < 1e-9 and abs(SIDE_STRAFE_TIBIA_REACH_DEG + 14.0) < 1e-9 and abs(SIDE_STRAFE_FEMUR_PULL_DEG + 5.0) < 1e-9 and abs(SIDE_STRAFE_TIBIA_PULL_DEG - 12.0) < 1e-9 and abs(SIDE_STRAFE_LIFT_FEMUR_DEG + 34.0) < 1e-9 and abs(SIDE_STRAFE_LIFT_TIBIA_DEG + 6.0) < 1e-9 and abs(SIDE_STRAFE_HOLD - 0.18) < 1e-9 and abs(SIDE_STRAFE_SETTLE - 0.10) < 1e-9)



def update_web_health(bus: DynamixelBus, label: str = "WEB HEALTH"):
    """Read motor health only when explicitly requested.

    Important: /api/state must NOT constantly read Dynamixel motors because
    browser polling can collide with READY/gait writes on the same serial bus.
    This cached health avoids false NO_REPLY while still showing last known health.
    """
    global WEB_LAST_HEALTH
    max_temp, min_volt, max_abs_load, any_no_reply, connected = read_bus_health(bus)
    WEB_LAST_HEALTH = {
        "connected": connected,
        "max_temp": max_temp,
        "min_volt": round(min_volt, 1),
        "max_abs_load": max_abs_load,
        "no_reply": any_no_reply,
        "status": health_status(max_temp, min_volt, max_abs_load, any_no_reply),
        "label": label,
        "time": time.strftime("%H:%M:%S"),
    }
    return WEB_LAST_HEALTH


def print_web_health_cached(bus: DynamixelBus, label: str = "WEB HEALTH CHECK"):
    h = update_web_health(bus, label)
    print()
    print("===================================================")
    print(f" {label}")
    print("===================================================")
    print(f"Current mode : {CURRENT_MODE}")
    print(f"Connected    : {h['connected']}/18")
    print(f"Max temp     : {h['max_temp']} C")
    print(f"Min voltage  : {h['min_volt']:.1f} V")
    print(f"Max abs load : {h['max_abs_load']}")
    print(f"No reply     : {h['no_reply']}")
    print(f"Status       : {h['status']}")

def web_log(message: str):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {message}"
    print(line)
    WEB_LOG_LINES.append(line)
    if len(WEB_LOG_LINES) > WEB_LOG_LIMIT:
        del WEB_LOG_LINES[:len(WEB_LOG_LINES) - WEB_LOG_LIMIT]


def capture_to_web_log(fn, *args, **kwargs):
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            result = fn(*args, **kwargs)
    except Exception as e:
        out = buf.getvalue().strip()
        if out:
            for line in out.splitlines():
                web_log(line)
        web_log(f"ERROR: {type(e).__name__}: {e}")
        return None
    out = buf.getvalue().strip()
    if out:
        for line in out.splitlines():
            web_log(line)
    return result


def web_stop_motion():
    global WEB_CURRENT_MOTION
    WEB_MOTION_STOP.set()
    if WEB_CURRENT_MOTION != "idle":
        WEB_CURRENT_MOTION = "stopping"
    web_log("STOP requested. Hold-release mode will return to READY after the current phase.")


def web_return_to_ready_hold_release(bus: DynamixelBus, direction: str):
    """
    Fast recovery used by web/controller hold-release mode.

    The original terminal gait is cycle-based: once you press W/A/S/D it tends
    to finish a whole cycle before final recenter. For controller use near
    walls, release must behave differently: stop after the current phase and
    return to READY instead of finishing the rest of the cycle.

    action_ready(..., use_safety_check=False) is already the script's fast
    tripod-lift return-to-ready routine, so we reuse it instead of changing the
    original gait math.
    """
    global ACTIVE_GOALS, CURRENT_MODE
    web_log(f"Hold-release return to READY from {direction}...")
    ready_pose = level_ready_pose()
    try:
        # Keep the original tripod-lift READY recovery, but skip the slow
        # automatic health print so web/controller can accept the next hold
        # command sooner after READY has actually been sent.
        capture_to_web_log(action_ready, bus, False, False)
    except Exception as e:
        web_log(f"READY recovery error: {type(e).__name__}: {e}")

    # capture_to_web_log logs exceptions internally, so verify the robot state.
    # If READY did not actually run, force a direct READY fallback instead of
    # leaving the robot stuck in a lifted gait phase.
    if dict(ACTIVE_GOALS) != dict(ready_pose) and CURRENT_MODE != "BODY_IK_READY_POSE_V16":
        web_log("READY recovery did not confirm READY; sending direct READY fallback.")
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = dict(ready_pose)
        bus.move_sync(ready_pose, speed=READY_SPEED)
        time.sleep(GAIT_FINAL_READY_DELAY)


def build_side_strafe_web_hold_phases(direction: str) -> List[Tuple[str, List[str], List[str], str, float]]:
    """
    Side-strafe phase list exposed for web/controller hold-release mode.
    This lets STOP interrupt between side-strafe phases instead of waiting for
    _run_side_strafe_cycle_body() to finish the entire 6-phase side cycle.
    """
    return [
        (f"SIDE_{direction}_B_UP_A_PULL",     CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "up_pull",    SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_B_REACH_A_PULL",  CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "reach_pull", SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_B_DOWN_A_PULL",   CRAB_FIRST_TRIPOD,  CRAB_SECOND_TRIPOD, "down_pull",  SIDE_STRAFE_SETTLE),
        (f"SIDE_{direction}_A_UP_B_PULL",     CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "up_pull",    SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_A_REACH_B_PULL",  CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "reach_pull", SIDE_STRAFE_HOLD),
        (f"SIDE_{direction}_A_DOWN_B_PULL",   CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD,  "down_pull",  SIDE_STRAFE_SETTLE),
    ]


def web_motion_loop(bus: DynamixelBus, direction: str):
    global ACTIVE_GOALS, CURRENT_MODE, WEB_CURRENT_MOTION
    direction = normalize_direction(direction)
    WEB_CURRENT_MOTION = direction
    WEB_MOTION_STOP.clear()

    # Own the Dynamixel bus for the whole motion. Web STOP can still set the
    # stop event, but health/status/ready commands will wait instead of reading
    # the serial bus at the same time.
    with WEB_BUSY_LOCK:
        stopped_by_release = False
        try:
            if not pre_motion_check(bus):
                WEB_CURRENT_MOTION = "blocked"
                return

            if direction in ["forward", "backward", "turn_left", "turn_right"]:
                cycle = 0
                web_log(f"HOLD Motion START: {direction}")
                while not WEB_MOTION_STOP.is_set():
                    cycle += 1
                    phases = build_simultaneous_gait_phases(direction)
                    web_log(f"{direction} hold cycle {cycle}")
                    for label, targets, hold in phases:
                        if WEB_MOTION_STOP.is_set():
                            stopped_by_release = True
                            break
                        CURRENT_MODE = label
                        send_phase_fullswing(bus, targets, GAIT_SPEED, hold, label if GAIT_PHASE_HEALTH else "")
                        if WEB_MOTION_STOP.is_set():
                            stopped_by_release = True
                            break
                        if GAIT_PHASE_HEALTH:
                            capture_to_web_log(print_web_health_cached, bus, label)
                    if stopped_by_release:
                        break

            elif direction in ["left", "right"]:
                cycle = 0
                web_log(f"HOLD Side strafe START: {direction}")
                while not WEB_MOTION_STOP.is_set():
                    cycle += 1
                    web_log(f"{direction} hold side cycle {cycle}")
                    phases = build_side_strafe_web_hold_phases(direction)
                    for mode_name, active, other, phase, delay in phases:
                        if WEB_MOTION_STOP.is_set():
                            stopped_by_release = True
                            break
                        CURRENT_MODE = mode_name
                        targets = build_side_strafe_targets(active, other, direction, phase)
                        effective_delay = SIDE_STRAFE_FLOW_HOLD if SIDE_STRAFE_FLOW_MODE else delay
                        send_phase_side_legacy(
                            bus,
                            targets,
                            GAIT_SPEED,
                            effective_delay,
                            mode_name if SIDE_STRAFE_FLOW_PRINT_PHASES else "",
                        )
                        if WEB_MOTION_STOP.is_set():
                            stopped_by_release = True
                            break
                        if GAIT_PHASE_HEALTH:
                            capture_to_web_log(print_web_health_cached, bus, CURRENT_MODE)
                    if stopped_by_release:
                        break
            else:
                web_log(f"Unsupported web motion: {direction}")
                return

            if stopped_by_release or WEB_MOTION_STOP.is_set():
                web_log(f"Release detected for {direction}; returning to READY now.")
                web_return_to_ready_hold_release(bus, direction)
            else:
                # This normally only happens if the loop exits unexpectedly.
                web_log(f"Motion ended for {direction}; returning to READY.")
                web_return_to_ready_hold_release(bus, direction)

            if WEB_POST_MOTION_IDLE_DELAY > 0:
                time.sleep(WEB_POST_MOTION_IDLE_DELAY)
            if WEB_POST_MOTION_HEALTH:
                capture_to_web_log(print_web_health_cached, bus, f"AFTER HOLD MOTION {direction}")
        except Exception as e:
            web_log(f"MOTION ERROR: {type(e).__name__}: {e}")
        finally:
            WEB_CURRENT_MOTION = "idle"
            WEB_MOTION_STOP.clear()
            web_log("Motion thread idle. Ready for next fresh hold command.")


def web_start_motion(bus: DynamixelBus, direction: str):
    global WEB_MOTION_THREAD
    direction = normalize_direction(direction)
    if direction not in ["forward", "backward", "left", "right", "turn_left", "turn_right"]:
        return {"ok": False, "message": f"Unsupported motion: {direction}"}

    # Controller/web hold mode: never queue a different direction while busy.
    # If user presses another direction during movement/recovery, ignore it.
    # They must release to neutral and press again after WEB_CURRENT_MOTION=idle.
    if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
        if WEB_CURRENT_MOTION == direction:
            return {"ok": True, "message": f"Already holding {direction}"}
        return {"ok": False, "message": f"Busy with {WEB_CURRENT_MOTION}; ignored {direction}. Release and wait for idle."}

    if WEB_CURRENT_MOTION not in ["idle", "blocked"]:
        return {"ok": False, "message": f"Not idle yet ({WEB_CURRENT_MOTION}); ignored {direction}"}

    WEB_MOTION_STOP.clear()
    WEB_MOTION_THREAD = threading.Thread(target=web_motion_loop, args=(bus, direction), daemon=True)
    WEB_MOTION_THREAD.start()
    return {"ok": True, "message": f"Started hold {direction}"}


def web_run_terminal_command(bus: DynamixelBus, raw_cmd: str):
    raw_cmd = (raw_cmd or "").strip()
    if not raw_cmd:
        return {"ok": True, "message": "No command"}
    web_log(f"> {raw_cmd}")
    parts = raw_cmd.split()
    cmd = parts[0].lower()
    try:
        if cmd in ["x", "exit", "quit"]:
            return {"ok": False, "message": "Exit disabled in web UI. Stop Python from terminal."}
        elif cmd in ["h", "help"]:
            capture_to_web_log(print_help)
        elif cmd == "p":
            capture_to_web_log(print_status, bus)
        elif cmd in ["calibmodel", "calib", "offsetcalib"]:
            capture_to_web_log(print_calibration_normalization_report)
        elif cmd == "health":
            capture_to_web_log(print_web_health_cached, bus, "WEB HEALTH CHECK")
        elif cmd in ["movestats", "stats", "mstats"]:
            capture_to_web_log(action_movement_stats, parts)
        elif cmd == "speed":
            capture_to_web_log(action_set_speed, parts, bus)
        elif cmd == "smooth":
            capture_to_web_log(action_smooth, parts)
        elif cmd in ["walklift", "clearance", "gaitlift"]:
            capture_to_web_log(action_walk_lift, parts)
        elif cmd in ["sidestrafe", "side", "ad"]:
            capture_to_web_log(action_side_strafe_settings, parts)
        elif cmd == "sideflow":
            capture_to_web_log(action_sideflow, parts)
        elif cmd in ["range"]:
            capture_to_web_log(action_range, parts)
        elif cmd in ["legtrim", "trim"]:
            capture_to_web_log(action_leg_trim, parts)
        elif cmd in ["rearm", "resync", "cm530sync", "powercycle"]:
            capture_to_web_log(bus.rearm_after_power_cycle, GAIT_SPEED, None, "manual rearm command")
        elif cmd == "torque_max":
            capture_to_web_log(action_torque_max, bus)
        elif cmd == "timing":
            capture_to_web_log(action_gait_timing, parts)
        elif cmd == "ik":
            capture_to_web_log(action_ik_settings, parts)
        elif cmd == "latency":
            capture_to_web_log(action_latency_profile, parts)
        elif cmd in ["r", "ready"]:
            capture_to_web_log(action_ready, bus, True)
        elif cmd == "force_r":
            web_log("FORCE_R: returning without safety check.")
            capture_to_web_log(action_ready, bus, False)
        elif cmd in ["bodysmooth", "heightsmooth"]:
            global BODY_HEIGHT_SMOOTH_ENABLED, BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL, BODY_HEIGHT_SMOOTH_STEP_DELAY
            if len(parts) == 1:
                web_log(f"Body smooth: enabled={BODY_HEIGHT_SMOOTH_ENABLED}, steps_per_level={BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL}, step_delay={BODY_HEIGHT_SMOOTH_STEP_DELAY:.3f}s")
            elif parts[1].lower() in ["on", "true", "1"]:
                BODY_HEIGHT_SMOOTH_ENABLED = True
                web_log("Body smooth ON")
            elif parts[1].lower() in ["off", "false", "0"]:
                BODY_HEIGHT_SMOOTH_ENABLED = False
                web_log("Body smooth OFF")
            elif parts[1].lower() == "steps" and len(parts) >= 3:
                BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL = max(1, min(50, int(parts[2])))
                web_log(f"Body smooth steps per level = {BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL}")
            elif parts[1].lower() == "delay" and len(parts) >= 3:
                BODY_HEIGHT_SMOOTH_STEP_DELAY = max(0.005, min(0.20, float(parts[2])))
                web_log(f"Body smooth step delay = {BODY_HEIGHT_SMOOTH_STEP_DELAY:.3f}s")
            else:
                web_log("Usage: bodysmooth on/off | bodysmooth steps 10 | bodysmooth delay 0.045")

        elif cmd in ["bodylevel", "height", "stancelevel"]:
            if len(parts) == 1:
                femur_deg, tibia_deg = body_height_degrees()
                web_log(f"Body level = {BODY_HEIGHT_LEVEL:+d} / range {BODY_HEIGHT_MIN}..{BODY_HEIGHT_MAX} | femur {femur_deg:+.1f}, tibia {tibia_deg:+.1f}")
            elif parts[1].lower() in ["reset", "zero", "default"]:
                action_body_level_reset(bus)
            elif parts[1].lower() in ["up", "+", "plus"]:
                action_body_level_delta(bus, +1)
            elif parts[1].lower() in ["down", "-", "minus"]:
                action_body_level_delta(bus, -1)
            else:
                action_body_level_set(bus, int(parts[1]), True)
        elif cmd in ["bodydelta", "heightdelta"]:
            delta = int(parts[1]) if len(parts) >= 2 else 0
            action_body_level_delta(bus, delta)
        elif cmd == "pushup":
            if len(parts) != 2: web_log("Usage: pushup 1/2/3/4")
            else: capture_to_web_log(action_pushup, bus, parts[1])
        elif cmd == "pushquick":
            if len(parts) != 2: web_log("Usage: pushquick 1/2/3/4")
            else: action_pushup_quick(bus, parts[1])
        elif cmd in ["liftall", "alllift"]:
            level = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else 7
            action_lift_all_quick(bus, level)
        elif cmd == "lift":
            try:
                level, legs = parse_lift_command(parts)
                capture_to_web_log(action_lift_legs, bus, level, legs)
            except ValueError as e: web_log(str(e))
        elif cmd == "gait":
            if len(parts) != 2: web_log("Usage: gait forward/backward/left/right/turn_left/turn_right")
            else: capture_to_web_log(action_gait_cycle, bus, parts[1], 1)
        elif cmd == "walk":
            if len(parts) < 2: web_log("Usage: walk forward 3")
            else:
                direction = parts[1]
                cycles = int(parts[2]) if len(parts) >= 3 else 1
                capture_to_web_log(action_gait_cycle, bus, direction, cycles)
        elif cmd == "turn":
            if len(parts) != 2: web_log("Usage: turn left / turn right")
            else:
                td = normalize_direction(parts[1])
                if td in ["turn_left", "turn_right"]: web_start_motion(bus, td)
                else: web_log("Usage: turn left / turn right")
        elif cmd in ["w", "s", "q", "e", "forward", "backward"]:
            web_start_motion(bus, normalize_direction(cmd))
        elif cmd in ["a", "d", "left", "right"]:
            web_start_motion(bus, normalize_direction(cmd))
        elif cmd in ["stop", "space"]:
            web_stop_motion()
        else:
            web_log(f"Unknown command: {raw_cmd}. Type help for help.")
            return {"ok": False, "message": f"Unknown command: {raw_cmd}"}
    except ValueError:
        web_log("Invalid number format."); return {"ok": False, "message": "Invalid number format"}
    except Exception as e:
        web_log(f"COMMAND ERROR: {type(e).__name__}: {e}"); return {"ok": False, "message": str(e)}
    return {"ok": True, "message": "Command accepted"}


def web_state(bus: Optional[DynamixelBus] = None):
    lf, lt = gait_lift_values()
    # IMPORTANT: return cached health only. Do not poll the Dynamixel bus here.
    # Browsers call /api/state repeatedly; live reads here caused false NO_REPLY
    # and blocked READY/motion commands. Use the Health button/command to refresh.
    health = dict(WEB_LAST_HEALTH)
    return {
        "motion": WEB_CURRENT_MOTION, "current_mode": CURRENT_MODE,
        "speeds": {"ready": READY_SPEED, "move": MOVE_SPEED, "lift": LIFT_SPEED, "gait": GAIT_SPEED},
        "walk_lift": {"femur": lf, "tibia": lt, "level": GAIT_LIFT_LEVEL, "profile": USE_WALK_LIFT_PROFILE},
        "body_height": {"level": BODY_HEIGHT_LEVEL, "min": BODY_HEIGHT_MIN, "max": BODY_HEIGHT_MAX, "femur_offset": body_height_degrees()[0], "tibia_offset": body_height_degrees()[1], "smooth": BODY_HEIGHT_SMOOTH_ENABLED, "smooth_steps": BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL, "smooth_delay": BODY_HEIGHT_SMOOTH_STEP_DELAY},
        "smooth": {"enabled": SMOOTH_GAIT, "steps": SMOOTH_STEPS, "step_delay": SMOOTH_STEP_DELAY},
        "timing": {"phase": GAIT_PHASE_DELAY, "settle": GAIT_SETTLE_DELAY, "final": GAIT_FINAL_READY_DELAY, "end_mode": GAIT_END_MODE},
        "profile": globals().get("CURRENT_MOTION_PROFILE", "fixed-gait"),
        "ik": {"enabled": globals().get("IK_ENABLED", False), "lift": globals().get("IK_LIFT_CM", None)},
        "bezier": {"enabled": globals().get("IK_NATIVE_BEZIER_ENABLED", False), "steps": globals().get("IK_BEZIER_STEPS", None), "arc": globals().get("IK_BEZIER_ARC_EXTRA_CM", None)},
        "bodyik": {"enabled": globals().get("BODY_IK_ENABLED", False), "height": globals().get("BODY_IK_Z_CM", 0.0), "roll": globals().get("BODY_IK_ROLL_DEG", 0.0), "pitch": globals().get("BODY_IK_PITCH_DEG", 0.0)},
        "range": {"forward_swing": GAIT_HIP_SWING_DEG, "forward_push": GAIT_SUPPORT_PUSH_DEG, "backward_swing": BACKWARD_HIP_SWING_DEG, "backward_push": BACKWARD_SUPPORT_PUSH_DEG, "strafe_swing": STRAFE_HIP_SWING_DEG, "strafe_push": STRAFE_SUPPORT_PUSH_DEG, "turn_swing": TURN_HIP_SWING_DEG, "turn_push": TURN_SUPPORT_PUSH_DEG},
        "side_strafe": {"flow": SIDE_STRAFE_FLOW_MODE, "reach_femur": SIDE_STRAFE_FEMUR_REACH_DEG, "reach_tibia": SIDE_STRAFE_TIBIA_REACH_DEG, "pull_femur": SIDE_STRAFE_FEMUR_PULL_DEG, "pull_tibia": SIDE_STRAFE_TIBIA_PULL_DEG, "lift_femur": SIDE_STRAFE_LIFT_FEMUR_DEG, "lift_tibia": SIDE_STRAFE_LIFT_TIBIA_DEG, "debug_steps": SIDE_STRAFE_DEBUG_STEPS_ENABLED, "phase_boost": SIDE_STRAFE_PHASE_BOOST_ENABLED},
        "movestats": {"enabled": MOVEMENT_STATS_ENABLED, "detail": MOVEMENT_STATS_DETAIL},
        "preset_flags": {"sidestrafe_good": is_sidestrafe_good_preset(), "sideflow_on": SIDE_STRAFE_FLOW_MODE and abs(SIDE_STRAFE_FLOW_HOLD) < 1e-9, "sideflow_off": not SIDE_STRAFE_FLOW_MODE, "movestats_off": not MOVEMENT_STATS_ENABLED, "smooth_fullstep": (not SMOOTH_GAIT and abs(GAIT_PHASE_DELAY - 0.30) < 1e-9 and abs(GAIT_SETTLE_DELAY - 0.14) < 1e-9), "smooth_smoothfull": (SMOOTH_GAIT and SMOOTH_STEPS == 5 and abs(GAIT_PHASE_DELAY - 0.26) < 1e-9 and abs(GAIT_SETTLE_DELAY - 0.14) < 1e-9), "speed_all_25": READY_SPEED == 25 and MOVE_SPEED == 25 and LIFT_SPEED == 25 and GAIT_SPEED == 25},
        "health": health, "motor_status": dict(WEB_LAST_MOTOR_STATUS), "logs": WEB_LOG_LINES[-120:],
    }


WEB_HTML = r'''
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>SControlX2 Web</title>
<style>:root{--bg:#0d1117;--panel:#161b22;--panel2:#0f1720;--text:#e6edf3;--muted:#8b949e;--line:#30363d;--accent:#58a6ff;--danger:#ff6b6b;--ok:#3fb950;--warn:#d29922}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif}.wrap{padding:16px;max-width:1700px;margin:0 auto}h1{font-size:22px;margin:0 0 6px}.sub{color:var(--muted);margin-bottom:14px}.grid{display:grid;grid-template-columns:520px 560px minmax(480px,1fr);gap:14px;align-items:start}.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:14px;min-width:0;overflow:hidden}.controller{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:16px}.dpad,.face{display:grid;grid-template-columns:72px 72px 72px;grid-template-rows:72px 72px 72px;gap:8px;justify-content:center;max-width:100%}.btn{border:1px solid var(--line);background:#21262d;color:var(--text);border-radius:14px;font-size:17px;font-weight:700;cursor:pointer;user-select:none;min-width:0;overflow:hidden}.btn:hover{border-color:var(--accent)}.btn.active{background:#1f6feb;border-color:var(--accent);box-shadow:0 0 0 3px rgba(88,166,255,.30),0 0 14px rgba(88,166,255,.30)}.btn.on{background:#17381f;border-color:var(--ok);box-shadow:0 0 0 2px rgba(63,185,80,.18) inset}.btn.flash{transform:scale(.98);border-color:var(--accent);box-shadow:0 0 0 2px rgba(88,166,255,.20) inset}.btn.small{font-size:13px;padding:10px}.btn.danger{background:#3b1717;border-color:#6b2b2b}.btn.ok{background:#17381f;border-color:#2f6f3a}.btn.warn{background:#3b2f13;border-color:#6f5a20}.wide{width:100%;margin-top:8px}.row{display:grid;grid-template-columns:130px minmax(0,1fr) minmax(0,1fr) auto;gap:8px;align-items:center;margin:9px 0;max-width:100%}.row label{color:var(--muted);font-size:13px}.row input[type=range]{width:100%}.row input,.row select{background:#0d1117;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:7px;min-width:0}.value{min-width:55px;text-align:right;color:var(--accent);font-family:Consolas,monospace}.statuspanel{margin-top:12px}.statuspanel h2{margin:0 0 8px;font-size:15px}.statuscards{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.pill{display:block;padding:8px 10px;border-radius:12px;border:1px solid var(--line);background:var(--panel2);font-size:12px}.pill strong{display:block;color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.04em;margin-bottom:3px}.pill span{font-family:Consolas,monospace;color:var(--text)}.log{height:500px;overflow:auto;background:#05080d;border:1px solid var(--line);border-radius:12px;padding:10px;font-family:Consolas,monospace;font-size:12px;white-space:pre-wrap;color:#d1d5db}.terminal{display:flex;gap:8px;margin-top:10px}.terminal input{flex:1;background:#05080d;color:var(--text);border:1px solid var(--line);border-radius:10px;padding:11px;font-family:Consolas,monospace;min-width:0}.statusgrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.metric{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:10px}.metric .k{color:var(--muted);font-size:12px}.metric .v{font-size:22px;font-weight:700;margin-top:5px}.section{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}summary{cursor:pointer;color:var(--accent);font-weight:700}.presetgrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}.kbd{font-family:Consolas,monospace;color:var(--muted);font-size:11px}details .row{grid-template-columns:120px minmax(0,1fr) minmax(0,1fr) 48px}details .presetgrid{grid-template-columns:repeat(3,minmax(0,1fr))}@media(max-width:1580px){.wrap{max-width:1500px}.grid{grid-template-columns:480px 520px minmax(420px,1fr)}.dpad,.face{grid-template-columns:66px 66px 66px;grid-template-rows:66px 66px 66px}.btn{font-size:15px}.kbd{font-size:10px}}@media(max-width:1250px){.grid{grid-template-columns:1fr}.controller{grid-template-columns:1fr 1fr}.wrap{max-width:760px}.log{height:360px}}.motorbox{margin-top:14px;border-top:1px solid var(--line);padding-top:10px}.motorbar{display:flex;gap:8px;align-items:center;justify-content:space-between;flex-wrap:wrap;margin:8px 0}.motorbar .summary{font-weight:700}.summary.ok{color:var(--ok)}.summary.warn{color:var(--warn)}.motorwrap{max-height:360px;overflow:auto;border:1px solid var(--line);border-radius:10px;background:#0d1117}.motortable{width:100%;border-collapse:collapse;font-size:12px;min-width:760px}.motortable th,.motortable td{border-bottom:1px solid #242b33;padding:6px 7px;text-align:left;white-space:nowrap}.motortable th{position:sticky;top:0;background:#111923;color:var(--muted);z-index:1}.motortable tr.warnrow{background:rgba(210,153,34,.10)}.motortable td.warntext{font-weight:700;color:var(--warn)}.motortable td.oktext{color:var(--ok)}.terminalbar{display:flex;justify-content:space-between;align-items:center;gap:8px;margin:6px 0 8px}.terminalbar .hint{color:var(--muted);font-size:12px}.toggle.followoff{border-color:var(--warn);color:var(--warn)}</style></head>
<body><div class="wrap"><h1>Hexapod Web Controller</h1><div class="sub">Controller-style WASDQE movement + stance-height controls. Health is cached; press Health to refresh motor reads safely.</div><div class="grid">
<div class="card"><h2>Controller Layout</h2><div class="sub">Hold button/key = move. Release = return to READY after current phase. Z/X/C control persistent stance height.</div><div class="controller"><div><div class="sub">Left side movement pad</div><div class="dpad"><div></div><button class="btn move" data-dir="forward">W<br><span class="kbd">Forward</span></button><div></div><button class="btn move" data-dir="turn_left">Q<br><span class="kbd">Turn L</span></button><button class="btn move" data-dir="left">A<br><span class="kbd">Strafe L</span></button><button class="btn move" data-dir="right">D<br><span class="kbd">Strafe R</span></button><div></div><button class="btn move" data-dir="backward">S<br><span class="kbd">Back</span></button><button class="btn move" data-dir="turn_right">E<br><span class="kbd">Turn R</span></button></div></div><div><div class="sub">Right side face buttons</div><div class="face"><div></div><button class="btn ok" onclick="cmd('health')">△<br><span class="kbd">Health</span></button><div></div><button class="btn warn" onclick="cmd('r')">□<br><span class="kbd">Ready</span></button><button class="btn danger" onclick="stopMove()">○<br><span class="kbd">STOP</span></button><button class="btn" onclick="cmd('p')">◇<br><span class="kbd">Status</span></button><div></div><button class="btn" onclick="cmd('force_r')">×<br><span class="kbd">Force R</span></button><div></div></div></div></div><div class="section"><h2>Stance Shortcuts</h2><div class="sub">Use Z / X / C or click the buttons below. These controls are persistent until Reset Height.</div><div class="presetgrid"><button id="btn_stance_z" class="btn small stance" onclick="bodyLevelDelta(-1)">Z<br><span class="kbd">Lower Stance</span></button><button id="btn_stance_x" class="btn small stance" onclick="bodyLevelDelta(1)">X<br><span class="kbd">Raise Stance</span></button><button id="btn_stance_c" class="btn small stance" onclick="bodyLevelReset()">C<br><span class="kbd">Reset Height</span></button></div></div><button class="btn ok wide" onclick="startup()">Safe Start Check: Ready → Health<br></button><button class="btn danger wide" onclick="stopMove()">RELEASE / RETURN TO READY</button><div class="section"><h2>Quick Presets</h2><div class="presetgrid"><button id="btn_side_good" class="btn small" onclick="presetCmd(this,'sidestrafe good')">SideStrafe Good</button><button id="btn_sideflow_on" class="btn small" onclick="presetCmd(this,'sideflow on')">SideFlow ON</button><button id="btn_sideflow_off" class="btn small" onclick="presetCmd(this,'sideflow off')">SideFlow OFF</button><button id="btn_smooth_fullstep" class="btn small" onclick="presetCmd(this,'smooth fullstep')">Smooth Fullstep</button><button id="btn_smooth_smoothfull" class="btn small" onclick="presetCmd(this,'smooth smoothfull')">Smooth Smoothfull</button><button id="btn_walklift_clear" class="btn small" onclick="presetCmd(this,'walklift clear')">WalkLift Clear</button><button id="btn_speed25" class="btn small" onclick="presetCmd(this,'speed all 25')">Speed All 25</button><button id="btn_movestats_off" class="btn small" onclick="presetCmd(this,'movestats off')">MoveStats OFF</button><button class="btn small" onclick="presetCmd(this,'health')">Health Refresh</button></div></div></div>
<div class="card"><h2>Cached Health / State</h2><div class="statusgrid"><div class="metric"><div class="k">Status</div><div class="v" id="h_status">--</div></div><div class="metric"><div class="k">Connected</div><div class="v" id="h_conn">--</div></div><div class="metric"><div class="k">Max Temp</div><div class="v" id="h_temp">--</div></div><div class="metric"><div class="k">Min Volt</div><div class="v" id="h_volt">--</div></div><div class="metric"><div class="k">Max Load</div><div class="v" id="h_load">--</div></div><div class="metric"><div class="k">Motion</div><div class="v" id="motion">--</div></div></div><div class="motorbox"><div class="motorbar"><div><h2 style="margin:0">Motor Status Board</h2><div class="sub" id="motor_meta">Run p to read all motor rows.</div></div><button class="btn small" onclick="cmd('p')">Refresh Motor Status Board (p)</button></div><details id="motor_details" open><summary id="motor_summary">No motor status board yet</summary><div class="motorwrap"><table class="motortable" id="motor_table"><thead><tr><th>ID</th><th>Joint</th><th>Leg</th><th>Part</th><th>Raw</th><th>DegZero</th><th>Goal</th><th>Load</th><th>Volt</th><th>Temp</th><th>Warnings</th></tr></thead><tbody><tr><td colspan="11">Press Refresh Motor Status Board or run p in terminal.</td></tr></tbody></table></div></details></div><div class="section"><h2>Main Tuning</h2><div class="row"><label>All Speed</label><input id="speed" type="range" min="1" max="80" value="25" oninput="sv('speedv',this.value)" onchange="cmd('speed all '+this.value)"><span class="value" id="speedv">25</span></div><div class="row"><label>Stance Height</label><input id="bodylevel" type="range" min="-7" max="7" value="0" oninput="sv('bodylevelv',this.value)" onchange="bodyLevelSet(this.value)"><span class="value" id="bodylevelv">0</span></div><div class="presetgrid"><button class="btn small" onclick="bodyLevelDelta(-1)">Lower Stance</button><button class="btn small" onclick="bodyLevelDelta(1)">Raise Stance</button><button class="btn small" onclick="bodyLevelSet(-7)">Lowest Stance</button><button class="btn small" onclick="bodyLevelSet(0)">Reset Height</button><button class="btn small" onclick="bodyLevelSet(7)">Highest Stance</button></div><div class="sub">Stance Height is persistent: movement and Ready use the selected height until Reset Height is pressed. Keyboard: Z lower, X raise, C reset.</div><div class="row"><label>Walk Lift Preset</label><select onchange="cmd('walklift '+this.value)"><option value="clear">clear</option><option value="high">high</option><option value="high2">high2</option><option value="max">max</option><option value="max12">max12</option><option value="old6">old6</option><option value="low">low</option></select></div><div class="row"><label>Walk Lift Level</label><input id="liftlevel" type="range" min="1" max="12" value="6" oninput="sv('liftlevelv',this.value)" onchange="cmd('walklift level '+this.value)"><span class="value" id="liftlevelv">6</span></div><div class="row"><label>IK Swing / Step</label><input id="ikstep" type="range" min="4.0" max="10.0" step="0.1" value="8.5" oninput="sv('ikstepv',Number(this.value).toFixed(1))" onchange="cmd('ik step '+this.value)"><span class="value" id="ikstepv">8.5</span></div><div class="row"><label>Phase Hold</label><input id="phase" type="range" min="0.02" max="0.60" step="0.01" value="0.30" oninput="sv('phasev',this.value)" onchange="cmd('timing phase '+this.value)"><span class="value" id="phasev">0.30</span></div><div class="row"><label>Settle</label><input id="settle" type="range" min="0.02" max="0.40" step="0.01" value="0.14" oninput="sv('settlev',this.value)" onchange="cmd('timing settle '+this.value)"><span class="value" id="settlev">0.14</span></div></div><details><summary>Advanced debug / all script features</summary><div class="row"><label>Forward Range</label><input type="number" id="fwSwing" value="24"><input type="number" id="fwPush" value="16"><button class="btn small" onclick="cmd('range forward '+v('fwSwing')+' '+v('fwPush'))">Set</button></div><div class="row"><label>Strafe Range</label><input type="number" id="stSwing" value="28"><input type="number" id="stPush" value="22"><button class="btn small" onclick="cmd('range strafe '+v('stSwing')+' '+v('stPush'))">Set</button></div><div class="row"><label>Turn Range</label><input type="number" id="tnSwing" value="30"><input type="number" id="tnPush" value="24"><button class="btn small" onclick="cmd('range turn '+v('tnSwing')+' '+v('tnPush'))">Set</button></div><div class="row"><label>Lift Legs</label><select id="liftLv"><option>3</option><option>4</option><option>5</option><option selected>6</option><option>7</option><option>8</option><option>9</option><option>10</option><option>11</option><option>12</option></select><input id="liftLegs" placeholder="FL MR RL"><button class="btn small" onclick="cmd('lift '+v('liftLv')+' '+v('liftLegs'))">Lift</button></div><div class="presetgrid"><button class="btn small" onclick="cmd('lift 6 FL MR RL')">Lift Tripod A</button><button class="btn small" onclick="cmd('lift 6 FR ML RR')">Lift Tripod B</button><button class="btn small" onclick="cmd('torque_max')">Torque Max</button><button class="btn small" onclick="cmd('movestats on')">MoveStats ON</button><button class="btn small" onclick="cmd('movestats off')">MoveStats OFF</button><button class="btn small" onclick="cmd('smooth on')">Smooth ON</button><button class="btn small" onclick="cmd('smooth off')">Smooth OFF</button><button class="btn small" onclick="cmd('pushup 1')">Pushup 1</button><button class="btn small" onclick="cmd('pushup 4')">Pushup 4</button><button class="btn small" onclick="cmd('liftall 7')">Lift All L7</button><button class="btn small" onclick="cmd('latency fast')">Latency FAST</button><button class="btn small" onclick="cmd('latency normal')">Latency NORMAL</button></div></details></div>
<div class="card"><h2>Terminal / Debug Output</h2><div class="terminalbar"><span class="hint">Scroll up to inspect old output. Auto-follow pauses when you scroll away from the bottom.</span><button id="termFollowBtn" class="btn small toggle" onclick="toggleTermFollow()">Auto-follow ON</button></div><div class="log" id="log"></div><div class="terminal"><input id="term" placeholder="Type terminal command: r, health, speed all 25, sidestrafe good, walk forward 1..." onkeydown="if(event.key==='Enter') sendTerm()"><button class="btn small" onclick="sendTerm()">Send</button></div><div class="section statuspanel"><h2>Live Robot Status</h2><div id="pills" class="statuscards"></div></div></div>
</div></div><script>
let terminalAutoFollow=true;
function logNearBottom(){const log=document.getElementById('log');if(!log)return true;return (log.scrollHeight-log.scrollTop-log.clientHeight)<16}
function updateTermFollowBtn(){const b=document.getElementById('termFollowBtn');if(!b)return;b.textContent=terminalAutoFollow?'Auto-follow ON':'Auto-follow OFF';b.classList.toggle('followoff',!terminalAutoFollow)}
function toggleTermFollow(){terminalAutoFollow=!terminalAutoFollow;const log=document.getElementById('log');if(terminalAutoFollow&&log)log.scrollTop=log.scrollHeight;updateTermFollowBtn()}
function sv(id,val){document.getElementById(id).textContent=val}function v(id){return document.getElementById(id).value}
function uiLog(msg){const log=document.getElementById('log');if(log){const oldTop=log.scrollTop;const wasBottom=logNearBottom();log.textContent+=(log.textContent?'\n':'')+msg;if(terminalAutoFollow||wasBottom)log.scrollTop=log.scrollHeight;else log.scrollTop=oldTop}}
async function api(path,body){const opt=body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{};const r=await fetch(path,opt);return await r.json()}async function bodyLevelSet(level){const res=await api('/api/action/bodylevel',{mode:'set',level:Number(level),delta:0});uiLog('[WEB] set stance height -> '+(res.level??level)+' '+(res.message||''));setTimeout(refresh,250)}async function bodyLevelDelta(delta){const res=await api('/api/action/bodylevel',{mode:'delta',level:0,delta:Number(delta)});uiLog('[WEB] '+(delta>0?'raise':'lower')+' stance -> '+(res.level??'?')+' '+(res.message||''));setTimeout(refresh,250)}async function bodyLevelReset(){const res=await api('/api/action/bodylevel',{mode:'reset',level:0,delta:0});uiLog('[WEB] reset stance height -> '+(res.level??0)+' '+(res.message||''));setTimeout(refresh,250)}async function cmd(c){await api('/api/command',{command:c});setTimeout(refresh,250)}async function presetCmd(btn,c){if(btn){btn.classList.add('flash');setTimeout(()=>btn.classList.remove('flash'),180)}await cmd(c)}async function startMove(dir){document.querySelectorAll('.move').forEach(b=>b.classList.remove('active'));const b=document.querySelector(`[data-dir="${dir}"]`);if(b)b.classList.add('active');await api('/api/move/start',{direction:dir})}async function stopMove(){document.querySelectorAll('.move').forEach(b=>b.classList.remove('active'));await api('/api/move/stop',{});setTimeout(refresh,300)}async function startup(){for(const c of ['r','health']){await cmd(c);await new Promise(r=>setTimeout(r,250))}}function sendTerm(){const el=document.getElementById('term');const c=el.value.trim();if(!c)return;el.value='';cmd(c)}function setOn(id,on){const el=document.getElementById(id);if(el)el.classList.toggle('on',!!on)}
function stanceActive(k,on){const el=document.getElementById('btn_stance_'+k);if(el)el.classList.toggle('active',!!on)}
function prettyProfile(p){p=(p||'fixed-gait').toLowerCase();if(p==='ik-motion')return 'IK Motion';if(p==='ik-bezier-motion')return 'IK Bézier Motion';if(p==='ik-bezier-demo-motion')return 'Bézier Demo Motion';if(p==='bodyik-posture-test')return 'Body IK Posture';return 'Fixed Gait'}
function prettyCommand(m){m=(m||'').trim();if(!m||m==='UNKNOWN')return 'Idle / Ready';return m.replaceAll('_',' ')}
function esc(x){return String(x??'').replace(/[&<>"']/g,m=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[m]))}
function renderMotorStatus(ms){ms=ms||{};const rows=ms.rows||[];const summary=document.getElementById('motor_summary');const meta=document.getElementById('motor_meta');const table=document.getElementById('motor_table');if(!summary||!meta||!table)return;const warn=Number(ms.warn_count||0);summary.textContent=ms.summary||'No motor status board yet';summary.className=warn>0?'summary warn':'summary ok';meta.textContent=ms.time?('Last p read: '+ms.time+' · '+rows.length+' motor rows'):'Run p to read all motor rows.';const body=table.querySelector('tbody');if(!rows.length){body.innerHTML='<tr><td colspan="11">Press Refresh Motor Status Board or run p in terminal.</td></tr>';return}body.innerHTML=rows.map(r=>`<tr class="${r.ok?'':'warnrow'}"><td>${esc(r.id)}</td><td>${esc(r.joint)}</td><td>${esc(r.leg)}</td><td>${esc(r.part)}</td><td>${esc(r.raw)}</td><td>${esc(r.deg_zero)}</td><td>${esc(r.goal)}</td><td>${esc(r.load)}</td><td>${esc(r.volt)}</td><td>${esc(r.temp)}</td><td class="${r.ok?'oktext':'warntext'}">${esc(r.warnings)}</td></tr>`).join('')}
for(const b of document.querySelectorAll('.stance')){b.addEventListener('mousedown',()=>b.classList.add('active'));b.addEventListener('mouseup',()=>b.classList.remove('active'));b.addEventListener('mouseleave',()=>b.classList.remove('active'));b.addEventListener('touchstart',(e)=>{e.preventDefault();b.classList.add('active')});b.addEventListener('touchend',(e)=>{e.preventDefault();b.classList.remove('active');b.click()})}for(const b of document.querySelectorAll('.move')){const dir=b.dataset.dir;b.addEventListener('mousedown',()=>startMove(dir));b.addEventListener('touchstart',(e)=>{e.preventDefault();startMove(dir)});b.addEventListener('mouseup',stopMove);b.addEventListener('touchend',(e)=>{e.preventDefault();stopMove()})}document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const k=e.key.toLowerCase();const map={w:'forward',s:'backward',a:'left',d:'right',q:'turn_left',e:'turn_right'};if(map[k]&&!e.repeat){e.preventDefault();startMove(map[k])}if(k==='z'&&!e.repeat){e.preventDefault();stanceActive('z',true);bodyLevelDelta(-1)}if(k==='x'&&!e.repeat){e.preventDefault();stanceActive('x',true);bodyLevelDelta(1)}if(k==='c'&&!e.repeat){e.preventDefault();stanceActive('c',true);bodyLevelReset()}if(e.key===' '){e.preventDefault();stopMove()}});document.addEventListener('keyup',e=>{const k=e.key.toLowerCase();if('wasdqe'.includes(k))stopMove();if('zxc'.includes(k))stanceActive(k,false)});async function refresh(){const s=await api('/api/state');const h=s.health||{};document.getElementById('h_status').textContent=h.status||'--';document.getElementById('h_conn').textContent=(h.connected??'--')+'/18';document.getElementById('h_temp').textContent=(h.max_temp??'--')+' C';document.getElementById('h_volt').textContent=(h.min_volt??'--')+' V';document.getElementById('h_load').textContent=h.max_abs_load??'--';document.getElementById('motion').textContent=s.motion||'--';document.getElementById('speed').value=s.speeds.gait;sv('speedv',s.speeds.gait);if(document.getElementById('bodylevel')){document.getElementById('bodylevel').value=s.body_height.level;sv('bodylevelv',s.body_height.level);}document.getElementById('liftlevel').value=s.walk_lift.level;sv('liftlevelv',s.walk_lift.level);if(document.getElementById('ikstep')&&s.ik){document.getElementById('ikstep').value=s.ik.step;sv('ikstepv',Number(s.ik.step).toFixed(1));}document.getElementById('phase').value=s.timing.phase;sv('phasev',Number(s.timing.phase).toFixed(2));document.getElementById('settle').value=s.timing.settle;sv('settlev',Number(s.timing.settle).toFixed(2));document.getElementById('pills').innerHTML=`<span class="pill"><strong>Motion Mode</strong><span>${prettyProfile(s.profile)}</span></span><span class="pill"><strong>Current Command</strong><span>${prettyCommand(s.current_mode)}</span></span><span class="pill"><strong>Stance Height</strong><span>${s.body_height.level} / ${s.body_height.min}..${s.body_height.max} &nbsp; F ${s.body_height.femur_offset} / T ${s.body_height.tibia_offset}</span></span><span class="pill"><strong>Walk Lift</strong><span>F ${s.walk_lift.femur} / T ${s.walk_lift.tibia}</span></span><span class="pill"><strong>IK Lift</strong><span>${s.ik&&s.ik.lift!==null?s.ik.lift:'--'} cm</span></span><span class="pill"><strong>Bézier</strong><span>${s.bezier&&s.bezier.enabled?'ON':'OFF'}${s.bezier&&s.bezier.enabled?' / '+s.bezier.steps+' steps':''}</span></span><span class="pill"><strong>Smooth / Step End</strong><span>${s.smooth.enabled?'ON':'OFF'} / ${s.timing.end_mode}</span></span><span class="pill"><strong>SideFlow / MoveStats</strong><span>${s.side_strafe.flow?'ON':'OFF'} / ${s.movestats.enabled?'ON':'OFF'}</span></span>`;const f=s.preset_flags||{};setOn('btn_side_good',f.sidestrafe_good);setOn('btn_sideflow_on',f.sideflow_on);setOn('btn_sideflow_off',f.sideflow_off);setOn('btn_smooth_fullstep',f.smooth_fullstep);setOn('btn_smooth_smoothfull',f.smooth_smoothfull);setOn('btn_speed25',f.speed_all_25);setOn('btn_movestats_off',f.movestats_off);renderMotorStatus(s.motor_status||{});const log=document.getElementById('log');if(log){const newText=(s.logs||[]).join('\n');const oldTop=log.scrollTop;const wasBottom=logNearBottom();if(log.textContent!==newText){log.textContent=newText;if(terminalAutoFollow||wasBottom)log.scrollTop=log.scrollHeight;else log.scrollTop=oldTop}}updateTermFollowBtn()}setTimeout(()=>{const log=document.getElementById('log');if(log){log.addEventListener('scroll',()=>{terminalAutoFollow=logNearBottom();updateTermFollowBtn()})}updateTermFollowBtn()},0);setInterval(refresh,1200);refresh();
</script></body></html>
'''


def create_web_app(bus: DynamixelBus):
    try:
        from fastapi import FastAPI
        from fastapi.responses import HTMLResponse
        from pydantic import BaseModel
    except ImportError:
        print("Missing web libraries. Install using:")
        print("  pip install fastapi uvicorn")
        raise
    class CommandRequest(BaseModel):
        command: str
    class MoveRequest(BaseModel):
        direction: str
    class PushupRequest(BaseModel):
        level: int
    class LiftAllRequest(BaseModel):
        level: int = 7
    class BodyLevelRequest(BaseModel):
        mode: str = "set"   # set / delta / reset
        level: int = 0
        delta: int = 0
    app = FastAPI(title="IKControl Hexapod Web Controller")
    @app.get("/", response_class=HTMLResponse)
    def index():
        return HTMLResponse(WEB_HTML)

    # Compatibility alias: the combined YOLO demo uses /hexapod, while this
    # standalone research controller originally serves the dashboard at /.
    # Keep both URLs working so old bookmarks do not show 404.
    @app.get("/hexapod", response_class=HTMLResponse)
    def index_hexapod_alias():
        return HTMLResponse(WEB_HTML)

    @app.get("/api/state")
    def api_state():
        return web_state(bus)
    @app.post("/api/command")
    def api_command(req: CommandRequest):
        with WEB_BUSY_LOCK:
            return web_run_terminal_command(bus, req.command)
    @app.post("/api/move/start")
    def api_move_start(req: MoveRequest):
        return web_start_motion(bus, req.direction)
    @app.post("/api/move/stop")
    def api_move_stop():
        web_stop_motion()
        return {"ok": True, "message": "Stop requested"}
    @app.post("/api/action/bodylevel")
    def api_action_bodylevel(req: BodyLevelRequest):
        if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
            return {"ok": False, "message": f"Busy with {WEB_CURRENT_MOTION}; bodylevel ignored"}
        if WEB_CURRENT_MOTION not in ["idle", "blocked"]:
            return {"ok": False, "message": f"Not idle ({WEB_CURRENT_MOTION}); bodylevel ignored"}
        with WEB_BUSY_LOCK:
            mode = (req.mode or "set").lower()
            if mode == "reset":
                ok = action_body_level_reset(bus)
            elif mode == "delta":
                ok = action_body_level_delta(bus, req.delta)
            else:
                ok = action_body_level_set(bus, req.level, True)
        return {"ok": bool(ok), "message": f"bodylevel {BODY_HEIGHT_LEVEL}", "level": BODY_HEIGHT_LEVEL}

    @app.post("/api/action/pushup")
    def api_action_pushup(req: PushupRequest):
        if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
            return {"ok": False, "message": f"Busy with {WEB_CURRENT_MOTION}; pushup ignored"}
        if WEB_CURRENT_MOTION not in ["idle", "blocked"]:
            return {"ok": False, "message": f"Not idle ({WEB_CURRENT_MOTION}); pushup ignored"}
        with WEB_BUSY_LOCK:
            ok = action_pushup_quick(bus, str(req.level))
        return {"ok": bool(ok), "message": f"pushup {req.level}"}
    @app.post("/api/action/liftall")
    def api_action_liftall(req: LiftAllRequest):
        if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
            return {"ok": False, "message": f"Busy with {WEB_CURRENT_MOTION}; liftall ignored"}
        if WEB_CURRENT_MOTION not in ["idle", "blocked"]:
            return {"ok": False, "message": f"Not idle ({WEB_CURRENT_MOTION}); liftall ignored"}
        with WEB_BUSY_LOCK:
            ok = action_lift_all_quick(bus, req.level)
        return {"ok": bool(ok), "message": f"liftall {req.level}"}
    return app

# ============================================================
# HELP
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" SCONTROLX2 - SEMI-OVERLAP TRIPOD GAIT")
    print(" HEXAPOD REFINED2K BALANCED CONTROL")
    print(" KEY CHANGE: sync write preserved")
    print("             safer 6-phase semi-overlap gait")
    print("             removed risky A_DOWN+B_UP handoff")
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
    print("  calib        = show offset-normalized calibration model")
    print()
    print("TUNING:")
    print("  speed all 25       = set all speeds to 23")
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
    print("  ik                 = show experimental inverse-kinematics settings")
    print("  ik on/off          = switch gait target builders between IK and fixed-degree")
    print("  ik step/lift/support = tune IK movement in centimetres")
    print()
    print("LIFT/PUSH:")
    print("  lift FL            = lift FL (level 3)")
    print("  lift 6 FL MR RL    = lift tripod A at level 6")
    print("  pushup 1/2/3/4     = body height")
    print("  pushquick 1/2/3/4  = quick web/controller pushup")
    print("  liftall 7           = lift all legs at level 7; use r to recover")
    print("  latency fast/normal = controller response timing preset")
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
    print("  speed all 25")
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


# ============================================================
# RUN MODE SELECTION / TERMINAL MODE
# ============================================================

def choose_run_mode() -> str:
    """Ask how to run SControlX2. Terminal is the safe default."""
    print()
    print("===================================================")
    print(" SCONTROLX2 RUN MODE")
    print("===================================================")
    print("Choose control mode:")
    print("  1) Terminal only  [default]")
    print("  2) Web UI server")
    print()
    print("Terminal mode keeps everything in this console.")
    print("Web mode starts the browser dashboard on port 8000.")

    choice = input("Run mode [terminal]: ").strip().lower()
    if choice in ["2", "web", "w", "webui", "ui", "server"]:
        return "web"
    return "terminal"


def terminal_execute_command(bus: DynamixelBus, raw_cmd: str) -> bool:
    """
    Execute one terminal command.
    Returns False when the terminal loop should exit.
    """
    raw_cmd = (raw_cmd or "").strip()
    if not raw_cmd:
        return True

    parts = raw_cmd.split()
    cmd = parts[0].lower()

    try:
        if cmd in ["x", "exit", "quit"]:
            return False
        elif cmd in ["h", "help", "?"]:
            print_help()
        elif cmd == "p":
            print_status(bus)
        elif cmd in ["calibmodel", "calib", "offsetcalib"]:
            print_calibration_normalization_report()
        elif cmd == "health":
            print_health(bus, "TERMINAL HEALTH CHECK")
        elif cmd in ["movestats", "stats", "mstats"]:
            action_movement_stats(parts)
        elif cmd == "speed":
            action_set_speed(parts, bus)
        elif cmd == "smooth":
            action_smooth(parts)
        elif cmd in ["walklift", "clearance", "gaitlift"]:
            action_walk_lift(parts)
        elif cmd in ["sidestrafe", "side", "ad"]:
            action_side_strafe_settings(parts)
        elif cmd == "sideflow":
            action_sideflow(parts)
        elif cmd == "range":
            action_range(parts)
        elif cmd in ["legtrim", "trim"]:
            action_leg_trim(parts)
        elif cmd in ["rearm", "resync", "cm530sync", "powercycle"]:
            bus.rearm_after_power_cycle(GAIT_SPEED, reason="manual rearm command")
        elif cmd == "torque_max":
            action_torque_max(bus)
        elif cmd == "timing":
            action_gait_timing(parts)
        elif cmd == "ik":
            action_ik_settings(parts)
        elif cmd in ["bodyik", "bik"]:
            action_body_ik_settings(parts)
        elif cmd == "latency":
            action_latency_profile(parts)
        elif cmd in ["r", "ready"]:
            action_ready(bus, True)
        elif cmd == "force_r":
            print("FORCE_R: returning without safety check.")
            action_ready(bus, False)
        elif cmd in ["bodysmooth", "heightsmooth"]:
            global BODY_HEIGHT_SMOOTH_ENABLED, BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL, BODY_HEIGHT_SMOOTH_STEP_DELAY
            if len(parts) == 1:
                print(f"Body smooth: enabled={BODY_HEIGHT_SMOOTH_ENABLED}, steps_per_level={BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL}, step_delay={BODY_HEIGHT_SMOOTH_STEP_DELAY:.3f}s")
            elif parts[1].lower() in ["on", "true", "1"]:
                BODY_HEIGHT_SMOOTH_ENABLED = True
                print("Body smooth ON")
            elif parts[1].lower() in ["off", "false", "0"]:
                BODY_HEIGHT_SMOOTH_ENABLED = False
                print("Body smooth OFF")
            elif parts[1].lower() == "steps" and len(parts) >= 3:
                BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL = max(1, min(50, int(parts[2])))
                print(f"Body smooth steps per level = {BODY_HEIGHT_SMOOTH_STEPS_PER_LEVEL}")
            elif parts[1].lower() == "delay" and len(parts) >= 3:
                BODY_HEIGHT_SMOOTH_STEP_DELAY = max(0.005, min(0.20, float(parts[2])))
                print(f"Body smooth step delay = {BODY_HEIGHT_SMOOTH_STEP_DELAY:.3f}s")
            else:
                print("Usage: bodysmooth on/off | bodysmooth steps 10 | bodysmooth delay 0.045")
        elif cmd in ["bodylevel", "height", "stancelevel"]:
            if len(parts) == 1:
                femur_deg, tibia_deg = body_height_degrees()
                print(f"Body level = {BODY_HEIGHT_LEVEL:+d} / range {BODY_HEIGHT_MIN}..{BODY_HEIGHT_MAX} | femur {femur_deg:+.1f}, tibia {tibia_deg:+.1f}")
            elif parts[1].lower() in ["reset", "zero", "default"]:
                action_body_level_reset(bus)
            elif parts[1].lower() in ["up", "+", "plus"]:
                action_body_level_delta(bus, +1)
            elif parts[1].lower() in ["down", "-", "minus"]:
                action_body_level_delta(bus, -1)
            else:
                action_body_level_set(bus, int(parts[1]), True)
        elif cmd in ["bodydelta", "heightdelta"]:
            delta = int(parts[1]) if len(parts) >= 2 else 0
            action_body_level_delta(bus, delta)
        elif cmd == "pushup":
            if len(parts) != 2:
                print("Usage: pushup 1/2/3/4")
            else:
                action_pushup(bus, parts[1])
        elif cmd == "pushquick":
            if len(parts) != 2:
                print("Usage: pushquick 1/2/3/4")
            else:
                action_pushup_quick(bus, parts[1])
        elif cmd in ["liftall", "alllift"]:
            level = int(parts[1]) if len(parts) >= 2 and parts[1].isdigit() else 7
            action_lift_all_quick(bus, level)
        elif cmd == "lift":
            level, legs = parse_lift_command(parts)
            action_lift_legs(bus, level, legs)
        elif cmd == "gait":
            if len(parts) != 2:
                print("Usage: gait forward/backward/left/right/turn_left/turn_right")
            else:
                action_gait_cycle(bus, parts[1], 1)
        elif cmd == "walk":
            if len(parts) < 2:
                print("Usage: walk forward 3")
            else:
                direction = parts[1]
                cycles = int(parts[2]) if len(parts) >= 3 else 1
                action_gait_cycle(bus, direction, cycles)
        elif cmd == "turn":
            if len(parts) != 2:
                print("Usage: turn left / turn right")
            else:
                td = normalize_direction(parts[1])
                if td in ["turn_left", "turn_right"]:
                    action_gait_continuous(bus, td)
                else:
                    print("Usage: turn left / turn right")
        elif cmd in ["w", "s", "q", "e", "forward", "backward"]:
            action_gait_continuous(bus, normalize_direction(cmd))
        elif cmd in ["a", "d", "left", "right"]:
            action_side_strafe_continuous(bus, normalize_direction(cmd))
        elif cmd in ["stop", "space"]:
            print("Terminal continuous movement stops by pressing Enter during the movement.")
        else:
            print(f"Unknown command: {raw_cmd}. Type h or help for help.")
    except ValueError:
        print("Invalid number format or command usage.")
    except KeyboardInterrupt:
        print("\nInterrupted command.")
    except Exception as e:
        print(f"COMMAND ERROR: {type(e).__name__}: {e}")

    return True


def terminal_loop(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" SCONTROLX2 TERMINAL MODE")
    print("===================================================")
    print("Startup defaults applied: sidestrafe good, movestats off, sideflow on, speed all 25.")
    print("Startup: NO automatic movement. Type r/ready manually when robot is safe.")
    print("Type h or help for commands. Type x to exit.")
    print("For continuous w/s/q/e/a/d movement, press Enter during movement to stop after the current cycle.")

    while True:
        try:
            raw_cmd = input("\nSControlX2 command [h help]: ")
        except (EOFError, KeyboardInterrupt):
            print("\nExiting terminal mode.")
            break
        if not terminal_execute_command(bus, raw_cmd):
            break


def run_web_mode(bus: DynamixelBus, selected_port: str):
    print()
    print("SControlX2 WEB UI HOLD-RELEASE MODE")
    print("Startup defaults applied: sidestrafe good, movestats off, sideflow on, speed all 25.")
    print("Startup: NO automatic movement. Press READY manually when robot is safe.")
    print("Open from laptop browser: http://<raspberry-pi-ip>:8000 or http://raspberrypi.local:8000")
    print("Install web dependencies if needed: pip install fastapi uvicorn")
    web_log("Connected to Dynamixel bus.")
    web_log(f"Selected port: {selected_port}")
    web_log("Startup defaults applied: sidestrafe good | movestats off | sideflow on | speed all 25")
    web_log("No automatic movement was sent. Use READY button or terminal 'r' when safe.")
    web_log("Health is cached. Browser polling no longer reads Dynamixel motors; press Health to refresh.")
    web_log("Open web dashboard on port 8000. Hold-release movement is enabled.")
    app = create_web_app(bus)
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


# ============================================================
# V5 NATIVE FOOT-SPACE IK OVERRIDE
# ============================================================
# v4 already proved the IK distances/signs work on the real robot.
# v5 changes the IK gait logic from "old phase -> IK output" into a more
# native foot-space tripod cycle:
#
#   swing tripod:   REAR + LIFT  -> FRONT + LIFT -> FRONT + DOWN
#   support tripod: FRONT        -> NEUTRAL      -> REAR
#
# This means the final movement is no longer trying to match the old hardcoded
# femur/tibia/hip positions. The gait is now generated from foot coordinates,
# then IK calculates the hip/femur/tibia angles needed for each foot position.
#
# Old/fallback IK-assisted mode still exists as "ik mode assist".
# Native foot-space mode is default when IK is ON.
# ============================================================

IK_NATIVE_GAIT = True
IK_NATIVE_SUPPORT_USES_FULL_STEP = True
# v6: prevent startup/pre-lift counter-twitch.
# In v5, the up phase already moved support legs to FRONT and swing legs to REAR,
# so from READY the robot could shove/twitch opposite before the foot was clearly airborne.
# v6 makes the first/native up phase a pure vertical lift at NEUTRAL, then does the stride
# movement only during the swing phase.
IK_NATIVE_UP_PHASE_NEUTRAL = True
# v7: grounded/support tripod still must move backward relative to the body,
# but the first support push is softened so the robot does not feel like it
# jerks backward before the swing tripod clearly moves forward.
IK_NATIVE_SOFT_SUPPORT_PUSH = True
IK_NATIVE_SUPPORT_SWING_SCALE = 1.00
IK_NATIVE_SUPPORT_DOWN_SCALE = 1.00


def ik_direction_unit(direction: str) -> Tuple[float, float]:
    """Return unit direction in body foot-space: +x forward, +y left."""
    direction = normalize_direction(direction)
    if direction == "forward":
        return 1.0, 0.0
    if direction == "backward":
        return -1.0, 0.0
    if direction == "left":
        return 0.0, 1.0
    if direction == "right":
        return 0.0, -1.0
    return 0.0, 0.0


def ik_native_linear_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    Native linear foot-space target for forward/back/left/right.

    station:
      front   = foot placed in the commanded direction
      neutral = default READY foot coordinate
      rear    = foot trails opposite commanded direction

    During the support phase, grounded legs naturally travel front -> neutral -> rear
    relative to the body. During the swing phase, lifted legs recover rear -> front
    through the air.
    """
    ux, uy = ik_direction_unit(direction)
    foot = copy_foot(IK_DEFAULT_FEET_CM[leg])

    # Use the same distance for swing and support so there is no sudden foot-state
    # mismatch when a leg switches from swing to support in the next half-cycle.
    half_stride = IK_STEP_CM * 0.5

    station_scale = 1.0
    if station == "front_soft":
        station = "front"
        station_scale = IK_NATIVE_SUPPORT_SWING_SCALE
    elif station == "rear_soft":
        station = "rear"
        station_scale = IK_NATIVE_SUPPORT_SWING_SCALE
    elif station == "rear_settle":
        station = "rear"
        station_scale = IK_NATIVE_SUPPORT_DOWN_SCALE

    move = half_stride * station_scale

    if station == "front":
        foot["x"] += ux * move
        foot["y"] += uy * move
    elif station == "rear":
        foot["x"] -= ux * move
        foot["y"] -= uy * move
    # neutral = unchanged

    if lifted:
        foot["z"] += IK_LIFT_CM
    return foot


def ik_native_turn_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    Native rotational foot-space target.

    For turning, front/rear are angular positions around the robot body:
      front = rotated in the desired turn direction
      rear  = rotated opposite the desired turn direction
    Support legs travel from front -> neutral -> rear while grounded.
    Swing legs recover rear -> front in the air.
    """
    foot = copy_foot(IK_DEFAULT_FEET_CM[leg])
    direction = normalize_direction(direction)
    turn = IK_TURN_DEG if direction == "turn_left" else -IK_TURN_DEG
    half_turn = turn * 0.5

    if station == "front":
        foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], half_turn)
    elif station == "rear":
        foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], -half_turn)
    # neutral = unchanged

    if lifted:
        foot["z"] += IK_LIFT_CM
    return foot


def ik_native_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    direction = normalize_direction(direction)
    if direction in ["turn_left", "turn_right"]:
        return ik_native_turn_target(leg, direction, station, lifted)
    return ik_native_linear_target(leg, direction, station, lifted)


def build_tripod_phase_ik_native(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,
    support_push_active: bool = True,
) -> Dict[int, int]:
    """
    Native foot-space IK tripod phase.

    Phase map v6:
      up    : swing tripod lifts vertically from NEUTRAL, support stays NEUTRAL
      swing : swing tripod moves to FRONT while airborne, support moves to REAR
      down  : swing tripod places foot at FRONT, support holds REAR

    v5 used up = lifted REAR + support FRONT. From READY this could create a
    small counter-twitch before the foot had enough clearance. v6 delays the
    push until the swing tripod is already lifted.
    """
    targets = level_ready_pose()
    direction = normalize_direction(direction)

    if phase == "up":
        # v6 anti-twitch: lift first, do not pre-push support legs yet.
        # This prevents the robot from nudging opposite the requested direction
        # when starting from READY.
        lifted_station = "neutral" if IK_NATIVE_UP_PHASE_NEUTRAL else "rear"
        lifted_air = True
        support_station = "neutral" if IK_NATIVE_UP_PHASE_NEUTRAL else "front"
    elif phase == "swing":
        # Once the swing tripod is airborne, move it toward the requested
        # direction and let the support tripod push in the opposite foot-space
        # direction. This is where body translation should happen.
        lifted_station = "front"
        lifted_air = True
        support_station = "rear_soft" if IK_NATIVE_SOFT_SUPPORT_PUSH else "rear"
    elif phase == "down":
        lifted_station = "front"
        lifted_air = False
        support_station = "rear_settle" if IK_NATIVE_SOFT_SUPPORT_PUSH else "rear"
    else:
        lifted_station = "neutral"
        lifted_air = False
        support_station = "neutral"

    for leg in lifted_legs:
        foot = ik_native_target(leg, direction, lifted_station, lifted_air)
        targets.update(build_leg_ik_targets(leg, foot))

    for leg in support_legs:
        if support_push_active:
            foot = ik_native_target(leg, direction, support_station, False)
        else:
            foot = ik_native_target(leg, direction, "neutral", False)
        targets.update(build_leg_ik_targets(leg, foot))

    return targets


def build_side_strafe_targets_ik_native(active_tripod, other_tripod, direction, phase) -> Dict[int, int]:
    """
    Native IK strafe uses the same foot-space model as forward/back:
    active tripod recovers from rear -> front in the air, while the other tripod
    supports from front -> rear on the ground.
    """
    targets = level_ready_pose()
    direction = normalize_direction(direction)

    if phase == "up_pull":
        # v6 anti-twitch for A/D strafe too: vertical lift first, no side push yet.
        active_station = "neutral" if IK_NATIVE_UP_PHASE_NEUTRAL else "rear"
        active_air = True
        support_station = "neutral" if IK_NATIVE_UP_PHASE_NEUTRAL else "front"
    elif phase == "reach_pull":
        active_station = "front"
        active_air = True
        support_station = "rear_soft" if IK_NATIVE_SOFT_SUPPORT_PUSH else "rear"
    elif phase == "down_pull":
        active_station = "front"
        active_air = False
        support_station = "rear_settle" if IK_NATIVE_SOFT_SUPPORT_PUSH else "rear"
    else:
        active_station = "neutral"
        active_air = False
        support_station = "neutral"

    for leg in active_tripod:
        foot = ik_native_target(leg, direction, active_station, active_air)
        targets.update(build_leg_ik_targets(leg, foot))

    for leg in other_tripod:
        foot = ik_native_target(leg, direction, support_station, False)
        targets.update(build_leg_ik_targets(leg, foot))

    return targets


# Keep the v4 IK-assisted builders as fallback, but route IK through native mode
# by default. These definitions intentionally override the earlier v4 versions.
_PREVIOUS_BUILD_TRIPOD_PHASE_IK = build_tripod_phase_ik
_PREVIOUS_BUILD_SIDE_STRAFE_TARGETS_IK = build_side_strafe_targets_ik
_PREVIOUS_PRINT_IK_STATUS = print_ik_status
_PREVIOUS_ACTION_IK_SETTINGS = action_ik_settings


def build_tripod_phase_ik(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,
    support_push_active: bool = True,
) -> Dict[int, int]:
    if IK_NATIVE_GAIT:
        return build_tripod_phase_ik_native(lifted_legs, support_legs, direction, phase, support_push_active)
    return _PREVIOUS_BUILD_TRIPOD_PHASE_IK(lifted_legs, support_legs, direction, phase, support_push_active)


def build_side_strafe_targets_ik(active_tripod, other_tripod, direction, phase) -> Dict[int, int]:
    if IK_NATIVE_GAIT:
        return build_side_strafe_targets_ik_native(active_tripod, other_tripod, direction, phase)
    return _PREVIOUS_BUILD_SIDE_STRAFE_TARGETS_IK(active_tripod, other_tripod, direction, phase)


def print_ik_status():
    print()
    print("===================================================")
    print(" IK STATUS / V5 NATIVE FOOT-SPACE MODE")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"IK mode           : {'native foot-space v11 higher-lift longer-reach' if IK_NATIVE_GAIT else 'assist / old phase-compatible'}")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step / support cm : step={IK_STEP_CM:.1f}, support={IK_SUPPORT_PUSH_CM:.1f}, lift={IK_LIFT_CM:.1f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f}")
    print("Native logic:")
    print("  swing tripod   : rear+lift -> front+lift -> front+down")
    print("  support tripod : front -> neutral -> rear")
    print("Commands:")
    print("  ik on / ik off")
    print("  ik mode native")
    print("  ik mode assist")
    print("  ik step 7.0")
    print("  ik lift 5.0")
    print("  ik turn 7")
    print("  ik preview FR forward")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_ENABLED, IK_STEP_CM, IK_SUPPORT_PUSH_CM, IK_LIFT_CM, IK_TURN_DEG, IK_NATIVE_GAIT

    if len(parts) == 1:
        print_ik_status()
        return

    sub = parts[1].lower()

    if sub == "mode" and len(parts) >= 3:
        mode = parts[2].lower()
        if mode in ["native", "foot", "footspace", "full", "ik"]:
            IK_NATIVE_GAIT = True
            print("IK mode = native foot-space. V11 uses higher lift and longer rear/front foot targets with smooth frame sending.")
        elif mode in ["assist", "assisted", "old", "legacy", "v4"]:
            IK_NATIVE_GAIT = False
            print("IK mode = assist/legacy. Uses v4 IK-assisted targets based on the old phase structure.")
        else:
            print("Usage: ik mode native OR ik mode assist")
        return

    if sub in ["native", "footspace", "full"]:
        IK_NATIVE_GAIT = True
        print("IK mode = native foot-space.")
        return

    if sub in ["assist", "assisted", "legacy", "v4"]:
        IK_NATIVE_GAIT = False
        print("IK mode = assist/legacy.")
        return

    if sub in ["on", "true", "1", "enable", "enabled"]:
        IK_ENABLED = True
        IK_NATIVE_GAIT = True
        print("IK gait ON. V11 higher-lift longer-reach smooth native mode active: stronger foot clearance and bigger horizontal stride.")
    elif sub in ["off", "false", "0", "disable", "disabled"]:
        IK_ENABLED = False
        print("IK gait OFF. Reverted to original fixed-degree gait builders.")
    elif sub == "step" and len(parts) >= 3:
        IK_STEP_CM = clamp_float(float(parts[2]), 0.5, 8.0)
        print(f"IK step/stride = {IK_STEP_CM:.2f} cm")
    elif sub == "support" and len(parts) >= 3:
        IK_SUPPORT_PUSH_CM = clamp_float(float(parts[2]), 0.0, 6.0)
        print(f"IK support push = {IK_SUPPORT_PUSH_CM:.2f} cm (used by assist mode; native mode uses step as stride)")
    elif sub == "lift" and len(parts) >= 3:
        IK_LIFT_CM = clamp_float(float(parts[2]), 0.5, 8.0)
        print(f"IK lift = {IK_LIFT_CM:.2f} cm")
    elif sub == "turn" and len(parts) >= 3:
        IK_TURN_DEG = clamp_float(float(parts[2]), 1.0, 35.0)
        print(f"IK turn = {IK_TURN_DEG:.2f} deg")
    elif sub == "preview" and len(parts) >= 4:
        leg = parts[2].upper()
        direction = normalize_direction(parts[3])
        if leg not in ALL_LEGS:
            print(f"Unknown leg: {leg}")
            return
        print(f"Preview mode: {'native foot-space' if IK_NATIVE_GAIT else 'assist/legacy'}")
        if IK_NATIVE_GAIT:
            native_roles = [
                ("rear_lift", "rear", True),
                ("front_lift", "front", True),
                ("front_down", "front", False),
                ("support_front", "front", False),
                ("support_neutral", "neutral", False),
                ("support_rear", "rear", False),
            ]
            for label, station, lifted in native_roles:
                foot = ik_native_target(leg, direction, station, lifted)
                h, f, t = ik_relative_leg_degrees(leg, foot)
                print(f"{leg} {direction:<10} {label:<16} foot={foot}  deg: hip={h:+.2f}, femur={f:+.2f}, tibia={t:+.2f}")
        else:
            for role in ["ready", "lifted_up", "lifted_swing", "lifted_down", "support_push"]:
                foot = ik_target_for_leg(leg, direction, role)
                h, f, t = ik_relative_leg_degrees(leg, foot)
                print(f"{leg} {direction:<10} {role:<13} foot={foot}  deg: hip={h:+.2f}, femur={f:+.2f}, tibia={t:+.2f}")
    else:
        print("Usage: ik on/off | ik mode native/assist | ik step 7.0 | ik lift 5.0 | ik turn 7 | ik preview FR forward")




# ============================================================
# V11 HIGHER-LIFT + LONGER-REACH PATCH
# ============================================================
# Based on v10 feedback: movement was bigger than v9, but lift was still not enough and horizontal reach still needed to be closer to pre-v9. Keeps smooth-native delivery while increasing clearance and reach.
# Previous v10 note: movement became smoother/quicker but hip/coxa reach
# looked smaller because the smooth sender distributed the target over frames
# and support push was softened. V10 keeps the smooth native sender but increases
# the native foot-space stride, support station scale, and hip clamp so the coxa
# can travel farther while avoiding the old mid-step cut.
# ============================================================

# ============================================================
# V9 NATIVE IK SMOOTH PHASE SENDER
# ============================================================
# v8 already removed the obvious double-push by using the same support target
# during swing and down. The remaining "cut" feeling comes from sending each
# native phase as one large target jump, especially when the coxa/hip moves from
# neutral to the rear/support position.
#
# v9 keeps the SAME 3 gait phases, but changes HOW each phase is sent:
#   - interpolate from current motor goals to the next IK target
#   - sync-write each interpolated frame so all motors update together
#   - do not add a second support push during touchdown
#
# Result: one smoother simultaneous coxa/femur/tibia movement inside each phase,
# instead of a visible cut in the middle of the coxa travel.
# ============================================================

IK_NATIVE_SMOOTH_SEND = True
IK_NATIVE_SMOOTH_STEPS = 8
IK_NATIVE_MIN_FRAME_DELAY = 0.022
IK_NATIVE_EXTRA_END_HOLD = 0.040

_PREVIOUS_SEND_PHASE_FULLSWING_V8 = send_phase_fullswing
_PREVIOUS_SEND_PHASE_SIDE_LEGACY_V8 = send_phase_side_legacy
_PREVIOUS_PRINT_IK_STATUS_V8 = print_ik_status
_PREVIOUS_ACTION_IK_SETTINGS_V8 = action_ik_settings


def ik_native_smooth_active() -> bool:
    return bool(IK_ENABLED and IK_NATIVE_GAIT and IK_NATIVE_SMOOTH_SEND)


def send_phase_ik_native_smooth(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    Smooth native IK phase sender.

    This is used only when IK native mode is ON. It does not change the IK math
    or the target phase logic. It only changes delivery from one big jump into
    a short stream of sync-written intermediate frames.
    """
    global ACTIVE_GOALS

    steps = max(2, int(IK_NATIVE_SMOOTH_STEPS))
    start = dict(ACTIVE_GOALS)
    frames = interpolate_targets(start, targets, steps)

    # Keep the total physical phase time close to the requested hold. Because
    # bus.move_sync is much faster than legacy per-motor writes, this frame delay
    # is what lets the AX motors visibly travel through the path.
    frame_delay = max(float(IK_NATIVE_MIN_FRAME_DELAY), float(hold) / float(steps))

    for frame in frames:
        bus.move_sync(frame, speed=speed)
        ACTIVE_GOALS = dict(frame)
        time.sleep(frame_delay)

    # Tiny settle only. Avoid a second visible push/cut after the phase ends.
    if IK_NATIVE_EXTRA_END_HOLD > 0:
        time.sleep(float(IK_NATIVE_EXTRA_END_HOLD))

    ACTIVE_GOALS = dict(targets)

    if label and SIDE_STRAFE_FLOW_PRINT_PHASES:
        print(f"  {label}: sent smooth-native")


def send_phase_fullswing(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    v9 override for forward/back/turn.

    Old fixed gait still uses the v8/original legacy sender.
    Native IK mode uses smooth sync-write frames so the support coxa does not
    cut from one station to another in a single hard jump.
    """
    if ik_native_smooth_active():
        send_phase_ik_native_smooth(bus, targets, speed, hold, label)
    else:
        _PREVIOUS_SEND_PHASE_FULLSWING_V8(bus, targets, speed, hold, label)


def send_phase_side_legacy(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    v9 override for A/D side strafe.

    In fixed gait mode, preserve the proven old side-strafe legacy pacing.
    In native IK mode, use the same smooth sync-written frame sender as W/S/Q/E.
    """
    if ik_native_smooth_active():
        send_phase_ik_native_smooth(bus, targets, speed, hold, label)
    else:
        _PREVIOUS_SEND_PHASE_SIDE_LEGACY_V8(bus, targets, speed, hold, label)


def print_ik_status():
    print()
    print("===================================================")
    print(" IK STATUS / V10 LARGER-REACH SMOOTH NATIVE MODE")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"IK mode           : {'native foot-space v10 larger-reach smooth sender' if IK_NATIVE_GAIT else 'assist / old phase-compatible'}")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step / support cm : step={IK_STEP_CM:.1f}, support={IK_SUPPORT_PUSH_CM:.1f}, lift={IK_LIFT_CM:.1f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f}")
    print(f"Smooth native send: {IK_NATIVE_SMOOTH_SEND} | steps={IK_NATIVE_SMOOTH_STEPS} | min frame delay={IK_NATIVE_MIN_FRAME_DELAY:.3f}s")
    print("Native logic:")
    print("  up    : swing tripod lifts vertically; support tripod stays neutral")
    print("  swing : swing tripod moves to front while support tripod pushes rear")
    print("  down  : swing tripod lands; support tripod holds the same rear target")
    print("Important:")
    print("  There is still a lift/swing/down structure, but v10 sends each movement as")
    print("  smooth sync-written frames with larger foot reach so the coxa should travel farther without cutting mid-motion.")
    print("Commands:")
    print("  ik on / ik off")
    print("  ik mode native")
    print("  ik mode assist")
    print("  ik smooth on/off")
    print("  ik smooth 9")
    print("  ik step 7.0")
    print("  ik lift 5.0")
    print("  ik turn 7")
    print("  ik preview FR forward")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_ENABLED, IK_STEP_CM, IK_SUPPORT_PUSH_CM, IK_LIFT_CM, IK_TURN_DEG, IK_NATIVE_GAIT
    global IK_NATIVE_SMOOTH_SEND, IK_NATIVE_SMOOTH_STEPS, IK_NATIVE_MIN_FRAME_DELAY

    if len(parts) >= 2 and parts[1].lower() == "smooth":
        if len(parts) == 2:
            print(f"IK smooth native send = {IK_NATIVE_SMOOTH_SEND}, steps={IK_NATIVE_SMOOTH_STEPS}, min delay={IK_NATIVE_MIN_FRAME_DELAY:.3f}s")
            return
        value = parts[2].lower()
        if value in ["on", "true", "1", "enable", "enabled"]:
            IK_NATIVE_SMOOTH_SEND = True
            print("IK native smooth sender ON.")
            return
        if value in ["off", "false", "0", "disable", "disabled"]:
            IK_NATIVE_SMOOTH_SEND = False
            print("IK native smooth sender OFF. Native IK will use the older phase sender.")
            return
        try:
            IK_NATIVE_SMOOTH_STEPS = int(max(2, min(20, int(value))))
            print(f"IK native smooth steps = {IK_NATIVE_SMOOTH_STEPS}")
            return
        except Exception:
            print("Usage: ik smooth on/off OR ik smooth 9")
            return

    if len(parts) >= 3 and parts[1].lower() in ["delay", "framedelay", "frame_delay"]:
        try:
            IK_NATIVE_MIN_FRAME_DELAY = clamp_float(float(parts[2]), 0.005, 0.080)
            print(f"IK native min frame delay = {IK_NATIVE_MIN_FRAME_DELAY:.3f}s")
        except Exception:
            print("Usage: ik delay 0.018")
        return

    _PREVIOUS_ACTION_IK_SETTINGS_V8(parts)



# ============================================================
# V12 LIFT-WAIT CLEARANCE PATCH
# ============================================================
# Real hardware feedback after v11:
#   - horizontal reach is better, but the feet are still too close to the floor.
#   - during backward movement, the swing foot can drag and divert the body.
#   - likely cause: the next phase is sent before AX motors physically finish
#     the vertical lift, especially under WARN voltage/load sag.
#
# V12 fix:
#   1) make lifted foot targets higher by scaling the lifted Z offset.
#   2) give UP/LIFT phases their own slower/longer smooth frame delivery.
#   3) add a real lift settle hold before the SWING command is allowed to start.
#
# This is intentionally not a new gait pattern. It is the same v11 native IK
# gait, but it waits for actual physical lift clearance before horizontal travel.
# ============================================================

# Higher clearance than v11/v12, but still clamped for AX safety.
IK_LIFT_CM = 7.0
IK_MAX_FEMUR_DELTA_DEG = 60.0
IK_MAX_TIBIA_DELTA_DEG = 60.0

# Multiplies z lift only while a leg is in the air. 1.0 = old behavior.
IK_LIFT_TARGET_SCALE = 1.18

# Lift phases need more time than horizontal swing phases because the loaded
# femur/tibia joints must raise part of the body/leg mass before coxa travel.
IK_NATIVE_LIFT_SMOOTH_STEPS = 12
IK_NATIVE_LIFT_MIN_FRAME_DELAY = 0.040
IK_NATIVE_LIFT_EXTRA_HOLD = 0.28

# Horizontal phases can remain quicker/smoother.
IK_NATIVE_SMOOTH_STEPS = 8
IK_NATIVE_MIN_FRAME_DELAY = 0.022
IK_NATIVE_EXTRA_END_HOLD = 0.035

_PREVIOUS_IK_NATIVE_TARGET_V11 = ik_native_target
_PREVIOUS_SEND_PHASE_IK_NATIVE_SMOOTH_V11 = send_phase_ik_native_smooth
_PREVIOUS_PRINT_IK_STATUS_V11 = print_ik_status
_PREVIOUS_ACTION_IK_SETTINGS_V11 = action_ik_settings


def ik_native_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    V12 override: preserve v11 native foot-space targets, but increase vertical
    clearance for lifted feet only. This makes lift visibly higher without
    changing the grounded/support foot path.
    """
    foot = _PREVIOUS_IK_NATIVE_TARGET_V11(leg, direction, station, lifted)
    if lifted and IK_LIFT_TARGET_SCALE > 1.0:
        # Previous function already added IK_LIFT_CM. Add only the extra part.
        foot["z"] += IK_LIFT_CM * (IK_LIFT_TARGET_SCALE - 1.0)
    return foot


def _ik_label_is_lift_phase(label: str) -> bool:
    u = (label or "").upper()
    # Gait lift phases: A_UP/B_UP. Final recenter lift also needs clearance.
    return (
        "_UP" in u
        or "LIFT_CURRENT" in u
        or u.endswith("_LIFT")
    ) and "HIP_READY" not in u and "DOWN" not in u


def send_phase_ik_native_smooth(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    V12 smooth native sender.

    The normal v9-v11 smooth sender is good for coxa/hip smoothness, but lift
    phases need longer physical travel time. Otherwise the next SWING command
    can be sent while femur/tibia are still low, making the foot scrape.
    """
    global ACTIVE_GOALS

    is_lift = _ik_label_is_lift_phase(label)

    if is_lift:
        steps = max(2, int(IK_NATIVE_LIFT_SMOOTH_STEPS))
        frame_delay = max(float(IK_NATIVE_LIFT_MIN_FRAME_DELAY), float(hold) / float(steps))
        extra_hold = float(IK_NATIVE_LIFT_EXTRA_HOLD)
    else:
        steps = max(2, int(IK_NATIVE_SMOOTH_STEPS))
        frame_delay = max(float(IK_NATIVE_MIN_FRAME_DELAY), float(hold) / float(steps))
        extra_hold = float(IK_NATIVE_EXTRA_END_HOLD)

    start = dict(ACTIVE_GOALS)
    frames = interpolate_targets(start, targets, steps)

    for frame in frames:
        bus.move_sync(frame, speed=speed)
        ACTIVE_GOALS = dict(frame)
        time.sleep(frame_delay)

    # This is the important anti-drag wait: do not begin horizontal travel until
    # the lift target has had time to physically complete.
    if extra_hold > 0:
        time.sleep(extra_hold)

    ACTIVE_GOALS = dict(targets)

    if label and SIDE_STRAFE_FLOW_PRINT_PHASES:
        suffix = " lift-wait" if is_lift else " smooth-native"
        print(f"  {label}: sent{suffix}")


def print_ik_status():
    print()
    print("===================================================")
    print(" IK STATUS / V12 LIFT-WAIT CLEARANCE MODE")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"IK mode           : {'native foot-space v12 lift-wait clearance' if IK_NATIVE_GAIT else 'assist / old phase-compatible'}")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step / support cm : step={IK_STEP_CM:.1f}, support={IK_SUPPORT_PUSH_CM:.1f}, lift={IK_LIFT_CM:.1f} x scale {IK_LIFT_TARGET_SCALE:.2f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f}")
    print(f"Lift sender       : steps={IK_NATIVE_LIFT_SMOOTH_STEPS}, min delay={IK_NATIVE_LIFT_MIN_FRAME_DELAY:.3f}s, extra hold={IK_NATIVE_LIFT_EXTRA_HOLD:.2f}s")
    print(f"Swing sender      : steps={IK_NATIVE_SMOOTH_STEPS}, min delay={IK_NATIVE_MIN_FRAME_DELAY:.3f}s, extra hold={IK_NATIVE_EXTRA_END_HOLD:.2f}s")
    print("Native logic:")
    print("  up    : lift vertically first and WAIT for clearance")
    print("  swing : move lifted tripod horizontally while support pushes")
    print("  down  : land after horizontal travel")
    print("Commands:")
    print("  ik on / ik off")
    print("  ik lift 6.2")
    print("  ik liftdelay 0.28")
    print("  ik liftdelay 0.040")
    print("  ik smooth 8")
    print("  ik preview FR backward")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_NATIVE_LIFT_EXTRA_HOLD, IK_NATIVE_LIFT_MIN_FRAME_DELAY, IK_NATIVE_LIFT_SMOOTH_STEPS
    global IK_LIFT_TARGET_SCALE, IK_LIFT_CM, IK_MAX_FEMUR_DELTA_DEG, IK_MAX_TIBIA_DELTA_DEG

    if len(parts) >= 2 and parts[1].lower() in ["liftdelay", "lift_hold", "lifthold", "wait"]:
        if len(parts) >= 3:
            try:
                IK_NATIVE_LIFT_EXTRA_HOLD = clamp_float(float(parts[2]), 0.0, 0.80)
                print(f"IK lift extra hold = {IK_NATIVE_LIFT_EXTRA_HOLD:.2f}s")
            except Exception:
                print("Usage: ik liftdelay 0.28")
        else:
            print(f"IK lift extra hold = {IK_NATIVE_LIFT_EXTRA_HOLD:.2f}s")
        return

    if len(parts) >= 2 and parts[1].lower() in ["liftframe", "liftdelay", "lift_frame_delay"]:
        if len(parts) >= 3:
            try:
                IK_NATIVE_LIFT_MIN_FRAME_DELAY = clamp_float(float(parts[2]), 0.015, 0.100)
                print(f"IK lift frame delay = {IK_NATIVE_LIFT_MIN_FRAME_DELAY:.3f}s")
            except Exception:
                print("Usage: ik liftframe 0.040")
        else:
            print(f"IK lift frame delay = {IK_NATIVE_LIFT_MIN_FRAME_DELAY:.3f}s")
        return

    if len(parts) >= 2 and parts[1].lower() in ["liftsteps", "lift_steps"]:
        if len(parts) >= 3:
            try:
                IK_NATIVE_LIFT_SMOOTH_STEPS = int(max(3, min(24, int(parts[2]))))
                print(f"IK lift smooth steps = {IK_NATIVE_LIFT_SMOOTH_STEPS}")
            except Exception:
                print("Usage: ik liftsteps 12")
        else:
            print(f"IK lift smooth steps = {IK_NATIVE_LIFT_SMOOTH_STEPS}")
        return

    if len(parts) >= 2 and parts[1].lower() in ["liftscale", "lift_scale"]:
        if len(parts) >= 3:
            try:
                IK_LIFT_TARGET_SCALE = clamp_float(float(parts[2]), 1.0, 1.60)
                print(f"IK lifted target scale = {IK_LIFT_TARGET_SCALE:.2f}")
            except Exception:
                print("Usage: ik liftscale 1.18")
        else:
            print(f"IK lifted target scale = {IK_LIFT_TARGET_SCALE:.2f}")
        return

    # Let the existing v11 handler process normal commands such as ik on,
    # ik step, ik lift, ik smooth, ik delay, ik preview, etc.
    _PREVIOUS_ACTION_IK_SETTINGS_V11(parts)



# ============================================================
# V13 TURN + STRAFE + LIFT BOOST PATCH
# ============================================================
# Real hardware feedback after v12:
#   - W/S movement and clearance are good overall.
#   - A/D strafe works, but ML/MR do not visibly reach outward enough.
#   - Q/E turning works only after many cycles because the turn target is too
#     small and rear_soft/rear_settle stations were not handled by the turn IK.
#   - User also wants a little more lift headroom than the previous 6.5 cm cap.
#
# V13 fixes:
#   1) Default turn becomes much larger and the turn solver now recognizes
#      rear_soft/rear_settle/front_soft stations instead of treating them as neutral.
#   2) A/D strafe gets a modest side-reach boost, with a larger boost for ML/MR.
#   3) IK lift default and command ceiling are increased, while keeping lift-wait.
# ============================================================

# Practical defaults. Turn was effectively tiny because v12 used half of 7 deg.
# V13 uses a bigger turn plus a stronger station scale.
IK_TURN_DEG = 18.0
IK_TURN_STATION_SCALE = 0.75       # effective front/rear angle = IK_TURN_DEG * 0.75
IK_TURN_SUPPORT_SCALE = 0.85       # support/rear station during swing/down
IK_TURN_LIFTED_SCALE = 1.00        # lifted/front station during swing/down

# Lift: keep the same lift-wait approach, but raise the requested target.
IK_LIFT_CM = 7.0
IK_LIFT_TARGET_SCALE = 1.22
IK_MAX_FEMUR_DELTA_DEG = 60.0
IK_MAX_TIBIA_DELTA_DEG = 60.0

# Strafe: give middle legs more visible outward reach/pull.
# Keep conservative because the log shows A/D can already hit high load.
IK_STRAFE_OUTER_REACH_MULT = 1.12
IK_STRAFE_MIDDLE_REACH_MULT = 1.38

_PREVIOUS_IK_NATIVE_LINEAR_TARGET_V12 = ik_native_linear_target
_PREVIOUS_IK_NATIVE_TURN_TARGET_V12 = ik_native_turn_target
_PREVIOUS_PRINT_IK_STATUS_V12 = print_ik_status
_PREVIOUS_ACTION_IK_SETTINGS_V12 = action_ik_settings


def _ik_station_base_and_scale(station: str, for_turn: bool = False) -> Tuple[str, float]:
    """Normalize native stations such as rear_soft/rear_settle."""
    station = (station or "neutral").lower()
    scale = 1.0
    if station == "front_soft":
        station = "front"
        scale = IK_NATIVE_SUPPORT_SWING_SCALE
    elif station == "rear_soft":
        station = "rear"
        scale = IK_TURN_SUPPORT_SCALE if for_turn else IK_NATIVE_SUPPORT_SWING_SCALE
    elif station == "rear_settle":
        station = "rear"
        scale = IK_TURN_SUPPORT_SCALE if for_turn else IK_NATIVE_SUPPORT_DOWN_SCALE
    return station, scale


def ik_native_linear_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    V13 linear target override.

    Same native foot-space idea as v12, but A/D receives a little more lateral
    station travel. ML/MR get extra because middle legs visually looked like
    they were not reaching/pulling outward enough.
    """
    direction = normalize_direction(direction)
    ux, uy = ik_direction_unit(direction)
    foot = copy_foot(IK_DEFAULT_FEET_CM[leg])

    half_stride = IK_STEP_CM * 0.5
    station, station_scale = _ik_station_base_and_scale(station, for_turn=False)

    move = half_stride * station_scale

    if direction in ["left", "right"]:
        if leg in ["ML", "MR"]:
            move *= IK_STRAFE_MIDDLE_REACH_MULT
        else:
            move *= IK_STRAFE_OUTER_REACH_MULT

    if station == "front":
        foot["x"] += ux * move
        foot["y"] += uy * move
    elif station == "rear":
        foot["x"] -= ux * move
        foot["y"] -= uy * move

    if lifted:
        foot["z"] += IK_LIFT_CM
    return foot


def ik_native_turn_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    V13 rotational target override.

    Important fix: v12 did not normalize rear_soft/rear_settle for turning, so
    the grounded support tripod could stay near neutral instead of producing a
    real rotational push. This made Q/E look extremely tiny. V13 normalizes those
    stations and uses a larger effective turn angle.
    """
    foot = copy_foot(IK_DEFAULT_FEET_CM[leg])
    direction = normalize_direction(direction)
    station, station_scale = _ik_station_base_and_scale(station, for_turn=True)

    turn_sign = 1.0 if direction == "turn_left" else -1.0
    base_turn = IK_TURN_DEG * IK_TURN_STATION_SCALE

    if lifted:
        station_scale *= IK_TURN_LIFTED_SCALE

    turn_amount = turn_sign * base_turn * station_scale

    if station == "front":
        foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], turn_amount)
    elif station == "rear":
        foot["x"], foot["y"] = rotate_xy(foot["x"], foot["y"], -turn_amount)

    if lifted:
        foot["z"] += IK_LIFT_CM
    return foot


def print_ik_status():
    print()
    print("===================================================")
    print(" IK STATUS / V13 TURN-STRAFE-LIFT BOOST MODE")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"IK mode           : {'native foot-space v13 turn/strafe/lift boost' if IK_NATIVE_GAIT else 'assist / old phase-compatible'}")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step/support/lift : step={IK_STEP_CM:.1f} cm, support={IK_SUPPORT_PUSH_CM:.1f} cm, lift={IK_LIFT_CM:.1f} cm x scale {IK_LIFT_TARGET_SCALE:.2f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f} | station scale={IK_TURN_STATION_SCALE:.2f} | support scale={IK_TURN_SUPPORT_SCALE:.2f}")
    print(f"Q/E turn mode     : {'touchdown-then-push' if IK_TURN_PUSH_AFTER_TOUCHDOWN else 'arc push delay'} | settle={IK_TURN_TOUCHDOWN_SETTLE:.2f}s | push steps={IK_TURN_GROUND_PUSH_STEPS} | push frame={IK_TURN_GROUND_PUSH_FRAME_DELAY:.3f}s")
    print(f"Strafe boost      : outer={IK_STRAFE_OUTER_REACH_MULT:.2f}x, ML/MR={IK_STRAFE_MIDDLE_REACH_MULT:.2f}x")
    print(f"Lift sender       : steps={IK_NATIVE_LIFT_SMOOTH_STEPS}, min delay={IK_NATIVE_LIFT_MIN_FRAME_DELAY:.3f}s, extra hold={IK_NATIVE_LIFT_EXTRA_HOLD:.2f}s")
    print(f"Swing sender      : steps={IK_NATIVE_SMOOTH_STEPS}, min delay={IK_NATIVE_MIN_FRAME_DELAY:.3f}s, extra hold={IK_NATIVE_EXTRA_END_HOLD:.2f}s")
    print("Commands:")
    print("  ik on / ik off")
    print("  ik lift 7.0        # allowed up to 8.0")
    print("  ik turn 18         # allowed up to 35")
    print("  ik strafe_mid 1.38")
    print("  ik strafe_outer 1.12")
    print("  ik liftdelay 0.28")
    print("  ik preview MR left")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_STRAFE_MIDDLE_REACH_MULT, IK_STRAFE_OUTER_REACH_MULT
    global IK_TURN_DEG, IK_LIFT_CM

    if len(parts) >= 2 and parts[1].lower() in ["strafe_mid", "strafemid", "middle", "midreach"]:
        if len(parts) >= 3:
            try:
                IK_STRAFE_MIDDLE_REACH_MULT = clamp_float(float(parts[2]), 0.80, 1.80)
                print(f"IK ML/MR strafe reach multiplier = {IK_STRAFE_MIDDLE_REACH_MULT:.2f}x")
            except Exception:
                print("Usage: ik strafe_mid 1.38")
        else:
            print(f"IK ML/MR strafe reach multiplier = {IK_STRAFE_MIDDLE_REACH_MULT:.2f}x")
        return

    if len(parts) >= 2 and parts[1].lower() in ["strafe_outer", "strafeout", "outer"]:
        if len(parts) >= 3:
            try:
                IK_STRAFE_OUTER_REACH_MULT = clamp_float(float(parts[2]), 0.80, 1.50)
                print(f"IK outer strafe reach multiplier = {IK_STRAFE_OUTER_REACH_MULT:.2f}x")
            except Exception:
                print("Usage: ik strafe_outer 1.12")
        else:
            print(f"IK outer strafe reach multiplier = {IK_STRAFE_OUTER_REACH_MULT:.2f}x")
        return

    # Handle lift/turn here so the new higher limits are definitely applied.
    if len(parts) >= 3 and parts[1].lower() == "lift":
        try:
            IK_LIFT_CM = clamp_float(float(parts[2]), 0.5, 8.0)
            print(f"IK lift = {IK_LIFT_CM:.2f} cm")
        except Exception:
            print("Usage: ik lift 7.0")
        return

    if len(parts) >= 3 and parts[1].lower() == "turn":
        try:
            IK_TURN_DEG = clamp_float(float(parts[2]), 1.0, 35.0)
            print(f"IK turn = {IK_TURN_DEG:.2f} deg")
        except Exception:
            print("Usage: ik turn 18")
        return

    _PREVIOUS_ACTION_IK_SETTINGS_V12(parts)



# ============================================================
# V14 BEZIER FOOT-ARC IK GAIT
# ============================================================
# Goal:
#   Keep the successful v13 native IK distances/signs, but remove the sharper
#   lift -> swing -> down corner by sending the swing foot through a smooth
#   cubic Bezier arc in FOOT SPACE before converting each frame through IK.
#
# Important:
#   - This is still the same fixed tripod order.
#   - The actual leg positions are IK-calculated from Bezier foot coordinates.
#   - The first lift phase is kept, because the real AX motors need time to
#     physically clear the ground before horizontal travel begins.
# ============================================================

IK_NATIVE_BEZIER_ENABLED = True
IK_BEZIER_STEPS = 10
IK_BEZIER_FRAME_DELAY = 0.030
IK_BEZIER_ARC_EXTRA_CM = 1.2
IK_BEZIER_SUPPORT_START = 0.08   # normal W/S support push timing
IK_TURN_BEZIER_SUPPORT_START = 0.55  # v8 fallback if after-touchdown mode is OFF

# V9 Q/E turn timing:
# The swing tripod must land first, then the grounded tripod pushes.
# This avoids wasting the rotational push while the lifted tripod is still in the air.
IK_TURN_PUSH_AFTER_TOUCHDOWN = True
IK_TURN_TOUCHDOWN_SETTLE = 0.18
IK_TURN_GROUND_PUSH_STEPS = 6
IK_TURN_GROUND_PUSH_FRAME_DELAY = 0.030
IK_TURN_GROUND_PUSH_SETTLE = 0.06

IK_BEZIER_PRINT_EACH_FRAME = False


def _smoothstep(t: float) -> float:
    t = clamp_float(float(t), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _bezier_value(a: float, b: float, c: float, d: float, t: float) -> float:
    u = 1.0 - t
    return (u ** 3) * a + 3.0 * (u ** 2) * t * b + 3.0 * u * (t ** 2) * c + (t ** 3) * d


def _bezier_foot(p0: Dict[str, float], p1: Dict[str, float], p2: Dict[str, float], p3: Dict[str, float], t: float) -> Dict[str, float]:
    return {
        "x": _bezier_value(p0["x"], p1["x"], p2["x"], p3["x"], t),
        "y": _bezier_value(p0["y"], p1["y"], p2["y"], p3["y"], t),
        "z": _bezier_value(p0["z"], p1["z"], p2["z"], p3["z"], t),
    }


def _lerp_foot(a: Dict[str, float], b: Dict[str, float], t: float) -> Dict[str, float]:
    return {
        "x": a["x"] + (b["x"] - a["x"]) * t,
        "y": a["y"] + (b["y"] - a["y"]) * t,
        "z": a["z"] + (b["z"] - a["z"]) * t,
    }


def _build_bezier_tripod_frames(
    active_tripod: List[str],
    support_tripod: List[str],
    direction: str,
    group_name: str,
) -> List[Tuple[str, Dict[int, int], float]]:
    """
    One tripod movement as:
      1) active tripod vertical lift from neutral, support neutral
      2) active tripod follows Bezier arc from lifted neutral -> front/down
         while support tripod smoothly travels neutral -> rear

    This replaces the old separate SWING and DOWN corner with a curved foot arc.
    """
    direction = normalize_direction(direction)
    frames: List[Tuple[str, Dict[int, int], float]] = []

    # Phase 1: real clearance first. Keep lift-wait behavior from v12/v13.
    lift_targets = level_ready_pose()
    for leg in active_tripod:
        foot = ik_native_target(leg, direction, "neutral", True)
        lift_targets.update(build_leg_ik_targets(leg, foot))
    for leg in support_tripod:
        foot = ik_native_target(leg, direction, "neutral", False)
        lift_targets.update(build_leg_ik_targets(leg, foot))
    frames.append((f"GAIT_{direction}_{group_name}_BEZIER_UP_CLEAR", lift_targets, GAIT_PHASE_DELAY))

    # Phase 2: foot-space Bezier arc. Each frame is already a small target, so
    # the sender will transmit it directly instead of interpolating in raw joint space.
    steps = max(4, int(IK_BEZIER_STEPS))
    for i in range(1, steps + 1):
        t = i / float(steps)
        ts = _smoothstep(t)

        # Slightly delay support push at the start so the active feet are already
        # clearly airborne before the grounded legs begin pushing hard.
        #
        # v8: For Q/E turning only, do NOT slow the whole movement.
        # Instead, keep the Bezier frame speed the same but hold support legs
        # near neutral until later in the foot arc. This delays the actual
        # rotational push until the swing tripod is closer to touchdown.
        is_turn = direction in ["turn_left", "turn_right"]
        if is_turn and IK_TURN_PUSH_AFTER_TOUCHDOWN:
            # V9: no support push while the other tripod is still swinging/landing.
            support_t = 0.0
        else:
            support_start = IK_TURN_BEZIER_SUPPORT_START if is_turn else IK_BEZIER_SUPPORT_START
            if t <= support_start:
                support_t = 0.0
            else:
                support_t = _smoothstep((t - support_start) / max(0.001, 1.0 - support_start))

        targets = level_ready_pose()

        for leg in active_tripod:
            p0 = ik_native_target(leg, direction, "neutral", True)
            p3 = ik_native_target(leg, direction, "front", False)
            p1 = _lerp_foot(p0, ik_native_target(leg, direction, "front", True), 0.25)
            p2 = _lerp_foot(p0, ik_native_target(leg, direction, "front", True), 0.78)
            p1["z"] += IK_BEZIER_ARC_EXTRA_CM
            p2["z"] += IK_BEZIER_ARC_EXTRA_CM
            foot = _bezier_foot(p0, p1, p2, p3, ts)
            targets.update(build_leg_ik_targets(leg, foot))

        for leg in support_tripod:
            s0 = ik_native_target(leg, direction, "neutral", False)
            s1 = ik_native_target(leg, direction, "rear_settle", False)
            foot = _lerp_foot(s0, s1, support_t)
            targets.update(build_leg_ik_targets(leg, foot))

        frames.append((f"GAIT_{direction}_{group_name}_BEZIER_{i:02d}/{steps:02d}", targets, IK_BEZIER_FRAME_DELAY))

    # V9: Q/E turn only.
    # Old behavior: support tripod already pushed during the swing arc, so much
    # of the turn happened while the swing tripod was still high.
    # New behavior: active tripod lands first, short settle, then support tripod
    # performs the turn push while all feet are on/near the floor.
    if direction in ["turn_left", "turn_right"] and IK_TURN_PUSH_AFTER_TOUCHDOWN:
        touchdown_targets = level_ready_pose()
        for leg in active_tripod:
            foot = ik_native_target(leg, direction, "front", False)
            touchdown_targets.update(build_leg_ik_targets(leg, foot))
        for leg in support_tripod:
            foot = ik_native_target(leg, direction, "neutral", False)
            touchdown_targets.update(build_leg_ik_targets(leg, foot))

        if float(IK_TURN_TOUCHDOWN_SETTLE) > 0:
            frames.append((
                f"GAIT_{direction}_{group_name}_TOUCHDOWN_SETTLE",
                touchdown_targets,
                float(IK_TURN_TOUCHDOWN_SETTLE),
            ))

        push_steps = max(1, int(IK_TURN_GROUND_PUSH_STEPS))
        for j in range(1, push_steps + 1):
            pt = _smoothstep(j / float(push_steps))
            push_targets = level_ready_pose()

            # Keep the just-landed tripod planted at its front/down target.
            for leg in active_tripod:
                foot = ik_native_target(leg, direction, "front", False)
                push_targets.update(build_leg_ik_targets(leg, foot))

            # Now and only now: support tripod turns from neutral -> rear.
            for leg in support_tripod:
                s0 = ik_native_target(leg, direction, "neutral", False)
                s1 = ik_native_target(leg, direction, "rear_settle", False)
                foot = _lerp_foot(s0, s1, pt)
                push_targets.update(build_leg_ik_targets(leg, foot))

            frames.append((
                f"GAIT_{direction}_{group_name}_GROUND_PUSH_{j:02d}/{push_steps:02d}",
                push_targets,
                float(IK_TURN_GROUND_PUSH_FRAME_DELAY),
            ))

        if float(IK_TURN_GROUND_PUSH_SETTLE) > 0:
            frames.append((
                f"GAIT_{direction}_{group_name}_GROUND_PUSH_SETTLE",
                push_targets,
                float(IK_TURN_GROUND_PUSH_SETTLE),
            ))

    return frames


_PREVIOUS_BUILD_SIMULTANEOUS_GAIT_PHASES_V13 = build_simultaneous_gait_phases
_PREVIOUS_RUN_SIDE_STRAFE_CYCLE_BODY_V13 = _run_side_strafe_cycle_body
_PREVIOUS_SEND_PHASE_IK_NATIVE_SMOOTH_V13 = send_phase_ik_native_smooth
_PREVIOUS_PRINT_IK_STATUS_V13 = print_ik_status
_PREVIOUS_ACTION_IK_SETTINGS_V13 = action_ik_settings


def build_simultaneous_gait_phases(direction: str) -> List[Tuple[str, Dict[int, int], float]]:
    """V14 override: W/S/Q/E use Bezier foot-arc frames when native IK is enabled."""
    direction = normalize_direction(direction)
    if IK_ENABLED and IK_NATIVE_GAIT and IK_NATIVE_BEZIER_ENABLED and direction in ["forward", "backward", "turn_left", "turn_right"]:
        phases: List[Tuple[str, Dict[int, int], float]] = []
        phases.extend(_build_bezier_tripod_frames(TRIPOD_A, TRIPOD_B, direction, "A"))
        phases.extend(_build_bezier_tripod_frames(TRIPOD_B, TRIPOD_A, direction, "B"))
        return phases
    return _PREVIOUS_BUILD_SIMULTANEOUS_GAIT_PHASES_V13(direction)


def _run_side_strafe_cycle_body(bus: DynamixelBus, direction: str, cycle_label: str = "") -> bool:
    """V14 override: A/D side strafe also uses Bezier foot-arc frames in IK native mode."""
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if not (IK_ENABLED and IK_NATIVE_GAIT and IK_NATIVE_BEZIER_ENABLED and direction in ["left", "right"]):
        return _PREVIOUS_RUN_SIDE_STRAFE_CYCLE_BODY_V13(bus, direction, cycle_label)

    if not pre_motion_check(bus):
        return False

    phases: List[Tuple[str, Dict[int, int], float]] = []
    phases.extend(_build_bezier_tripod_frames(CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, direction, "B"))
    phases.extend(_build_bezier_tripod_frames(CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, direction, "A"))

    for mode_name, targets, delay in phases:
        CURRENT_MODE = mode_name
        label = mode_name if SIDE_STRAFE_FLOW_PRINT_PHASES else ""
        send_phase_side_legacy(bus, targets, GAIT_SPEED, delay, label)
        if GAIT_PHASE_HEALTH:
            print_health(bus, CURRENT_MODE)

    return True


def _ik_label_is_bezier_frame(label: str) -> bool:
    u = (label or "").upper()
    return "_BEZIER_" in u and "UP_CLEAR" not in u


def send_phase_ik_native_smooth(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold: float,
    label: str = "",
):
    """
    V14 sender override.

    Bezier frames are already small foot-space frames, so sending each frame via
    sync-write directly gives a smooth continuous arc without extra raw-space
    interpolation that can make the step smaller/slower. Non-Bezier phases keep
    the v13 lift-wait/smooth-native sender.
    """
    global ACTIVE_GOALS

    if IK_ENABLED and IK_NATIVE_GAIT and IK_NATIVE_BEZIER_ENABLED and _ik_label_is_bezier_frame(label):
        bus.move_sync(targets, speed=speed)
        ACTIVE_GOALS = dict(targets)
        time.sleep(max(0.0, float(hold)))
        if label and SIDE_STRAFE_FLOW_PRINT_PHASES and IK_BEZIER_PRINT_EACH_FRAME:
            print(f"  {label}: sent bezier-frame")
        return

    _PREVIOUS_SEND_PHASE_IK_NATIVE_SMOOTH_V13(bus, targets, speed, hold, label)


def print_ik_status():
    print()
    print("===================================================")
    print(" IK STATUS / V14 BEZIER FOOT-ARC MODE")
    print("===================================================")
    print(f"IK enabled        : {IK_ENABLED}")
    print(f"IK mode           : {'native foot-space v14 Bezier foot arc' if IK_NATIVE_GAIT else 'assist / old phase-compatible'}")
    print(f"Bezier enabled    : {IK_NATIVE_BEZIER_ENABLED} | steps={IK_BEZIER_STEPS} | frame delay={IK_BEZIER_FRAME_DELAY:.3f}s | arc extra={IK_BEZIER_ARC_EXTRA_CM:.1f} cm")
    print(f"Lengths cm        : coxa={IK_COXA_CM:.1f}, femur={IK_FEMUR_CM:.1f}, tibia={IK_TIBIA_CM:.1f}")
    print(f"Step/support/lift : step={IK_STEP_CM:.1f} cm, support={IK_SUPPORT_PUSH_CM:.1f} cm, lift={IK_LIFT_CM:.1f} cm x scale {IK_LIFT_TARGET_SCALE:.2f}")
    print(f"Turn deg          : {IK_TURN_DEG:.1f} | station scale={IK_TURN_STATION_SCALE:.2f} | support scale={IK_TURN_SUPPORT_SCALE:.2f}")
    print(f"Strafe boost      : outer={IK_STRAFE_OUTER_REACH_MULT:.2f}x, ML/MR={IK_STRAFE_MIDDLE_REACH_MULT:.2f}x")
    print(f"Lift sender       : steps={IK_NATIVE_LIFT_SMOOTH_STEPS}, min delay={IK_NATIVE_LIFT_MIN_FRAME_DELAY:.3f}s, extra hold={IK_NATIVE_LIFT_EXTRA_HOLD:.2f}s")
    print("Commands:")
    print("  ik bezier on/off")
    print("  ik bezier_steps 10")
    print("  ik bezier_delay 0.030")
    print("  ik bezier_arc 1.2")
    print("  ik lift 7.0 | ik turn 18 | ik strafe_mid 1.70")
    print("===================================================")


def action_ik_settings(parts: List[str]):
    global IK_NATIVE_BEZIER_ENABLED, IK_BEZIER_STEPS, IK_BEZIER_FRAME_DELAY, IK_BEZIER_ARC_EXTRA_CM

    if len(parts) >= 2 and parts[1].lower() in ["bezier", "curve", "arc"]:
        if len(parts) == 2:
            print(f"IK Bezier foot arc = {IK_NATIVE_BEZIER_ENABLED}, steps={IK_BEZIER_STEPS}, delay={IK_BEZIER_FRAME_DELAY:.3f}s, arc={IK_BEZIER_ARC_EXTRA_CM:.1f}cm")
            return
        sub = parts[2].lower()
        if sub in ["on", "true", "1", "yes"]:
            IK_NATIVE_BEZIER_ENABLED = True
            print("IK Bezier foot arc ON.")
            return
        if sub in ["off", "false", "0", "no"]:
            IK_NATIVE_BEZIER_ENABLED = False
            print("IK Bezier foot arc OFF. Falling back to v13 phase-style native IK.")
            return
        print("Usage: ik bezier on OR ik bezier off")
        return

    if len(parts) >= 2 and parts[1].lower() in ["bezier_steps", "curve_steps", "arc_steps"]:
        if len(parts) >= 3:
            try:
                IK_BEZIER_STEPS = int(clamp_float(float(parts[2]), 4, 18))
                print(f"IK Bezier steps = {IK_BEZIER_STEPS}")
            except Exception:
                print("Usage: ik bezier_steps 10")
        else:
            print(f"IK Bezier steps = {IK_BEZIER_STEPS}")
        return

    if len(parts) >= 2 and parts[1].lower() in ["bezier_delay", "curve_delay", "arc_delay"]:
        if len(parts) >= 3:
            try:
                IK_BEZIER_FRAME_DELAY = clamp_float(float(parts[2]), 0.010, 0.080)
                print(f"IK Bezier frame delay = {IK_BEZIER_FRAME_DELAY:.3f}s")
            except Exception:
                print("Usage: ik bezier_delay 0.030")
        else:
            print(f"IK Bezier frame delay = {IK_BEZIER_FRAME_DELAY:.3f}s")
        return

    if len(parts) >= 2 and parts[1].lower() in ["bezier_arc", "curve_arc", "arc_extra"]:
        if len(parts) >= 3:
            try:
                IK_BEZIER_ARC_EXTRA_CM = clamp_float(float(parts[2]), 0.0, 4.0)
                print(f"IK Bezier arc extra = {IK_BEZIER_ARC_EXTRA_CM:.2f} cm")
            except Exception:
                print("Usage: ik bezier_arc 1.2")
        else:
            print(f"IK Bezier arc extra = {IK_BEZIER_ARC_EXTRA_CM:.2f} cm")
        return

    _PREVIOUS_ACTION_IK_SETTINGS_V13(parts)



# ============================================================
# V15 EXPERIMENTAL BODY IK LAYER
# ============================================================
# What this adds:
#   The existing v14 system already does LEG IK:
#       foot target in cm -> hip/femur/tibia angles -> Dynamixel raw position.
#
#   V15 adds a first experimental BODY IK offset before the leg IK solve:
#       body translation / roll / pitch / yaw -> adjusted foot target -> leg IK.
#
# Important assumptions:
#   - This is an approximate first hardware test because exact body mount
#     coordinates have not been measured yet.
#   - It uses the current IK_DEFAULT_FEET_CM as the approximate foot/body
#     coordinate reference.
#   - Keep BODY_IK_ENABLED off for normal gait; turn it on only for testing.
#
# Safe first tests:
#   r
#   ik on
#   bodyik on
#   bodyik height 1
#   r
#   bodyik reset
#
# Useful posture tests:
#   bodyik pitch 3    # front/rear height compensation
#   bodyik roll 3     # left/right height compensation
#   bodyik yaw 5      # small body twist compensation
# ============================================================

BODY_IK_ENABLED = False
BODY_IK_X_CM = 0.0       # body forward +cm; feet are adjusted opposite
BODY_IK_Y_CM = 0.0       # body left +cm; feet are adjusted opposite
BODY_IK_Z_CM = 0.0       # body up +cm; feet are adjusted downward relative to body
BODY_IK_ROLL_DEG = 0.0   # body roll: left/right tilt
BODY_IK_PITCH_DEG = 0.0  # body pitch: front/back tilt
BODY_IK_YAW_DEG = 0.0    # body yaw/twist

# Conservative first-test clamps. These are intentionally small.
BODY_IK_MAX_TRANSLATE_CM = 3.0
BODY_IK_MAX_HEIGHT_CM = 4.0
BODY_IK_MAX_ROLL_DEG = 8.0
BODY_IK_MAX_PITCH_DEG = 8.0
BODY_IK_MAX_YAW_DEG = 12.0


def _rotate_x(vx: float, vy: float, vz: float, deg: float) -> Tuple[float, float, float]:
    r = math.radians(deg)
    c = math.cos(r)
    s = math.sin(r)
    return vx, vy * c - vz * s, vy * s + vz * c


def _rotate_y(vx: float, vy: float, vz: float, deg: float) -> Tuple[float, float, float]:
    r = math.radians(deg)
    c = math.cos(r)
    s = math.sin(r)
    return vx * c + vz * s, vy, -vx * s + vz * c


def _rotate_z(vx: float, vy: float, vz: float, deg: float) -> Tuple[float, float, float]:
    r = math.radians(deg)
    c = math.cos(r)
    s = math.sin(r)
    return vx * c - vy * s, vx * s + vy * c, vz


def apply_body_ik_to_foot(leg: str, foot: Dict[str, float]) -> Dict[str, float]:
    """
    Convert a requested foot target through an approximate inverse body transform.

    If the body moves/rotates while the foot is assumed to stay planted in the
    world, the foot coordinate in the body's new frame changes in the opposite
    direction. This adjusted coordinate is then sent to the existing leg IK.
    """
    if not BODY_IK_ENABLED:
        return foot

    # Start with the requested foot coordinate from the gait/Bezier generator.
    x = float(foot["x"])
    y = float(foot["y"])
    z = float(foot["z"])

    # Body translation: body moves +X/+Y/+Z, feet appear -X/-Y/-Z relative to body.
    x -= BODY_IK_X_CM
    y -= BODY_IK_Y_CM
    z -= BODY_IK_Z_CM

    # Body rotation: use inverse body rotation so feet stay approximately fixed
    # in world space while the body pose changes.
    # Order is yaw, pitch, roll inverse. For small angles this is safe enough.
    x, y, z = _rotate_z(x, y, z, -BODY_IK_YAW_DEG)
    x, y, z = _rotate_y(x, y, z, -BODY_IK_PITCH_DEG)
    x, y, z = _rotate_x(x, y, z, -BODY_IK_ROLL_DEG)

    # Keep z from going above the body plane too aggressively.
    z = clamp_float(z, -18.0, -3.0)
    return {"x": x, "y": y, "z": z}


# Override the normal IK target builder so EVERY existing IK path can optionally
# pass through the body IK transform first: W/S/A/D/Q/E, walk commands, side
# strafe, turning, Bezier frames, and recenter.
def build_leg_ik_targets(leg: str, foot_target: Dict[str, float]) -> Dict[int, int]:
    adjusted = apply_body_ik_to_foot(leg, foot_target)
    hip_deg, femur_deg, tibia_deg = ik_relative_leg_degrees(leg, adjusted)
    return build_leg_offset_targets(leg, hip_deg, femur_deg, tibia_deg)


def reset_body_ik_values():
    global BODY_IK_X_CM, BODY_IK_Y_CM, BODY_IK_Z_CM, BODY_IK_ROLL_DEG, BODY_IK_PITCH_DEG, BODY_IK_YAW_DEG
    BODY_IK_X_CM = 0.0
    BODY_IK_Y_CM = 0.0
    BODY_IK_Z_CM = 0.0
    BODY_IK_ROLL_DEG = 0.0
    BODY_IK_PITCH_DEG = 0.0
    BODY_IK_YAW_DEG = 0.0


def print_body_ik_status():
    print()
    print("===================================================")
    print(" EXPERIMENTAL BODY IK STATUS / V15")
    print("===================================================")
    print(f"Body IK enabled : {BODY_IK_ENABLED}")
    print(f"Translate cm    : x={BODY_IK_X_CM:+.2f}, y={BODY_IK_Y_CM:+.2f}, z/height={BODY_IK_Z_CM:+.2f}")
    print(f"Rotation deg    : roll={BODY_IK_ROLL_DEG:+.2f}, pitch={BODY_IK_PITCH_DEG:+.2f}, yaw={BODY_IK_YAW_DEG:+.2f}")
    print("Meaning:")
    print("  bodyik height 1.0  = raise body target about 1 cm by moving feet down relative to body")
    print("  bodyik pitch 3     = tilt body front/back approximately 3 degrees")
    print("  bodyik roll 3      = tilt body left/right approximately 3 degrees")
    print("  bodyik yaw 5       = twist body approximately 5 degrees")
    print("Commands:")
    print("  bodyik on / off")
    print("  bodyik reset")
    print("  bodyik height 1.0")
    print("  bodyik forward 1.0")
    print("  bodyik side 1.0")
    print("  bodyik roll 3")
    print("  bodyik pitch 3")
    print("  bodyik yaw 5")
    print("===================================================")


def action_body_ik_settings(parts: List[str]):
    global BODY_IK_ENABLED, BODY_IK_X_CM, BODY_IK_Y_CM, BODY_IK_Z_CM
    global BODY_IK_ROLL_DEG, BODY_IK_PITCH_DEG, BODY_IK_YAW_DEG

    if len(parts) == 1:
        print_body_ik_status()
        return

    sub = parts[1].lower()

    if sub in ["on", "true", "1", "enable", "enabled"]:
        BODY_IK_ENABLED = True
        print("Experimental Body IK ON. Use tiny values first: bodyik height 1 | bodyik pitch 2 | bodyik roll 2")
        return

    if sub in ["off", "false", "0", "disable", "disabled"]:
        BODY_IK_ENABLED = False
        print("Experimental Body IK OFF. Normal leg IK/Bézier gait remains available.")
        return

    if sub in ["reset", "zero", "clear"]:
        reset_body_ik_values()
        print("Body IK values reset to zero.")
        return

    if len(parts) < 3:
        print_body_ik_status()
        return

    try:
        value = float(parts[2])
    except Exception:
        print("Body IK value must be a number. Example: bodyik pitch 3")
        return

    if sub in ["height", "z", "up"]:
        BODY_IK_Z_CM = clamp_float(value, -BODY_IK_MAX_HEIGHT_CM, BODY_IK_MAX_HEIGHT_CM)
        print(f"Body IK height/z = {BODY_IK_Z_CM:+.2f} cm")
        return

    if sub in ["forward", "x"]:
        BODY_IK_X_CM = clamp_float(value, -BODY_IK_MAX_TRANSLATE_CM, BODY_IK_MAX_TRANSLATE_CM)
        print(f"Body IK x/forward = {BODY_IK_X_CM:+.2f} cm")
        return

    if sub in ["side", "y", "left"]:
        BODY_IK_Y_CM = clamp_float(value, -BODY_IK_MAX_TRANSLATE_CM, BODY_IK_MAX_TRANSLATE_CM)
        print(f"Body IK y/side = {BODY_IK_Y_CM:+.2f} cm")
        return

    if sub == "roll":
        BODY_IK_ROLL_DEG = clamp_float(value, -BODY_IK_MAX_ROLL_DEG, BODY_IK_MAX_ROLL_DEG)
        print(f"Body IK roll = {BODY_IK_ROLL_DEG:+.2f} deg")
        return

    if sub == "pitch":
        BODY_IK_PITCH_DEG = clamp_float(value, -BODY_IK_MAX_PITCH_DEG, BODY_IK_MAX_PITCH_DEG)
        print(f"Body IK pitch = {BODY_IK_PITCH_DEG:+.2f} deg")
        return

    if sub == "yaw":
        BODY_IK_YAW_DEG = clamp_float(value, -BODY_IK_MAX_YAW_DEG, BODY_IK_MAX_YAW_DEG)
        print(f"Body IK yaw = {BODY_IK_YAW_DEG:+.2f} deg")
        return

    print_body_ik_status()


# V15 status override adds body IK info below the existing v14 IK status.
_PREVIOUS_PRINT_IK_STATUS_V14 = print_ik_status


def print_ik_status():
    _PREVIOUS_PRINT_IK_STATUS_V14()
    print_body_ik_status()



# ============================================================
# BODY IK V16 - VISIBLE POSE TEST OVERRIDES
# ============================================================
# V15 accepted body IK settings, but normal `r` used the old READY_POSE path,
# so the posture change was barely visible. V16 makes READY apply a visible
# body-IK pose when BODY_IK_ENABLED is on and any body IK value is nonzero.
#
# This is still experimental. It is intentionally a posture test first:
#   bodyik on
#   bodyik scale 2.0
#   bodyik height 1
#   r
#
# The scale value exaggerates small body IK tests so the effect can be seen on
# real hardware before exact body mount geometry is measured.
# ============================================================

BODY_IK_EFFECT_SCALE = 2.0
BODY_IK_POSE_SMOOTH_STEPS = 18
BODY_IK_POSE_FRAME_DELAY = 0.035


def body_ik_has_nonzero_pose() -> bool:
    return (
        abs(BODY_IK_X_CM) > 1e-6 or
        abs(BODY_IK_Y_CM) > 1e-6 or
        abs(BODY_IK_Z_CM) > 1e-6 or
        abs(BODY_IK_ROLL_DEG) > 1e-6 or
        abs(BODY_IK_PITCH_DEG) > 1e-6 or
        abs(BODY_IK_YAW_DEG) > 1e-6
    )


# Replace V15 body transform with a more visible scaled first-test transform.
def apply_body_ik_to_foot(leg: str, foot: Dict[str, float]) -> Dict[str, float]:
    if not BODY_IK_ENABLED:
        return foot

    scale = float(BODY_IK_EFFECT_SCALE)

    x = float(foot["x"])
    y = float(foot["y"])
    z = float(foot["z"])

    # Translation: body moves +X/+Y/+Z, foot target appears opposite in body frame.
    x -= BODY_IK_X_CM * scale
    y -= BODY_IK_Y_CM * scale
    z -= BODY_IK_Z_CM * scale

    # Rotation: exaggerate tiny pitch/roll/yaw values so the real robot visibly reacts.
    x, y, z = _rotate_z(x, y, z, -BODY_IK_YAW_DEG * scale)
    x, y, z = _rotate_y(x, y, z, -BODY_IK_PITCH_DEG * scale)
    x, y, z = _rotate_x(x, y, z, -BODY_IK_ROLL_DEG * scale)

    z = clamp_float(z, -19.0, -2.5)
    return {"x": x, "y": y, "z": z}


# Rebind build_leg_ik_targets again so the V16 scaled transform is used.
def build_leg_ik_targets(leg: str, foot_target: Dict[str, float]) -> Dict[int, int]:
    adjusted = apply_body_ik_to_foot(leg, foot_target)
    hip_deg, femur_deg, tibia_deg = ik_relative_leg_degrees(leg, adjusted)
    return build_leg_offset_targets(leg, hip_deg, femur_deg, tibia_deg)


def build_body_ik_ready_pose() -> Dict[int, int]:
    targets: Dict[int, int] = {}
    for leg in ALL_LEGS:
        targets.update(build_leg_ik_targets(leg, dict(IK_DEFAULT_FEET_CM[leg])))
    return targets


_PREVIOUS_ACTION_READY_V15 = action_ready


def action_ready(bus: DynamixelBus, use_safety_check: bool = True, print_after_health: bool = True):
    """
    V16 override:
    - If body IK is OFF, keep original ready behavior.
    - If body IK is ON and nonzero, apply a visible body-IK posture pose.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    if not (BODY_IK_ENABLED and body_ik_has_nonzero_pose()):
        return _PREVIOUS_ACTION_READY_V15(bus, use_safety_check, print_after_health)

    if use_safety_check and not pre_motion_check(bus):
        return

    print("\nACTION: V16 BODY IK VISIBLE READY POSE")
    print(f"Body IK scale={BODY_IK_EFFECT_SCALE:.2f} | height={BODY_IK_Z_CM:+.2f}cm | roll={BODY_IK_ROLL_DEG:+.2f} | pitch={BODY_IK_PITCH_DEG:+.2f} | yaw={BODY_IK_YAW_DEG:+.2f}")
    print("This applies body pose through IK foot targets, not the old raw READY_POSE.")

    try:
        bus.rearm_after_power_cycle(READY_SPEED, reason="body-ik ready")
    except Exception as e:
        print(f"[REARM WARNING] {type(e).__name__}: {e}")
    bus.enable_torque_all()
    targets = build_body_ik_ready_pose()

    start = dict(ACTIVE_GOALS)
    frames = interpolate_targets(start, targets, BODY_IK_POSE_SMOOTH_STEPS)
    for frame in frames:
        bus.move_sync(frame, speed=READY_SPEED)
        ACTIVE_GOALS = dict(frame)
        time.sleep(BODY_IK_POSE_FRAME_DELAY)

    CURRENT_MODE = "BODY_IK_READY_POSE_V16"
    ACTIVE_GOALS = dict(targets)
    time.sleep(0.20)
    if print_after_health:
        print_health(bus, "AFTER BODY IK READY")


_PREVIOUS_ACTION_BODY_IK_SETTINGS_V15 = action_body_ik_settings


def action_body_ik_settings(parts: List[str]):
    global BODY_IK_EFFECT_SCALE, BODY_IK_POSE_SMOOTH_STEPS, BODY_IK_POSE_FRAME_DELAY

    if len(parts) >= 2:
        sub = parts[1].lower()
        if sub == "scale":
            if len(parts) < 3:
                print(f"Body IK effect scale = {BODY_IK_EFFECT_SCALE:.2f}")
                print("Usage: bodyik scale 2.0")
                return
            try:
                BODY_IK_EFFECT_SCALE = clamp_float(float(parts[2]), 0.25, 5.0)
                print(f"Body IK effect scale = {BODY_IK_EFFECT_SCALE:.2f}")
            except Exception:
                print("Usage: bodyik scale 2.0")
            return
        if sub == "posesmooth":
            if len(parts) < 3:
                print(f"Body IK pose smooth steps={BODY_IK_POSE_SMOOTH_STEPS}, delay={BODY_IK_POSE_FRAME_DELAY:.3f}s")
                return
            try:
                BODY_IK_POSE_SMOOTH_STEPS = max(3, min(40, int(parts[2])))
                print(f"Body IK pose smooth steps = {BODY_IK_POSE_SMOOTH_STEPS}")
            except Exception:
                print("Usage: bodyik posesmooth 18")
            return
        if sub == "posedelay":
            if len(parts) < 3:
                print(f"Body IK pose delay = {BODY_IK_POSE_FRAME_DELAY:.3f}s")
                return
            try:
                BODY_IK_POSE_FRAME_DELAY = max(0.005, min(0.15, float(parts[2])))
                print(f"Body IK pose delay = {BODY_IK_POSE_FRAME_DELAY:.3f}s")
            except Exception:
                print("Usage: bodyik posedelay 0.035")
            return

    _PREVIOUS_ACTION_BODY_IK_SETTINGS_V15(parts)
    if len(parts) == 1:
        print(f"V16 visible-pose scale : {BODY_IK_EFFECT_SCALE:.2f}")
        print("V16 test commands:")
        print("  bodyik on")
        print("  bodyik scale 2.0")
        print("  bodyik height 1")
        print("  r")
        print("  bodyik pitch 4")
        print("  r")
        print("  bodyik reset")
        print("  r")



# ============================================================
# V18 MODE SWITCHES + BEZIER SHOWCASE + SAVED IK/BEZIER DEFAULTS
# ============================================================
# Purpose:
#   Keep all successful modes available from BOTH terminal and Web UI:
#     1) fixed / hardcoded gait    -> original joint-space movement
#     2) IK motion mode            -> current best reach/speed native IK, Bézier OFF
#     3) IK Bézier motion mode     -> smoother motor-friendly curved foot arc
#     4) Body IK posture test      -> optional posture/tilt proof-of-concept
#
# Recommendation:
#   - Default at startup stays safe: hardcoded/fixed unless the user enables IK.
#   - `ik motion` is the best practical walking mode for distance/speed.
#   - `ik bezier_motion` is smoother and gentler, but less distance-efficient.
# ============================================================

IK_STEP_CM = 8.5
IK_SUPPORT_PUSH_CM = 4.2
IK_LIFT_CM = 7.0
IK_TURN_DEG = 18.0
IK_LIFT_TARGET_SCALE = 1.22
IK_NATIVE_LIFT_EXTRA_HOLD = 0.28
IK_NATIVE_LIFT_MIN_FRAME_DELAY = 0.040
IK_NATIVE_LIFT_SMOOTH_STEPS = 12
IK_NATIVE_SMOOTH_STEPS = 8
IK_NATIVE_MIN_FRAME_DELAY = 0.022
IK_NATIVE_EXTRA_END_HOLD = 0.035
IK_STRAFE_MIDDLE_REACH_MULT = 1.70
IK_STRAFE_OUTER_REACH_MULT = 1.12
IK_BEZIER_STEPS = 10
IK_BEZIER_FRAME_DELAY = 0.030
IK_BEZIER_ARC_EXTRA_CM = 1.20
IK_NATIVE_BEZIER_ENABLED = False
CURRENT_MOTION_PROFILE = "fixed-gait"


def apply_motion_profile(profile: str):
    global IK_ENABLED, IK_NATIVE_GAIT, IK_NATIVE_BEZIER_ENABLED, IK_NATIVE_SMOOTH_SEND
    global IK_STEP_CM, IK_SUPPORT_PUSH_CM, IK_LIFT_CM, IK_TURN_DEG
    global IK_LIFT_TARGET_SCALE, IK_NATIVE_LIFT_EXTRA_HOLD, IK_NATIVE_LIFT_MIN_FRAME_DELAY, IK_NATIVE_LIFT_SMOOTH_STEPS
    global IK_NATIVE_SMOOTH_STEPS, IK_NATIVE_MIN_FRAME_DELAY, IK_NATIVE_EXTRA_END_HOLD
    global IK_STRAFE_MIDDLE_REACH_MULT, IK_STRAFE_OUTER_REACH_MULT
    global IK_BEZIER_STEPS, IK_BEZIER_FRAME_DELAY, IK_BEZIER_ARC_EXTRA_CM
    global BODY_IK_ENABLED, CURRENT_MOTION_PROFILE

    p = (profile or "").lower().strip()

    IK_STEP_CM = 8.5
    IK_SUPPORT_PUSH_CM = 4.2
    IK_LIFT_CM = 7.0
    IK_TURN_DEG = 18.0
    IK_LIFT_TARGET_SCALE = 1.22
    IK_NATIVE_LIFT_EXTRA_HOLD = 0.28
    IK_NATIVE_LIFT_MIN_FRAME_DELAY = 0.040
    IK_NATIVE_LIFT_SMOOTH_STEPS = 12
    IK_NATIVE_SMOOTH_STEPS = 8
    IK_NATIVE_MIN_FRAME_DELAY = 0.022
    IK_NATIVE_EXTRA_END_HOLD = 0.035
    IK_STRAFE_MIDDLE_REACH_MULT = 1.70
    IK_STRAFE_OUTER_REACH_MULT = 1.12
    IK_BEZIER_STEPS = 10
    IK_BEZIER_FRAME_DELAY = 0.030
    IK_BEZIER_ARC_EXTRA_CM = 1.20
    IK_NATIVE_SMOOTH_SEND = True

    if p in ["fixed", "hardcoded", "raw", "legacy", "default"]:
        IK_ENABLED = False
        IK_NATIVE_GAIT = True
        IK_NATIVE_BEZIER_ENABLED = False
        BODY_IK_ENABLED = False
        CURRENT_MOTION_PROFILE = "fixed-gait"
        return "Fixed Gait selected. Original hardcoded joint-space movement will be used."

    if p in ["ik", "motion", "ik_motion", "efficient", "native", "normal", "fast"]:
        IK_ENABLED = True
        IK_NATIVE_GAIT = True
        IK_NATIVE_BEZIER_ENABLED = False
        BODY_IK_ENABLED = False
        CURRENT_MOTION_PROFILE = "ik-motion"
        return "IK Motion selected. Uses IK foot targets without Bézier curve. Best reach/speed mode."

    if p in ["bezier", "bezier_motion", "ik_bezier_motion", "curve", "smooth", "arc"]:
        IK_ENABLED = True
        IK_NATIVE_GAIT = True
        IK_NATIVE_BEZIER_ENABLED = True
        BODY_IK_ENABLED = False
        CURRENT_MOTION_PROFILE = "ik-bezier-motion"
        return "IK Bézier Motion selected. Uses curved foot arc. Smoother/gentler but less distance-efficient."

    if p in ["bezier_demo", "bezier_showcase", "showcase", "big_curve", "obvious_curve", "curve_show", "bezier_extreme", "extreme_curve", "demo_curve"]:
        IK_ENABLED = True
        IK_NATIVE_GAIT = True
        IK_NATIVE_BEZIER_ENABLED = True
        BODY_IK_ENABLED = False
        # More obvious curve profile for visual comparison, not the most efficient walking preset.
        IK_STEP_CM = 5.8
        IK_SUPPORT_PUSH_CM = 3.0
        IK_LIFT_CM = 8.0
        IK_TURN_DEG = 18.0
        IK_LIFT_TARGET_SCALE = 1.35
        IK_NATIVE_LIFT_EXTRA_HOLD = 0.38
        IK_BEZIER_STEPS = 20
        IK_BEZIER_FRAME_DELAY = 0.040
        IK_BEZIER_ARC_EXTRA_CM = 6.0
        IK_NATIVE_EXTRA_END_HOLD = 0.070
        CURRENT_MOTION_PROFILE = "ik-bezier-demo-motion"
        return "Bézier Demo Motion selected. Extreme visible foot arc for demo/comparison; slower and less distance-efficient."

    if p in ["bodyik", "body", "posture"]:
        IK_ENABLED = True
        IK_NATIVE_GAIT = True
        IK_NATIVE_BEZIER_ENABLED = False
        BODY_IK_ENABLED = True
        CURRENT_MOTION_PROFILE = "bodyik-posture-test"
        return "Body IK Posture Test selected. Use bodyik height/pitch/roll then r."

    return "Unknown profile. Use: fixed, motion, bezier_motion, bezier_demo, bodyik."


def print_motion_profile_status():
    print()
    print("===================================================")
    print(" MOTION PROFILE SWITCHES / V19")
    print("===================================================")
    print(f"Current profile : {CURRENT_MOTION_PROFILE}")
    print(f"IK enabled      : {IK_ENABLED}")
    print(f"Native IK       : {IK_NATIVE_GAIT}")
    print(f"Bezier arc      : {IK_NATIVE_BEZIER_ENABLED}")
    print(f"Body IK         : {BODY_IK_ENABLED}")
    print(f"Step/Lift/Turn  : step={IK_STEP_CM:.1f} cm, lift={IK_LIFT_CM:.1f} cm, turn={IK_TURN_DEG:.1f} deg")
    print(f"Strafe mid      : {IK_STRAFE_MIDDLE_REACH_MULT:.2f}x")
    print(f"Bezier          : steps={IK_BEZIER_STEPS}, delay={IK_BEZIER_FRAME_DELAY:.3f}s, arc={IK_BEZIER_ARC_EXTRA_CM:.1f} cm")
    print("Terminal switches:")
    print("  ik fixed             # original hardcoded movement")
    print("  ik motion            # best IK reach/speed, Bézier OFF")
    print("  ik bezier_motion     # smoother IK Bézier motion with saved good settings")
    print("  ik bezier_demo       # extreme demo curve to clearly see Bézier foot arc")
    print("  ik bezier on/off     # quick Bezier toggle after IK is on")
    print("  ik bodyik            # body IK posture test mode")
    print("===================================================")


_PREVIOUS_ACTION_IK_SETTINGS_V16 = action_ik_settings
_PREVIOUS_PRINT_IK_STATUS_V16 = print_ik_status


def print_ik_status():
    _PREVIOUS_PRINT_IK_STATUS_V16()
    print_motion_profile_status()


def action_ik_settings(parts: List[str]):
    global IK_ENABLED, IK_NATIVE_GAIT, IK_NATIVE_BEZIER_ENABLED, CURRENT_MOTION_PROFILE

    if len(parts) == 1:
        print_ik_status()
        return

    sub = parts[1].lower()

    if sub in ["fixed", "hardcoded", "raw", "legacy", "default"]:
        print(apply_motion_profile("fixed"))
        return
    if sub in ["motion", "ik_motion", "efficient", "native_default", "normal", "fast_default"]:
        print(apply_motion_profile("ik"))
        return
    if sub in ["bezier_motion", "ik_bezier_motion", "bezier_default", "curve_default", "smooth_default"]:
        print(apply_motion_profile("bezier"))
        return
    if sub in ["bezier_demo", "bezier_showcase", "showcase", "big_curve", "obvious_curve", "curve_show", "bezier_extreme", "extreme_curve", "demo_curve"]:
        print(apply_motion_profile("bezier_showcase"))
        return
    if sub in ["bodyik", "body_default", "posture_default"]:
        print(apply_motion_profile("bodyik"))
        return
    if sub in ["on", "true", "1", "enable", "enabled"]:
        print(apply_motion_profile("ik"))
        return
    if sub in ["off", "false", "0", "disable", "disabled"]:
        print(apply_motion_profile("fixed"))
        return

    if sub in ["bezier", "curve", "arc"] and len(parts) >= 3:
        value = parts[2].lower()
        if value in ["on", "true", "1", "yes"]:
            IK_ENABLED = True
            IK_NATIVE_GAIT = True
            IK_NATIVE_BEZIER_ENABLED = True
            CURRENT_MOTION_PROFILE = "ik-bezier-motion"
            print("IK Bézier foot arc ON. Current profile = ik-bezier-motion.")
            return
        if value in ["off", "false", "0", "no"]:
            IK_NATIVE_BEZIER_ENABLED = False
            CURRENT_MOTION_PROFILE = "ik-motion" if IK_ENABLED else "fixed"
            print("IK Bézier foot arc OFF. Current profile = ik-motion if IK is enabled.")
            return

    _PREVIOUS_ACTION_IK_SETTINGS_V16(parts)


_PREVIOUS_WEB_STATE_V16 = web_state


def web_state(bus: Optional[DynamixelBus] = None):
    s = _PREVIOUS_WEB_STATE_V16(bus)
    s["ik"] = {
        "profile": CURRENT_MOTION_PROFILE,
        "enabled": IK_ENABLED,
        "native": IK_NATIVE_GAIT,
        "bezier": IK_NATIVE_BEZIER_ENABLED,
        "step": IK_STEP_CM,
        "lift": IK_LIFT_CM,
        "turn": IK_TURN_DEG,
        "strafe_mid": IK_STRAFE_MIDDLE_REACH_MULT,
        "bezier_steps": IK_BEZIER_STEPS,
        "bezier_delay": IK_BEZIER_FRAME_DELAY,
        "bezier_arc": IK_BEZIER_ARC_EXTRA_CM,
        "bodyik": BODY_IK_ENABLED,
    }
    flags = s.setdefault("preset_flags", {})
    flags.update({
        "mode_fixed": (not IK_ENABLED),
        "mode_ik": bool(IK_ENABLED and not IK_NATIVE_BEZIER_ENABLED and not BODY_IK_ENABLED),
        "mode_bezier": bool(IK_ENABLED and IK_NATIVE_BEZIER_ENABLED),
        "mode_bodyik": bool(BODY_IK_ENABLED),
    })
    return s


_WEB_PROFILE_SECTION = """<div class=\"section\"><h2>Movement Control Mode</h2><div class=\"sub\">Switch between Fixed Gait, IK Motion, IK Bézier Motion, and Body IK posture testing. These buttons only change the motion mode</div><div class=\"presetgrid\"><button id=\"btn_mode_fixed\" class=\"btn small\" onclick=\"presetCmd(this,'ik fixed')\">Fixed Gait</button><button id=\"btn_mode_ik\" class=\"btn small\" onclick=\"presetCmd(this,'ik motion')\">IK Motion</button><button id=\"btn_mode_bezier\" class=\"btn small\" onclick=\"presetCmd(this,'ik bezier_motion')\">IK Bézier Motion</button><button class=\"btn small\" onclick=\"presetCmd(this,'ik bezier_demo')\">Bézier Demo Motion</button><button class=\"btn small\" onclick=\"presetCmd(this,'ik bezier on')\">Bézier ON</button><button class=\"btn small\" onclick=\"presetCmd(this,'ik bezier off')\">Bézier OFF</button><button id=\"btn_mode_bodyik\" class=\"btn small\" onclick=\"presetCmd(this,'ik bodyik')\">Body IK Posture</button><button class=\"btn small\" onclick=\"presetCmd(this,'ik')\">IK Status</button><button class=\"btn small\" onclick=\"presetCmd(this,'bodyik')\">Body IK Status</button><button class=\"btn small\" onclick=\"presetCmd(this,'health')\">Health Refresh</button></div></div>"""

try:
    WEB_HTML = WEB_HTML.replace('<div class="section"><h2>Quick Presets</h2>', _WEB_PROFILE_SECTION + '<div class="section"><h2>Quick Presets</h2>')
    WEB_HTML = WEB_HTML.replace("setOn('btn_movestats_off',f.movestats_off);", "setOn('btn_movestats_off',f.movestats_off);setOn('btn_mode_fixed',f.mode_fixed);setOn('btn_mode_ik',f.mode_ik);setOn('btn_mode_bezier',f.mode_bezier);setOn('btn_mode_bodyik',f.mode_bodyik);")
    WEB_HTML = WEB_HTML.replace("<span class=\"pill\">End: ${s.timing.end_mode}</span>", "<span class=\"pill\">End: ${s.timing.end_mode}</span><span class=\"pill\">Motion Mode: ${s.ik?s.ik.profile:'--'}</span><span class=\"pill\">Bézier: ${s.ik?s.ik.bezier:false}</span><span class=\"pill\">IK Lift: ${s.ik?s.ik.lift:'--'} cm</span>")
except Exception as _web_patch_error:
    print(f"[V17 WEB PATCH WARNING] {type(_web_patch_error).__name__}: {_web_patch_error}")


# ============================================================
# V20 CM530 REARM + IK STEP/LIFT UI FINAL PATCH
# ============================================================
# Extra final-layer safety/tuning patch added after V18 so it wins over all
# earlier versioned overrides in this file.

IK_STEP_CM = 8.5
IK_SUPPORT_PUSH_CM = 4.2
IK_MR_RL_EXTRA_LIFT_CM = {"MR": 1.0, "RL": 1.0}

_PREVIOUS_IK_NATIVE_TARGET_V20 = ik_native_target
def ik_native_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    foot = _PREVIOUS_IK_NATIVE_TARGET_V20(leg, direction, station, lifted)
    if lifted:
        foot["z"] += float(IK_MR_RL_EXTRA_LIFT_CM.get(leg, 0.0))
    return foot

_PREVIOUS_ACTION_IK_SETTINGS_V20 = action_ik_settings
def action_ik_settings(parts: List[str]):
    global IK_STEP_CM, IK_SUPPORT_PUSH_CM, IK_MR_RL_EXTRA_LIFT_CM

    if len(parts) >= 2 and parts[1].lower() in ["step", "swing", "stride", "reach"]:
        if len(parts) >= 3:
            try:
                IK_STEP_CM = clamp_float(float(parts[2]), 2.0, 10.0)
                # Keep support push roughly proportional for native IK.
                IK_SUPPORT_PUSH_CM = clamp_float(IK_STEP_CM * 0.50, 0.0, 5.5)
                print(f"IK swing/step = {IK_STEP_CM:.2f} cm | support = {IK_SUPPORT_PUSH_CM:.2f} cm")
            except Exception:
                print("Usage: ik step 8.5")
        else:
            print(f"IK swing/step = {IK_STEP_CM:.2f} cm")
        return

    if len(parts) >= 2 and parts[1].lower() in ["mrrllift", "leglift", "mrrl_lift", "unevenlift"]:
        if len(parts) >= 3:
            try:
                value = clamp_float(float(parts[2]), 0.0, 2.5)
                IK_MR_RL_EXTRA_LIFT_CM["MR"] = value
                IK_MR_RL_EXTRA_LIFT_CM["RL"] = value
                print(f"IK MR/RL extra lifted foot height = {value:.2f} cm")
            except Exception:
                print("Usage: ik leglift 1.0")
        else:
            print(f"IK MR/RL extra lifted foot height = MR {IK_MR_RL_EXTRA_LIFT_CM['MR']:.2f} cm, RL {IK_MR_RL_EXTRA_LIFT_CM['RL']:.2f} cm")
        return

    return _PREVIOUS_ACTION_IK_SETTINGS_V20(parts)

# Patch web HTML after every earlier web patch has run.
try:
    WEB_HTML = WEB_HTML.replace('id="liftlevel" type="range" min="1" max="9"', 'id="liftlevel" type="range" min="1" max="12"')
    WEB_HTML = WEB_HTML.replace('<option>8</option><option>9</option></select><input id="liftLegs"', '<option>8</option><option>9</option><option>10</option><option>11</option><option>12</option></select><input id="liftLegs"')
    if 'id="ikstep"' not in WEB_HTML:
        WEB_HTML = WEB_HTML.replace(
            '<div class="row"><label>Walk Lift Level</label><input id="liftlevel" type="range" min="1" max="12" value="6" oninput="sv(\'liftlevelv\',this.value)" onchange="cmd(\'walklift level \'+this.value)"><span class="value" id="liftlevelv">6</span></div>',
            '<div class="row"><label>Walk Lift Level</label><input id="liftlevel" type="range" min="1" max="12" value="6" oninput="sv(\'liftlevelv\',this.value)" onchange="cmd(\'walklift level \'+this.value)"><span class="value" id="liftlevelv">6</span></div><div class="row"><label>IK Swing / Step</label><input id="ikstep" type="range" min="4.0" max="10.0" step="0.1" value="8.5" oninput="sv(\'ikstepv\',Number(this.value).toFixed(1))" onchange="cmd(\'ik step \'+this.value)"><span class="value" id="ikstepv">8.5</span></div>'
        )
    if "document.getElementById('ikstep')" not in WEB_HTML:
        WEB_HTML = WEB_HTML.replace(
            "document.getElementById('liftlevel').value=s.walk_lift.level;sv('liftlevelv',s.walk_lift.level);document.getElementById('phase').value=s.timing.phase;",
            "document.getElementById('liftlevel').value=s.walk_lift.level;sv('liftlevelv',s.walk_lift.level);if(document.getElementById('ikstep')&&s.ik){document.getElementById('ikstep').value=s.ik.step;sv('ikstepv',Number(s.ik.step).toFixed(1));}document.getElementById('phase').value=s.timing.phase;"
        )
except Exception as _v20_web_patch_error:
    print(f"[V20 WEB PATCH WARNING] {type(_v20_web_patch_error).__name__}: {_v20_web_patch_error}")



# ============================================================
# OPEN_DAY_ML_MR_SIDE_STRAFE_STABILITY_PATCH
# ============================================================
# Open-day hardware tuning ONLY for side-strafe foot placement.
# Cooldown / READY timing is intentionally kept exactly the same as the last
# working IKControlC_open_day_mr_right.py file because the user said cooldown
# is perfect and must not be touched.
#
# What this changes:
# - MR on RIGHT strafe reaches farther outward before the pull phase.
# - MR on RIGHT strafe is prevented from tucking too far inward under the body.
# - ML gets the same mirrored protection for LEFT strafe.
# ============================================================

# MR right-strafe foot-space guard. y is negative on the robot's right side.
# More negative = farther outward/right; less negative = closer to body centre.
IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM = 1.60
IK_MR_RIGHT_MIN_OUTWARD_Y_CM = -8.80
IK_MR_RIGHT_SUPPORT_REAR_SCALE = 0.42

# ML left-strafe mirror guard. y is positive on the robot's left side.
# More positive = farther outward/left; smaller positive = closer to body centre.
IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM = 1.60
IK_ML_LEFT_MIN_OUTWARD_Y_CM = 8.80
IK_ML_LEFT_SUPPORT_REAR_SCALE = 0.42

# Web-only fast READY recovery timings from IKControlC.
# DO NOT change these in this patch; cooldown was reported as perfect.
WEB_POST_MOTION_IDLE_DELAY = 0.0
WEB_READY_RECOVERY_REARM = False
WEB_READY_UP_HOLD = 0.075
WEB_READY_DOWN_HOLD = 0.045
WEB_READY_FINAL_HOLD = 0.050

_PREVIOUS_IK_NATIVE_LINEAR_TARGET_MIDDLE_SIDE = ik_native_linear_target


def _middle_side_station_base(station: str) -> str:
    station_base, _station_scale = _ik_station_base_and_scale((station or "neutral").lower(), for_turn=False)
    return station_base


def _clamp_mr_right_outward(foot: Dict[str, float]) -> None:
    if float(foot["y"]) > float(IK_MR_RIGHT_MIN_OUTWARD_Y_CM):
        foot["y"] = float(IK_MR_RIGHT_MIN_OUTWARD_Y_CM)


def _clamp_ml_left_outward(foot: Dict[str, float]) -> None:
    if float(foot["y"]) < float(IK_ML_LEFT_MIN_OUTWARD_Y_CM):
        foot["y"] = float(IK_ML_LEFT_MIN_OUTWARD_Y_CM)


def ik_native_linear_target(leg: str, direction: str, station: str, lifted: bool = False) -> Dict[str, float]:
    """
    Final open-day ML/MR side-strafe override.

    For RIGHT strafe:
      - MR front/reach target goes farther outward/right before pulling.
      - MR rear/support target is softened and clamped so the tibia does not
        bend inward too much under the body while supporting the robot.

    For LEFT strafe:
      - ML gets the same mirrored behaviour.

    READY recovery/cooldown is not modified by this override.
    """
    direction_n = normalize_direction(direction)
    station_base = _middle_side_station_base(station)
    foot = _PREVIOUS_IK_NATIVE_LINEAR_TARGET_MIDDLE_SIDE(leg, direction_n, station, lifted)

    if leg == "MR" and direction_n == "right":
        base_y = float(IK_DEFAULT_FEET_CM["MR"]["y"])

        if station_base == "front":
            # The issue seen on hardware: MR barely reaches outward, then starts
            # pulling inward, so the robot tips. Give it more outward/right reach
            # before the pull phase begins.
            foot["y"] -= float(IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM)

        elif station_base == "rear":
            # Rear/support is where MR can tuck too close under the body. Keep
            # only part of the inward movement, then clamp it safely outward.
            inward_delta = (float(foot["y"]) - base_y) * float(IK_MR_RIGHT_SUPPORT_REAR_SCALE)
            foot["y"] = base_y + inward_delta

        if station_base in ["front", "rear"]:
            _clamp_mr_right_outward(foot)

    elif leg == "ML" and direction_n == "left":
        base_y = float(IK_DEFAULT_FEET_CM["ML"]["y"])

        if station_base == "front":
            # Mirrored version of MR-right: ML reaches farther outward/left
            # before the support/pull phase.
            foot["y"] += float(IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM)

        elif station_base == "rear":
            # Prevent ML from tucking inward under the body during left strafe.
            inward_delta = (float(foot["y"]) - base_y) * float(IK_ML_LEFT_SUPPORT_REAR_SCALE)
            foot["y"] = base_y + inward_delta

        if station_base in ["front", "rear"]:
            _clamp_ml_left_outward(foot)

    return foot


def web_return_to_ready_hold_release(bus: DynamixelBus, direction: str):
    """
    Web/controller release recovery.

    This is intentionally the same READY/cooldown behaviour as the previous
    IKControlC open-day file. It still sends READY before becoming idle, and the
    user reported this cooldown timing is perfect.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    web_log(f"Fast hold-release return to READY from {direction}...")
    ready_pose = level_ready_pose()

    try:
        if WEB_READY_RECOVERY_REARM:
            bus.rearm_after_power_cycle(READY_SPEED, reason="web fast ready")
    except Exception as e:
        web_log(f"WEB ready rearm warning: {type(e).__name__}: {e}")

    def lift_tripod_for_ready(legs):
        t = level_ready_pose()
        for leg_name in legs:
            t.update(build_leg_offset_targets(leg_name, 0.0, SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG))
        return t

    for mode, legs, hold in [
        ("WEB_READY_B_UP",   CRAB_FIRST_TRIPOD,  WEB_READY_UP_HOLD),
        ("WEB_READY_B_DOWN", None,               WEB_READY_DOWN_HOLD),
        ("WEB_READY_A_UP",   CRAB_SECOND_TRIPOD, WEB_READY_UP_HOLD),
        ("WEB_READY_A_DOWN", None,               WEB_READY_DOWN_HOLD),
    ]:
        CURRENT_MODE = mode
        targets = lift_tripod_for_ready(legs) if legs else level_ready_pose()
        bus.move_sync(targets, speed=READY_SPEED)
        ACTIVE_GOALS = dict(targets)
        if hold > 0:
            time.sleep(float(hold))

    CURRENT_MODE = "READY_REFINED2K"
    ready_pose = level_ready_pose()
    bus.move_sync(ready_pose, speed=READY_SPEED)
    ACTIVE_GOALS = dict(ready_pose)
    if WEB_READY_FINAL_HOLD > 0:
        time.sleep(float(WEB_READY_FINAL_HOLD))
    web_log("Fast READY recovery sent. Idle is available now.")


_PREVIOUS_ACTION_IK_SETTINGS_MIDDLE_SIDE = action_ik_settings


def action_ik_settings(parts: List[str]):
    global IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM, IK_MR_RIGHT_MIN_OUTWARD_Y_CM, IK_MR_RIGHT_SUPPORT_REAR_SCALE
    global IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM, IK_ML_LEFT_MIN_OUTWARD_Y_CM, IK_ML_LEFT_SUPPORT_REAR_SCALE
    global WEB_READY_UP_HOLD, WEB_READY_DOWN_HOLD, WEB_READY_FINAL_HOLD

    if len(parts) >= 2 and parts[1].lower() in ["turnmode", "qemode", "turnpushmode"]:
        global IK_TURN_PUSH_AFTER_TOUCHDOWN
        if len(parts) == 2:
            print(f"IK Q/E turn mode = {'afterdown' if IK_TURN_PUSH_AFTER_TOUCHDOWN else 'arc'}")
            print("Usage: ik turnmode afterdown   OR   ik turnmode arc")
            return
        mode = parts[2].lower()
        if mode in ["afterdown", "touchdown", "ground", "land", "landing"]:
            IK_TURN_PUSH_AFTER_TOUCHDOWN = True
            print("IK Q/E turn mode = afterdown: swing tripod lands/settles first, then support tripod pushes.")
            return
        if mode in ["arc", "bezier", "old", "during"]:
            IK_TURN_PUSH_AFTER_TOUCHDOWN = False
            print("IK Q/E turn mode = arc: support push happens during Bezier swing using ik turnpushdelay.")
            return
        print("Usage: ik turnmode afterdown   OR   ik turnmode arc")
        return

    if len(parts) >= 2 and parts[1].lower() in ["turnsettle", "qesettle", "landsettle", "touchdown"]:
        global IK_TURN_TOUCHDOWN_SETTLE
        if len(parts) == 2:
            print(f"IK Q/E touchdown settle = {IK_TURN_TOUCHDOWN_SETTLE:.2f}s")
            print("Usage: ik turnsettle 0.18   # try 0.10..0.35")
            return
        try:
            IK_TURN_TOUCHDOWN_SETTLE = clamp_float(float(parts[2]), 0.0, 0.60)
            print(f"IK Q/E touchdown settle = {IK_TURN_TOUCHDOWN_SETTLE:.2f}s")
        except Exception:
            print("Usage: ik turnsettle 0.18")
        return

    if len(parts) >= 2 and parts[1].lower() in ["turnpushframes", "qepushframes", "groundpushframes"]:
        global IK_TURN_GROUND_PUSH_STEPS
        if len(parts) == 2:
            print(f"IK Q/E ground push frames = {IK_TURN_GROUND_PUSH_STEPS}")
            print("Usage: ik turnpushframes 6   # try 4..10")
            return
        try:
            IK_TURN_GROUND_PUSH_STEPS = int(clamp_float(float(parts[2]), 1, 14))
            print(f"IK Q/E ground push frames = {IK_TURN_GROUND_PUSH_STEPS}")
        except Exception:
            print("Usage: ik turnpushframes 6")
        return

    if len(parts) >= 2 and parts[1].lower() in ["turnpushframe", "qepushframe", "groundpushframe"]:
        global IK_TURN_GROUND_PUSH_FRAME_DELAY
        if len(parts) == 2:
            print(f"IK Q/E ground push frame delay = {IK_TURN_GROUND_PUSH_FRAME_DELAY:.3f}s")
            print("Usage: ik turnpushframe 0.030   # try 0.020..0.055")
            return
        try:
            IK_TURN_GROUND_PUSH_FRAME_DELAY = clamp_float(float(parts[2]), 0.005, 0.120)
            print(f"IK Q/E ground push frame delay = {IK_TURN_GROUND_PUSH_FRAME_DELAY:.3f}s")
        except Exception:
            print("Usage: ik turnpushframe 0.030")
        return

    if len(parts) >= 2 and parts[1].lower() in ["turnpushsettle", "qepushsettle"]:
        global IK_TURN_GROUND_PUSH_SETTLE
        if len(parts) == 2:
            print(f"IK Q/E ground push settle = {IK_TURN_GROUND_PUSH_SETTLE:.2f}s")
            print("Usage: ik turnpushsettle 0.06   # try 0.00..0.20")
            return
        try:
            IK_TURN_GROUND_PUSH_SETTLE = clamp_float(float(parts[2]), 0.0, 0.40)
            print(f"IK Q/E ground push settle = {IK_TURN_GROUND_PUSH_SETTLE:.2f}s")
        except Exception:
            print("Usage: ik turnpushsettle 0.06")
        return

    if len(parts) >= 2 and parts[1].lower() in ["turnpushdelay", "turn_push_delay", "qepushdelay", "qe_delay", "pushdelay"]:
        global IK_TURN_BEZIER_SUPPORT_START
        if len(parts) == 2:
            print(f"IK Q/E turn support-push delay = {IK_TURN_BEZIER_SUPPORT_START:.2f} of Bezier arc")
            print("Usage: ik turnpushdelay 0.55   # try 0.45..0.75")
            return
        try:
            IK_TURN_BEZIER_SUPPORT_START = clamp_float(float(parts[2]), 0.08, 0.85)
            print(f"IK Q/E turn support-push delay = {IK_TURN_BEZIER_SUPPORT_START:.2f} of Bezier arc")
            print("This does not slow the whole motion; it only starts the support push later inside Q/E frames.")
        except Exception:
            print("Usage: ik turnpushdelay 0.55")
        return

    if len(parts) >= 2 and parts[1].lower() in ["mrright", "mr_right", "rightmr"]:
        if len(parts) == 2:
            print("MR right strafe guard:")
            print(f"  extra outward reach = {IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM:.2f} cm")
            print(f"  min outward y clamp = {IK_MR_RIGHT_MIN_OUTWARD_Y_CM:.2f} cm")
            print(f"  rear/support scale  = {IK_MR_RIGHT_SUPPORT_REAR_SCALE:.2f}")
            print("Usage: ik mrright reach 1.6 | ik mrright clamp -8.8 | ik mrright rear 0.42")
            return
        if len(parts) >= 4:
            key = parts[2].lower()
            try:
                value = float(parts[3])
            except Exception:
                print("MR right value must be a number.")
                return
            if key in ["reach", "out", "outward"]:
                IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM = clamp_float(value, 0.0, 3.0)
                print(f"MR right extra outward reach = {IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM:.2f} cm")
                return
            if key in ["clamp", "miny", "min"]:
                IK_MR_RIGHT_MIN_OUTWARD_Y_CM = clamp_float(value, -13.0, -5.0)
                print(f"MR right inward clamp y = {IK_MR_RIGHT_MIN_OUTWARD_Y_CM:.2f} cm")
                return
            if key in ["rear", "support", "pull"]:
                IK_MR_RIGHT_SUPPORT_REAR_SCALE = clamp_float(value, 0.15, 1.0)
                print(f"MR right rear/support scale = {IK_MR_RIGHT_SUPPORT_REAR_SCALE:.2f}")
                return
        print("Usage: ik mrright reach 1.6 | ik mrright clamp -8.8 | ik mrright rear 0.42")
        return

    if len(parts) >= 2 and parts[1].lower() in ["mlleft", "ml_left", "leftml"]:
        if len(parts) == 2:
            print("ML left strafe guard:")
            print(f"  extra outward reach = {IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM:.2f} cm")
            print(f"  min outward y clamp = {IK_ML_LEFT_MIN_OUTWARD_Y_CM:.2f} cm")
            print(f"  rear/support scale  = {IK_ML_LEFT_SUPPORT_REAR_SCALE:.2f}")
            print("Usage: ik mlleft reach 1.6 | ik mlleft clamp 8.8 | ik mlleft rear 0.42")
            return
        if len(parts) >= 4:
            key = parts[2].lower()
            try:
                value = float(parts[3])
            except Exception:
                print("ML left value must be a number.")
                return
            if key in ["reach", "out", "outward"]:
                IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM = clamp_float(value, 0.0, 3.0)
                print(f"ML left extra outward reach = {IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM:.2f} cm")
                return
            if key in ["clamp", "miny", "min"]:
                IK_ML_LEFT_MIN_OUTWARD_Y_CM = clamp_float(value, 5.0, 13.0)
                print(f"ML left inward clamp y = {IK_ML_LEFT_MIN_OUTWARD_Y_CM:.2f} cm")
                return
            if key in ["rear", "support", "pull"]:
                IK_ML_LEFT_SUPPORT_REAR_SCALE = clamp_float(value, 0.15, 1.0)
                print(f"ML left rear/support scale = {IK_ML_LEFT_SUPPORT_REAR_SCALE:.2f}")
                return
        print("Usage: ik mlleft reach 1.6 | ik mlleft clamp 8.8 | ik mlleft rear 0.42")
        return

    if len(parts) >= 2 and parts[1].lower() in ["webready", "readyfast", "fastready"]:
        if len(parts) == 2:
            print(f"Web fast READY holds: up={WEB_READY_UP_HOLD:.3f}s down={WEB_READY_DOWN_HOLD:.3f}s final={WEB_READY_FINAL_HOLD:.3f}s")
            print("Usage: ik webready up 0.075 | ik webready down 0.045 | ik webready final 0.050")
            return
        if len(parts) >= 4:
            key = parts[2].lower()
            try:
                value = clamp_float(float(parts[3]), 0.0, 0.20)
            except Exception:
                print("Web ready hold value must be a number.")
                return
            if key == "up":
                WEB_READY_UP_HOLD = value
                print(f"Web READY up hold = {WEB_READY_UP_HOLD:.3f}s")
                return
            if key in ["down", "settle"]:
                WEB_READY_DOWN_HOLD = value
                print(f"Web READY down hold = {WEB_READY_DOWN_HOLD:.3f}s")
                return
            if key in ["final", "idle"]:
                WEB_READY_FINAL_HOLD = value
                print(f"Web READY final hold = {WEB_READY_FINAL_HOLD:.3f}s")
                return
        print("Usage: ik webready up 0.075 | ik webready down 0.045 | ik webready final 0.050")
        return

    return _PREVIOUS_ACTION_IK_SETTINGS_MIDDLE_SIDE(parts)


_PREVIOUS_WEB_STATE_MIDDLE_SIDE = web_state


def web_state(bus: Optional[DynamixelBus] = None):
    s = _PREVIOUS_WEB_STATE_MIDDLE_SIDE(bus)
    if "ik" in s:
        s["ik"].update({
            "mr_right_reach": IK_MR_RIGHT_EXTRA_OUTWARD_REACH_CM,
            "mr_right_clamp_y": IK_MR_RIGHT_MIN_OUTWARD_Y_CM,
            "mr_right_rear_scale": IK_MR_RIGHT_SUPPORT_REAR_SCALE,
            "ml_left_reach": IK_ML_LEFT_EXTRA_OUTWARD_REACH_CM,
            "ml_left_clamp_y": IK_ML_LEFT_MIN_OUTWARD_Y_CM,
            "ml_left_rear_scale": IK_ML_LEFT_SUPPORT_REAR_SCALE,
            "web_ready_up": WEB_READY_UP_HOLD,
            "web_ready_down": WEB_READY_DOWN_HOLD,
            "web_ready_final": WEB_READY_FINAL_HOLD,
        })
    return s

# OPEN_DAY_ML_MR_SIDE_STRAFE_STABILITY_PATCH applied.

def main():
    global WEB_BUS
    run_mode = choose_run_mode()
    selected_port = choose_serial_port()
    bus = DynamixelBus(selected_port)
    WEB_BUS = bus
    if not bus.open():
        return
    try:
        apply_web_startup_defaults()
        if run_mode == "web":
            run_web_mode(bus, selected_port)
        else:
            terminal_loop(bus)
    finally:
        if run_mode == "web":
            web_stop_motion()
            try:
                if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
                    WEB_MOTION_THREAD.join(timeout=3.0)
            except Exception:
                pass
        bus.close()


if __name__ == "__main__":
    main()

# OPEN_DAY_READY_FIX: web release keeps READY recovery and skips only post-motion health cooldown.


# OFFSET_NORMALIZED_TEST_NOT_STAR_READY: READY_POSE keeps old standing shape but normalizes per-joint physical offsets.


# ============================================================
# CALIBRATED_OFFSETS_V3_KEEP_HIPS
# ============================================================
# This test branch intentionally keeps the calibrated hip normalization.
# It does NOT revert hips to the old ready pose.
#
# Calibration idea used here:
#   measured flat/star center = physical joint reference
#   old standing READY        = original stance shape
#   normalized READY          = measured center + shared group offset
#
# Hips, femurs, and tibias are all included in the normalized READY_POSE.
# Fixed gait and IK movement both use READY_POSE / level_ready_pose() as the
# base reference, so they automatically use this calibration.
# Absolute raw pose dictionaries such as PUSHUP_LEVELS were already rebased.
# ============================================================

# CALIBRATED_OFFSETS_V4_DIRECT_HIP_CENTERS: hips use calibrated center raw values; femur/tibia use normalized standing offsets.


# CALIBRATED_OFFSETS_V5_ALL_HIPS_DIRECT: hips 1/2/8/13/14=520, ID7=575; femur/tibia normalized; pushup rebased.

# ============================================================
# V8 IK Q/E TURN PUSH-DELAY-ONLY PATCH
# ============================================================
# Based from v5 calibrated pose.
# Fixed gait timing unchanged.
# Normal IK timing unchanged.
# Q/E IK turning only: support legs stay neutral longer inside the Bezier arc,
# so the rotational push begins later, closer to swing-foot touchdown.
# Runtime tuning:
#   ik turnpushdelay 0.45
#   ik turnpushdelay 0.55
#   ik turnpushdelay 0.70
# ============================================================

# ============================================================
# V9 IK Q/E TOUCHDOWN-THEN-PUSH PATCH
# ============================================================
# Based from v8/v5 calibrated pose.
# Fixed gait unchanged.
# Normal IK W/S/A/D unchanged.
# Q/E IK turning only:
#   1) swing tripod lifts and lands while support tripod stays neutral
#   2) short touchdown settle
#   3) support tripod performs ground push while landed tripod remains planted
#   4) only then the next tripod is allowed to lift
# Runtime tuning:
#   ik turnmode afterdown
#   ik turnsettle 0.18
#   ik turnpushframes 6
#   ik turnpushframe 0.030
#   ik turnpushsettle 0.06
# ============================================================
