# ============================================================
# SCONTROLX2 - SEMI-OVERLAP TRIPOD GAIT
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
import threading
import io
import contextlib
from typing import Dict, Optional, Tuple, List, Any
from dataclasses import dataclass, asdict

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


# ============================================================
# RESEARCH / FORMULA GAIT MODEL LAYER (SAFE ADD-ON)
# ============================================================
# This layer does NOT change the tuned gait values. It names the
# existing constants as a reusable model so the movement can be
# explained, exported, compared, and reproduced in a paper.
#
# Core formula used by the current joint-space gait:
#   raw_target = level_ready_pose(body_level)[motor_id]
#              + logical_deg * RAW_PER_DEG
#              * leg_movement_sign * joint_direction
#
# Gait frame construction:
#   lifted leg hip   = lift_hip_for_leg(leg, direction, swing_deg)
#   support leg hip  = support_hip_for_leg(leg, direction, push_deg)
#   lifted femur/tibia use gait_lift_values()
#   grounded femur/tibia return to body-height ready pose
#
# The important research improvement is that the robot is no longer
# described only as random hardcoded raw positions. READY_POSE remains
# calibration data, while gait is documented as: body base + directional
# profile + leg sign matrix + phase generator.
# ============================================================

RESEARCH_MODEL_VERSION = "v8_formula_safe_2026_05_25"


@dataclass(frozen=True)
class GaitFormulaProfile:
    direction_group: str
    swing_deg: float
    support_push_deg: float
    lift_source: str
    phase_pattern: str
    notes: str


def gait_formula_profile(direction: str) -> GaitFormulaProfile:
    """Return the current tuned movement profile without changing any values."""
    d = normalize_direction(direction)
    if d == "backward":
        return GaitFormulaProfile(
            "backward", BACKWARD_HIP_SWING_DEG, BACKWARD_SUPPORT_PUSH_DEG,
            "gait_lift_values()",
            "A_UP+B_PUSH -> A_SWING+B_PUSH -> A_DOWN+B_HOLD -> B_UP+A_PUSH -> B_SWING+A_PUSH -> B_DOWN+A_HOLD",
            "Backward uses the forward hip sign matrix with opposite support direction.",
        )
    if d in ["turn_left", "turn_right"]:
        turn_scale = TURN_LEFT_SCALE if d == "turn_left" else TURN_RIGHT_SCALE
        return GaitFormulaProfile(
            "turn", TURN_HIP_SWING_DEG * turn_scale, TURN_SUPPORT_PUSH_DEG * turn_scale,
            "gait_lift_values()",
            "A_UP+B_PUSH -> A_SWING+B_PUSH -> A_DOWN+B_HOLD -> B_UP+A_PUSH -> B_SWING+A_PUSH -> B_DOWN+A_HOLD",
            f"Turn uses HIP_TURN_SIGN and per-direction scale {turn_scale:.2f}.",
        )
    if d in ["left", "right"]:
        return GaitFormulaProfile(
            "strafe_generic", STRAFE_HIP_SWING_DEG, STRAFE_SUPPORT_PUSH_DEG,
            "gait_lift_values(); note: web hold-release uses dedicated side-strafe W23 profile",
            "Generic tripod strafe phase builder. Dedicated side-strafe functions preserve the tested W23 A/D behavior.",
            "Left/right are intentionally preserved because this is the working side-strafe branch.",
        )
    return GaitFormulaProfile(
        "forward", GAIT_HIP_SWING_DEG, GAIT_SUPPORT_PUSH_DEG,
        "gait_lift_values()",
        "A_UP+B_PUSH -> A_SWING+B_PUSH -> A_DOWN+B_HOLD -> B_UP+A_PUSH -> B_SWING+A_PUSH -> B_DOWN+A_HOLD",
        "Forward baseline profile.",
    )


def research_model_snapshot() -> Dict[str, Any]:
    """Machine-readable summary for README, paper methods, or future ROS bridge."""
    lf, lt = gait_lift_values()
    return {
        "model_version": RESEARCH_MODEL_VERSION,
        "conversion": {
            "raw_per_deg": RAW_PER_DEG,
            "formula": "raw_target = base_raw + logical_deg * RAW_PER_DEG * leg_movement_sign * joint_direction",
        },
        "calibration_base": {
            "ready_pose": dict(READY_POSE),
            "body_height": {
                "level": BODY_HEIGHT_LEVEL,
                "min": BODY_HEIGHT_MIN,
                "max": BODY_HEIGHT_MAX,
                "femur_step_deg": BODY_HEIGHT_FEMUR_STEP_DEG,
                "tibia_step_deg": BODY_HEIGHT_TIBIA_STEP_DEG,
            },
        },
        "robot_model": {
            "leg_joints": LEG_JOINTS,
            "joint_info": JOINT_INFO,
            "leg_movement_sign": LEG_MOVEMENT_SIGN,
            "joint_directions": JOINT_DIRECTIONS,
            "tripod_a": TRIPOD_A,
            "tripod_b": TRIPOD_B,
        },
        "current_gait_values": {
            "lift_femur_deg": lf,
            "lift_tibia_deg": lt,
            "forward": asdict(gait_formula_profile("forward")),
            "backward": asdict(gait_formula_profile("backward")),
            "left": asdict(gait_formula_profile("left")),
            "right": asdict(gait_formula_profile("right")),
            "turn_left": asdict(gait_formula_profile("turn_left")),
            "turn_right": asdict(gait_formula_profile("turn_right")),
        },
        "side_strafe_w23_profile": {
            "reach_femur_deg": SIDE_STRAFE_FEMUR_REACH_DEG,
            "reach_tibia_deg": SIDE_STRAFE_TIBIA_REACH_DEG,
            "pull_femur_deg": SIDE_STRAFE_FEMUR_PULL_DEG,
            "pull_tibia_deg": SIDE_STRAFE_TIBIA_PULL_DEG,
            "lift_femur_deg": SIDE_STRAFE_LIFT_FEMUR_DEG,
            "lift_tibia_deg": SIDE_STRAFE_LIFT_TIBIA_DEG,
            "flow_mode": SIDE_STRAFE_FLOW_MODE,
        },
        "safety_thresholds": {
            "temp_warn_c": TEMP_WARN_C,
            "temp_stop_c": TEMP_STOP_C,
            "load_warn": LOAD_WARN,
            "load_stop": LOAD_STOP,
            "volt_warn_v": VOLT_WARN_V,
            "volt_stop_v": VOLT_STOP_V,
            "volt_danger_v": VOLT_DANGER_V,
        },
    }


def print_research_model(direction: str = "forward"):
    d = normalize_direction(direction)
    profile = gait_formula_profile(d)
    lf, lt = gait_lift_values()
    print()
    print("===================================================")
    print(" RESEARCH FORMULA MODEL")
    print("===================================================")
    print(f"Model version : {RESEARCH_MODEL_VERSION}")
    print(f"Direction     : {d}")
    print(f"Profile group : {profile.direction_group}")
    print(f"Swing deg     : {profile.swing_deg:+.2f}")
    print(f"Support push  : {profile.support_push_deg:+.2f}")
    print(f"Lift femur    : {lf:+.2f}")
    print(f"Lift tibia    : {lt:+.2f}")
    print(f"Phase pattern : {profile.phase_pattern}")
    print()
    print("Formula:")
    print("  raw_target = base_raw + logical_deg * RAW_PER_DEG * leg_movement_sign * joint_direction")
    print()
    print("Meaning:")
    print("  READY_POSE is calibration data.")
    print("  body-height creates the base pose.")
    print("  gait profile supplies swing/support/lift degrees.")
    print("  leg sign tables adapt the same formula to all six legs.")
    print("===================================================")

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
    # Data/model-driven wrapper around the existing tuned constants.
    # Output is intentionally identical to the previous if/else function,
    # except turn scaling remains inside turn_hip_for_leg() as before.
    profile = gait_formula_profile(direction)
    d = normalize_direction(direction)
    if d in ["turn_left", "turn_right"]:
        return TURN_HIP_SWING_DEG, TURN_SUPPORT_PUSH_DEG
    return profile.swing_deg, profile.support_push_deg


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

def action_ready(bus: DynamixelBus, use_safety_check: bool = True):
    global ACTIVE_GOALS, CURRENT_MODE

    if use_safety_check and not pre_motion_check(bus):
        return

    print("\nACTION: FAST TRIPOD-LIFT RETURN TO READY")
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
        web_log("Usage: liftall 1-9")
        return False

    if level not in LIFT_LEVELS:
        web_log("Liftall level must be 1-9.")
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
WEB_LAST_HEALTH = {"status": "NOT_READ", "connected": None, "max_temp": None, "min_volt": None, "max_abs_load": None, "no_reply": None}


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
    try:
        capture_to_web_log(action_ready, bus, False)
    except Exception as e:
        web_log(f"READY recovery error: {type(e).__name__}: {e}")
        # Last fallback: direct ready pose. This is less graceful but prevents
        # staying in a half-gait pose if the ready routine fails.
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = level_ready_pose()
        send_phase(bus, level_ready_pose(), GAIT_SPEED, GAIT_FINAL_READY_DELAY)


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

            time.sleep(0.10)
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
        elif cmd == "health":
            capture_to_web_log(print_web_health_cached, bus, "WEB HEALTH CHECK")
        elif cmd in ["movestats", "stats", "mstats"]:
            capture_to_web_log(action_movement_stats, parts)
        elif cmd == "speed":
            capture_to_web_log(action_set_speed, parts)
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
        elif cmd in ["formula", "model", "researchmodel"]:
            capture_to_web_log(print_research_model, parts[1] if len(parts) >= 2 else "forward")
        elif cmd in ["legtrim", "trim"]:
            capture_to_web_log(action_leg_trim, parts)
        elif cmd == "torque_max":
            capture_to_web_log(action_torque_max, bus)
        elif cmd == "timing":
            capture_to_web_log(action_gait_timing, parts)
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
        "range": {"forward_swing": GAIT_HIP_SWING_DEG, "forward_push": GAIT_SUPPORT_PUSH_DEG, "backward_swing": BACKWARD_HIP_SWING_DEG, "backward_push": BACKWARD_SUPPORT_PUSH_DEG, "strafe_swing": STRAFE_HIP_SWING_DEG, "strafe_push": STRAFE_SUPPORT_PUSH_DEG, "turn_swing": TURN_HIP_SWING_DEG, "turn_push": TURN_SUPPORT_PUSH_DEG},
        "side_strafe": {"flow": SIDE_STRAFE_FLOW_MODE, "reach_femur": SIDE_STRAFE_FEMUR_REACH_DEG, "reach_tibia": SIDE_STRAFE_TIBIA_REACH_DEG, "pull_femur": SIDE_STRAFE_FEMUR_PULL_DEG, "pull_tibia": SIDE_STRAFE_TIBIA_PULL_DEG, "lift_femur": SIDE_STRAFE_LIFT_FEMUR_DEG, "lift_tibia": SIDE_STRAFE_LIFT_TIBIA_DEG, "debug_steps": SIDE_STRAFE_DEBUG_STEPS_ENABLED, "phase_boost": SIDE_STRAFE_PHASE_BOOST_ENABLED},
        "movestats": {"enabled": MOVEMENT_STATS_ENABLED, "detail": MOVEMENT_STATS_DETAIL},
        "research_model": {"version": RESEARCH_MODEL_VERSION, "formula": "raw_target = base_raw + logical_deg * RAW_PER_DEG * leg_movement_sign * joint_direction"},
        "preset_flags": {"sidestrafe_good": is_sidestrafe_good_preset(), "sideflow_on": SIDE_STRAFE_FLOW_MODE and abs(SIDE_STRAFE_FLOW_HOLD) < 1e-9, "sideflow_off": not SIDE_STRAFE_FLOW_MODE, "movestats_off": not MOVEMENT_STATS_ENABLED, "smooth_fullstep": (not SMOOTH_GAIT and abs(GAIT_PHASE_DELAY - 0.30) < 1e-9 and abs(GAIT_SETTLE_DELAY - 0.14) < 1e-9), "smooth_smoothfull": (SMOOTH_GAIT and SMOOTH_STEPS == 5 and abs(GAIT_PHASE_DELAY - 0.26) < 1e-9 and abs(GAIT_SETTLE_DELAY - 0.14) < 1e-9), "speed_all_25": READY_SPEED == 25 and MOVE_SPEED == 25 and LIFT_SPEED == 25 and GAIT_SPEED == 25},
        "health": health, "logs": WEB_LOG_LINES[-120:],
    }


WEB_HTML = r'''
<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>SControlX2 Web</title>
<style>:root{--bg:#0d1117;--panel:#161b22;--panel2:#0f1720;--text:#e6edf3;--muted:#8b949e;--line:#30363d;--accent:#58a6ff;--danger:#ff6b6b;--ok:#3fb950;--warn:#d29922}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Inter,Segoe UI,Arial,sans-serif}.wrap{padding:16px;max-width:1450px;margin:0 auto}h1{font-size:22px;margin:0 0 6px}.sub{color:var(--muted);margin-bottom:14px}.grid{display:grid;grid-template-columns:1.1fr 1fr 1.35fr;gap:14px}.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:14px}.controller{display:grid;grid-template-columns:1fr 1fr;gap:16px}.dpad,.face{display:grid;grid-template-columns:72px 72px 72px;grid-template-rows:72px 72px 72px;gap:8px;justify-content:center}.btn{border:1px solid var(--line);background:#21262d;color:var(--text);border-radius:14px;font-size:17px;font-weight:700;cursor:pointer;user-select:none}.btn:hover{border-color:var(--accent)}.btn.active{background:#1f6feb}.btn.on{background:#17381f;border-color:var(--ok);box-shadow:0 0 0 2px rgba(63,185,80,.18) inset}.btn.flash{transform:scale(.98);border-color:var(--accent);box-shadow:0 0 0 2px rgba(88,166,255,.20) inset}.btn.small{font-size:13px;padding:10px}.btn.danger{background:#3b1717;border-color:#6b2b2b}.btn.ok{background:#17381f;border-color:#2f6f3a}.btn.warn{background:#3b2f13;border-color:#6f5a20}.wide{width:100%;margin-top:8px}.row{display:flex;gap:8px;align-items:center;margin:9px 0}.row label{min-width:130px;color:var(--muted);font-size:13px}.row input[type=range]{flex:1}.row input,.row select{background:#0d1117;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:7px}.value{min-width:55px;text-align:right;color:var(--accent);font-family:Consolas,monospace}.pill{display:inline-block;padding:5px 8px;border-radius:999px;border:1px solid var(--line);background:var(--panel2);font-size:12px;margin:2px}.log{height:500px;overflow:auto;background:#05080d;border:1px solid var(--line);border-radius:12px;padding:10px;font-family:Consolas,monospace;font-size:12px;white-space:pre-wrap;color:#d1d5db}.terminal{display:flex;gap:8px;margin-top:10px}.terminal input{flex:1;background:#05080d;color:var(--text);border:1px solid var(--line);border-radius:10px;padding:11px;font-family:Consolas,monospace}.statusgrid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px}.metric{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:10px}.metric .k{color:var(--muted);font-size:12px}.metric .v{font-size:22px;font-weight:700;margin-top:5px}.section{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}summary{cursor:pointer;color:var(--accent);font-weight:700}.presetgrid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.kbd{font-family:Consolas,monospace;color:var(--muted);font-size:11px}@media(max-width:1100px){.grid{grid-template-columns:1fr}.controller{grid-template-columns:1fr}.log{height:360px}}</style></head>
<body><div class="wrap"><h1>SControlX2 Hexapod Web Controller</h1><div class="sub">Controller-style WASDQE movement + debug/tuning dashboard. Health is cached; press Health to refresh motor reads safely.</div><div class="grid">
<div class="card"><h2>Controller Layout</h2><div class="sub">Hold button/key = move. Release = return to READY after current phase. New directions are ignored until idle.</div><div class="controller"><div><div class="sub">Left side movement pad</div><div class="dpad"><div></div><button class="btn move" data-dir="forward">W<br><span class="kbd">Forward</span></button><div></div><button class="btn move" data-dir="turn_left">Q<br><span class="kbd">Turn L</span></button><button class="btn move" data-dir="left">A<br><span class="kbd">Strafe L</span></button><button class="btn move" data-dir="right">D<br><span class="kbd">Strafe R</span></button><div></div><button class="btn move" data-dir="backward">S<br><span class="kbd">Back</span></button><button class="btn move" data-dir="turn_right">E<br><span class="kbd">Turn R</span></button></div></div><div><div class="sub">Right side face buttons</div><div class="face"><div></div><button class="btn ok" onclick="cmd('health')">△<br><span class="kbd">Health</span></button><div></div><button class="btn warn" onclick="cmd('r')">□<br><span class="kbd">Ready</span></button><button class="btn danger" onclick="stopMove()">○<br><span class="kbd">STOP</span></button><button class="btn" onclick="cmd('p')">◇<br><span class="kbd">Status</span></button><div></div><button class="btn" onclick="cmd('force_r')">×<br><span class="kbd">Force R</span></button><div></div></div></div></div><button class="btn ok wide" onclick="startup()">Startup Setup: r → health → sidestrafe good → movestats off → sideflow on → speed all 25</button><button class="btn danger wide" onclick="stopMove()">RELEASE / RETURN TO READY</button><div class="section"><h2>Quick Presets</h2><div class="presetgrid"><button id="btn_side_good" class="btn small" onclick="presetCmd(this,'sidestrafe good')">SideStrafe Good</button><button id="btn_sideflow_on" class="btn small" onclick="presetCmd(this,'sideflow on')">SideFlow ON</button><button id="btn_sideflow_off" class="btn small" onclick="presetCmd(this,'sideflow off')">SideFlow OFF</button><button id="btn_smooth_fullstep" class="btn small" onclick="presetCmd(this,'smooth fullstep')">Smooth Fullstep</button><button id="btn_smooth_smoothfull" class="btn small" onclick="presetCmd(this,'smooth smoothfull')">Smooth Smoothfull</button><button id="btn_walklift_clear" class="btn small" onclick="presetCmd(this,'walklift clear')">WalkLift Clear</button><button id="btn_speed25" class="btn small" onclick="presetCmd(this,'speed all 25')">Speed All 25</button><button id="btn_movestats_off" class="btn small" onclick="presetCmd(this,'movestats off')">MoveStats OFF</button><button class="btn small" onclick="presetCmd(this,'health')">Health Refresh</button></div></div></div>
<div class="card"><h2>Cached Health / State</h2><div class="statusgrid"><div class="metric"><div class="k">Status</div><div class="v" id="h_status">--</div></div><div class="metric"><div class="k">Connected</div><div class="v" id="h_conn">--</div></div><div class="metric"><div class="k">Max Temp</div><div class="v" id="h_temp">--</div></div><div class="metric"><div class="k">Min Volt</div><div class="v" id="h_volt">--</div></div><div class="metric"><div class="k">Max Load</div><div class="v" id="h_load">--</div></div><div class="metric"><div class="k">Motion</div><div class="v" id="motion">--</div></div></div><div class="section"><h2>Main Tuning</h2><div class="row"><label>All Speed</label><input id="speed" type="range" min="1" max="80" value="25" oninput="sv('speedv',this.value)" onchange="cmd('speed all '+this.value)"><span class="value" id="speedv">25</span></div><div class="row"><label>Body Height</label><input id="bodylevel" type="range" min="-7" max="7" value="0" oninput="sv('bodylevelv',this.value)" onchange="bodyLevelSet(this.value)"><span class="value" id="bodylevelv">0</span></div><div class="presetgrid"><button class="btn small" onclick="bodyLevelDelta(-1)">L2 Smooth Lower -1</button><button class="btn small" onclick="bodyLevelDelta(1)">R2 Smooth Raise +1</button><button class="btn small" onclick="bodyLevelSet(-7)">Lowest -7</button><button class="btn small" onclick="bodyLevelSet(0)">Reset 0</button><button class="btn small" onclick="bodyLevelSet(7)">Highest +7</button><button class="btn small" onclick="cmd('r')">Ready at Level</button></div><div class="row"><label>Walk Lift Preset</label><select onchange="cmd('walklift '+this.value)"><option value="clear">clear</option><option value="high">high</option><option value="high2">high2</option><option value="max">max</option><option value="old6">old6</option><option value="low">low</option></select></div><div class="row"><label>Walk Lift Level</label><input id="liftlevel" type="range" min="1" max="9" value="6" oninput="sv('liftlevelv',this.value)" onchange="cmd('walklift level '+this.value)"><span class="value" id="liftlevelv">6</span></div><div class="row"><label>Phase Hold</label><input id="phase" type="range" min="0.02" max="0.60" step="0.01" value="0.30" oninput="sv('phasev',this.value)" onchange="cmd('timing phase '+this.value)"><span class="value" id="phasev">0.30</span></div><div class="row"><label>Settle</label><input id="settle" type="range" min="0.02" max="0.40" step="0.01" value="0.14" oninput="sv('settlev',this.value)" onchange="cmd('timing settle '+this.value)"><span class="value" id="settlev">0.14</span></div></div><details><summary>Advanced debug / all script features</summary><div class="row"><label>Forward Range</label><input type="number" id="fwSwing" value="24"><input type="number" id="fwPush" value="16"><button class="btn small" onclick="cmd('range forward '+v('fwSwing')+' '+v('fwPush'))">Set</button></div><div class="row"><label>Strafe Range</label><input type="number" id="stSwing" value="28"><input type="number" id="stPush" value="22"><button class="btn small" onclick="cmd('range strafe '+v('stSwing')+' '+v('stPush'))">Set</button></div><div class="row"><label>Turn Range</label><input type="number" id="tnSwing" value="30"><input type="number" id="tnPush" value="24"><button class="btn small" onclick="cmd('range turn '+v('tnSwing')+' '+v('tnPush'))">Set</button></div><div class="row"><label>Lift Legs</label><select id="liftLv"><option>3</option><option>4</option><option>5</option><option selected>6</option><option>7</option><option>8</option><option>9</option></select><input id="liftLegs" placeholder="FL MR RL"><button class="btn small" onclick="cmd('lift '+v('liftLv')+' '+v('liftLegs'))">Lift</button></div><div class="presetgrid"><button class="btn small" onclick="cmd('lift 6 FL MR RL')">Lift Tripod A</button><button class="btn small" onclick="cmd('lift 6 FR ML RR')">Lift Tripod B</button><button class="btn small" onclick="cmd('torque_max')">Torque Max</button><button class="btn small" onclick="cmd('movestats on')">MoveStats ON</button><button class="btn small" onclick="cmd('movestats off')">MoveStats OFF</button><button class="btn small" onclick="cmd('smooth on')">Smooth ON</button><button class="btn small" onclick="cmd('smooth off')">Smooth OFF</button><button class="btn small" onclick="cmd('pushup 1')">Pushup 1</button><button class="btn small" onclick="cmd('pushup 4')">Pushup 4</button><button class="btn small" onclick="cmd('liftall 7')">Lift All L7</button><button class="btn small" onclick="cmd('latency fast')">Latency FAST</button><button class="btn small" onclick="cmd('latency normal')">Latency NORMAL</button></div></details><div class="section" id="pills"></div></div>
<div class="card"><h2>Terminal / Debug Output</h2><div class="log" id="log"></div><div class="terminal"><input id="term" placeholder="Type terminal command: r, health, speed all 25, sidestrafe good, walk forward 1..." onkeydown="if(event.key==='Enter') sendTerm()"><button class="btn small" onclick="sendTerm()">Send</button></div></div>
</div></div><script>
function sv(id,val){document.getElementById(id).textContent=val}function v(id){return document.getElementById(id).value}async function api(path,body){const opt=body?{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}:{};const r=await fetch(path,opt);return await r.json()}async function bodyLevelSet(level){await api('/api/action/bodylevel',{mode:'set',level:Number(level),delta:0});setTimeout(refresh,250)}async function bodyLevelDelta(delta){await api('/api/action/bodylevel',{mode:'delta',level:0,delta:Number(delta)});setTimeout(refresh,250)}async function bodyLevelReset(){await api('/api/action/bodylevel',{mode:'reset',level:0,delta:0});setTimeout(refresh,250)}async function cmd(c){await api('/api/command',{command:c});setTimeout(refresh,250)}async function presetCmd(btn,c){if(btn){btn.classList.add('flash');setTimeout(()=>btn.classList.remove('flash'),180)}await cmd(c)}async function startMove(dir){document.querySelectorAll('.move').forEach(b=>b.classList.remove('active'));const b=document.querySelector(`[data-dir="${dir}"]`);if(b)b.classList.add('active');await api('/api/move/start',{direction:dir})}async function stopMove(){document.querySelectorAll('.move').forEach(b=>b.classList.remove('active'));await api('/api/move/stop',{});setTimeout(refresh,300)}async function startup(){for(const c of ['r','health','sidestrafe good','movestats off','sideflow on','speed all 25']){await cmd(c);await new Promise(r=>setTimeout(r,250))}}function sendTerm(){const el=document.getElementById('term');const c=el.value.trim();if(!c)return;el.value='';cmd(c)}function setOn(id,on){const el=document.getElementById(id);if(el)el.classList.toggle('on',!!on)}for(const b of document.querySelectorAll('.move')){const dir=b.dataset.dir;b.addEventListener('mousedown',()=>startMove(dir));b.addEventListener('touchstart',(e)=>{e.preventDefault();startMove(dir)});b.addEventListener('mouseup',stopMove);b.addEventListener('touchend',(e)=>{e.preventDefault();stopMove()})}document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const map={w:'forward',s:'backward',a:'left',d:'right',q:'turn_left',e:'turn_right'};if(map[e.key.toLowerCase()]&&!e.repeat)startMove(map[e.key.toLowerCase()]);if(e.key===' ')stopMove()});document.addEventListener('keyup',e=>{if('wasdqe'.includes(e.key.toLowerCase()))stopMove()});async function refresh(){const s=await api('/api/state');const h=s.health||{};document.getElementById('h_status').textContent=h.status||'--';document.getElementById('h_conn').textContent=(h.connected??'--')+'/18';document.getElementById('h_temp').textContent=(h.max_temp??'--')+' C';document.getElementById('h_volt').textContent=(h.min_volt??'--')+' V';document.getElementById('h_load').textContent=h.max_abs_load??'--';document.getElementById('motion').textContent=s.motion||'--';document.getElementById('speed').value=s.speeds.gait;sv('speedv',s.speeds.gait);if(document.getElementById('bodylevel')){document.getElementById('bodylevel').value=s.body_height.level;sv('bodylevelv',s.body_height.level);}document.getElementById('liftlevel').value=s.walk_lift.level;sv('liftlevelv',s.walk_lift.level);document.getElementById('phase').value=s.timing.phase;sv('phasev',Number(s.timing.phase).toFixed(2));document.getElementById('settle').value=s.timing.settle;sv('settlev',Number(s.timing.settle).toFixed(2));document.getElementById('pills').innerHTML=`<span class="pill">Mode: ${s.current_mode}</span><span class="pill">Smooth: ${s.smooth.enabled}</span><span class="pill">Sideflow: ${s.side_strafe.flow}</span><span class="pill">MoveStats: ${s.movestats.enabled}</span><span class="pill">Lift: F ${s.walk_lift.femur} / T ${s.walk_lift.tibia}</span><span class="pill">Body Level: ${s.body_height.level} (F ${s.body_height.femur_offset} / T ${s.body_height.tibia_offset})</span><span class="pill">End: ${s.timing.end_mode}</span>`;const f=s.preset_flags||{};setOn('btn_side_good',f.sidestrafe_good);setOn('btn_sideflow_on',f.sideflow_on);setOn('btn_sideflow_off',f.sideflow_off);setOn('btn_smooth_fullstep',f.smooth_fullstep);setOn('btn_smooth_smoothfull',f.smooth_smoothfull);setOn('btn_speed25',f.speed_all_25);setOn('btn_movestats_off',f.movestats_off);const log=document.getElementById('log');log.textContent=(s.logs||[]).join('\n');log.scrollTop=log.scrollHeight}setInterval(refresh,1200);refresh();
</script></body></html>
'''


def create_web_app(bus: DynamixelBus):
    try:
        from fastapi import FastAPI
        from fastapi.responses import HTMLResponse
        from pydantic import BaseModel
    except Exception as e:
        print("Web libraries failed to import. This is usually a FastAPI/Pydantic version mismatch.")
        print(f"Reason: {type(e).__name__}: {e}")
        print("Fix using: python -m pip install --upgrade fastapi uvicorn pydantic pydantic-core")
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
    app = FastAPI(title="SControlX2 Hexapod Web Controller")
    @app.get("/", response_class=HTMLResponse)
    def index():
        return HTMLResponse(WEB_HTML)
    @app.get("/api/state")
    def api_state():
        return web_state(bus)

    @app.get("/api/research/model")
    def api_research_model():
        return research_model_snapshot()

    @app.get("/api/research/gait/{direction}")
    def api_research_gait(direction: str):
        return asdict(gait_formula_profile(direction))

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
    print()
    print("TUNING:")
    print("  speed all 25       = set all speeds to 23")
    print("  speed gait 18      = set gait speed")
    print("  range strafe 28 22 = tune strafe hip/push")
    print("  range turn 30 24   = tune turn hip/push")
    print("  formula forward    = print reusable gait formula/model for paper")
    print("  model turn_left    = same as formula; useful for research notes")
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
# LAUNCH MODE: WEB UI OR TERMINAL DEBUG
# ============================================================


def choose_launch_mode() -> str:
    """
    After the Dynamixel COM port is connected, choose how this same control
    program should run:
      web      = FastAPI/Uvicorn dashboard and controller HTTP API
      terminal = VS Code / command-line terminal debug mode

    This does not change gait values, READY_POSE, controller mappings, or motor
    math. It only changes the user interface layer used to send commands.
    """
    print()
    print("===================================================")
    print(" LAUNCH MODE")
    print("===================================================")
    print("Choose control interface:")
    print("  1) web       = Web UI + controller API using Uvicorn")
    print("  2) terminal  = VS Code terminal command control/debug")
    print()
    print("Terminal mode is useful when debugging because commands and errors")
    print("print directly in this console without needing the browser.")

    while True:
        choice = input("Launch mode [web/terminal, default web]: ").strip().lower()
        if choice == "":
            return "web"
        if choice in ["1", "w", "web", "webui", "ui", "uvicorn"]:
            return "web"
        if choice in ["2", "t", "term", "terminal", "console", "code"]:
            return "terminal"
        print("Please type web or terminal.")


def print_terminal_mode_help():
    print()
    print("===================================================")
    print(" TERMINAL DEBUG MODE COMMANDS")
    print("===================================================")
    print("BASIC:")
    print("  h / help            = show full original help")
    print("  th / terminalhelp   = show this terminal mode help")
    print("  r / ready           = ready pose at current body-height level")
    print("  force_r             = ready pose without safety precheck")
    print("  health              = read motor health now")
    print("  p / status          = print motor positions/goals")
    print("  x / exit / quit     = stop program and close COM port")
    print()
    print("MOVEMENT:")
    print("  w / forward         = continuous forward until Enter")
    print("  s / backward        = continuous backward until Enter")
    print("  a / left            = continuous strafe left until Enter")
    print("  d / right           = continuous strafe right until Enter")
    print("  q / turn_left       = continuous turn left until Enter")
    print("  e / turn_right      = continuous turn right until Enter")
    print("  walk forward 3      = run 3 cycles")
    print("  gait forward        = run 1 cycle")
    print("  turn left/right     = run continuous turn until Enter")
    print()
    print("TUNING:")
    print("  speed all 25")
    print("  sidestrafe good")
    print("  sideflow on/off")
    print("  smooth fullstep")
    print("  walklift clear")
    print("  bodylevel -4 / bodylevel up / bodylevel reset")
    print("  bodysmooth on / bodysmooth steps 10 / bodysmooth delay 0.045")
    print("  formula forward / model turn_left")
    print("===================================================")


def terminal_run_command(bus: DynamixelBus, raw_cmd: str) -> bool:
    """
    Return True to keep running, False to exit terminal mode.

    This is intentionally direct/console-based. It reuses the same action_*
    functions as the original terminal workflow so debugging output appears in
    VS Code terminal instead of being captured into the web log.
    """
    raw_cmd = (raw_cmd or "").strip()
    if not raw_cmd:
        return True

    parts = raw_cmd.split()
    cmd = parts[0].lower()

    try:
        if cmd in ["x", "exit", "quit"]:
            print("Exiting terminal mode...")
            return False

        if cmd in ["h", "help"]:
            print_help()
        elif cmd in ["th", "terminalhelp", "terminal_help"]:
            print_terminal_mode_help()
        elif cmd in ["p", "status"]:
            print_status(bus)
        elif cmd == "health":
            print_health(bus, "TERMINAL HEALTH CHECK")

        elif cmd in ["r", "ready"]:
            action_ready(bus, True)
        elif cmd == "force_r":
            print("FORCE_R: returning without safety check.")
            action_ready(bus, False)

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
        elif cmd in ["formula", "model", "researchmodel"]:
            print_research_model(parts[1] if len(parts) >= 2 else "forward")
        elif cmd in ["legtrim", "trim"]:
            action_leg_trim(parts)
        elif cmd == "torque_max":
            action_torque_max(bus)
        elif cmd == "timing":
            action_gait_timing(parts)
        elif cmd == "latency":
            action_latency_profile(parts)

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

        elif cmd in ["w", "s", "q", "e", "forward", "backward", "turn_left", "turn_right"]:
            action_gait_continuous(bus, normalize_direction(cmd))
        elif cmd in ["a", "d", "left", "right"]:
            action_side_strafe_continuous(bus, normalize_direction(cmd))
        elif cmd in ["stop", "space"]:
            print("In terminal mode, continuous movement stops by pressing Enter during the movement command.")
        else:
            print(f"Unknown command: {raw_cmd}. Type h or th for help.")

    except ValueError:
        print("Invalid number format or invalid command argument.")
    except Exception as e:
        print(f"COMMAND ERROR: {type(e).__name__}: {e}")

    return True


def run_terminal_debug_mode(bus: DynamixelBus, selected_port: str):
    """VS Code / command-line control loop."""
    apply_web_startup_defaults()
    print()
    print("===================================================")
    print(" SCONTROLX2 TERMINAL DEBUG MODE")
    print("===================================================")
    print(f"Connected port: {selected_port}")
    print("Startup defaults applied: sidestrafe good, movestats off, sideflow on, speed all 25.")
    print("Startup: NO automatic movement. Type r when robot is safe.")
    print("Type th for terminal command help, h for full original help, x to exit.")
    print("===================================================")

    while True:
        try:
            raw_cmd = input("SControlX2 terminal command [h help]: ")
        except (EOFError, KeyboardInterrupt):
            print("\nKeyboard/EOF exit requested.")
            break
        keep_running = terminal_run_command(bus, raw_cmd)
        if not keep_running:
            break


def run_web_ui_mode(bus: DynamixelBus, selected_port: str):
    """Original FastAPI/Uvicorn web dashboard mode."""
    if not check_web_dependencies():
        print("Web dependencies failed. COM port will stay connected until program exits.")
        return

    apply_web_startup_defaults()
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
# WEB DEPENDENCY PREFLIGHT
# ============================================================

def check_web_dependencies() -> bool:
    """
    Check FastAPI/Uvicorn before opening the Dynamixel serial port.

    This prevents the robot from connecting to the bus and then crashing
    later because of a broken FastAPI / Pydantic / pydantic-core install.
    It does not change gait logic, READY_POSE, controller commands, or motor values.
    """
    try:
        import fastapi  # noqa: F401
        import uvicorn  # noqa: F401
        import pydantic  # noqa: F401
        try:
            import pydantic_core  # noqa: F401
        except Exception:
            pydantic_core = None
        return True
    except Exception as e:
        print()
        print("===================================================")
        print(" WEB DEPENDENCY ERROR")
        print("===================================================")
        print("The Dynamixel/gait code is not the problem.")
        print("FastAPI could not start because your Python web packages are mismatched.")
        print()
        print(f"Import error: {type(e).__name__}: {e}")
        print()
        print("Fix on the SAME Python you use to run this script:")
        print("  python -m pip install --upgrade fastapi uvicorn pydantic pydantic-core")
        print()
        print("If you run with the Windows Store Python launcher, use:")
        print("  py -3.12 -m pip install --upgrade fastapi uvicorn pydantic pydantic-core")
        print()
        print("Then run this file again. The serial port will not be opened until this check passes.")
        print("===================================================")
        return False

# ============================================================
# MAIN
# ============================================================


def main():
    global WEB_BUS

    selected_port = choose_serial_port()
    bus = DynamixelBus(selected_port)
    WEB_BUS = bus

    if not bus.open():
        return

    try:
        mode = choose_launch_mode()
        if mode == "terminal":
            run_terminal_debug_mode(bus, selected_port)
        else:
            run_web_ui_mode(bus, selected_port)
    finally:
        web_stop_motion()
        try:
            if WEB_MOTION_THREAD and WEB_MOTION_THREAD.is_alive():
                WEB_MOTION_THREAD.join(timeout=3.0)
        except Exception:
            pass
        bus.close()

if __name__ == "__main__":
    main()
