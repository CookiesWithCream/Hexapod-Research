# ============================================================
# HEXAPOD KINEMATICS / ROBOT MODEL CONFIG
# ============================================================
#
# This file stores:
#   - motor ID mapping
#   - calibrated saved ready pose
#   - joint directions
#   - rough discovered raw limits
#   - conversion constant
#
# IMPORTANT:
#   READY_POSE below is the updated post-calibration pose.
#   It replaces the old pre-calibration ready pose.
#
# ============================================================


# ============================================================
# DYNAMIXEL RAW / DEGREE CONVERSION
# ============================================================
#
# AX-series approximate:
#   0 to 1023 raw
#   about 300 degrees total range
#
# 1023 / 300 = 3.41 raw per degree
#
# ============================================================

RAW_PER_DEG = 1023.0 / 300.0


# ============================================================
# UPDATED SAVED READY POSE
# ============================================================
#
# Latest good captured pose after calibration:
#
# ID  1 RL_hip    : 599
# ID  2 FL_hip    : 773
# ID  3 FR_femur  : 752
# ID  4 FL_femur  : 606
# ID  5 FR_tibia  : 494
# ID  6 FL_tibia  : 661
# ID  7 MR_hip    : 772
# ID  8 ML_hip    : 781
# ID  9 MR_femur  : 342
# ID 10 ML_femur  : 661
# ID 11 MR_tibia  : 521
# ID 12 ML_tibia  : 467
# ID 13 RR_hip    : 473
# ID 14 FR_hip    : 512
# ID 15 RR_femur  : 277
# ID 16 RL_femur  : 820
# ID 17 RR_tibia  : 494
# ID 18 RL_tibia  : 421
#
# ============================================================

READY_POSE = {
    1: 599,    # RL_hip
    2: 773,    # FL_hip
    3: 752,    # FR_femur
    4: 606,    # FL_femur
    5: 494,    # FR_tibia
    6: 661,    # FL_tibia
    7: 772,    # MR_hip
    8: 781,    # ML_hip
    9: 342,    # MR_femur
    10: 661,   # ML_femur
    11: 521,   # MR_tibia
    12: 467,   # ML_tibia
    13: 473,   # RR_hip
    14: 512,   # FR_hip
    15: 277,   # RR_femur
    16: 820,   # RL_femur
    17: 494,   # RR_tibia
    18: 421,   # RL_tibia
}


# ============================================================
# LEG JOINT MAP
# ============================================================
#
# Robot body orientation:
#   FL = front left
#   ML = middle left
#   RL = rear left
#   FR = front right
#   MR = middle right
#   RR = rear right
#
# Each leg:
#   hip   = side/forward swing joint
#   femur = upper leg lift/lower joint
#   tibia = lower leg extension joint
#
# ============================================================

LEG_JOINTS = {
    "FL": {
        "hip": "FL_hip",
        "femur": "FL_femur",
        "tibia": "FL_tibia",
    },
    "ML": {
        "hip": "ML_hip",
        "femur": "ML_femur",
        "tibia": "ML_tibia",
    },
    "RL": {
        "hip": "RL_hip",
        "femur": "RL_femur",
        "tibia": "RL_tibia",
    },
    "FR": {
        "hip": "FR_hip",
        "femur": "FR_femur",
        "tibia": "FR_tibia",
    },
    "MR": {
        "hip": "MR_hip",
        "femur": "MR_femur",
        "tibia": "MR_tibia",
    },
    "RR": {
        "hip": "RR_hip",
        "femur": "RR_femur",
        "tibia": "RR_tibia",
    },
}


# ============================================================
# JOINT LIMITS / MOTOR ID INFO
# ============================================================
#
# These limits are rough raw-discovery ranges.
#
# IMPORTANT:
#   For active movement control, use software-safe limits inside
#   hexapod_limited_control_console.py.
#
#   These raw limits are mainly metadata and older discovery data.
#
# Since you recalibrated, avoid using old min_raw/max_raw for
# probing. Probe scripts should clamp only to absolute 0–1023.
#
# ============================================================

JOINT_LIMITS = {
    # --------------------
    # HIPS
    # --------------------
    "FL_hip": {
        "id": 2,
        "type": "hip",
        "min_deg": -95.60,
        "max_deg": 61.29,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "ML_hip": {
        "id": 8,
        "type": "hip",
        "min_deg": -109.38,
        "max_deg": 52.79,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RL_hip": {
        "id": 1,
        "type": "hip",
        "min_deg": -97.65,
        "max_deg": 90.91,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "FR_hip": {
        "id": 14,
        "type": "hip",
        "min_deg": -96.48,
        "max_deg": 95.60,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "MR_hip": {
        "id": 7,
        "type": "hip",
        "min_deg": -76.25,
        "max_deg": 99.12,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RR_hip": {
        "id": 13,
        "type": "hip",
        "min_deg": -91.79,
        "max_deg": 100.59,
        "min_raw": 0,
        "max_raw": 1023,
    },

    # --------------------
    # FEMURS
    # --------------------
    "FL_femur": {
        "id": 4,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "ML_femur": {
        "id": 10,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RL_femur": {
        "id": 16,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "FR_femur": {
        "id": 3,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "MR_femur": {
        "id": 9,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RR_femur": {
        "id": 15,
        "type": "femur",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },

    # --------------------
    # TIBIAS
    # --------------------
    "FL_tibia": {
        "id": 6,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "ML_tibia": {
        "id": 12,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RL_tibia": {
        "id": 18,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "FR_tibia": {
        "id": 5,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "MR_tibia": {
        "id": 11,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
    "RR_tibia": {
        "id": 17,
        "type": "tibia",
        "min_deg": -55.0,
        "max_deg": 55.0,
        "min_raw": 0,
        "max_raw": 1023,
    },
}


# ============================================================
# JOINT DIRECTIONS
# ============================================================
#
# Direction describes how raw values convert to model-space angle.
#
# angle = ((raw - ready_raw) / RAW_PER_DEG) * JOINT_DIRECTIONS[joint]
#
# Keep all as 1 unless a joint's angle display is reversed.
#
# Physical movement reversal for MR/RR femur/tibia should be handled
# in the movement bridge using LEG_MOVEMENT_SIGN, not here.
#
# ============================================================

JOINT_DIRECTIONS = {
    # HIPS
    "FL_hip": 1,
    "ML_hip": 1,
    "RL_hip": 1,
    "FR_hip": 1,
    "MR_hip": 1,
    "RR_hip": 1,

    # FEMURS
    "FL_femur": 1,
    "ML_femur": 1,
    "RL_femur": 1,
    "FR_femur": 1,
    "MR_femur": 1,
    "RR_femur": 1,

    # TIBIAS
    "FL_tibia": 1,
    "ML_tibia": 1,
    "RL_tibia": 1,
    "FR_tibia": 1,
    "MR_tibia": 1,
    "RR_tibia": 1,
}


# ============================================================
# OPTIONAL HELPERS
# ============================================================

def get_joint_name_by_id(motor_id: int) -> str:
    for joint_name, info in JOINT_LIMITS.items():
        if int(info["id"]) == int(motor_id):
            return joint_name

    return "UNKNOWN"


def get_motor_id_by_joint(joint_name: str) -> int:
    return int(JOINT_LIMITS[joint_name]["id"])


def raw_to_session_deg(joint_name: str, raw: int, ready_pose: dict = None) -> float:
    if ready_pose is None:
        ready_pose = READY_POSE

    motor_id = get_motor_id_by_joint(joint_name)
    center = ready_pose[motor_id]
    direction = JOINT_DIRECTIONS.get(joint_name, 1)

    return ((raw - center) / RAW_PER_DEG) * direction


def session_deg_to_raw(joint_name: str, deg: float, ready_pose: dict = None) -> int:
    if ready_pose is None:
        ready_pose = READY_POSE

    motor_id = get_motor_id_by_joint(joint_name)
    center = ready_pose[motor_id]
    direction = JOINT_DIRECTIONS.get(joint_name, 1)

    raw = int(round(center + deg * RAW_PER_DEG * direction))
    raw = max(0, min(1023, raw))

    return raw


def print_ready_pose():
    print("READY_POSE = {")
    for motor_id in sorted(READY_POSE.keys()):
        joint_name = get_joint_name_by_id(motor_id)
        print(f"    {motor_id}: {READY_POSE[motor_id]},   # {joint_name}")
    print("}")