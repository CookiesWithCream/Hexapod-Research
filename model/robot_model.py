import math
from dataclasses import dataclass
from typing import Dict, Tuple, Optional


# ============================================================
# HEXAPOD KINEMATICS MODEL
# ============================================================
# This file DOES NOT talk to motors directly.
# It only converts:
#
#   foot XYZ target  ->  hip/femur/tibia model degrees
#   model degrees    ->  Dynamixel raw position
#   raw position     ->  model degrees
#
# READY_POSE is treated as 0 degrees in model space.
# ============================================================


# ============================================================
# READY POSE RAW VALUES
# ============================================================

READY_POSE: Dict[int, int] = {
    1: 529,
    2: 814,
    3: 746,
    4: 57,
    5: 971,
    6: 63,
    7: 260,
    8: 843,
    9: 323,
    10: 966,
    11: 648,
    12: 397,
    13: 507,
    14: 514,
    15: 72,
    16: 982,
    17: 29,
    18: 395,
}


# ============================================================
# JOINT LIMITS
# ============================================================
# Hip limits are from your current discovery.
# Femur and tibia limits are TEMPORARY conservative limits.
#
# Later, replace femur/tibia limits with your real discovered limits.
# ============================================================

JOINT_LIMITS: Dict[str, Dict[str, float]] = {
    # --------------------
    # HIPS - DISCOVERED
    # --------------------
    "FL_hip": {"id": 2, "min_deg": -95.60, "max_deg": 61.29, "min_raw": 488, "max_raw": 1023},
    "ML_hip": {"id": 8, "min_deg": -109.38, "max_deg": 52.79, "min_raw": 470, "max_raw": 1023},
    "RL_hip": {"id": 1, "min_deg": -97.65, "max_deg": 90.91, "min_raw": 196, "max_raw": 839},
    "FR_hip": {"id": 14, "min_deg": -96.48, "max_deg": 95.60, "min_raw": 185, "max_raw": 840},
    "MR_hip": {"id": 7, "min_deg": -76.25, "max_deg": 99.12, "min_raw": 0, "max_raw": 598},
    "RR_hip": {"id": 13, "min_deg": -91.79, "max_deg": 100.59, "min_raw": 194, "max_raw": 850},

    # --------------------
    # FEMUR - TEMP SAFE LIMITS
    # --------------------
    "FL_femur": {"id": 4, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "ML_femur": {"id": 10, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "RL_femur": {"id": 16, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},

    "FR_femur": {"id": 3, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "MR_femur": {"id": 9, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "RR_femur": {"id": 15, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},

    # --------------------
    # TIBIA - TEMP SAFE LIMITS
    # --------------------
    "FL_tibia": {"id": 6, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "ML_tibia": {"id": 12, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "RL_tibia": {"id": 18, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},

    "FR_tibia": {"id": 5, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "MR_tibia": {"id": 11, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
    "RR_tibia": {"id": 17, "min_deg": -35.0, "max_deg": 35.0, "min_raw": 0, "max_raw": 1023},
}


# ============================================================
# LEG MOTOR MAP
# ============================================================

LEG_JOINTS: Dict[str, Dict[str, str]] = {
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
# JOINT DIRECTION CONFIG
# ============================================================
# If a joint moves opposite during testing, flip 1 to -1.
#
# Example:
#   "FL_femur": -1
#
# Keep changes here, not inside the IK formula.
# ============================================================

JOINT_DIRECTIONS: Dict[str, int] = {
    "FL_hip": 1,
    "FL_femur": 1,
    "FL_tibia": 1,

    "ML_hip": 1,
    "ML_femur": 1,
    "ML_tibia": 1,

    "RL_hip": 1,
    "RL_femur": 1,
    "RL_tibia": 1,

    "FR_hip": 1,
    "FR_femur": 1,
    "FR_tibia": 1,

    "MR_hip": 1,
    "MR_femur": 1,
    "MR_tibia": 1,

    "RR_hip": 1,
    "RR_femur": 1,
    "RR_tibia": 1,
}


# ============================================================
# ROBOT GEOMETRY
# ============================================================
# IMPORTANT:
# These are placeholder lengths in millimeters.
# Measure your actual robot and update these.
#
# coxa  = small horizontal hip link
# femur = upper leg
# tibia = lower leg
# ============================================================

@dataclass
class LegGeometry:
    coxa: float = 45.0
    femur: float = 75.0
    tibia: float = 105.0


GEOMETRY = LegGeometry()


# ============================================================
# DEFAULT FOOT POSITIONS
# ============================================================
# This is the local XYZ target when robot is in ready pose.
#
# x = forward/back relative to leg
# y = outward from robot body
# z = up/down
#
# Negative z means foot is below body.
# ============================================================

DEFAULT_FOOT_TARGETS: Dict[str, Tuple[float, float, float]] = {
    "FL": (80.0, 75.0, -85.0),
    "ML": (0.0, 90.0, -85.0),
    "RL": (-80.0, 75.0, -85.0),

    "FR": (80.0, -75.0, -85.0),
    "MR": (0.0, -90.0, -85.0),
    "RR": (-80.0, -75.0, -85.0),
}


# ============================================================
# BASIC HELPERS
# ============================================================

DYNAMIXEL_MAX_RAW = 1023
DYNAMIXEL_DEG_RANGE = 300.0
RAW_PER_DEG = DYNAMIXEL_MAX_RAW / DYNAMIXEL_DEG_RANGE


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def raw_to_model_deg(joint_name: str, raw: int) -> float:
    """
    Convert Dynamixel raw position to model-space degrees.

    READY_POSE raw = 0 degrees.
    """
    motor_id = JOINT_LIMITS[joint_name]["id"]
    center_raw = READY_POSE[motor_id]
    direction = JOINT_DIRECTIONS[joint_name]

    return ((raw - center_raw) / RAW_PER_DEG) * direction


def model_deg_to_raw(joint_name: str, model_deg: float, clamp_to_limits: bool = True) -> int:
    """
    Convert model-space degrees to Dynamixel raw position.

    0 degrees = READY_POSE.
    """
    info = JOINT_LIMITS[joint_name]
    motor_id = info["id"]
    center_raw = READY_POSE[motor_id]
    direction = JOINT_DIRECTIONS[joint_name]

    if clamp_to_limits:
        model_deg = clamp(model_deg, info["min_deg"], info["max_deg"])

    raw = int(round(center_raw + (model_deg * RAW_PER_DEG * direction)))

    raw = int(clamp(raw, info["min_raw"], info["max_raw"]))
    raw = int(clamp(raw, 0, 1023))

    return raw


def check_joint_angle(joint_name: str, model_deg: float) -> bool:
    info = JOINT_LIMITS[joint_name]
    return info["min_deg"] <= model_deg <= info["max_deg"]


def print_leg_info() -> None:
    print("\n=== LEG JOINT MAP ===")
    for leg, joints in LEG_JOINTS.items():
        print(f"\n[{leg}]")
        for part, joint_name in joints.items():
            info = JOINT_LIMITS[joint_name]
            motor_id = info["id"]
            ready_raw = READY_POSE[motor_id]
            print(
                f"  {part:<6} {joint_name:<10} "
                f"ID={motor_id:<2} ready={ready_raw:<4} "
                f"deg=[{info['min_deg']:.2f}, {info['max_deg']:.2f}]"
            )


# ============================================================
# FORWARD KINEMATICS
# ============================================================

def forward_kinematics(
    hip_deg: float,
    femur_deg: float,
    tibia_deg: float,
    geometry: LegGeometry = GEOMETRY,
) -> Tuple[float, float, float]:
    """
    Convert 3 joint angles into foot XYZ position.

    This is a standard 3DOF leg model.

    hip_deg   = horizontal rotation
    femur_deg = upper leg pitch
    tibia_deg = lower leg pitch

    Returns:
        x, y, z in millimeters
    """

    hip = math.radians(hip_deg)
    femur = math.radians(femur_deg)
    tibia = math.radians(tibia_deg)

    # distance from hip joint to foot in the horizontal leg plane
    planar = geometry.coxa + (
        geometry.femur * math.cos(femur)
    ) + (
        geometry.tibia * math.cos(femur + tibia)
    )

    z = (
        geometry.femur * math.sin(femur)
    ) + (
        geometry.tibia * math.sin(femur + tibia)
    )

    x = planar * math.cos(hip)
    y = planar * math.sin(hip)

    return x, y, z


# ============================================================
# INVERSE KINEMATICS
# ============================================================

def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    geometry: LegGeometry = GEOMETRY,
    elbow_down: bool = True,
) -> Optional[Tuple[float, float, float]]:
    """
    Convert foot XYZ position into hip/femur/tibia angles.

    Returns:
        hip_deg, femur_deg, tibia_deg

    If the target is unreachable:
        returns None
    """

    coxa = geometry.coxa
    femur = geometry.femur
    tibia = geometry.tibia

    hip_rad = math.atan2(y, x)
    hip_deg = math.degrees(hip_rad)

    horizontal_distance = math.sqrt(x * x + y * y)
    leg_plane_x = horizontal_distance - coxa

    distance = math.sqrt(leg_plane_x * leg_plane_x + z * z)

    # unreachable target
    if distance > femur + tibia:
        return None

    # too close / physically impossible
    if distance < abs(femur - tibia):
        return None

    # angle from horizontal to target
    target_angle = math.atan2(z, leg_plane_x)

    # law of cosines for femur angle
    cos_femur = (
        (femur * femur) + (distance * distance) - (tibia * tibia)
    ) / (
        2 * femur * distance
    )
    cos_femur = clamp(cos_femur, -1.0, 1.0)

    femur_offset = math.acos(cos_femur)

    if elbow_down:
        femur_rad = target_angle + femur_offset
    else:
        femur_rad = target_angle - femur_offset

    # law of cosines for tibia angle
    cos_tibia = (
        (femur * femur) + (tibia * tibia) - (distance * distance)
    ) / (
        2 * femur * tibia
    )
    cos_tibia = clamp(cos_tibia, -1.0, 1.0)

    tibia_inside_rad = math.acos(cos_tibia)

    # convert inner knee angle to servo-style relative bend
    tibia_rad = tibia_inside_rad - math.pi

    hip_deg = math.degrees(hip_rad)
    femur_deg = math.degrees(femur_rad)
    tibia_deg = math.degrees(tibia_rad)

    return hip_deg, femur_deg, tibia_deg


# ============================================================
# LEG TARGET CONVERSION
# ============================================================

def leg_target_to_joint_degrees(
    leg_name: str,
    x: float,
    y: float,
    z: float,
    elbow_down: bool = True,
    clamp_to_limits: bool = True,
) -> Optional[Dict[str, float]]:
    """
    Convert one leg foot target XYZ into model-space joint degrees.

    Returns:
        {
            "FL_hip": value,
            "FL_femur": value,
            "FL_tibia": value,
        }
    """

    leg_name = leg_name.upper()

    if leg_name not in LEG_JOINTS:
        raise ValueError(f"Unknown leg name: {leg_name}")

    result = inverse_kinematics(x, y, z, GEOMETRY, elbow_down=elbow_down)

    if result is None:
        return None

    hip_deg, femur_deg, tibia_deg = result

    hip_joint = LEG_JOINTS[leg_name]["hip"]
    femur_joint = LEG_JOINTS[leg_name]["femur"]
    tibia_joint = LEG_JOINTS[leg_name]["tibia"]

    joint_degrees = {
        hip_joint: hip_deg,
        femur_joint: femur_deg,
        tibia_joint: tibia_deg,
    }

    if clamp_to_limits:
        for joint_name, deg in list(joint_degrees.items()):
            info = JOINT_LIMITS[joint_name]
            joint_degrees[joint_name] = clamp(deg, info["min_deg"], info["max_deg"])

    return joint_degrees


def leg_target_to_motor_raws(
    leg_name: str,
    x: float,
    y: float,
    z: float,
    elbow_down: bool = True,
) -> Optional[Dict[int, int]]:
    """
    Convert one leg foot target XYZ into motor ID -> raw position.

    Example return:
        {
            2: 820,
            4: 80,
            6: 100,
        }
    """

    degrees = leg_target_to_joint_degrees(
        leg_name=leg_name,
        x=x,
        y=y,
        z=z,
        elbow_down=elbow_down,
        clamp_to_limits=True,
    )

    if degrees is None:
        return None

    raw_targets: Dict[int, int] = {}

    for joint_name, model_deg in degrees.items():
        motor_id = JOINT_LIMITS[joint_name]["id"]
        raw_targets[motor_id] = model_deg_to_raw(joint_name, model_deg)

    return raw_targets


def ready_pose_raws_for_leg(leg_name: str) -> Dict[int, int]:
    leg_name = leg_name.upper()

    if leg_name not in LEG_JOINTS:
        raise ValueError(f"Unknown leg name: {leg_name}")

    result: Dict[int, int] = {}

    for joint_name in LEG_JOINTS[leg_name].values():
        motor_id = JOINT_LIMITS[joint_name]["id"]
        result[motor_id] = READY_POSE[motor_id]

    return result


def all_ready_pose_raws() -> Dict[int, int]:
    return dict(READY_POSE)


# ============================================================
# DEBUG TEST
# ============================================================

if __name__ == "__main__":
    print_leg_info()

    print("\n=== FK TEST ===")
    print("Angles: hip=0, femur=0, tibia=0")
    print("XYZ:", forward_kinematics(0, 0, 0))

    print("\n=== IK TEST ===")
    test_target = DEFAULT_FOOT_TARGETS["FL"]
    print("Target:", test_target)

    ik = inverse_kinematics(*test_target)
    print("IK:", ik)

    if ik:
        raw = leg_target_to_motor_raws("FL", *test_target)
        print("Raw targets:", raw)