# ============================================================
# WCONTROL71 CHANGE NOTE
# ============================================================
# Source base:
#   WControl70_working_strafe_with_movement_stats.py
#
# Current upgrade:
#   - Keeps the working WControl23/WControl41 no-hip A/D side-strafe.
#   - Adds A/D-only flow mode so left/right strafe runs like one continuous
#     gait instead of pause -> read -> pause between phases.
#   - Forward/backward/turn are unchanged.
#
# New default:
#   SIDE_STRAFE_FLOW_MODE = True
#
# New commands:
#   sideflow              = show flow settings
#   sideflow on           = no extra hold pauses between A/D phases
#   sideflow off          = original WControl23 phase holds
#   sideflow tiny         = tiny visible hold, default 0.015s
#   sideflow hold 0.02    = custom flow hold cap
#
# Recommended smooth A/D test:
#   r
#   health
#   sidestrafe good
#   movestats off
#   sideflow on
#   speed all 23
#   a
#   r
#   d
#
# Note:
#   movestats on will still add read/print pauses. For smooth whole-move
#   testing, keep movestats off and use health after the movement.
# ============================================================

# src/control/WControl_final_v6_clean.py
#
# HEXAPOD REFINED2K BALANCED CONTROL SCRIPT
# VERSION: WCONTROL70 - WORKING W23 A/D STRAFE + GLOBAL MOVEMENT STATS
#
# Main updates:
#   WCONTROL32 reboot: restored the WControl23 side-strafe that physically worked best.
#   Startup no longer auto-moves the robot; speeds default to 22.
#   A/D uses the working no-hip lift-out + planted-pull pattern.
#   1. Forward gait uses mirrored left/right hip signs.
#   2. Gait uses lift level 6 for higher foot clearance.
#   3. Runtime speed command added:
#        speed
#        speed gait 18
#        speed gait 12
#        speed all 10
#   4. Backward, strafe left/right, and turn left/right are available.
#      Strafe now locks all hip/coxa joints at ready and uses femur/tibia support-tripod push while the opposite tripod is lifted.
#   5. Smooth gait mode reduces stop-start pauses by interpolating between poses
#      and avoiding full motor health reads between every tiny gait phase.
#   6. Optional torque_max command sets AX torque limit cap to 1023.
#      This does NOT force full torque constantly; it only removes a low cap.
#   6. Lift command supports:
#        lift FL
#        lift 3 FL
#        lift 5 FL
#        lift 6 FL
#        lift 3 FL FR
#        lift 6 FL FR ML MR RL RR
#
# Safety:
#   AX Present Load max magnitude is 1023.
#   This script warns at 450 and blocks at 700.
#   Stop testing if temp approaches 50C or voltage stays below 10.8V.

import sys
import time
from typing import Dict, Optional, Tuple, List

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
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_LOAD = 40
ADDR_PRESENT_VOLTAGE = 42
ADDR_PRESENT_TEMPERATURE = 43

TORQUE_ENABLE = 1
COMM_SUCCESS = 0

RAW_PER_DEG = 1023.0 / 300.0

READ_RETRIES = 3
READ_RETRY_DELAY = 0.04

# AX Moving Speed notes:
#   0   = maximum speed / no speed control. Avoid for tuning.
#   1   = very slow.
#   1023 = fastest controlled speed.
# Your working forward gait used GAIT_SPEED = 18, so that remains the default.
READY_SPEED = 22
MOVE_SPEED = 22
LIFT_SPEED = 22
GAIT_SPEED = 22

MIN_SAFE_SPEED = 1
MAX_SAFE_SPEED = 1023

# Torque limit cap. 1023 is max cap on AX-series.
# This is NOT constant full torque; it only lets the motor use up to this if required.
TORQUE_LIMIT_RAW = 1023

TEMP_WARN_C = 50
TEMP_STOP_C = 58

# Present Load magnitude is 0-1023.
# 450 ~= 44% inferred load.
# 700 ~= 68% inferred load.
LOAD_WARN = 450
LOAD_STOP = 700

VOLT_WARN_V = 10.8
VOLT_STOP_V = 9.5
VOLT_DANGER_V = 9.2


# ============================================================
# BALANCED REFINED2K READY POSE
# ============================================================
# This READY_POSE is the refined2k stance from the leg contribution tuner.
#
# It replaces the old refined2k balanced base stance with the best balanced result:
#   FL = BALANCED
#   ML = BALANCED
#   RL = BALANCED
#   FR = BALANCED
#   MR = BALANCED
#   RR = BALANCED
#
# Final refined2k health from your tuner test:
#   Connected: 18/18
#   Max temp: 37C
#   Min voltage: 11.4V
#   Max abs load: 224
#   Status: OK
#
# Refined2k offsets from old LOW14:
#   ML: femur +5.0, tibia -5.0
#   RR: femur +3.5, tibia -3.5
#   FL: femur -2.5, tibia +2.5
#   FR: femur -0.3, tibia +0.3
#   RL: femur +0.5, tibia -0.5
#   MR: femur +0.5, tibia -0.5
#
# From now on:
#   r / ready returns to this balanced stance.
#   walking, backward, strafe, and turn are calculated from this balanced stance.

READY_POSE = {
    1: 460,   # RL_hip
    2: 747,   # FL_hip

    3: 411,   # FR_femur  old 412, refined2k FR femur -0.3 deg
    4: 366,   # FL_femur  old 375, refined2k FL femur -2.5 deg

    5: 798,   # FR_tibia  old 797, refined2k FR tibia +0.3 deg
    6: 796,   # FL_tibia  old 787, refined2k FL tibia +2.5 deg

    7: 608,   # MR_hip
    8: 753,   # ML_hip

    9: 627,   # MR_femur  old 629, refined2k MR femur +0.5 deg
    10: 437,  # ML_femur  old 420, refined2k ML femur +5.0 deg

    11: 216,  # MR_tibia  old 214, refined2k MR tibia -0.5 deg
    12: 787,  # ML_tibia  old 804, refined2k ML tibia -5.0 deg

    13: 578,  # RR_hip
    14: 575,  # FR_hip

    15: 641,  # RR_femur  old 653, refined2k RR femur +3.5 deg
    16: 412,  # RL_femur  old 410, refined2k RL femur +0.5 deg

    17: 189,  # RR_tibia  old 177, refined2k RR tibia -3.5 deg
    18: 817,  # RL_tibia  old 819, refined2k RL tibia -0.5 deg
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

TRIPOD_A = ["FL", "MR", "RL"]
TRIPOD_B = ["FR", "ML", "RR"]


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


# ============================================================
# MOTION SETTINGS
# ============================================================

# Lift levels:
#   femur negative = upward lift from refined2k
#   tibia positive = stronger lift/tuck from refined2k
LIFT_LEVELS = {
    1: {"femur": -6.0,  "tibia": 6.0},
    2: {"femur": -10.0, "tibia": 10.0},
    3: {"femur": -14.0, "tibia": 14.0},
    4: {"femur": -18.0, "tibia": 18.0},
    5: {"femur": -22.0, "tibia": 22.0},

    # Higher-clearance gait lift. Use this for walking when feet drag or push back.
    # Speeds and hip swing are unchanged; only swing-leg lift clearance is increased.
    6: {"femur": -28.0, "tibia": 28.0},

    # Optional max test only. Do not use first unless level 6 still drags.
    7: {"femur": -32.0, "tibia": 32.0},

    # Extra split-lift test levels. These are only for gait lift testing.
    8: {"femur": -36.0, "tibia": 34.0},
    9: {"femur": -40.0, "tibia": 36.0},
}

DEFAULT_LIFT_LEVEL = 3

# Higher-clearance real tripod gait.
# IMPORTANT: speed, hip swing, and support push are kept unchanged.
# Only the swing-leg lift level is increased to reduce ground friction/push-back.
GAIT_HIP_SWING_DEG = 24.0
GAIT_SUPPORT_PUSH_DEG = 16.0

# Backward uses the same size as forward, just reversed.
BACKWARD_HIP_SWING_DEG = 24.0
BACKWARD_SUPPORT_PUSH_DEG = 16.0

# WControl10: strafe/turn range increased because earlier a/d/q/e barely moved.
# Forward/backward stays unchanged.
STRAFE_HIP_SWING_DEG = 28.0
STRAFE_SUPPORT_PUSH_DEG = 22.0
TURN_HIP_SWING_DEG = 30.0
TURN_SUPPORT_PUSH_DEG = 24.0

GAIT_LIFT_LEVEL = 6

# Walking lift profile override.
# Your observation: full tibia tuck wastes movement because the tibia folds all the
# way in before the leg comes down. For walking, femur should provide most of the
# ground clearance while tibia only tucks enough to avoid scraping.
#
# Manual lift commands still use LIFT_LEVELS. Walking gait uses this profile when
# USE_WALK_LIFT_PROFILE = True.
USE_WALK_LIFT_PROFILE = False

# V6-style default: walking uses LIFT_LEVELS[GAIT_LIFT_LEVEL] instead of the separate profile.
# Your refined2k stance is lower/better balanced than LOW14, so the old
# "femur -32, tibia +12" walking profile can look too low during the actual
# gait phases. For this version we use more tibia tuck again, but not as much
# as the old full level-6 unless you select old6.
WALK_LIFT_FEMUR_DEG = -32.0   # used only if profile is turned on
WALK_LIFT_TIBIA_DEG = 12.0    # used only if profile is turned on

# Extra-high presets for finding the correct gait lift.
#
# Since the refined2k stance is lower, the previous gait lift may not visibly
# clear enough. This version adds higher femur lift and stronger tibia tuck.
#
# Recommended test order:
#   walklift high1 -> w -> r
#   walklift high2 -> w -> r
#   walklift high3 -> w -> r
#   walklift max   -> w -> r
#
# Choose the lowest preset that clearly clears the ground. Do not spam max.
WALK_LIFT_PRESETS = {
    "test1": {"femur": -32.0, "tibia": 18.0},
    "test2": {"femur": -34.0, "tibia": 22.0},
    "test3": {"femur": -36.0, "tibia": 24.0},

    # New extra-high refined2k lift presets.
    "high1": {"femur": -38.0, "tibia": 26.0},
    "high2": {"femur": -40.0, "tibia": 28.0},
    "high3": {"femur": -42.0, "tibia": 30.0},
    "max":   {"femur": -44.0, "tibia": 32.0},

    # Old full level-6 style for comparison.
    "old6":  {"femur": -28.0, "tibia": 28.0},

    # Aliases.
    "low":   {"femur": -30.0, "tibia": 16.0},
    "clear": {"femur": -38.0, "tibia": 28.0},
    "high":  {"femur": -40.0, "tibia": 28.0},
}

# Per-leg lift fine tuning.
# Use this when one leg visually lifts/tucks more than the others.
# 1.00 = normal movement, 0.90 = 10% less, 0.85 = 15% less.
#
# Today's note: RR tibia looked slightly excessive, especially on high/manual lift.
# So RR tibia is trimmed a little by default. This affects manual lift and walking.
LEG_FEMUR_LIFT_SCALE = {
    "FL": 1.00,
    "ML": 1.00,
    "RL": 1.00,
    "FR": 1.00,
    "MR": 1.00,
    "RR": 1.00,
}

LEG_TIBIA_LIFT_SCALE = {
    "FL": 1.00,
    "ML": 1.00,
    "RL": 1.00,
    "FR": 1.00,
    "MR": 1.00,

    # RR tibia trim: reduce just a little.
    # Change to 0.90 if too low, or 0.80 if still too excessive.
    "RR": 0.85,
}

# Original was 0.80 / 0.45. That worked, but looked stop-start.
# Gait-lift finder uses slightly longer holds so the lift becomes visible/reaches target.
GAIT_PHASE_DELAY = 0.30
GAIT_SETTLE_DELAY = 0.14
GAIT_FINAL_READY_DELAY = 0.35

# Final cycle handling.
# direct = old behavior: after final cycle, slide all hips back to READY at once.
# tripod = improved behavior: lift one tripod, recenter its hips, place it down,
#          then repeat for the other tripod. This avoids the visible all-feet-down
#          hip reset/drag at the end of a 1-cycle walk.
# hold   = do not return to READY automatically; stay in last gait stance.
GAIT_END_MODE = "tripod"
GAIT_END_RECENTER_DELAY = 0.10

# Smooth gait settings.
# True = less pause between lift/down phases.
# It does not increase torque. It just sends smaller intermediate goal positions.
SMOOTH_GAIT = False
SMOOTH_STEPS = 3
SMOOTH_STEP_DELAY = 0.025

# Printing/reading full health after every phase creates a visible pause because
# it reads 18 motors several times. Keep this off for smoother walking.
GAIT_PHASE_HEALTH = False

# Safety pre-check every cycle instead of every phase to reduce pause.
# Safety still runs before each cycle and after final status.
GAIT_PRECHECK_EACH_PHASE = False

# Forward hip sign per leg.
# Left and right legs must mirror.
# If forward becomes backward, flip all signs.
HIP_FORWARD_SIGN = {
    "FL": -1,
    "ML": -1,
    "RL": -1,

    "FR": 1,
    "MR": 1,
    "RR": 1,
}

# Strafe direction sign. Experimental.
HIP_STRAFE_SIGN = {
    "FL": -1,
    "ML": 1,
    "RL": -1,

    "FR": -1,
    "MR": 1,
    "RR": -1,
}

# Turn direction sign. Same side sign intentionally creates yaw.
HIP_TURN_SIGN = {
    "FL": -1,
    "ML": -1,
    "RL": -1,

    "FR": -1,
    "MR": -1,
    "RR": -1,
}

# ============================================================
# WCONTROL11 SIDE / TURN LOGIC
# ============================================================
#
# Forward/backward already works, so WControl11 does NOT change it.
#
# Turn logic:
#   turn_left:
#       left legs  = backward-walk hip direction
#       right legs = forward-walk hip direction
#   turn_right:
#       left legs  = forward-walk hip direction
#       right legs = backward-walk hip direction
#
# This is stronger and more physically meaningful than the old same-sign turn map.
#
# Strafe logic:
#   This is still approximate without IK, but stronger than WControl10.
#   Front/rear legs sweep diagonally while middle legs push sideways.
#   If left/right reverses, flip STRAFE_DIRECTION_MULTIPLIER.
LEFT_LEGS = ["FL", "ML", "RL"]
RIGHT_LEGS = ["FR", "MR", "RR"]

STRAFE_DIRECTION_MULTIPLIER = 1.0
TURN_DIRECTION_MULTIPLIER = 1.0

# WControl12 precision turn scaling.
# WControl11 turn was strong/juicy, but q was too much.
# q/turn_left is reduced by 25%:
#   30 hip -> 22.5 effective
#   24 support -> 18.0 effective
#
# e/turn_right is reduced slightly less, around 22%:
#   30 hip -> 23.4 effective
#   24 support -> 18.72 effective
#
# This gives finer single-click control while still letting user cycle q/e if needed.
TURN_LEFT_SCALE = 0.75
TURN_RIGHT_SCALE = 0.78

# ============================================================
# WCONTROL14 CLASSIC TRIPOD CRAB STRAFE
# ============================================================
#
# Online/reference style summary:
#   True crab/strafe gait is normally done by moving FOOT TARGETS sideways
#   using IK. Since this robot is currently joint-space only, WControl14 uses
#   a pseudo-foot-target sequence:
#
#     1. Lift tripod B: FR + ML + RR
#     2. Move that lifted tripod sideways
#     3. Put it down
#     4. Lift tripod A: FL + MR + RL
#     5. Move that lifted tripod sideways
#     6. Put it down
#
# For left strafe:
#   left-side legs move outward-left
#   right-side legs move inward-left
#
# For right strafe:
#   mirrored.
#
# w/s/q/e are intentionally unchanged from WControl12.
CRAB_FIRST_TRIPOD = ["FR", "ML", "RR"]
CRAB_SECOND_TRIPOD = ["FL", "MR", "RL"]

CRAB_STRAFE_DIRECTION_MULTIPLIER = 1.0

# Side placement while leg is in the air.
# WControl15 keeps the support tripod PLANTED at ready during strafe,
# because WControl14 made support legs push while the other tripod moved,
# causing the robot to look like it was fighting left/right.
# WControl17: femur/tibia side reach is kept small.
# The real side placement now comes mostly from coxa/hip yaw on the LIFTED tripod.
# Side placement while the leg is in the air.
# Bigger than WControl17 because hip yaw is no longer doing the sideways movement.
# Start conservative; tune with `crab power 8`, `crab power 10`, `crab power 12`.
CRAB_REACH_FEMUR_DEG = 10.0
CRAB_REACH_TIBIA_DEG = -8.0

# Support tripod stays planted during lifted-leg reach.
CRAB_SUPPORT_FEMUR_DEG = 0.0
CRAB_SUPPORT_TIBIA_DEG = 0.0
CRAB_SUPPORT_PUSH_ENABLED = False

# WControl18 body shift after the reached tripod lands.
# Main body translation now uses femur/tibia radial placement, NOT hip yaw.
# This is closer to the online crab/sideways gait idea where feet are placed sideways
# and the body translates over planted feet. Hip yaw is kept near zero by default.
CRAB_BODY_SHIFT_ENABLED = True
CRAB_BODY_SHIFT_FEMUR_DEG = 8.0
CRAB_BODY_SHIFT_TIBIA_DEG = -8.0
CRAB_BODY_SHIFT_HIP_DEG = 0.0
CRAB_BODY_SHIFT_HOLD = 0.26

# Minimum visible lift while the lifted tripod is reaching sideways.
# This prevents the side-reach femur offset from cancelling the vertical lift,
# which is likely why ML looked like it pushed down instead of lifting.
CRAB_MIN_REACH_LIFT_FEMUR_DEG = -26.0
CRAB_MIN_REACH_TUCK_TIBIA_DEG = 22.0

# WControl18: hip yaw is OFF by default for strafe.
# Your observation was correct: using coxa/hip yaw made sideways walking look like
# diagonal/forward walking. For crab strafe, femur/tibia should do most of the
# side placement. Use `crab hip on` or `crab hipamount 4` only as a small trim.
CRAB_USE_HIP_YAW = False
CRAB_HIP_DEG = 0.0

# Stronger crab-walk experimental hip map.
# These values are multipliers applied to STRAFE_HIP_SWING_DEG.
# Middle legs contribute more directly; front/rear help shift the body.
CRAB_STRAFE_LEFT_SWING = {
    "FL": -1.00,
    "ML": +1.25,
    "RL": -1.00,
    "FR": -1.00,
    "MR": +1.25,
    "RR": -1.00,
}

CRAB_STRAFE_LEFT_SUPPORT = {
    "FL": +0.85,
    "ML": -1.25,
    "RL": +0.85,
    "FR": +0.85,
    "MR": -1.25,
    "RR": +0.85,
}

# ============================================================
# WCONTROL23 NO-HIP LIFT-OUT + PLANTED-PULL STRAFE FOR A/D ONLY
# ============================================================
# This replaces WControl19 a/d only. Forward/backward/turn stay unchanged.
#
# Why WControl19 failed:
#   Femur/tibia offsets mostly looked like vertical lift + tibia tuck, so the
#   robot pushed one side down but did not translate sideways.
#
# WControl20 uses the motion you described:
#   1. Tripod B = ML + FR + RR lifts.
#   2. Tripod B reaches sideways while still lifted.
#   3. Tripod B lands at the reached position.
#   4. Tripod A = FL + MR + RL lifts while Tripod B holds/pulls the body.
#   5. Tripod A reaches sideways and lands.
#   6. Repeat.
#
# Important: this is still joint-space, not true IK. The coxa/hip is used only
# for A/D side-step reach/push. Femur/tibia mainly lift and keep clearance.
# If a/d is reversed, use: sidestrafe flip
# If movement is too small, use: sidestrafe hip 18
# If it drags, use: sidestrafe lift 38 30

SIDE_STRAFE_DIRECTION_MULTIPLIER = 1.0

# NO HIP MOVEMENT FOR A/D STRAFE.
# These stay zero on purpose. A/D strafe locks all coxa/hip joints at READY_POSE.
SIDE_STRAFE_HIP_REACH_DEG = 0.0
SIDE_STRAFE_HIP_PUSH_DEG = 0.0

# Main no-hip side reach using femur+tibia only.
# WControl23 change:
#   The lifted foot should NOT tuck inward then extend back to ready.
#   It should lift OUTWARD, land OUTWARD, then the planted foot PULLS the body sideways.
#
# Meaning in this joint-space model:
#   femur negative = lift up
#   tibia positive = tuck inward/retract  (bad for strafe if too much)
#   tibia negative = extend/outward       (needed for sideways reach/pull)
SIDE_STRAFE_FEMUR_REACH_DEG = 6.0
SIDE_STRAFE_TIBIA_REACH_DEG = -14.0

# Planted tripod pull after landing.
# This is intentionally the OPPOSITE of the reached-out pose.
# Land out -> pull toward body/side direction under load -> body moves sideways.
SIDE_STRAFE_FEMUR_PULL_DEG = -5.0
SIDE_STRAFE_TIBIA_PULL_DEG = 12.0

# Lift-out profile.
# Old WControl22 used +30 tibia, which tucked/shrank the leg inward.
# This uses femur for clearance and tibia slightly OUTWARD instead.
SIDE_STRAFE_LIFT_FEMUR_DEG = -34.0
SIDE_STRAFE_LIFT_TIBIA_DEG = -6.0

SIDE_STRAFE_HOLD = 0.30
SIDE_STRAFE_SETTLE = 0.14

# WControl35 special phase tuning.
# Boost ONLY the phase where A tripod is reaching and B tripod is standing:
# SIDE_left_A_REACH_B_PULL / SIDE_right_A_REACH_B_PULL.
# Left strafe effect: FR/RR femur extend more; ML femur contracts more.
# Right strafe is mirrored automatically.
SIDE_STRAFE_PHASE_BOOST_ENABLED = True
SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG = 9.0
SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG = 12.0
SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG = 8.0
SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG = 12.0

# WControl33 A/D debug-step mode.
# This does NOT lower AX motor speed. It breaks each side-strafe phase into
# visible micro-poses so you can watch where each leg drags, skids, or overloads.
# Use:
#   sidestrafe debug on
#   sidestrafe debug slow
#   sidestrafe debug enter on
#   sidestrafe debug off
SIDE_STRAFE_DEBUG_STEPS_ENABLED = False
SIDE_STRAFE_DEBUG_STEPS = 10
SIDE_STRAFE_DEBUG_STEP_DELAY = 0.070
SIDE_STRAFE_DEBUG_PRINT_FRAMES = False
SIDE_STRAFE_DEBUG_ENTER_STEP = False


# Pushup/higher body test levels.
PUSHUP_LEVELS = {
    "1": {
        1: 470, 2: 757, 3: 425, 4: 388, 5: 776, 6: 766,
        7: 598, 8: 763, 9: 616, 10: 433, 11: 235, 12: 783,
        13: 568, 14: 565, 15: 640, 16: 423, 17: 198, 18: 798,
    },
    "2": {
        1: 480, 2: 767, 3: 439, 4: 402, 5: 756, 6: 746,
        7: 588, 8: 773, 9: 602, 10: 447, 11: 255, 12: 763,
        13: 558, 14: 555, 15: 626, 16: 437, 17: 218, 18: 778,
    },
    "3": {
        1: 491, 2: 778, 3: 453, 4: 416, 5: 715, 6: 705,
        7: 577, 8: 784, 9: 588, 10: 461, 11: 296, 12: 722,
        13: 547, 14: 544, 15: 612, 16: 451, 17: 259, 18: 737,
    },
    "4": {
        1: 501, 2: 788, 3: 466, 4: 429, 5: 674, 6: 664,
        7: 567, 8: 794, 9: 575, 10: 474, 11: 337, 12: 681,
        13: 537, 14: 534, 15: 599, 16: 464, 17: 300, 18: 696,
    },
}


# ============================================================
# RUNTIME STATE
# ============================================================

ACTIVE_GOALS: Dict[int, int] = dict(READY_POSE)
CURRENT_MODE = "UNKNOWN"

# WControl70 movement-stat debug mode.
# Off by default so the working gait stays fast/smooth.
# When enabled, prints per-leg load/voltage/temp after every major gait phase.
MOVEMENT_STATS_ENABLED = False
MOVEMENT_STATS_DETAIL = "compact"  # compact or detail
MOVEMENT_STATS_WARN_ONLY = False

# WControl71 A/D-only flow mode.
# True = remove extra hold/sleep between side-strafe phases so a/d feels like
# one continuous controller gait. This only affects left/right strafe.
SIDE_STRAFE_FLOW_MODE = True
SIDE_STRAFE_FLOW_HOLD = 0.0
SIDE_STRAFE_FLOW_TINY_HOLD = 0.015
SIDE_STRAFE_FLOW_PRINT_PHASES = True



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


def offset_from_ready(joint_name: str, deg: float) -> int:
    motor_id = joint_to_motor_id(joint_name)
    return clamp_raw(READY_POSE[motor_id] + logical_deg_to_raw_delta(joint_name, deg))


def build_leg_offset_targets(
    leg: str,
    hip_deg: float = 0.0,
    femur_deg: float = 0.0,
    tibia_deg: float = 0.0,
) -> Dict[int, int]:
    hip_joint = leg_part_to_joint(leg, "hip")
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")

    hip_id = joint_to_motor_id(hip_joint)
    femur_id = joint_to_motor_id(femur_joint)
    tibia_id = joint_to_motor_id(tibia_joint)

    # Per-leg lift trim.
    # This only changes femur/tibia lift/tuck size, not hip stride.
    femur_deg = femur_deg * LEG_FEMUR_LIFT_SCALE.get(leg, 1.0)
    tibia_deg = tibia_deg * LEG_TIBIA_LIFT_SCALE.get(leg, 1.0)

    return {
        hip_id: offset_from_ready(hip_joint, hip_deg),
        femur_id: offset_from_ready(femur_joint, femur_deg),
        tibia_id: offset_from_ready(tibia_joint, tibia_deg),
    }


def normalize_direction(text: str) -> str:
    text = text.lower().strip()

    aliases = {
        "foward": "forward",
        "forwad": "forward",
        "fw": "forward",
        "w": "forward",

        "back": "backward",
        "backwards": "backward",
        "bw": "backward",
        "s": "backward",

        "l": "left",
        "a": "left",

        "r": "right",
        "d": "right",

        "tl": "turn_left",
        "q": "turn_left",

        "tr": "turn_right",
        "e": "turn_right",
    }

    return aliases.get(text, text)


def parse_lift_command(parts: List[str]) -> Tuple[int, List[str]]:
    """
    Supported:
      lift FL
      lift 3 FL
      lift 5 FL FR
      lift 2 FL MR RR
    """
    if len(parts) < 2:
        raise ValueError("Usage: lift FL OR lift 3 FL FR")

    level = DEFAULT_LIFT_LEVEL
    leg_tokens = parts[1:]

    if parts[1].isdigit():
        level = int(parts[1])
        leg_tokens = parts[2:]

    if level not in LIFT_LEVELS:
        raise ValueError("Lift level must be 1, 2, 3, 4, 5, 6, 7, 8, or 9.")

    if not leg_tokens:
        raise ValueError("No leg selected. Example: lift 3 FL FR")

    legs = [token.upper() for token in leg_tokens]

    for leg in legs:
        if leg not in ALL_LEGS:
            raise ValueError(f"Unknown leg: {leg}. Valid legs: {ALL_LEGS}")

    if len(set(legs)) != len(legs):
        raise ValueError("Duplicate leg in command.")

    return level, legs


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
        speed = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, speed)))
        self.write2(motor_id, ADDR_MOVING_SPEED, speed)

    def set_torque_limit(self, motor_id: int, torque_limit: int):
        torque_limit = int(max(0, min(1023, torque_limit)))
        self.write2(motor_id, ADDR_TORQUE_LIMIT, torque_limit)

    def set_torque_limit_all(self, torque_limit: int = TORQUE_LIMIT_RAW):
        for motor_id in ALL_MOTOR_IDS:
            self.set_torque_limit(motor_id, torque_limit)
            time.sleep(0.006)

    def move_many(self, targets: Dict[int, int], speed: int):
        global ACTIVE_GOALS

        for motor_id in targets:
            self.enable_torque(motor_id)
            self.set_speed(motor_id, speed)
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
    print(f"Current mode : {CURRENT_MODE}")
    print(f"Connected    : {connected}/18")
    print(f"Max temp     : {max_temp} C")
    print(f"Min voltage  : {min_volt:.1f} V")
    print(f"Max abs load : {max_abs_load}")
    print(f"No reply     : {any_no_reply}")
    print(f"Status       : {status}")


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
        print("Use force_r only if physically supporting the robot.")
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



def read_leg_movement_stats(bus: DynamixelBus) -> Dict[str, Dict[str, object]]:
    """
    Read compact leg-level stats.

    Returns one row per leg:
      hip/femur/tibia loads
      max joint load
      total absolute leg load
      min voltage
      max temperature

    This is intentionally lightweight enough for debugging, but still reads
    all 18 motors, so only enable while testing/tuning.
    """
    rows: Dict[str, Dict[str, object]] = {}

    for leg in ALL_LEGS:
        rows[leg] = {
            "hip_load": None,
            "femur_load": None,
            "tibia_load": None,
            "abs_sum": 0,
            "max_abs": 0,
            "max_part": "",
            "min_volt": 99.0,
            "max_temp": 0,
            "warnings": [],
        }

    for motor_id in ALL_MOTOR_IDS:
        joint_name = motor_id_to_joint(motor_id)
        leg, part = joint_to_leg_part(joint_name)
        if leg not in rows or part not in ["hip", "femur", "tibia"]:
            continue

        load_raw = bus.read2(motor_id, ADDR_PRESENT_LOAD)
        volt_raw = bus.read1(motor_id, ADDR_PRESENT_VOLTAGE)
        temp = bus.read1(motor_id, ADDR_PRESENT_TEMPERATURE)

        load_value = decode_load_value(load_raw)
        if load_value is not None:
            rows[leg][f"{part}_load"] = load_value
            abs_load = abs(load_value)
            rows[leg]["abs_sum"] += abs_load
            if abs_load > rows[leg]["max_abs"]:
                rows[leg]["max_abs"] = abs_load
                rows[leg]["max_part"] = part

        if volt_raw is not None:
            rows[leg]["min_volt"] = min(rows[leg]["min_volt"], volt_raw / 10.0)

        if temp is not None:
            rows[leg]["max_temp"] = max(rows[leg]["max_temp"], int(temp))

    for leg, row in rows.items():
        warns = []
        max_abs = int(row["max_abs"])
        min_v = float(row["min_volt"])
        max_t = int(row["max_temp"])

        if max_abs >= LOAD_STOP:
            warns.append("LOAD_STOP")
        elif max_abs >= LOAD_WARN:
            warns.append("LOAD_WARN")

        if min_v <= VOLT_STOP_V:
            warns.append("VOLT_STOP")
        elif min_v <= VOLT_WARN_V:
            warns.append("LOW_V")

        if max_t >= TEMP_STOP_C:
            warns.append("TEMP_STOP")
        elif max_t >= TEMP_WARN_C:
            warns.append("TEMP_WARN")

        if row["min_volt"] == 99.0:
            row["min_volt"] = 0.0

        row["warnings"] = warns

    return rows


def print_movement_stats(bus: DynamixelBus, label: str = "MOVEMENT PHASE", active: Optional[List[str]] = None, support: Optional[List[str]] = None):
    """
    WControl70 global movement statistics.

    Shows whether load is balanced after a phase. Useful for spotting:
      - ML too light
      - FR/RR overload
      - one leg not contributing
      - voltage sag
      - temperature rise
    """
    if not MOVEMENT_STATS_ENABLED:
        return

    active = active or []
    support = support or []
    rows = read_leg_movement_stats(bus)

    print()
    print("---------------------------------------------------")
    print(f" MOVEMENT STATS: {label}")
    print("---------------------------------------------------")
    print("Role: ACTIVE = lifted/swinging tripod, SUPPORT = planted/support tripod")
    print(f"{'Leg':<3} {'Role':<8} {'HipL':>6} {'FemL':>6} {'TibL':>6} {'AbsSum':>7} {'Max':>5} {'Vmin':>5} {'Tmax':>5} Warnings")
    print("-" * 92)

    support_total = 0
    support_leg_abs: Dict[str, int] = {}

    for leg in ALL_LEGS:
        row = rows[leg]
        if leg in active:
            role = "ACTIVE"
        elif leg in support:
            role = "SUPPORT"
        else:
            role = "-"

        hip = row["hip_load"]
        fem = row["femur_load"]
        tib = row["tibia_load"]

        hip_s = "----" if hip is None else f"{int(hip):+d}"
        fem_s = "----" if fem is None else f"{int(fem):+d}"
        tib_s = "----" if tib is None else f"{int(tib):+d}"

        abs_sum = int(row["abs_sum"])
        max_abs = int(row["max_abs"])
        min_v = float(row["min_volt"])
        max_t = int(row["max_temp"])
        warn_text = "OK" if not row["warnings"] else ",".join(row["warnings"])

        if MOVEMENT_STATS_DETAIL == "detail" or role == "SUPPORT" or warn_text != "OK":
            print(f"{leg:<3} {role:<8} {hip_s:>6} {fem_s:>6} {tib_s:>6} {abs_sum:>7} {max_abs:>5} {min_v:>5.1f} {max_t:>5} {warn_text}")

        if role == "SUPPORT":
            support_total += abs_sum
            support_leg_abs[leg] = abs_sum

    if support_leg_abs and support_total > 0:
        shares = []
        for leg, value in support_leg_abs.items():
            shares.append(f"{leg}={value / support_total * 100:.0f}%")
        print("-" * 92)
        print("Support load share: " + " | ".join(shares))

        light_leg = min(support_leg_abs, key=support_leg_abs.get)
        heavy_leg = max(support_leg_abs, key=support_leg_abs.get)
        if support_leg_abs[light_leg] > 0:
            ratio = support_leg_abs[heavy_leg] / max(1, support_leg_abs[light_leg])
            if ratio >= 2.0:
                print(f"[LOAD IMBALANCE] {heavy_leg} is carrying about {ratio:.1f}x {light_leg}.")
        else:
            print(f"[LOAD IMBALANCE] {light_leg} is almost not contributing.")

    print("---------------------------------------------------")


def print_movement_stats_settings():
    print()
    print("===================================================")
    print(" MOVEMENT STATS SETTINGS")
    print("===================================================")
    print(f"MOVEMENT_STATS_ENABLED = {MOVEMENT_STATS_ENABLED}")
    print(f"MOVEMENT_STATS_DETAIL  = {MOVEMENT_STATS_DETAIL}")
    print("Commands:")
    print("  movestats")
    print("  movestats on        = print per-leg stats after every movement phase")
    print("  movestats off       = no extra phase stats")
    print("  movestats compact   = support legs + warning legs only")
    print("  movestats detail    = all six legs every phase")
    print("===================================================")


def action_movement_stats(parts: List[str]):
    global MOVEMENT_STATS_ENABLED, MOVEMENT_STATS_DETAIL

    if len(parts) == 1:
        print_movement_stats_settings()
        return

    sub = parts[1].lower()

    if sub in ["on", "enable", "true", "1"]:
        MOVEMENT_STATS_ENABLED = True
    elif sub in ["off", "disable", "false", "0"]:
        MOVEMENT_STATS_ENABLED = False
    elif sub in ["compact", "normal"]:
        MOVEMENT_STATS_DETAIL = "compact"
        MOVEMENT_STATS_ENABLED = True
    elif sub in ["detail", "full", "all"]:
        MOVEMENT_STATS_DETAIL = "detail"
        MOVEMENT_STATS_ENABLED = True
    else:
        print("Usage: movestats on/off/compact/detail")
        return

    print_movement_stats_settings()

def print_status(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" MOTOR STATUS / REFINED2K BALANCED CONTROL")
    print("===================================================")
    print(
        f"{'ID':>2} {'Joint':<10} {'Leg':<2} {'Part':<5} "
        f"{'Raw':>4} {'DegReady':>8} {'Ready':>5} {'Goal':>5} "
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

        ready = READY_POSE[motor_id]
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
            f"{raw:>4} {deg:>8.2f} {ready:>5} {goal:>5} "
            f"{decode_load_text(load_raw):>7} {volt_text:>5} {temp_text:>5} {warn_text}"
        )

    print("-" * 120)
    print(f"Connected: {connected}/18")
    print(f"Health: maxTemp={max_temp}C, minVolt={min_volt:.1f}V, maxAbsLoad={max_abs_load}")
    print(f"Current mode: {CURRENT_MODE}")


# ============================================================
# ACTIONS
# ============================================================

def print_speed_settings():
    print()
    print("===================================================")
    print(" SPEED SETTINGS")
    print("===================================================")
    print(f"READY_SPEED = {READY_SPEED}")
    print(f"MOVE_SPEED  = {MOVE_SPEED}")
    print(f"LIFT_SPEED  = {LIFT_SPEED}")
    print(f"GAIT_SPEED  = {GAIT_SPEED}")
    print("Note: AX speed 1 is very slow; higher number is faster. Avoid 0 while tuning.")
    print("===================================================")


def action_set_speed(parts: List[str]):
    global READY_SPEED, MOVE_SPEED, LIFT_SPEED, GAIT_SPEED

    if len(parts) == 1:
        print_speed_settings()
        return

    # Shortcut: speed 22 means set GAIT_SPEED only.
    if len(parts) == 2:
        target = "gait"
        value_text = parts[1]
    elif len(parts) == 3:
        target = parts[1].lower()
        value_text = parts[2]
    else:
        print("Usage:")
        print("  speed")
        print("  speed 22          # shortcut for speed gait 22")
        print("  speed gait 18")
        print("  speed gait 12")
        print("  speed lift 10")
        print("  speed ready 12")
        print("  speed move 10")
        print("  speed all 10")
        return

    try:
        value = int(value_text)
    except ValueError:
        print("Speed must be a number. Example: speed gait 18")
        return

    value = int(max(MIN_SAFE_SPEED, min(MAX_SAFE_SPEED, value)))

    if target == "gait":
        GAIT_SPEED = value
    elif target == "lift":
        LIFT_SPEED = value
    elif target == "ready":
        READY_SPEED = value
    elif target == "move":
        MOVE_SPEED = value
    elif target == "all":
        READY_SPEED = value
        MOVE_SPEED = value
        LIFT_SPEED = value
        GAIT_SPEED = value
    else:
        print("Unknown speed target. Use gait/lift/ready/move/all.")
        return

    print_speed_settings()




def print_side_logic_settings():
    print()
    print("===================================================")
    print(" WCONTROL11 SIDE/TURN LOGIC SETTINGS")
    print("===================================================")
    print(f"STRAFE_DIRECTION_MULTIPLIER = {STRAFE_DIRECTION_MULTIPLIER:+.1f}")
    print(f"TURN_DIRECTION_MULTIPLIER   = {TURN_DIRECTION_MULTIPLIER:+.1f}")
    print(f"TURN_LEFT_SCALE             = {TURN_LEFT_SCALE:.2f}  # q")
    print(f"TURN_RIGHT_SCALE            = {TURN_RIGHT_SCALE:.2f}  # e")
    print(f"Effective q turn            = hip {TURN_HIP_SWING_DEG * TURN_LEFT_SCALE:.1f}, support {TURN_SUPPORT_PUSH_DEG * TURN_LEFT_SCALE:.1f}")
    print(f"Effective e turn            = hip {TURN_HIP_SWING_DEG * TURN_RIGHT_SCALE:.1f}, support {TURN_SUPPORT_PUSH_DEG * TURN_RIGHT_SCALE:.1f}")
    print("---------------------------------------------------")
    print("If a and d are reversed:")
    print("  sideflip strafe")
    print("If q and e are reversed:")
    print("  sideflip turn")
    print("If both are reversed:")
    print("  sideflip all")
    print("===================================================")



def action_turnscale(parts: List[str]):
    """
    Runtime precision tuning for q/e.

    Usage:
      turnscale
      turnscale left 0.75
      turnscale right 0.78
      turnscale both 0.75
      turnscale reset

    Lower number = smaller turn per command.
    Example:
      0.75 = 25% less movement
      0.78 = 22% less movement
      0.90 = 10% less movement
    """
    global TURN_LEFT_SCALE, TURN_RIGHT_SCALE

    if len(parts) == 1:
        print_side_logic_settings()
        return

    target = parts[1].lower()

    if target == "reset":
        TURN_LEFT_SCALE = 0.75
        TURN_RIGHT_SCALE = 0.78
        print_side_logic_settings()
        return

    if len(parts) != 3:
        print("Usage:")
        print("  turnscale")
        print("  turnscale left 0.75")
        print("  turnscale right 0.78")
        print("  turnscale both 0.75")
        print("  turnscale reset")
        return

    try:
        value = float(parts[2])
    except ValueError:
        print("Scale must be a number, example 0.75")
        return

    # Clamp for safety/precision.
    value = max(0.40, min(1.10, value))

    if target in ["left", "q"]:
        TURN_LEFT_SCALE = value
    elif target in ["right", "e"]:
        TURN_RIGHT_SCALE = value
    elif target == "both":
        TURN_LEFT_SCALE = value
        TURN_RIGHT_SCALE = value
    else:
        print("Target must be left, right, both, q, or e.")
        return

    print_side_logic_settings()


def action_sideflip(parts: List[str]):
    """
    Flip strafe or turn direction without editing the code.

    Usage:
      sideflip
      sideflip strafe
      sideflip turn
      sideflip all
    """
    global STRAFE_DIRECTION_MULTIPLIER, TURN_DIRECTION_MULTIPLIER

    if len(parts) == 1:
        print_side_logic_settings()
        return

    target = parts[1].lower()

    if target == "strafe":
        STRAFE_DIRECTION_MULTIPLIER *= -1.0
    elif target == "turn":
        TURN_DIRECTION_MULTIPLIER *= -1.0
    elif target == "all":
        STRAFE_DIRECTION_MULTIPLIER *= -1.0
        TURN_DIRECTION_MULTIPLIER *= -1.0
    else:
        print("Usage: sideflip / sideflip strafe / sideflip turn / sideflip all")
        return

    print_side_logic_settings()


def print_range_settings():
    print()
    print("===================================================")
    print(" MOVEMENT RANGE SETTINGS")
    print("===================================================")
    print(f"Forward hip/support  = {GAIT_HIP_SWING_DEG:.1f} / {GAIT_SUPPORT_PUSH_DEG:.1f} deg")
    print(f"Backward hip/support = {BACKWARD_HIP_SWING_DEG:.1f} / {BACKWARD_SUPPORT_PUSH_DEG:.1f} deg")
    print(f"Strafe hip/support   = {STRAFE_HIP_SWING_DEG:.1f} / {STRAFE_SUPPORT_PUSH_DEG:.1f} deg")
    print(f"Turn hip/support     = {TURN_HIP_SWING_DEG:.1f} / {TURN_SUPPORT_PUSH_DEG:.1f} deg")
    print("---------------------------------------------------")
    print("WControl10 keeps forward/backward unchanged.")
    print("Only strafe and turn ranges are increased/tunable.")
    print("===================================================")


def action_set_range(parts: List[str]):
    """
    Runtime range tuning without editing code.

    Usage:
      range
      range strafe 26 20
      range turn 30 22
      range all_side 24 18 28 20
    """
    global STRAFE_HIP_SWING_DEG, STRAFE_SUPPORT_PUSH_DEG
    global TURN_HIP_SWING_DEG, TURN_SUPPORT_PUSH_DEG

    if len(parts) == 1:
        print_range_settings()
        return

    target = parts[1].lower()

    try:
        if target == "strafe" and len(parts) == 4:
            STRAFE_HIP_SWING_DEG = float(parts[2])
            STRAFE_SUPPORT_PUSH_DEG = float(parts[3])

        elif target == "turn" and len(parts) == 4:
            TURN_HIP_SWING_DEG = float(parts[2])
            TURN_SUPPORT_PUSH_DEG = float(parts[3])

        elif target == "all_side" and len(parts) == 6:
            STRAFE_HIP_SWING_DEG = float(parts[2])
            STRAFE_SUPPORT_PUSH_DEG = float(parts[3])
            TURN_HIP_SWING_DEG = float(parts[4])
            TURN_SUPPORT_PUSH_DEG = float(parts[5])

        else:
            print("Usage:")
            print("  range")
            print("  range strafe 26 20")
            print("  range turn 30 22")
            print("  range all_side 24 18 28 20")
            return

    except ValueError:
        print("Range values must be numbers.")
        return

    # Clamp to sane tuning range.
    STRAFE_HIP_SWING_DEG = max(8.0, min(36.0, STRAFE_HIP_SWING_DEG))
    STRAFE_SUPPORT_PUSH_DEG = max(6.0, min(28.0, STRAFE_SUPPORT_PUSH_DEG))
    TURN_HIP_SWING_DEG = max(8.0, min(40.0, TURN_HIP_SWING_DEG))
    TURN_SUPPORT_PUSH_DEG = max(6.0, min(30.0, TURN_SUPPORT_PUSH_DEG))

    print_range_settings()


def action_torque_max(bus: DynamixelBus):
    print()
    print("===================================================")
    print(" ACTION: SET TORQUE LIMIT CAP TO MAX")
    print("===================================================")
    print("Setting AX torque limit cap to 1023 for all motors.")
    print("This does NOT command full torque constantly.")
    print("It only lets each servo use more torque if required.")
    print("Safety health/load warnings are still active.")
    print("===================================================")
    bus.set_torque_limit_all(TORQUE_LIMIT_RAW)
    print_health(bus, "AFTER TORQUE_MAX")

def build_ready_lift_targets(legs: List[str]) -> Dict[int, int]:
    """
    Fast tripod-lift ready reset helper.

    This prevents all six feet from dragging back to READY at the same time.
    Hips stay locked at READY. Femur lifts; tibia stays slightly outward
    using the same successful WControl23 lift-out idea.
    """
    targets = dict(READY_POSE)

    for leg in legs:
        targets.update(
            build_leg_offset_targets(
                leg,
                hip_deg=0.0,
                femur_deg=SIDE_STRAFE_LIFT_FEMUR_DEG,
                tibia_deg=SIDE_STRAFE_LIFT_TIBIA_DEG,
            )
        )

    return targets


def action_ready(bus: DynamixelBus, use_safety_check: bool = True):
    global ACTIVE_GOALS, CURRENT_MODE

    if use_safety_check:
        if not pre_motion_check(bus):
            return

    print()
    print("ACTION: FAST TRIPOD-LIFT RETURN TO REFINED2K BALANCED READY_POSE")
    print("Reset method: lift B -> ready, lift A -> ready, final soft ready.")
    print("This avoids dragging all six feet back to stance at once.")

    reset_speed = READY_SPEED

    # Tripod B first: FR + ML + RR
    CURRENT_MODE = "READY_RESET_B_UP"
    targets = build_ready_lift_targets(CRAB_FIRST_TRIPOD)
    ACTIVE_GOALS = dict(targets)
    bus.move_many(targets, speed=reset_speed)
    time.sleep(0.12)

    CURRENT_MODE = "READY_RESET_B_DOWN"
    targets = dict(READY_POSE)
    ACTIVE_GOALS = dict(targets)
    bus.move_many(targets, speed=reset_speed)
    time.sleep(0.10)

    # Tripod A second: FL + MR + RL
    CURRENT_MODE = "READY_RESET_A_UP"
    targets = build_ready_lift_targets(CRAB_SECOND_TRIPOD)
    ACTIVE_GOALS = dict(targets)
    bus.move_many(targets, speed=reset_speed)
    time.sleep(0.12)

    CURRENT_MODE = "READY_RESET_A_DOWN"
    targets = dict(READY_POSE)
    ACTIVE_GOALS = dict(targets)
    bus.move_many(targets, speed=reset_speed)
    time.sleep(0.10)

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = dict(READY_POSE)
    bus.move_many(dict(READY_POSE), speed=reset_speed)
    time.sleep(0.20)

    print_status(bus)
    print_health(bus, "AFTER READY")


def action_pushup(bus: DynamixelBus, level: str):
    global ACTIVE_GOALS, CURRENT_MODE

    if level not in PUSHUP_LEVELS:
        print("Usage: pushup 1 / pushup 2 / pushup 3 / pushup 4")
        return

    if not pre_motion_check(bus):
        return

    print()
    print("===================================================")
    print(f" ACTION: PUSHUP {level}")
    print("===================================================")

    targets = PUSHUP_LEVELS[level]
    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"PUSHUP_{level}"

    bus.move_many(targets, speed=MOVE_SPEED)
    time.sleep(1.0)

    print_status(bus)
    print_health(bus, f"AFTER PUSHUP {level}")


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

    print()
    print("===================================================")
    print(f" ACTION: LIFT LEVEL {level} LEGS {' '.join(legs)}")
    print("===================================================")
    print(f"Femur lift: {femur_deg:+.1f} deg")
    print(f"Tibia move: {tibia_deg:+.1f} deg")
    print("Use r to return to refined2k balanced ready.")
    print("===================================================")

    targets = dict(READY_POSE)

    for leg in legs:
        targets.update(
            build_leg_offset_targets(
                leg,
                hip_deg=0.0,
                femur_deg=femur_deg,
                tibia_deg=tibia_deg,
            )
        )

    ACTIVE_GOALS = dict(targets)
    CURRENT_MODE = f"LIFT_L{level}_{'_'.join(legs)}"

    bus.move_many(targets, speed=LIFT_SPEED)
    time.sleep(0.8)

    print_status(bus)
    print_health(bus, f"AFTER LIFT LEVEL {level} {' '.join(legs)}")


def gait_lift_values() -> Tuple[float, float]:
    """
    Return the walking lift values.

    If USE_WALK_LIFT_PROFILE is True, walking does NOT use the full manual lift
    table. This avoids the wasted motion where tibia folds fully inward before
    the leg comes down. Femur gives most of the clearance; tibia only tucks a bit.
    """
    if USE_WALK_LIFT_PROFILE:
        return WALK_LIFT_FEMUR_DEG, WALK_LIFT_TIBIA_DEG

    level = GAIT_LIFT_LEVEL
    return LIFT_LEVELS[level]["femur"], LIFT_LEVELS[level]["tibia"]


def movement_profile(direction: str) -> Tuple[float, float]:
    """
    Returns (hip_swing_deg, support_push_deg) for the selected movement.
    Forward/backward keep your proven walking size.
    Strafe/turn are deliberately smaller for first tests.
    """
    direction = normalize_direction(direction)

    if direction == "backward":
        return BACKWARD_HIP_SWING_DEG, BACKWARD_SUPPORT_PUSH_DEG

    if direction in ["left", "right"]:
        return STRAFE_HIP_SWING_DEG, STRAFE_SUPPORT_PUSH_DEG

    if direction in ["turn_left", "turn_right"]:
        return TURN_HIP_SWING_DEG, TURN_SUPPORT_PUSH_DEG

    return GAIT_HIP_SWING_DEG, GAIT_SUPPORT_PUSH_DEG



def turn_hip_for_leg(leg: str, direction: str, amount: float, lifted: bool) -> float:
    """
    WControl11 side-opposed turning.

    For turn_left:
      left side acts like backward
      right side acts like forward

    For turn_right:
      left side acts like forward
      right side acts like backward

    lifted=True uses swing amount.
    lifted=False uses support amount with opposite push.
    """
    direction = normalize_direction(direction)

    # Use existing forward sign as the base because forward/backward are already tested.
    forward_sign = HIP_FORWARD_SIGN[leg]

    if direction == "turn_left":
        # Left side backward, right side forward.
        side_dir = -1 if leg in LEFT_LEGS else +1
    elif direction == "turn_right":
        # Left side forward, right side backward.
        side_dir = +1 if leg in LEFT_LEGS else -1
    else:
        side_dir = 0

    # WControl12 precision scaling.
    # q/turn_left and e/turn_right can have different effective strength.
    if direction == "turn_left":
        turn_scale = TURN_LEFT_SCALE
    elif direction == "turn_right":
        turn_scale = TURN_RIGHT_SCALE
    else:
        turn_scale = 1.0

    # lifted foot swings in intended side direction.
    # support foot pulls opposite against ground.
    phase_sign = +1 if lifted else -1

    return TURN_DIRECTION_MULTIPLIER * turn_scale * side_dir * forward_sign * amount * phase_sign


def strafe_hip_for_leg(leg: str, direction: str, amount: float, lifted: bool) -> float:
    """
    WControl11 approximate crab strafe.

    This is not true IK crab walking, but it uses a stronger dedicated strafe
    map instead of reusing the old small generic hip map.

    For right strafe, signs are mirrored from left strafe.
    """
    direction = normalize_direction(direction)

    if lifted:
        base = CRAB_STRAFE_LEFT_SWING[leg]
    else:
        base = CRAB_STRAFE_LEFT_SUPPORT[leg]

    if direction == "right":
        base = -base

    return STRAFE_DIRECTION_MULTIPLIER * base * amount



def crab_side_for_leg(leg: str, direction: str) -> float:
    """
    Side sign for pseudo-foot-target crab strafe.

    +1 = outward/side reach for that side
    -1 = inward/retract toward body centerline

    left strafe:
      left legs outward, right legs inward

    right strafe:
      right legs outward, left legs inward
    """
    direction = normalize_direction(direction)

    if direction == "left":
        return +1.0 if leg in LEFT_LEGS else -1.0
    elif direction == "right":
        return +1.0 if leg in RIGHT_LEGS else -1.0

    return 0.0


def crab_hip_for_leg(leg: str, direction: str, lifted: bool) -> float:
    """
    WControl18 optional crab hip trim.

    This deliberately does NOT reuse the old strafe hip map because that looked
    like diagonal/forward walking.

    Rule:
      lifted tripod placement sweeps the lifted feet sideways.
      body-shift/support phase sweeps the opposite direction against planted feet.

    If a/d is physically reversed, use:
      crab flip
    """
    # Default WControl18 behavior: no hip yaw for strafe.
    # This prevents the coxa joints from turning the strafe into diagonal walking.
    if not CRAB_USE_HIP_YAW or CRAB_HIP_DEG == 0.0:
        return 0.0

    direction = normalize_direction(direction)

    if direction == "left":
        direction_sign = +1.0
    elif direction == "right":
        direction_sign = -1.0
    else:
        direction_sign = 0.0

    # Use the known forward sign map for left/right mirroring.
    base = HIP_FORWARD_SIGN[leg]

    phase_sign = +1.0 if lifted else -1.0

    return CRAB_STRAFE_DIRECTION_MULTIPLIER * direction_sign * base * phase_sign * CRAB_HIP_DEG


def crab_leg_offsets(leg: str, direction: str, role: str) -> Tuple[float, float, float]:
    """
    role:
      "lift_neutral"  = lift vertically only
      "reach_lifted"  = lift + sideways foot placement
      "reach_ground"  = foot down at sideways placement
      "support_push"  = stance tripod pulls body sideways
      "ready"         = neutral ready

    Returns hip_deg, femur_deg, tibia_deg relative to READY_POSE.
    """
    direction = normalize_direction(direction)
    side = CRAB_STRAFE_DIRECTION_MULTIPLIER * crab_side_for_leg(leg, direction)
    lift_femur, lift_tibia = gait_lift_values()

    if role == "lift_neutral":
        return 0.0, lift_femur, lift_tibia

    if role == "reach_lifted":
        hip = crab_hip_for_leg(leg, direction, lifted=True)

        # Apply side reach, but never allow the reach to cancel visible lift.
        # Negative femur = higher lift in our current model.
        femur = lift_femur + side * CRAB_REACH_FEMUR_DEG
        tibia = lift_tibia + side * CRAB_REACH_TIBIA_DEG

        if femur > CRAB_MIN_REACH_LIFT_FEMUR_DEG:
            femur = CRAB_MIN_REACH_LIFT_FEMUR_DEG

        if tibia < CRAB_MIN_REACH_TUCK_TIBIA_DEG:
            tibia = CRAB_MIN_REACH_TUCK_TIBIA_DEG

        return (hip, femur, tibia)

    if role == "reach_ground":
        hip = crab_hip_for_leg(leg, direction, lifted=True)
        return (
            hip,
            side * CRAB_REACH_FEMUR_DEG,
            side * CRAB_REACH_TIBIA_DEG,
        )

    if role == "support_push":
        # WControl15 default: support tripod stays planted at READY.
        # This matches your observation: three legs should stand still while
        # the other three lift and reach.
        if not CRAB_SUPPORT_PUSH_ENABLED:
            return 0.0, 0.0, 0.0

        # Optional experimental support push if enabled.
        hip = crab_hip_for_leg(leg, direction, lifted=False)
        return (
            hip,
            -side * CRAB_SUPPORT_FEMUR_DEG,
            -side * CRAB_SUPPORT_TIBIA_DEG,
        )

    if role == "body_shift":
        # WControl18: controlled side push after the reaching tripod is down.
        # The main body shift is femur/tibia radial placement. Hip yaw is optional trim only.
        if not CRAB_BODY_SHIFT_ENABLED:
            return 0.0, 0.0, 0.0

        hip = crab_hip_for_leg(leg, direction, lifted=False)
        hip = hip * (CRAB_BODY_SHIFT_HIP_DEG / max(1.0, CRAB_HIP_DEG))

        return (
            hip,
            -side * CRAB_BODY_SHIFT_FEMUR_DEG,
            -side * CRAB_BODY_SHIFT_TIBIA_DEG,
        )

    return 0.0, 0.0, 0.0


def build_crab_targets(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,
) -> Dict[int, int]:
    """
    Build full-body target for WControl14 crab strafe.

    phase:
      up      = lifted tripod lifts vertically, support tripod pulls
      reach   = lifted tripod moves sideways while in air, support pulls
      down    = lifted tripod placed down at side position, support holds push
      neutral = lifted tripod returns to ready after being lifted during recenter
    """
    targets = dict(READY_POSE)

    if phase == "up":
        for leg in lifted_legs:
            h, f, t = crab_leg_offsets(leg, direction, "lift_neutral")
            targets.update(build_leg_offset_targets(leg, h, f, t))

        for leg in support_legs:
            h, f, t = crab_leg_offsets(leg, direction, "support_push")
            targets.update(build_leg_offset_targets(leg, h, f, t))

    elif phase == "reach":
        for leg in lifted_legs:
            h, f, t = crab_leg_offsets(leg, direction, "reach_lifted")
            targets.update(build_leg_offset_targets(leg, h, f, t))

        for leg in support_legs:
            h, f, t = crab_leg_offsets(leg, direction, "support_push")
            targets.update(build_leg_offset_targets(leg, h, f, t))

    elif phase == "down":
        for leg in lifted_legs:
            h, f, t = crab_leg_offsets(leg, direction, "reach_ground")
            targets.update(build_leg_offset_targets(leg, h, f, t))

        for leg in support_legs:
            h, f, t = crab_leg_offsets(leg, direction, "support_push")
            targets.update(build_leg_offset_targets(leg, h, f, t))

    elif phase == "body_shift":
        # After the reaching tripod is down, make that tripod push the body sideways.
        # Other tripod stays ready/planted for stability.
        for leg in lifted_legs:
            h, f, t = crab_leg_offsets(leg, direction, "body_shift")
            targets.update(build_leg_offset_targets(leg, h, f, t))

        for leg in support_legs:
            targets.update(build_leg_offset_targets(leg, 0.0, 0.0, 0.0))

    elif phase == "neutral":
        for leg in lifted_legs:
            h, f, t = crab_leg_offsets(leg, direction, "lift_neutral")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        # other legs stay ready

    return targets


def crab_final_recenter(bus: DynamixelBus, direction: str):
    """
    Recenter after crab without dragging all feet.
    Lift first tripod, return it to ready, down.
    Then lift second tripod, return it to ready, down.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    sequence = [
        ("CRAB_END_B_UP", CRAB_FIRST_TRIPOD),
        ("CRAB_END_B_READY", CRAB_FIRST_TRIPOD),
        ("CRAB_END_A_UP", CRAB_SECOND_TRIPOD),
        ("CRAB_END_A_READY", CRAB_SECOND_TRIPOD),
    ]

    for mode_name, legs in sequence:
        CURRENT_MODE = f"{direction}_{mode_name}"

        if mode_name.endswith("_UP"):
            targets = dict(READY_POSE)
            for leg in legs:
                h, f, t = crab_leg_offsets(leg, direction, "lift_neutral")
                targets.update(build_leg_offset_targets(leg, h, f, t))
        else:
            targets = dict(READY_POSE)

        ACTIVE_GOALS = dict(targets)
        move_targets_for_side_strafe(bus, targets, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY, phase_label=CURRENT_MODE)
        print(f"{CURRENT_MODE}: sent")

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = dict(READY_POSE)
    move_targets_for_side_strafe(bus, dict(READY_POSE), speed=GAIT_SPEED, hold_delay=GAIT_FINAL_READY_DELAY)


def action_crab_strafe_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    """
    WControl14 dedicated crab strafe.
    Does not affect forward/backward/turn.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)

    if direction not in ["left", "right"]:
        print("Crab strafe only supports left/right.")
        return

    cycles = max(1, min(5, int(cycles)))

    lift_femur, lift_tibia = gait_lift_values()

    print()
    print("===================================================")
    print(f" ACTION: WCONTROL19 IK-STYLE A/D SIDE STRAFE {direction.upper()} x{cycles}")
    print("===================================================")
    print(f"Tripod order: B={CRAB_FIRST_TRIPOD}, then A={CRAB_SECOND_TRIPOD}")
    print(f"Lift: femur {lift_femur:+.1f}, tibia {lift_tibia:+.1f}")
    print(f"Reach: femur {CRAB_REACH_FEMUR_DEG:+.1f}, tibia {CRAB_REACH_TIBIA_DEG:+.1f}")
    print(f"Support planted: {not CRAB_SUPPORT_PUSH_ENABLED}")
    print(f"Optional support push: femur {CRAB_SUPPORT_FEMUR_DEG:+.1f}, tibia {CRAB_SUPPORT_TIBIA_DEG:+.1f}")
    print(f"Min reach lift/tuck: femur {CRAB_MIN_REACH_LIFT_FEMUR_DEG:+.1f}, tibia {CRAB_MIN_REACH_TUCK_TIBIA_DEG:+.1f}")
    print(f"Body shift enabled: {CRAB_BODY_SHIFT_ENABLED}, hip {CRAB_BODY_SHIFT_HIP_DEG:+.1f}, femur {CRAB_BODY_SHIFT_FEMUR_DEG:+.1f}, tibia {CRAB_BODY_SHIFT_TIBIA_DEG:+.1f}, hold {CRAB_BODY_SHIFT_HOLD:.2f}s")
    print(f"Hip yaw enabled: {CRAB_USE_HIP_YAW}, hip {CRAB_HIP_DEG:+.1f}  # should stay OFF/near 0 for true strafe")
    print("Theory: lift tripod -> place feet sideways with femur/tibia -> land -> body shifts sideways.")
    print("===================================================")

    for i in range(cycles):
        print()
        print(f"--- CRAB STRAFE CYCLE {i + 1}/{cycles} ---")

        if not pre_motion_check(bus):
            return

        phases = [
            (f"CRAB_{direction}_B_UP", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "up", GAIT_PHASE_DELAY),
            (f"CRAB_{direction}_B_REACH", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "reach", GAIT_PHASE_DELAY),
            (f"CRAB_{direction}_B_DOWN", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "down", GAIT_SETTLE_DELAY),
            (f"CRAB_{direction}_B_BODY_SHIFT", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "body_shift", CRAB_BODY_SHIFT_HOLD),

            (f"CRAB_{direction}_A_UP", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "up", GAIT_PHASE_DELAY),
            (f"CRAB_{direction}_A_REACH", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "reach", GAIT_PHASE_DELAY),
            (f"CRAB_{direction}_A_DOWN", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "down", GAIT_SETTLE_DELAY),
            (f"CRAB_{direction}_A_BODY_SHIFT", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "body_shift", CRAB_BODY_SHIFT_HOLD),
        ]

        for mode_name, lifted, support, phase, delay in phases:
            CURRENT_MODE = mode_name
            targets = build_crab_targets(lifted, support, direction, phase)
            ACTIVE_GOALS = dict(targets)
            move_targets_for_gait(bus, targets, speed=GAIT_SPEED, hold_delay=delay)

            if GAIT_PHASE_HEALTH:
                print_health(bus, CURRENT_MODE)
            else:
                print(f"{CURRENT_MODE}: sent")

        print_health(bus, f"AFTER CRAB CYCLE {i + 1} {direction}")

    print("Final mode: CRAB TRIPOD RECENTER.")
    crab_final_recenter(bus, direction)

    print_status(bus)
    time.sleep(0.25)
    print_health(bus, f"AFTER CRAB STRAFE {direction}")


def print_crab_settings():
    print()
    print("===================================================")
    print(" WCONTROL19 IK-STYLE A/D SIDE STRAFE SETTINGS")
    print("===================================================")
    print(f"Direction multiplier = {CRAB_STRAFE_DIRECTION_MULTIPLIER:+.1f}")
    print(f"Reach femur/tibia    = {CRAB_REACH_FEMUR_DEG:+.1f} / {CRAB_REACH_TIBIA_DEG:+.1f}")
    print(f"Support push enabled = {CRAB_SUPPORT_PUSH_ENABLED}")
    print(f"Support femur/tibia  = {CRAB_SUPPORT_FEMUR_DEG:+.1f} / {CRAB_SUPPORT_TIBIA_DEG:+.1f}")
    print(f"Body shift enabled   = {CRAB_BODY_SHIFT_ENABLED}")
    print(f"Body shift hip       = {CRAB_BODY_SHIFT_HIP_DEG:+.1f}")
    print(f"Body shift fem/tib   = {CRAB_BODY_SHIFT_FEMUR_DEG:+.1f} / {CRAB_BODY_SHIFT_TIBIA_DEG:+.1f}")
    print(f"Body shift hold      = {CRAB_BODY_SHIFT_HOLD:.2f}s")
    print(f"Min reach lift/tuck  = {CRAB_MIN_REACH_LIFT_FEMUR_DEG:+.1f} / {CRAB_MIN_REACH_TUCK_TIBIA_DEG:+.1f}")
    print(f"Use hip yaw          = {CRAB_USE_HIP_YAW}")
    print(f"Hip yaw amount       = {CRAB_HIP_DEG:+.1f}")
    print("---------------------------------------------------")
    print("Commands:")
    print("  crab")
    print("  crab power 8       # gentler lateral reach")
    print("  crab power 10      # default-ish lateral reach")
    print("  crab power 12      # stronger lateral reach")
    print("  crab lift -30 26")
    print("  crab hipamount 4   # optional small hip trim only")
    print("  crab bodyhip 0     # default; keeps body shift hip-free")
    print("  crab shift 3")
    print("  crab shift 5")
    print("  crab shifthold 0.25")
    print("  crab shiftoff / crab shifton")
    print("  crab support on/off")
    print("  crab hip on        # experimental small trim")
    print("  crab hip off       # recommended default")
    print("  crab flip")
    print("===================================================")


def action_crab_settings(parts: List[str]):
    """
    Runtime crab tuning.

    Usage:
      crab
      crab power 12
      crab power 14
      crab lift -30 26
      crab support on/off
      crab hip on
      crab hip off
      crab flip
    """
    global CRAB_REACH_FEMUR_DEG, CRAB_REACH_TIBIA_DEG
    global CRAB_SUPPORT_FEMUR_DEG, CRAB_SUPPORT_TIBIA_DEG
    global CRAB_MIN_REACH_LIFT_FEMUR_DEG, CRAB_MIN_REACH_TUCK_TIBIA_DEG
    global CRAB_SUPPORT_PUSH_ENABLED
    global CRAB_BODY_SHIFT_ENABLED, CRAB_BODY_SHIFT_FEMUR_DEG, CRAB_BODY_SHIFT_TIBIA_DEG, CRAB_BODY_SHIFT_HIP_DEG, CRAB_BODY_SHIFT_HOLD
    global CRAB_USE_HIP_YAW, CRAB_STRAFE_DIRECTION_MULTIPLIER

    if len(parts) == 1:
        print_crab_settings()
        return

    sub = parts[1].lower()

    if sub == "power":
        if len(parts) not in [3, 4]:
            print("Usage: crab power 12 OR crab power 12 8")
            return
        try:
            reach = float(parts[2])
            support = float(parts[3]) if len(parts) == 4 else CRAB_SUPPORT_FEMUR_DEG
        except ValueError:
            print("Power values must be numbers.")
            return

        reach = max(4.0, min(20.0, reach))
        support = max(0.0, min(18.0, support))

        CRAB_REACH_FEMUR_DEG = reach
        CRAB_REACH_TIBIA_DEG = -0.8 * reach
        CRAB_SUPPORT_FEMUR_DEG = support
        CRAB_SUPPORT_TIBIA_DEG = -support

    elif sub == "lift":
        if len(parts) != 4:
            print("Usage: crab lift -30 26")
            return
        try:
            femur = float(parts[2])
            tibia = float(parts[3])
        except ValueError:
            print("Lift values must be numbers.")
            return

        CRAB_MIN_REACH_LIFT_FEMUR_DEG = max(-42.0, min(-18.0, femur))
        CRAB_MIN_REACH_TUCK_TIBIA_DEG = max(10.0, min(36.0, tibia))

    elif sub == "shift":
        if len(parts) != 3:
            print("Usage: crab shift 3")
            return
        try:
            shift = float(parts[2])
        except ValueError:
            print("Shift value must be a number.")
            return

        shift = max(0.0, min(10.0, shift))
        CRAB_BODY_SHIFT_FEMUR_DEG = shift
        CRAB_BODY_SHIFT_TIBIA_DEG = -shift
        CRAB_BODY_SHIFT_ENABLED = shift > 0.0

    elif sub == "hipamount":
        if len(parts) != 3:
            print("Usage: crab hipamount 16")
            return
        try:
            CRAB_HIP_DEG = float(parts[2])
        except ValueError:
            print("Hip amount must be a number.")
            return
        CRAB_HIP_DEG = max(0.0, min(8.0, CRAB_HIP_DEG))
        CRAB_USE_HIP_YAW = CRAB_HIP_DEG > 0.0

    elif sub == "bodyhip":
        if len(parts) != 3:
            print("Usage: crab bodyhip 12")
            return
        try:
            CRAB_BODY_SHIFT_HIP_DEG = float(parts[2])
        except ValueError:
            print("Body hip amount must be a number.")
            return
        CRAB_BODY_SHIFT_HIP_DEG = max(0.0, min(8.0, CRAB_BODY_SHIFT_HIP_DEG))
        CRAB_BODY_SHIFT_ENABLED = CRAB_BODY_SHIFT_HIP_DEG > 0.0

    elif sub == "shifthold":
        if len(parts) != 3:
            print("Usage: crab shifthold 0.25")
            return
        try:
            CRAB_BODY_SHIFT_HOLD = float(parts[2])
        except ValueError:
            print("Hold value must be a number.")
            return

        CRAB_BODY_SHIFT_HOLD = max(0.05, min(0.60, CRAB_BODY_SHIFT_HOLD))

    elif sub == "shifton":
        CRAB_BODY_SHIFT_ENABLED = True

    elif sub == "shiftoff":
        CRAB_BODY_SHIFT_ENABLED = False

    elif sub == "support":
        if len(parts) != 3:
            print("Usage: crab support on/off")
            return
        if parts[2].lower() == "on":
            CRAB_SUPPORT_PUSH_ENABLED = True
            if CRAB_SUPPORT_FEMUR_DEG == 0.0:
                CRAB_SUPPORT_FEMUR_DEG = 6.0
                CRAB_SUPPORT_TIBIA_DEG = -6.0
        elif parts[2].lower() == "off":
            CRAB_SUPPORT_PUSH_ENABLED = False
        else:
            print("Use on/off.")
            return

    elif sub == "hip":
        if len(parts) != 3:
            print("Usage: crab hip on/off")
            return
        if parts[2].lower() == "on":
            CRAB_USE_HIP_YAW = True
        elif parts[2].lower() == "off":
            CRAB_USE_HIP_YAW = False
        else:
            print("Use on/off.")
            return

    elif sub == "flip":
        CRAB_STRAFE_DIRECTION_MULTIPLIER *= -1.0

    else:
        print("Usage: crab / crab hipamount 16 / crab bodyhip 12 / crab shift 3 / crab power 8 / crab lift -30 26 / crab hip on/off / crab flip")
        return

    print_crab_settings()


def tripod_phase_targets(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
    phase: str,
) -> Dict[int, int]:
    targets = dict(READY_POSE)

    direction = normalize_direction(direction)
    lift_femur, lift_tibia = gait_lift_values()
    hip_swing_deg, support_push_deg = movement_profile(direction)

    for leg in lifted_legs:
        if direction == "forward":
            lift_hip = HIP_FORWARD_SIGN[leg] * hip_swing_deg

        elif direction == "backward":
            lift_hip = -HIP_FORWARD_SIGN[leg] * hip_swing_deg

        elif direction in ["left", "right"]:
            lift_hip = strafe_hip_for_leg(leg, direction, hip_swing_deg, lifted=True)

        elif direction in ["turn_left", "turn_right"]:
            lift_hip = turn_hip_for_leg(leg, direction, hip_swing_deg, lifted=True)

        else:
            lift_hip = 0.0

        if phase == "lift":
            targets.update(
                build_leg_offset_targets(
                    leg,
                    hip_deg=lift_hip,
                    femur_deg=lift_femur,
                    tibia_deg=lift_tibia,
                )
            )
        else:
            targets.update(
                build_leg_offset_targets(
                    leg,
                    hip_deg=lift_hip,
                    femur_deg=0.0,
                    tibia_deg=0.0,
                )
            )

    for leg in support_legs:
        if direction == "forward":
            support_hip = -HIP_FORWARD_SIGN[leg] * support_push_deg

        elif direction == "backward":
            support_hip = HIP_FORWARD_SIGN[leg] * support_push_deg

        elif direction in ["left", "right"]:
            support_hip = strafe_hip_for_leg(leg, direction, support_push_deg, lifted=False)

        elif direction in ["turn_left", "turn_right"]:
            support_hip = turn_hip_for_leg(leg, direction, support_push_deg, lifted=False)

        else:
            support_hip = 0.0

        targets.update(
            build_leg_offset_targets(
                leg,
                hip_deg=support_hip,
                femur_deg=0.0,
                tibia_deg=0.0,
            )
        )

    return targets


def interpolate_targets(start: Dict[int, int], end: Dict[int, int], steps: int) -> List[Dict[int, int]]:
    """
    Build small intermediate poses so the gait looks less like:
      step -> pause -> step -> pause
    and more like a continuous transition.

    This does not change the final movement size. It only breaks the motion into
    smaller goal updates.
    """
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


def move_targets_for_gait(bus: DynamixelBus, targets: Dict[int, int], speed: int, hold_delay: float):
    """
    Smooth gait move wrapper.

    With SMOOTH_GAIT on, the robot receives intermediate positions. This reduces
    sudden pose jumps and lets the next phase begin sooner.
    """
    global ACTIVE_GOALS

    if SMOOTH_GAIT:
        start = dict(ACTIVE_GOALS)
        frames = interpolate_targets(start, targets, SMOOTH_STEPS)

        for frame in frames:
            bus.move_many(frame, speed=speed)
            ACTIVE_GOALS = dict(frame)
            time.sleep(SMOOTH_STEP_DELAY)

        # Short hold only, because the interpolation already gave time to move.
        time.sleep(hold_delay)
    else:
        bus.move_many(targets, speed=speed)
        ACTIVE_GOALS = dict(targets)
        time.sleep(hold_delay)




def move_targets_for_side_strafe(
    bus: DynamixelBus,
    targets: Dict[int, int],
    speed: int,
    hold_delay: float,
    phase_label: str = "",
):
    """
    WControl34 A/D-only debug-step move wrapper.

    Important difference from lowering speed:
      speed all 22 still keeps the motor command speed at 22.
      debug-step mode simply sends smaller intermediate poses with short delays.

    New WControl34 behavior:
      - debug on/slow is faster than WControl33 to avoid long stall holds.
      - optional enter-step pauses AFTER each named phase is completed.
        Use: sidestrafe debug enter on
    """
    global ACTIVE_GOALS

    if SIDE_STRAFE_DEBUG_STEPS_ENABLED:
        start = dict(ACTIVE_GOALS)
        frames = interpolate_targets(start, targets, SIDE_STRAFE_DEBUG_STEPS)

        for idx, frame in enumerate(frames, start=1):
            bus.move_many(frame, speed=speed)
            ACTIVE_GOALS = dict(frame)

            if SIDE_STRAFE_DEBUG_PRINT_FRAMES:
                print(f"    debug frame {idx}/{SIDE_STRAFE_DEBUG_STEPS}")

            time.sleep(SIDE_STRAFE_DEBUG_STEP_DELAY)

        # Important: in debug mode we do NOT add a long extra hold.
        # Long holds were likely causing extra stall/overload during observation.
        time.sleep(min(hold_delay, 0.06))
    else:
        move_targets_for_gait(bus, targets, speed=speed, hold_delay=hold_delay)

    if SIDE_STRAFE_DEBUG_ENTER_STEP:
        label = phase_label if phase_label else "side-strafe phase"
        try:
            input(f"[ENTER STEP] {label} complete. Press Enter for next phase...")
        except KeyboardInterrupt:
            raise


def pose_set_leg_lift(pose: Dict[int, int], leg: str, femur_deg: float, tibia_deg: float):
    """
    Modify only femur/tibia for one leg in an existing pose.
    Hip is preserved, so the leg can be lifted without changing stride position.
    """
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")
    femur_id = joint_to_motor_id(femur_joint)
    tibia_id = joint_to_motor_id(tibia_joint)
    pose[femur_id] = offset_from_ready(femur_joint, femur_deg)
    pose[tibia_id] = offset_from_ready(tibia_joint, tibia_deg)


def pose_set_leg_down(pose: Dict[int, int], leg: str):
    """
    Modify only femur/tibia back to READY for one leg.
    Hip is preserved.
    """
    femur_joint = leg_part_to_joint(leg, "femur")
    tibia_joint = leg_part_to_joint(leg, "tibia")
    femur_id = joint_to_motor_id(femur_joint)
    tibia_id = joint_to_motor_id(tibia_joint)
    pose[femur_id] = READY_POSE[femur_id]
    pose[tibia_id] = READY_POSE[tibia_id]


def pose_set_leg_hip_ready(pose: Dict[int, int], leg: str):
    """
    Modify only hip back to READY for one leg.
    Femur/tibia are preserved.
    """
    hip_joint = leg_part_to_joint(leg, "hip")
    hip_id = joint_to_motor_id(hip_joint)
    pose[hip_id] = READY_POSE[hip_id]


def final_tripod_recenter(bus: DynamixelBus, direction: str):
    """
    End-of-walk cleanup that avoids the old visible behavior:
      all feet down -> all hips slide back to READY.

    Instead:
      1. lift tripod A while keeping its current hip positions
      2. recenter tripod A hips while those feet are in the air
      3. place tripod A down
      4. repeat for tripod B

    This is especially useful when cycles=1, where the final hip reset was obvious.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    lift_femur, lift_tibia = gait_lift_values()
    pose = dict(ACTIVE_GOALS)

    for group_name, tripod in [("A", TRIPOD_A), ("B", TRIPOD_B)]:
        # Lift tripod without changing hip position.
        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_LIFT"
        lift_pose = dict(pose)
        for leg in tripod:
            pose_set_leg_lift(lift_pose, leg, lift_femur, lift_tibia)
        move_targets_for_gait(bus, lift_pose, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY)
        print(f"{CURRENT_MODE}: sent")

        # Recenter hips while lifted.
        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_HIP_READY"
        hip_ready_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            pose_set_leg_hip_ready(hip_ready_pose, leg)
        move_targets_for_gait(bus, hip_ready_pose, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY)
        print(f"{CURRENT_MODE}: sent")

        # Place tripod back down with hip already centered.
        CURRENT_MODE = f"GAIT_{direction}_END_RECENTER_{group_name}_DOWN"
        down_pose = dict(ACTIVE_GOALS)
        for leg in tripod:
            pose_set_leg_down(down_pose, leg)
        move_targets_for_gait(bus, down_pose, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY)
        print(f"{CURRENT_MODE}: sent")

        pose = dict(ACTIVE_GOALS)

    # Make sure every joint is exactly back to READY, but by now hips should
    # already be centered, so this should be a tiny correction, not a visible drag.
    CURRENT_MODE = "READY_REFINED2K"
    move_targets_for_gait(bus, dict(READY_POSE), speed=GAIT_SPEED, hold_delay=GAIT_FINAL_READY_DELAY)
    ACTIVE_GOALS = dict(READY_POSE)

def print_gait_timing_settings():
    print()
    print("===================================================")
    print(" GAIT SMOOTHNESS SETTINGS")
    print("===================================================")
    print(f"SMOOTH_GAIT              = {SMOOTH_GAIT}")
    print(f"SMOOTH_STEPS             = {SMOOTH_STEPS}")
    print(f"SMOOTH_STEP_DELAY        = {SMOOTH_STEP_DELAY:.3f}s")
    print(f"GAIT_PHASE_DELAY         = {GAIT_PHASE_DELAY:.3f}s")
    print(f"GAIT_SETTLE_DELAY        = {GAIT_SETTLE_DELAY:.3f}s")
    print(f"GAIT_FINAL_READY_DELAY   = {GAIT_FINAL_READY_DELAY:.3f}s")
    print(f"GAIT_END_MODE            = {GAIT_END_MODE}")
    print(f"GAIT_END_RECENTER_DELAY  = {GAIT_END_RECENTER_DELAY:.3f}s")
    print(f"GAIT_PHASE_HEALTH        = {GAIT_PHASE_HEALTH}")
    print(f"GAIT_PRECHECK_EACH_PHASE = {GAIT_PRECHECK_EACH_PHASE}")
    print("===================================================")


def print_walk_lift_settings():
    lift_femur, lift_tibia = gait_lift_values()

    print()
    print("===================================================")
    print(" WALKING LIFT PROFILE")
    print("===================================================")
    print(f"USE_WALK_LIFT_PROFILE = {USE_WALK_LIFT_PROFILE}")
    print(f"GAIT_LIFT_LEVEL       = {GAIT_LIFT_LEVEL}  # used only if profile is off")
    print(f"Walk femur lift       = {WALK_LIFT_FEMUR_DEG:+.1f} deg")
    print(f"Walk tibia tuck       = {WALK_LIFT_TIBIA_DEG:+.1f} deg")
    print(f"Actual walking lift   = femur {lift_femur:+.1f} deg, tibia {lift_tibia:+.1f} deg")
    print("---------------------------------------------------")
    print("Meaning:")
    print("  femur more negative = leg lifts higher from the ground")
    print("  tibia positive      = foot tucks inward/retracts")
    print("  too much tibia tuck = wasted folding movement and slower step, but helps clearance")
    print("---------------------------------------------------")
    print("Presets:")
    for name, values in WALK_LIFT_PRESETS.items():
        print(f"  walklift {name:<5} -> femur {values['femur']:+.1f}, tibia {values['tibia']:+.1f}")
    print("===================================================")


def action_walk_lift(parts: List[str]):
    global USE_WALK_LIFT_PROFILE, WALK_LIFT_FEMUR_DEG, WALK_LIFT_TIBIA_DEG, GAIT_LIFT_LEVEL

    if len(parts) == 1:
        print_walk_lift_settings()
        return

    sub = parts[1].lower()

    if sub in ["on", "profile", "custom"]:
        USE_WALK_LIFT_PROFILE = True
        print_walk_lift_settings()
        return

    if sub in ["off", "level", "levels"]:
        USE_WALK_LIFT_PROFILE = False
        print_walk_lift_settings()
        return

    if sub in WALK_LIFT_PRESETS:
        USE_WALK_LIFT_PROFILE = True
        WALK_LIFT_FEMUR_DEG = WALK_LIFT_PRESETS[sub]["femur"]
        WALK_LIFT_TIBIA_DEG = WALK_LIFT_PRESETS[sub]["tibia"]
        print_walk_lift_settings()
        return

    if sub == "femur" and len(parts) == 3:
        try:
            value = float(parts[2])
        except ValueError:
            print("Usage: walklift femur -32")
            return
        # Clamp to cautious walking range.
        WALK_LIFT_FEMUR_DEG = max(-46.0, min(-18.0, value))
        USE_WALK_LIFT_PROFILE = True
        print_walk_lift_settings()
        return

    if sub == "tibia" and len(parts) == 3:
        try:
            value = float(parts[2])
        except ValueError:
            print("Usage: walklift tibia 12")
            return
        # Keep tibia tuck small for the new walking profile.
        WALK_LIFT_TIBIA_DEG = max(0.0, min(36.0, value))
        USE_WALK_LIFT_PROFILE = True
        print_walk_lift_settings()
        return

    if sub == "both" and len(parts) == 4:
        try:
            femur = float(parts[2])
            tibia = float(parts[3])
        except ValueError:
            print("Usage: walklift both -32 12")
            return
        WALK_LIFT_FEMUR_DEG = max(-46.0, min(-18.0, femur))
        WALK_LIFT_TIBIA_DEG = max(0.0, min(36.0, tibia))
        USE_WALK_LIFT_PROFILE = True
        print_walk_lift_settings()
        return

    if sub == "level" and len(parts) == 3:
        try:
            level = int(parts[2])
        except ValueError:
            print("Usage: walklift level 6")
            return
        if level not in LIFT_LEVELS:
            print(f"Level must be one of: {sorted(LIFT_LEVELS.keys())}")
            return
        GAIT_LIFT_LEVEL = level
        USE_WALK_LIFT_PROFILE = False
        print_walk_lift_settings()
        return

    print("Usage:")
    print("  walklift              = show walking lift settings")
    print("  walklift high1        = extra-high gait lift: -38 / +26")
    print("  walklift high2        = stronger lift: -40 / +28")
    print("  walklift high3        = very high lift: -42 / +30")
    print("  walklift max          = maximum test lift: -44 / +32")
    print("  walklift old6         = old full tibia tuck behavior: -28 / +28")
    print("  walklift high         = high femur clearance: -38 / +24")
    print("  walklift low          = gentler test: -30 / +16")
    print("  walklift femur -34    = customize femur lift")
    print("  walklift tibia 22     = customize tibia tuck")
    print("  walklift both -40 28  = customize both")
    print("  walklift level 6      = disable profile and use LIFT_LEVELS[6]")
    print("  liftfind high2        = set preset and run one forward gait cycle")



def action_lift_find(bus: DynamixelBus, parts: List[str]):
    """
    Convenience command for gait lift testing.

    Usage:
      liftfind test1
      liftfind test2
      liftfind test3
      liftfind old6
      liftfind all

    It sets the walklift preset, runs one forward gait cycle, then asks you to
    visually judge clearance. It does not automatically run strafe/turn.
    """
    if len(parts) != 2:
        print("Usage: liftfind high1 / high2 / high3 / max / old6 / all")
        return

    choice = parts[1].lower()

    if choice == "all":
        for preset in ["high1", "high2", "high3", "max"]:
            print()
            print("===================================================")
            print(f" LIFTFIND ALL: TESTING {preset.upper()}")
            print("===================================================")
            action_walk_lift(["walklift", preset])
            action_gait_cycle(bus, "forward", cycles=1)
            print("Use your eyes: did the swing legs clear the ground? Then type r before the next manual test if needed.")
        return

    if choice not in WALK_LIFT_PRESETS:
        print(f"Unknown liftfind preset: {choice}")
        print(f"Available: {', '.join(WALK_LIFT_PRESETS.keys())}, all")
        return

    print()
    print("===================================================")
    print(f" LIFTFIND: {choice.upper()}")
    print("===================================================")
    print("This sets the walking lift preset and runs one FORWARD gait cycle.")
    print("Focus only on foot clearance. Ignore strafe/turn for now.")
    print("===================================================")

    action_walk_lift(["walklift", choice])
    action_gait_cycle(bus, "forward", cycles=1)


def apply_smooth_preset(name: str):
    """Apply gait timing presets. These only change timing/smoothness, not geometry."""
    global SMOOTH_GAIT, SMOOTH_STEPS, SMOOTH_STEP_DELAY
    global GAIT_PHASE_DELAY, GAIT_SETTLE_DELAY, GAIT_FINAL_READY_DELAY
    global GAIT_PHASE_HEALTH, GAIT_PRECHECK_EACH_PHASE

    name = name.lower().strip()

    if name in ["walk", "flow", "smoothwalk"]:
        # Best current balance: less choke than 6 steps, smoother than 2 steps.
        SMOOTH_GAIT = True
        SMOOTH_STEPS = 3
        SMOOTH_STEP_DELAY = 0.025
        GAIT_PHASE_DELAY = 0.08
        GAIT_SETTLE_DELAY = 0.05
        GAIT_FINAL_READY_DELAY = 0.25
        GAIT_PHASE_HEALTH = False
        GAIT_PRECHECK_EACH_PHASE = False
        return True

    if name in ["stable", "safe"]:
        # Slightly more waiting. Use this if the robot shakes or misses footing.
        SMOOTH_GAIT = True
        SMOOTH_STEPS = 4
        SMOOTH_STEP_DELAY = 0.030
        GAIT_PHASE_DELAY = 0.12
        GAIT_SETTLE_DELAY = 0.07
        GAIT_FINAL_READY_DELAY = 0.30
        GAIT_PHASE_HEALTH = False
        GAIT_PRECHECK_EACH_PHASE = False
        return True

    if name in ["quick", "fast"]:
        # Less pause, but less smooth. Use only if walk preset still feels too delayed.
        SMOOTH_GAIT = True
        SMOOTH_STEPS = 2
        SMOOTH_STEP_DELAY = 0.025
        GAIT_PHASE_DELAY = 0.06
        GAIT_SETTLE_DELAY = 0.03
        GAIT_FINAL_READY_DELAY = 0.20
        GAIT_PHASE_HEALTH = False
        GAIT_PRECHECK_EACH_PHASE = False
        return True

    if name in ["debug", "safecheck"]:
        # More checks and longer pauses. Use when testing risky new geometry.
        SMOOTH_GAIT = True
        SMOOTH_STEPS = 4
        SMOOTH_STEP_DELAY = 0.045
        GAIT_PHASE_DELAY = 0.18
        GAIT_SETTLE_DELAY = 0.10
        GAIT_FINAL_READY_DELAY = 0.35
        GAIT_PHASE_HEALTH = True
        GAIT_PRECHECK_EACH_PHASE = True
        return True

    return False




def action_finish_mode(mode: str):
    """Set how the robot returns to ready after the final gait cycle."""
    global GAIT_END_MODE

    mode = mode.lower().strip()
    aliases = {
        "tripod": "tripod",
        "recenter": "tripod",
        "smooth": "tripod",
        "direct": "direct",
        "old": "direct",
        "ready": "direct",
        "hold": "hold",
        "stay": "hold",
        "none": "hold",
    }

    if mode not in aliases:
        print("Usage: smooth end tripod / smooth end direct / smooth end hold")
        return

    GAIT_END_MODE = aliases[mode]
    print_gait_timing_settings()

def action_smooth(parts: List[str]):
    global SMOOTH_GAIT, SMOOTH_STEPS, SMOOTH_STEP_DELAY
    global GAIT_PHASE_DELAY, GAIT_SETTLE_DELAY, GAIT_PHASE_HEALTH, GAIT_PRECHECK_EACH_PHASE

    if len(parts) == 1:
        print_gait_timing_settings()
        return

    sub = parts[1].lower()

    if sub in ["preset", "mode"] and len(parts) == 3:
        if not apply_smooth_preset(parts[2]):
            print("Unknown smooth preset. Use: walk / stable / quick / debug")
            return
        print_gait_timing_settings()
        return

    # Shortcut: smooth walk / smooth stable / smooth quick / smooth debug
    if sub in ["walk", "flow", "smoothwalk", "stable", "safe", "quick", "fast", "debug", "safecheck"]:
        apply_smooth_preset(sub)
        print_gait_timing_settings()
        return

    if sub in ["on", "true", "1"]:
        SMOOTH_GAIT = True
        print_gait_timing_settings()
        return

    if sub in ["off", "false", "0"]:
        SMOOTH_GAIT = False
        print_gait_timing_settings()
        return

    if sub == "steps" and len(parts) == 3:
        try:
            SMOOTH_STEPS = max(1, min(12, int(parts[2])))
        except ValueError:
            print("Usage: smooth steps 4")
            return
        print_gait_timing_settings()
        return

    if sub == "stepdelay" and len(parts) == 3:
        try:
            SMOOTH_STEP_DELAY = max(0.005, min(0.20, float(parts[2])))
        except ValueError:
            print("Usage: smooth stepdelay 0.045")
            return
        print_gait_timing_settings()
        return

    if sub == "hold" and len(parts) == 3:
        try:
            value = max(0.03, min(1.00, float(parts[2])))
        except ValueError:
            print("Usage: smooth hold 0.18")
            return
        GAIT_PHASE_DELAY = value
        print_gait_timing_settings()
        return

    if sub == "settle" and len(parts) == 3:
        try:
            value = max(0.03, min(1.00, float(parts[2])))
        except ValueError:
            print("Usage: smooth settle 0.10")
            return
        GAIT_SETTLE_DELAY = value
        print_gait_timing_settings()
        return

    if sub in ["end", "finish", "stopmode"] and len(parts) == 3:
        action_finish_mode(parts[2])
        return

    if sub == "phasehealth" and len(parts) == 3:
        value = parts[2].lower()
        GAIT_PHASE_HEALTH = value in ["on", "true", "1", "yes"]
        print_gait_timing_settings()
        return

    if sub == "phasecheck" and len(parts) == 3:
        value = parts[2].lower()
        GAIT_PRECHECK_EACH_PHASE = value in ["on", "true", "1", "yes"]
        print_gait_timing_settings()
        return

    print("Usage:")
    print("  smooth")
    print("  smooth on / smooth off")
    print("  smooth walk        = recommended smooth-walk preset")
    print("  smooth stable      = safer, slightly slower preset")
    print("  smooth quick       = faster, less pause preset")
    print("  smooth debug       = more checks and longer pauses")
    print("  smooth steps 3")
    print("  smooth stepdelay 0.025")
    print("  smooth hold 0.08")
    print("  smooth settle 0.05")
    print("  smooth end tripod   = recenter hips one tripod at a time")
    print("  smooth end direct   = old final all-joint READY reset")
    print("  smooth end hold     = stay in final gait stance, no auto READY")
    print("  smooth phasehealth off")
    print("  smooth phasecheck off")



def tripod_pre_lift_targets(
    lifted_legs: List[str],
    support_legs: List[str],
    direction: str,
) -> Dict[int, int]:
    """
    V6-style split lift phase.

    Important difference from the previous refined script:
      - Previous walking gait combined hip swing + femur/tibia lift in one target.
      - That can make the foot look like it is sweeping low instead of lifting first.
      - This phase lifts the tripod vertically first while keeping lifted hips centered.

    Then the next phase swings the lifted tripod while it is already in the air.
    This should look closer to the good-looking ending recenter motion.
    """
    targets = dict(READY_POSE)
    direction = normalize_direction(direction)
    lift_femur, lift_tibia = gait_lift_values()
    _, support_push_deg = movement_profile(direction)

    # Lifted tripod: lift only, no hip swing yet.
    for leg in lifted_legs:
        targets.update(
            build_leg_offset_targets(
                leg,
                hip_deg=0.0,
                femur_deg=lift_femur,
                tibia_deg=lift_tibia,
            )
        )

    # Support tripod may begin pulling while lifted tripod clears the ground.
    for leg in support_legs:
        if direction == "forward":
            support_hip = -HIP_FORWARD_SIGN[leg] * support_push_deg
        elif direction == "backward":
            support_hip = HIP_FORWARD_SIGN[leg] * support_push_deg
        elif direction == "left":
            support_hip = -HIP_STRAFE_SIGN[leg] * support_push_deg
        elif direction == "right":
            support_hip = HIP_STRAFE_SIGN[leg] * support_push_deg
        elif direction == "turn_left":
            support_hip = HIP_TURN_SIGN[leg] * support_push_deg
        elif direction == "turn_right":
            support_hip = -HIP_TURN_SIGN[leg] * support_push_deg
        else:
            support_hip = 0.0

        targets.update(
            build_leg_offset_targets(
                leg,
                hip_deg=support_hip,
                femur_deg=0.0,
                tibia_deg=0.0,
            )
        )

    return targets


def side_strafe_direction_sign(direction: str) -> float:
    """+1 for left, -1 for right, with runtime flip support."""
    direction = normalize_direction(direction)
    if direction == "left":
        return SIDE_STRAFE_DIRECTION_MULTIPLIER * 1.0
    if direction == "right":
        return SIDE_STRAFE_DIRECTION_MULTIPLIER * -1.0
    return 0.0


def side_strafe_side_sign(leg: str, direction: str) -> float:
    """
    No-hip A/D femur+tibia side sign.

    For LEFT strafe:
      left legs  = outward/left lean
      right legs = inward/opposite push

    For RIGHT strafe, signs mirror.

    This does not move the coxa/hip at all. It only changes femur/tibia offsets.
    """
    direction = normalize_direction(direction)

    if direction == "left":
        base = +1.0 if leg in LEFT_LEGS else -1.0
    elif direction == "right":
        base = -1.0 if leg in LEFT_LEGS else +1.0
    else:
        base = 0.0

    return SIDE_STRAFE_DIRECTION_MULTIPLIER * base


def side_strafe_leg_offsets(leg: str, direction: str, role: str) -> Tuple[float, float, float]:
    """
    WControl21 A/D no-hip tripod side-step offsets.

    Roles:
      ready         = neutral ready
      lift          = vertical lift only
      reach_lifted  = lifted leg reaches/leans sideways while off the ground
      reach_ground  = landed at side-reach/lean position
      pull_ground   = planted tripod pulls body sideways while other tripod is lifted

    Critical rule:
      hip_deg is ALWAYS 0.0 for A/D strafe.
      All coxa/hip motors stay at READY_POSE.
    """
    side = side_strafe_side_sign(leg, direction)

    if role == "lift":
        # Lift while already beginning to extend the tibia outward.
        # This avoids the old shrink-inward motion that cancelled the strafe.
        return 0.0, SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG

    if role == "reach_lifted":
        # Foot is in the air: keep it high, and reach OUTWARD.
        femur = SIDE_STRAFE_LIFT_FEMUR_DEG + side * SIDE_STRAFE_FEMUR_REACH_DEG
        tibia = SIDE_STRAFE_LIFT_TIBIA_DEG + side * SIDE_STRAFE_TIBIA_REACH_DEG
        return 0.0, femur, tibia

    if role == "reach_ground":
        # Foot lands OUTWARD, not tucked.
        return 0.0, side * SIDE_STRAFE_FEMUR_REACH_DEG, side * SIDE_STRAFE_TIBIA_REACH_DEG

    if role == "pull_ground":
        # Planted tripod pulls the body sideways after the foot is already down.
        # This is the opposite of reach_ground, so the foot acts like an anchor.
        return 0.0, side * SIDE_STRAFE_FEMUR_PULL_DEG, side * SIDE_STRAFE_TIBIA_PULL_DEG

    return 0.0, 0.0, 0.0


def side_strafe_phase_support_offsets(leg: str, direction: str, active_tripod: List[str], phase: str) -> Tuple[float, float, float]:
    """
    Phase-specific standing-tripod support tuning.

    Normal support uses pull_ground. WControl35 only boosts this exact debug phase:
        SIDE_<direction>_A_REACH_B_PULL

    For left strafe in that phase, B tripod is standing: FR + ML + RR.
      FR/RR femur extend more to push the body farther left.
      ML femur contracts more so the left-side support hooks/pulls toward the target.
    """
    if not SIDE_STRAFE_PHASE_BOOST_ENABLED:
        return side_strafe_leg_offsets(leg, direction, "pull_ground")

    # Boost only when tripod A is active/reaching and tripod B is support.
    if phase != "reach_pull" or active_tripod != CRAB_SECOND_TRIPOD:
        return side_strafe_leg_offsets(leg, direction, "pull_ground")

    side = side_strafe_side_sign(leg, direction)

    if leg in ["ML", "MR"]:
        # Left strafe: ML side=+ => femur negative/contract.
        # Right strafe mirror: MR/ML signs reverse naturally.
        femur = -side * SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG
        tibia = side * SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG
    else:
        # Left strafe: FR/RR side=- => femur positive/extend.
        femur = -side * SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG
        tibia = side * SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG

    return 0.0, femur, tibia

def build_side_strafe_targets(
    active_tripod: List[str],
    other_tripod: List[str],
    direction: str,
    phase: str,
) -> Dict[int, int]:
    """
    WControl22 A/D target builder.

    Main rule requested by user:
      while one tripod is lifted, the other tripod must already lean/push sideways.

    phase:
      up_pull       active tripod lifts; support tripod pulls/leans sideways
      reach_pull    active tripod reaches while lifted; support tripod keeps pulling
      down_pull     active tripod lands; support tripod still pulls until weight transfers
      ready         both tripods neutral/ready

    Hips are always held at READY because side strafe is no-hip by design.
    """
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
    """Recenter A/D by lifting one tripod at a time so feet do not drag."""
    global ACTIVE_GOALS, CURRENT_MODE

    for idx, legs in enumerate([CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD], start=1):
        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_UP"
        targets = dict(READY_POSE)
        for leg in legs:
            h, f, t = side_strafe_leg_offsets(leg, direction, "lift")
            targets.update(build_leg_offset_targets(leg, h, f, t))
        ACTIVE_GOALS = dict(targets)
        move_targets_for_side_strafe(bus, targets, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY, phase_label=CURRENT_MODE)
        print(f"{CURRENT_MODE}: sent")

        CURRENT_MODE = f"SIDE_STRAFE_END_{idx}_READY"
        targets = dict(READY_POSE)
        ACTIVE_GOALS = dict(targets)
        move_targets_for_gait(bus, targets, speed=GAIT_SPEED, hold_delay=GAIT_END_RECENTER_DELAY)
        print(f"{CURRENT_MODE}: sent")

    CURRENT_MODE = "READY_REFINED2K"
    ACTIVE_GOALS = dict(READY_POSE)
    move_targets_for_gait(bus, dict(READY_POSE), speed=GAIT_SPEED, hold_delay=GAIT_FINAL_READY_DELAY)



def print_sideflow_settings():
    print()
    print("===================================================")
    print(" SIDE STRAFE FLOW SETTINGS")
    print("===================================================")
    print(f"SIDE_STRAFE_FLOW_MODE         = {SIDE_STRAFE_FLOW_MODE}")
    print(f"SIDE_STRAFE_FLOW_HOLD         = {SIDE_STRAFE_FLOW_HOLD:.3f}s")
    print(f"SIDE_STRAFE_FLOW_TINY_HOLD    = {SIDE_STRAFE_FLOW_TINY_HOLD:.3f}s")
    print(f"SIDE_STRAFE_FLOW_PRINT_PHASES = {SIDE_STRAFE_FLOW_PRINT_PHASES}")
    print("---------------------------------------------------")
    print("Meaning:")
    print("  on   = A/D phases are sent back-to-back with no extra hold pause")
    print("  tiny = small visible pause only, useful if movement is too sudden")
    print("  off  = original WControl23 hold/settle timing")
    print("---------------------------------------------------")
    print("Commands:")
    print("  sideflow")
    print("  sideflow on")
    print("  sideflow off")
    print("  sideflow tiny")
    print("  sideflow hold 0.02")
    print("  sideflow print on/off")
    print("===================================================")


def action_sideflow(parts: List[str]):
    global SIDE_STRAFE_FLOW_MODE, SIDE_STRAFE_FLOW_HOLD
    global SIDE_STRAFE_FLOW_PRINT_PHASES

    if len(parts) == 1:
        print_sideflow_settings()
        return

    sub = parts[1].lower()

    if sub in ["on", "enable", "flow", "smooth"]:
        SIDE_STRAFE_FLOW_MODE = True
        SIDE_STRAFE_FLOW_HOLD = 0.0
        print_sideflow_settings()
        return

    if sub in ["tiny", "small"]:
        SIDE_STRAFE_FLOW_MODE = True
        SIDE_STRAFE_FLOW_HOLD = SIDE_STRAFE_FLOW_TINY_HOLD
        print_sideflow_settings()
        return

    if sub in ["off", "disable", "original", "w23"]:
        SIDE_STRAFE_FLOW_MODE = False
        print_sideflow_settings()
        return

    if sub == "hold" and len(parts) >= 3:
        try:
            SIDE_STRAFE_FLOW_HOLD = max(0.0, min(0.10, float(parts[2])))
            SIDE_STRAFE_FLOW_MODE = True
        except ValueError:
            print("Usage: sideflow hold 0.02")
            return
        print_sideflow_settings()
        return

    if sub == "print" and len(parts) >= 3:
        SIDE_STRAFE_FLOW_PRINT_PHASES = parts[2].lower() in ["on", "true", "1", "yes"]
        print_sideflow_settings()
        return

    print("Usage: sideflow / sideflow on / sideflow off / sideflow tiny / sideflow hold 0.02")


def action_side_strafe_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    """
    WControl22: replacement for a/d only.
    Movement idea requested by user:
      one tripod lifts and waits/reaches while the opposite tripod already leans/pulls sideways.
      Then the lifted tripod lands, becomes support, and the other tripod lifts.
    """
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)
    if direction not in ["left", "right"]:
        print("Side strafe only supports left/right.")
        return

    cycles = max(1, min(5, int(cycles)))

    print()
    print("===================================================")
    print(f" ACTION: WCONTROL23 NO-HIP LIFT-OUT + PLANTED-PULL STRAFE {direction.upper()} x{cycles}")
    print("===================================================")
    print("a/d only: w/s/q/e are unchanged.")
    print(f"Tripod B first: {CRAB_FIRST_TRIPOD}")
    print(f"Tripod A second: {CRAB_SECOND_TRIPOD}")
    print(f"Hip reach/push: 0.0 / 0.0  # locked, no hip movement")
    print(f"Femur/tibia reach-out: {SIDE_STRAFE_FEMUR_REACH_DEG:+.1f} / {SIDE_STRAFE_TIBIA_REACH_DEG:+.1f}")
    print(f"Femur/tibia planted-pull: {SIDE_STRAFE_FEMUR_PULL_DEG:+.1f} / {SIDE_STRAFE_TIBIA_PULL_DEG:+.1f}")
    print(f"Lift-out: femur {SIDE_STRAFE_LIFT_FEMUR_DEG:+.1f}, tibia {SIDE_STRAFE_LIFT_TIBIA_DEG:+.1f}")
    print(f"Debug-step mode: {SIDE_STRAFE_DEBUG_STEPS_ENABLED}, steps={SIDE_STRAFE_DEBUG_STEPS}, stepDelay={SIDE_STRAFE_DEBUG_STEP_DELAY:.3f}s, enterStep={SIDE_STRAFE_DEBUG_ENTER_STEP}")
    print(f"Flow mode: {SIDE_STRAFE_FLOW_MODE}, flow hold={SIDE_STRAFE_FLOW_HOLD:.3f}s  # A/D only")
    print("Sequence: B lift OUT -> B reach OUT -> B land OUT -> B PULLS while A lift/reach/land -> A PULLS on next half-step.")
    print("If reversed: sidestrafe flip")
    print("If too small: sidestrafe reach 14 -14 OR sidestrafe push 16 -16")
    print("===================================================")

    for i in range(cycles):
        print()
        print(f"--- SIDE STRAFE CYCLE {i + 1}/{cycles} ---")
        if not pre_motion_check(bus):
            return

        # B = ML + FR + RR first, exactly as requested.
        # IMPORTANT: the support tripod pulls during the lift/reach/down of the other tripod.
        phases = [
            (f"SIDE_{direction}_B_UP_A_PULL", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "up_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_B_REACH_A_PULL", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "reach_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_B_DOWN_A_PULL", CRAB_FIRST_TRIPOD, CRAB_SECOND_TRIPOD, "down_pull", SIDE_STRAFE_SETTLE),

            (f"SIDE_{direction}_A_UP_B_PULL", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "up_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_A_REACH_B_PULL", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "reach_pull", SIDE_STRAFE_HOLD),
            (f"SIDE_{direction}_A_DOWN_B_PULL", CRAB_SECOND_TRIPOD, CRAB_FIRST_TRIPOD, "down_pull", SIDE_STRAFE_SETTLE),
        ]

        for mode_name, active, other, phase, delay in phases:
            CURRENT_MODE = mode_name
            targets = build_side_strafe_targets(active, other, direction, phase)
            ACTIVE_GOALS = dict(targets)

            # WControl71: for A/D flow mode, remove the extra hold pauses between
            # side-strafe phases. move_many() still sends all motor commands; this
            # only removes the added sleep after each phase.
            effective_delay = SIDE_STRAFE_FLOW_HOLD if SIDE_STRAFE_FLOW_MODE else delay

            move_targets_for_side_strafe(
                bus,
                targets,
                speed=GAIT_SPEED,
                hold_delay=effective_delay,
                phase_label=CURRENT_MODE,
            )

            if GAIT_PHASE_HEALTH:
                print_health(bus, CURRENT_MODE)
            elif SIDE_STRAFE_FLOW_PRINT_PHASES:
                print(f"{CURRENT_MODE}: sent")

            # Movement stats are useful, but they require 18-motor reads and will
            # create pauses. Keep movestats off for smooth whole-move testing.
            print_movement_stats(bus, CURRENT_MODE, active=active, support=other)

        print_health(bus, f"AFTER SIDE STRAFE CYCLE {i + 1} {direction}")

    print("Final mode: SIDE STRAFE TRIPOD RECENTER.")
    side_strafe_final_recenter(bus, direction)
    print_status(bus)
    time.sleep(0.25)
    print_health(bus, f"AFTER SIDE STRAFE {direction}")


def print_side_strafe_settings():
    print()
    print("===================================================")
    print(" WCONTROL34 RESTORED W23 A/D NO-HIP LIFT-OUT + PLANTED-PULL SETTINGS")
    print("===================================================")
    print(f"Direction multiplier = {SIDE_STRAFE_DIRECTION_MULTIPLIER:+.1f}")
    print(f"Hip reach / push     = 0.0 / 0.0  # LOCKED, no hip movement")
    print(f"Femur/tibia reach    = {SIDE_STRAFE_FEMUR_REACH_DEG:+.1f} / {SIDE_STRAFE_TIBIA_REACH_DEG:+.1f}")
    print(f"Femur/tibia pull     = {SIDE_STRAFE_FEMUR_PULL_DEG:+.1f} / {SIDE_STRAFE_TIBIA_PULL_DEG:+.1f}")
    print(f"Lift femur/tibia     = {SIDE_STRAFE_LIFT_FEMUR_DEG:+.1f} / {SIDE_STRAFE_LIFT_TIBIA_DEG:+.1f}")
    print(f"Hold/settle          = {SIDE_STRAFE_HOLD:.2f}s / {SIDE_STRAFE_SETTLE:.2f}s")
    print(f"Debug-step mode      = {SIDE_STRAFE_DEBUG_STEPS_ENABLED}, steps={SIDE_STRAFE_DEBUG_STEPS}, stepDelay={SIDE_STRAFE_DEBUG_STEP_DELAY:.3f}s, enterStep={SIDE_STRAFE_DEBUG_ENTER_STEP}")
    print(f"Phase boost A_REACH  = {SIDE_STRAFE_PHASE_BOOST_ENABLED}, FR/RR femur/tibia {SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG:+.1f}/{SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG:+.1f}, ML/MR femur/tibia {SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG:+.1f}/{SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG:+.1f}")
    print("Commands:")
    print("  sidestrafe")
    print("  sidestrafe good          = restore the working WControl23 values")
    print("  sidestrafe gentle        = lower-force debug preset")
    print("  sidestrafe stronger      = slightly bigger, not aggressive")
    print("  sidestrafe debug on      = fast visible micro-steps, motor speed unchanged")
    print("  sidestrafe debug slow    = moderate visible debug steps, not too slow")
    print("  sidestrafe debug enter on/off = pause after each named phase")
    print("  sidestrafe debug off     = normal direct W23 movement")
    print("  sidestrafe debug steps 12")
    print("  sidestrafe debug delay 0.03")
    print("  sidestrafe flip")
    print("  sidestrafe reach 6 -14")
    print("  sidestrafe pull -5 12")
    print("  sidestrafe lift 34 -6    = femur -34, tibia -6 outward")
    print("  sidestrafe hold 0.25")
    print("  sidestrafe settle 0.12")
    print("===================================================")


def action_side_strafe_settings(parts: List[str]):
    global SIDE_STRAFE_DIRECTION_MULTIPLIER
    global SIDE_STRAFE_HIP_REACH_DEG, SIDE_STRAFE_HIP_PUSH_DEG
    global SIDE_STRAFE_FEMUR_REACH_DEG, SIDE_STRAFE_TIBIA_REACH_DEG
    global SIDE_STRAFE_FEMUR_PULL_DEG, SIDE_STRAFE_TIBIA_PULL_DEG
    global SIDE_STRAFE_LIFT_FEMUR_DEG, SIDE_STRAFE_LIFT_TIBIA_DEG
    global SIDE_STRAFE_HOLD, SIDE_STRAFE_SETTLE
    global SIDE_STRAFE_DEBUG_STEPS_ENABLED, SIDE_STRAFE_DEBUG_STEPS, SIDE_STRAFE_DEBUG_STEP_DELAY, SIDE_STRAFE_DEBUG_PRINT_FRAMES, SIDE_STRAFE_DEBUG_ENTER_STEP
    global SIDE_STRAFE_PHASE_BOOST_ENABLED, SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG, SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG
    global SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG, SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG

    if len(parts) == 1:
        print_side_strafe_settings()
        return

    sub = parts[1].lower()

    try:
        if sub == "debug":
            if len(parts) == 2:
                print_side_strafe_settings()
                return

            mode = parts[2].lower()

            if mode in ["on", "enable", "normal"]:
                # Faster than WControl33. Enough to observe without long stall holds.
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = True
                SIDE_STRAFE_DEBUG_STEPS = 6
                SIDE_STRAFE_DEBUG_STEP_DELAY = 0.030
                SIDE_STRAFE_DEBUG_PRINT_FRAMES = False

            elif mode in ["slow", "slomo"]:
                # Moderate slow motion, not ultra-slow.
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = True
                SIDE_STRAFE_DEBUG_STEPS = 8
                SIDE_STRAFE_DEBUG_STEP_DELAY = 0.045
                SIDE_STRAFE_DEBUG_PRINT_FRAMES = False

            elif mode in ["ultra", "veryslow"]:
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = True
                SIDE_STRAFE_DEBUG_STEPS = 10
                SIDE_STRAFE_DEBUG_STEP_DELAY = 0.060
                SIDE_STRAFE_DEBUG_PRINT_FRAMES = True

            elif mode in ["off", "disable", "fast"]:
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = False
                SIDE_STRAFE_DEBUG_PRINT_FRAMES = False
                SIDE_STRAFE_DEBUG_ENTER_STEP = False

            elif mode in ["enter", "pause", "manual"] and len(parts) == 4:
                value = parts[3].lower()
                SIDE_STRAFE_DEBUG_ENTER_STEP = value in ["on", "true", "1", "yes", "enable"]

            elif mode in ["steps", "step"] and len(parts) == 4:
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = True
                SIDE_STRAFE_DEBUG_STEPS = int(max(2, min(20, int(parts[3]))))

            elif mode in ["delay", "stepdelay"] and len(parts) == 4:
                SIDE_STRAFE_DEBUG_STEPS_ENABLED = True
                SIDE_STRAFE_DEBUG_STEP_DELAY = max(0.005, min(0.150, float(parts[3])))

            elif mode in ["print", "frames"] and len(parts) == 4:
                value = parts[3].lower()
                SIDE_STRAFE_DEBUG_PRINT_FRAMES = value in ["on", "true", "1", "yes"]

            else:
                print("Usage:")
                print("  sidestrafe debug on")
                print("  sidestrafe debug slow")
                print("  sidestrafe debug ultra")
                print("  sidestrafe debug off")
                print("  sidestrafe debug steps 12")
                print("  sidestrafe debug delay 0.03")
                print("  sidestrafe debug print on")
                return

        elif sub == "phaseboost":
            if len(parts) == 3:
                value = parts[2].lower()
                SIDE_STRAFE_PHASE_BOOST_ENABLED = value in ["on", "true", "1", "yes", "enable"]

            elif len(parts) == 6:
                SIDE_STRAFE_PHASE_BOOST_ENABLED = True
                SIDE_STRAFE_PHASE_BOOST_FEMUR_DEG = float(parts[2])
                SIDE_STRAFE_PHASE_BOOST_TIBIA_DEG = float(parts[3])
                SIDE_STRAFE_PHASE_BOOST_MIDDLE_FEMUR_DEG = float(parts[4])
                SIDE_STRAFE_PHASE_BOOST_MIDDLE_TIBIA_DEG = float(parts[5])

            else:
                print("Usage: sidestrafe phaseboost on/off OR sidestrafe phaseboost 9 12 8 12")
                return

        elif sub == "flip":
            SIDE_STRAFE_DIRECTION_MULTIPLIER *= -1.0

        elif sub in ["good", "w23", "reset"]:
            # Restores the exact side-strafe shape that you said worked beautifully.
            SIDE_STRAFE_FEMUR_REACH_DEG = 6.0
            SIDE_STRAFE_TIBIA_REACH_DEG = -14.0
            SIDE_STRAFE_FEMUR_PULL_DEG = -5.0
            SIDE_STRAFE_TIBIA_PULL_DEG = 12.0
            SIDE_STRAFE_LIFT_FEMUR_DEG = -34.0
            SIDE_STRAFE_LIFT_TIBIA_DEG = -6.0
            SIDE_STRAFE_HOLD = 0.30
            SIDE_STRAFE_SETTLE = 0.14

        elif sub in ["gentle", "safe"]:
            # Lower force for debugging after a new day / cold start.
            SIDE_STRAFE_FEMUR_REACH_DEG = 5.0
            SIDE_STRAFE_TIBIA_REACH_DEG = -11.0
            SIDE_STRAFE_FEMUR_PULL_DEG = -3.5
            SIDE_STRAFE_TIBIA_PULL_DEG = 8.0
            SIDE_STRAFE_LIFT_FEMUR_DEG = -32.0
            SIDE_STRAFE_LIFT_TIBIA_DEG = -5.0
            SIDE_STRAFE_HOLD = 0.24
            SIDE_STRAFE_SETTLE = 0.12

        elif sub in ["slightlystronger", "stronger"]:
            # Same working W23 logic, just a little more visible, not the later aggressive versions.
            SIDE_STRAFE_FEMUR_REACH_DEG = 7.0
            SIDE_STRAFE_TIBIA_REACH_DEG = -16.0
            SIDE_STRAFE_FEMUR_PULL_DEG = -5.5
            SIDE_STRAFE_TIBIA_PULL_DEG = 13.5
            SIDE_STRAFE_LIFT_FEMUR_DEG = -35.0
            SIDE_STRAFE_LIFT_TIBIA_DEG = -6.5
            SIDE_STRAFE_HOLD = 0.26
            SIDE_STRAFE_SETTLE = 0.12

        elif sub in ["reach", "assist"] and len(parts) == 4:
            SIDE_STRAFE_FEMUR_REACH_DEG = max(-22.0, min(22.0, float(parts[2])))
            SIDE_STRAFE_TIBIA_REACH_DEG = max(-24.0, min(24.0, float(parts[3])))

        elif sub in ["pull", "push"] and len(parts) == 4:
            SIDE_STRAFE_FEMUR_PULL_DEG = max(-22.0, min(22.0, float(parts[2])))
            SIDE_STRAFE_TIBIA_PULL_DEG = max(-22.0, min(22.0, float(parts[3])))

        elif sub == "hip":
            print("A/D strafe hip movement is locked to 0. This restored W23 version intentionally uses NO hips.")
            return

        elif sub == "lift" and len(parts) == 4:
            # Example: sidestrafe lift 34 -6 -> femur -34, tibia -6.
            # IMPORTANT: tibia is allowed negative/outward. The old parser accidentally forced it positive.
            femur_mag = abs(float(parts[2]))
            tibia_value = float(parts[3])
            SIDE_STRAFE_LIFT_FEMUR_DEG = -max(20.0, min(48.0, femur_mag))
            SIDE_STRAFE_LIFT_TIBIA_DEG = max(-20.0, min(20.0, tibia_value))

        elif sub == "hold" and len(parts) == 3:
            SIDE_STRAFE_HOLD = max(0.05, min(0.80, float(parts[2])))

        elif sub == "settle" and len(parts) == 3:
            SIDE_STRAFE_SETTLE = max(0.04, min(0.50, float(parts[2])))

        else:
            print("Usage:")
            print("  sidestrafe")
            print("  sidestrafe good          # restore the working WControl23 values")
            print("  sidestrafe gentle        # lower force debug preset")
            print("  sidestrafe stronger      # small increase only")
            print("  sidestrafe debug on      # slow visible micro-steps, speed still 22")
            print("  sidestrafe debug slow")
            print("  sidestrafe debug enter on")
            print("  sidestrafe debug enter off")
            print("  sidestrafe debug off")
            print("  sidestrafe flip")
            print("  sidestrafe reach 6 -14")
            print("  sidestrafe pull -5 12")
            print("  sidestrafe lift 34 -6")
            print("  sidestrafe hold 0.25")
            print("  sidestrafe settle 0.12")
            return

    except ValueError:
        print("Invalid sidestrafe value.")
        return

    print_side_strafe_settings()


def action_gait_cycle(bus: DynamixelBus, direction: str, cycles: int = 1):
    global ACTIVE_GOALS, CURRENT_MODE

    direction = normalize_direction(direction)

    if direction not in ["forward", "backward", "left", "right", "turn_left", "turn_right"]:
        print("Usage: gait forward/backward/left/right OR turn left/right")
        return

    cycles = max(1, min(10, int(cycles)))

    # WControl19: use the new IK-style side strafe for a/d only.
    # w/s/q/e still use the existing gait/turn logic.
    if direction in ["left", "right"]:
        action_side_strafe_cycle(bus, direction, cycles=cycles)
        return

    print()
    print("===================================================")
    print(f" ACTION: SMOOTH HIGHER-LIFT TRIPOD GAIT {direction.upper()} x{cycles}")
    print("===================================================")
    hip_swing_deg, support_push_deg = movement_profile(direction)
    lift_femur, lift_tibia = gait_lift_values()
    print(f"Gait lift level: {GAIT_LIFT_LEVEL}")
    print(f"Walking lift: femur {lift_femur:+.1f} deg, tibia {lift_tibia:+.1f} deg")
    print(f"Hip swing: {hip_swing_deg} deg")
    print(f"Support push: {support_push_deg} deg")
    print(f"Gait speed: {GAIT_SPEED}")
    print(f"Smooth gait: {SMOOTH_GAIT}, steps={SMOOTH_STEPS}, stepDelay={SMOOTH_STEP_DELAY:.3f}s")
    print(f"Hold/settle: {GAIT_PHASE_DELAY:.3f}s / {GAIT_SETTLE_DELAY:.3f}s")
    print("===================================================")

    for i in range(cycles):
        print()
        print(f"--- GAIT CYCLE {i + 1}/{cycles} ---")

        # One pre-check per cycle gives much smoother movement than checking before every phase.
        if not pre_motion_check(bus):
            return

        phases = [
            # Split-lift gait:
            # UP    = lift tripod vertically first, hips still centered
            # SWING = keep tripod lifted, then swing hips
            # DOWN  = place foot down at swing position
            (f"GAIT_{direction}_A_UP", TRIPOD_A, TRIPOD_B, "up", GAIT_PHASE_DELAY),
            (f"GAIT_{direction}_A_SWING", TRIPOD_A, TRIPOD_B, "lift", GAIT_PHASE_DELAY),
            (f"GAIT_{direction}_A_DOWN", TRIPOD_A, TRIPOD_B, "down", GAIT_SETTLE_DELAY),
            (f"GAIT_{direction}_B_UP", TRIPOD_B, TRIPOD_A, "up", GAIT_PHASE_DELAY),
            (f"GAIT_{direction}_B_SWING", TRIPOD_B, TRIPOD_A, "lift", GAIT_PHASE_DELAY),
            (f"GAIT_{direction}_B_DOWN", TRIPOD_B, TRIPOD_A, "down", GAIT_SETTLE_DELAY),
        ]

        for mode_name, lifted, support, phase, delay in phases:
            if GAIT_PRECHECK_EACH_PHASE:
                if not pre_motion_check(bus):
                    return

            CURRENT_MODE = mode_name
            if phase == "up":
                targets = tripod_pre_lift_targets(lifted, support, direction)
            else:
                targets = tripod_phase_targets(lifted, support, direction, phase)
            move_targets_for_gait(bus, targets, speed=GAIT_SPEED, hold_delay=delay)

            if GAIT_PHASE_HEALTH:
                print_health(bus, CURRENT_MODE)
            else:
                print(f"{CURRENT_MODE}: sent")

            print_movement_stats(bus, CURRENT_MODE, active=lifted, support=support)

        # A light health check at the end of each cycle is a good balance between safety and smoothness.
        print_health(bus, f"AFTER CYCLE {i + 1} {direction}")

    if GAIT_END_MODE == "hold":
        print("Final mode: HOLD. Robot stays in last gait stance. Use r to return to READY.")
    elif GAIT_END_MODE == "tripod":
        print("Final mode: TRIPOD RECENTER. Re-centering hips one tripod at a time.")
        final_tripod_recenter(bus, direction)
    else:
        CURRENT_MODE = "READY_REFINED2K"
        ACTIVE_GOALS = dict(READY_POSE)
        move_targets_for_gait(bus, dict(READY_POSE), speed=GAIT_SPEED, hold_delay=GAIT_FINAL_READY_DELAY)

    print_status(bus)
    # Short bus recovery pause before the second full health read. At higher gait
    # speeds, reading again immediately can cause occasional false NO_REPLY.
    time.sleep(0.25)
    print_health(bus, f"AFTER GAIT {direction}")


def print_leg_trim():
    print()
    print("===================================================")
    print(" PER-LEG LIFT TRIM")
    print("===================================================")
    print("Scale meaning:")
    print("  1.00 = normal")
    print("  0.90 = 10% less movement")
    print("  0.85 = 15% less movement")
    print("  0.80 = 20% less movement")
    print("---------------------------------------------------")
    print("Femur lift scale:")
    for leg in ALL_LEGS:
        print(f"  {leg}: {LEG_FEMUR_LIFT_SCALE.get(leg, 1.0):.2f}")
    print("Tibia lift scale:")
    for leg in ALL_LEGS:
        print(f"  {leg}: {LEG_TIBIA_LIFT_SCALE.get(leg, 1.0):.2f}")
    print("---------------------------------------------------")
    print("Useful commands:")
    print("  legtrim")
    print("  legtrim RR tibia 0.90")
    print("  legtrim RR tibia 0.85")
    print("  legtrim RR tibia 0.80")
    print("  legtrim RR femur 1.00")
    print("===================================================")


def action_leg_trim(parts: List[str]):
    if len(parts) == 1:
        print_leg_trim()
        return

    if len(parts) != 4:
        print("Usage:")
        print("  legtrim")
        print("  legtrim RR tibia 0.85")
        print("  legtrim RR femur 1.00")
        return

    leg = parts[1].upper()
    part = parts[2].lower()

    if leg not in ALL_LEGS:
        print(f"Unknown leg: {leg}. Valid legs: {ALL_LEGS}")
        return

    if part not in ["femur", "tibia"]:
        print("Part must be femur or tibia.")
        return

    try:
        value = float(parts[3])
    except ValueError:
        print("Scale must be a number. Example: legtrim RR tibia 0.85")
        return

    value = max(0.50, min(1.20, value))

    if part == "femur":
        LEG_FEMUR_LIFT_SCALE[leg] = value
    else:
        LEG_TIBIA_LIFT_SCALE[leg] = value

    print_leg_trim()


# ============================================================
# HELP / MAIN
# ============================================================

def print_help():
    print()
    print("===================================================")
    print(" HEXAPOD REFINED2K BALANCED CONTROL")
    print(" READY_POSE = refined2k balanced stance")
    print(" WCONTROL71 WORKING W23 A/D STRAFE + FLOW MODE")
    print("===================================================")
    print("p                         = print full motor status")
    print("health                    = compact health summary")
    print("movestats                 = show movement stats settings")
    print("movestats on/off          = print per-leg load stats after every movement phase")
    print("movestats detail          = print all six legs after every phase")
    print("sideflow                  = show A/D flow mode settings")
    print("sideflow on               = remove extra pauses between A/D phases")
    print("sideflow off              = original WControl23 A/D timing")
    print("sideflow tiny             = tiny visible pause between A/D phases")
    print("speed                     = show current speed settings")
    print("speed 22                  = shortcut: set gait speed to 22")
    print("speed gait 18             = set walking speed; higher number = faster")
    print("speed gait 12             = slower walking speed")
    print("speed lift 10             = set manual lift speed")
    print("speed all 10              = set ready/move/lift/gait all together")
    print("range                     = show movement range settings")
    print("range strafe 26 20        = tune strafe hip/support only")
    print("range turn 30 22          = tune turn hip/support only")
    print("sideflip                  = show strafe/turn direction flip settings")
    print("sideflip strafe           = flip a/d direction if reversed")
    print("sideflip turn             = flip q/e direction if reversed")
    print("crab                      = show WControl14 crab strafe settings")
    print("crab power 12             = stronger side reach")
    print("crab hipamount 16         = lifted-foot sideways hip placement")
    print("crab bodyhip 12           = body-shift hip push")
    print("crab shift 5              = stronger femur/tibia assist")
    print("crab shifthold 0.25       = hold body-shift longer")
    print("crab lift -30 26          = force more visible lift during reach")
    print("crab support on/off       = optional support push; default off")
    print("crab hip on/off           = optional small hip yaw during strafe")
    print("crab flip                 = flip a/d crab direction")
    print("turnscale                 = show q/e precision scale")
    print("turnscale left 0.75       = reduce q turn to 75%")
    print("turnscale right 0.78      = reduce e turn to 78%")
    print("smooth                    = show smooth gait settings")
    print("smooth on/off             = enable/disable interpolated gait")
    print("smooth walk               = optional smooth-walk timing preset")
    print("smooth stable             = safer, slightly slower smooth preset")
    print("smooth quick              = faster, less pause smooth preset")
    print("smooth steps 3            = number of micro-steps per phase")
    print("smooth stepdelay 0.025    = delay between micro-steps")
    print("smooth hold 0.08          = reduce/increase lift hold delay")
    print("smooth settle 0.05        = reduce/increase down settle delay")
    print("smooth end tripod         = default final recenter, avoids all-feet hip drag")
    print("smooth end direct         = old final ready reset")
    print("smooth end hold           = do not auto-return to ready")
    print("walklift                  = show walking lift profile")
    print("walklift level 6          = V6 gait lift: femur -28, tibia +28")
    print("walklift level 7          = stronger V6-style lift: femur -32, tibia +32")
    print("walklift level 8          = extra split-lift test: femur -36, tibia +34")
    print("walklift level 9          = maximum split-lift test: femur -40, tibia +36")
    print("walklift old6             = old full tibia tuck: femur -28, tibia +28")
    print("walklift both -32 32      = custom profile if profile mode is needed")
    print("Main change: walking now lifts UP first, then swings, like the final recenter motion")
    print("legtrim                   = show per-leg lift/tibia trim")
    print("legtrim RR tibia 0.85     = reduce RR tibia lift/tuck slightly")
    print("torque_max                = optional: set AX torque limit cap to 1023")
    print("r / ready                  = return to refined2k balanced ready pose")
    print("force_r                    = force return to LOW14 without safety check")
    print("pushup 1                   = raise body slightly")
    print("pushup 2/3/4               = raise body more")
    print("---------------------------------------------------")
    print("Lift commands:")
    print("lift FL                    = default level 3 lift FL")
    print("lift 5 FL                  = previous max-tested lift FL")
    print("lift 6 FL                  = higher clearance lift FL")
    print("lift 7 FL                  = optional max clearance test only")
    print("lift 6 FL MR RL            = lift tripod A")
    print("lift 6 FR ML RR            = lift tripod B")
    print("---------------------------------------------------")
    print("Movement commands:")
    print("w / gait forward           = one forward gait cycle")
    print("s / gait backward          = one backward gait cycle")
    print("walk forward 3             = repeat forward gait 3 cycles")
    print("walk backward 3            = repeat backward gait 3 cycles")
    print("a / gait left              = one left strafe cycle, gentler profile")
    print("d / gait right             = one right strafe cycle, gentler profile")
    print("q / turn left              = one turn-left cycle, gentler profile")
    print("e / turn right             = one turn-right cycle, gentler profile")
    print("x                          = exit")
    print("---------------------------------------------------")
    print("Recommended debug start:")
    print("Speed defaults already = 22, so no need to type speed all 22.")
    print("r")
    print("health")
    print("sidestrafe good")
    print("a")
    print("r")
    print("health")
    print("d")
    print("r")
    print("health")
    print("Then test w/s/q/e separately if needed.")
    print("---------------------------------------------------")
    print("Current gait-lift finder defaults:")
    print(f"  Forward/backward hip swing   = {GAIT_HIP_SWING_DEG} deg")
    print(f"  Forward/backward support     = {GAIT_SUPPORT_PUSH_DEG} deg")
    print(f"  Strafe hip/support           = {STRAFE_HIP_SWING_DEG}/{STRAFE_SUPPORT_PUSH_DEG} deg")
    print(f"  Turn hip/support             = {TURN_HIP_SWING_DEG}/{TURN_SUPPORT_PUSH_DEG} deg")
    print(f"  Gait lift level              = {GAIT_LIFT_LEVEL}")
    print(f"  Walking lift profile         = {USE_WALK_LIFT_PROFILE}, femur {WALK_LIFT_FEMUR_DEG:+.1f}, tibia {WALK_LIFT_TIBIA_DEG:+.1f}")
    print(f"  RR tibia trim                = {LEG_TIBIA_LIFT_SCALE.get('RR', 1.0):.2f}")
    print(f"  Gait speed                   = {GAIT_SPEED}")
    print(f"  Smooth gait                  = {SMOOTH_GAIT}, steps={SMOOTH_STEPS}")
    print(f"  Phase hold/settle            = {GAIT_PHASE_DELAY:.2f}s / {GAIT_SETTLE_DELAY:.2f}s")
    print("===================================================")

def main():
    bus = DynamixelBus(DEFAULT_PORT)

    if not bus.open():
        return

    try:
        print()
        print("Startup: NO automatic movement.")
        print("Speed defaults are already set to 22 for ready/move/lift/gait.")
        print("Recommended debug start: r -> health -> sidestrafe good -> a -> r -> d -> r.")
        print("Debug observe: sidestrafe debug on")
        print("Movement stats: movestats on / movestats detail")
        print("Sideflow: sideflow on/off/tiny  # A/D only")
        print_help()

        while True:
            try:
                raw_cmd = input("\nLow14Control command [h help]: ").strip()
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
                    print("Exit requested.")
                    break

                elif cmd == "h":
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

                elif cmd == "liftfind":
                    action_lift_find(bus, parts)

                elif cmd == "crab":

                    action_crab_settings(parts)

                elif cmd in ["sidestrafe", "side", "ad"]:

                    action_side_strafe_settings(parts)


                elif cmd == "turnscale":

                    action_turnscale(parts)


                elif cmd == "sideflip":

                    action_sideflip(parts)


                elif cmd == "range":

                    action_set_range(parts)


                elif cmd in ["legtrim", "trim"]:
                    action_leg_trim(parts)

                elif cmd == "torque_max":
                    action_torque_max(bus)

                elif cmd in ["r", "ready"]:
                    action_ready(bus, use_safety_check=True)

                elif cmd == "force_r":
                    print()
                    print("FORCE_R: returning to LOW14 without safety check.")
                    print("Physically support the robot before using this.")
                    action_ready(bus, use_safety_check=False)

                elif cmd == "pushup":
                    if len(parts) != 2:
                        print("Usage: pushup 1 / pushup 2 / pushup 3 / pushup 4")
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
                        print("Usage: gait forward / backward / left / right")
                        continue

                    action_gait_cycle(bus, parts[1], cycles=1)

                elif cmd == "walk":
                    if len(parts) < 2:
                        print("Usage: walk forward 3")
                        continue

                    direction = parts[1]
                    cycles = 1

                    if len(parts) >= 3:
                        cycles = int(parts[2])

                    action_gait_cycle(bus, direction, cycles=cycles)

                elif cmd == "turn":
                    if len(parts) != 2:
                        print("Usage: turn left / turn right")
                        continue

                    turn_dir = normalize_direction(parts[1])

                    if turn_dir == "left":
                        action_gait_cycle(bus, "turn_left", cycles=1)
                    elif turn_dir == "right":
                        action_gait_cycle(bus, "turn_right", cycles=1)
                    else:
                        print("Usage: turn left / turn right")

                elif cmd in ["w", "s", "a", "d", "q", "e", "foward", "forwad", "fw"]:
                    direction = normalize_direction(cmd)
                    action_gait_cycle(bus, direction, cycles=1)

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
