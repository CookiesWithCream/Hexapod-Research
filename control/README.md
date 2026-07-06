# Control Folder

This folder contains the current runnable hexapod control scripts.

## Files

### `main.py`
Current launcher for the cleaned package. It wraps the main IKControl-based system with terminal/web mode selection, local web access, and safe serial/COM port selection or switching. This is the recommended entry point for Raspberry Pi web-control demonstration.

Typical examples:

```text
python main.py --mode web --com COM6
python main.py --mode terminal --com COM6
python main.py --mode web --com /dev/ttyUSB0
python main.py --mode web --com /dev/serial/by-id/<device>
```

### `IKControl.py`
Main current controller. This includes:

- fixed/hardcoded gait fallback mode
- efficient IK foot-space gait mode
- Bézier IK trajectory test mode
- Body IK posture experiment support
- terminal and Web UI mode switching
- controller-client support
- motor health/status feedback

Useful terminal commands:

```text
ik fixed            # original hardcoded movement
ik efficient        # IK movement without Bézier, best reach/speed
ik bezier_default   # smoother Bézier IK mode
ik bezier_showcase  # exaggerated Bézier demo mode
ik                  # show IK/mode status
health              # check motor health
```

### `SControl.py`
Stable/original hardcoded controller. Use this when you want the proven fixed joint-space gait behavior.

Typical terminal test:

```text
r
health
walk w 1
r
walk s 1
r
```

### `ControllerClient.py`
Current controller/gamepad client. This script sends controller input to the server for movement commands such as forward, backward, strafing, turning, ready pose, reset, and height-related controls.

### `StatsPos.py` and `CalibrationPose.py`
Supporting scripts for status reading and calibration/pose work.

### `legacy/`
Contains preserved older controller versions for reference.

### `notes/`
Contains only a folder README. Earlier personal progress notes and future-task reminders were removed from the clean package.
