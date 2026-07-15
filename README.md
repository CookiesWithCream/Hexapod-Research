# Hexapod Research Project Clean Package

This folder contains the cleaned working files for the AX-series hexapod research project. It keeps only project-relevant code, hardware assets, documentation, tools, and selected experiment milestones. Personal task notes, terminal logs, temporary package notes, virtual environments, Python cache files, and redundant non-project clutter were removed.

## Reproducibility Note

This Code Ocean capsule provides the software workflow, dependency list, setup instructions, calibration notes, and movement-test procedure for the Dynamixel-based hexapod control bridge. The physical movement results require the actual robot hardware, including the CM-530/Dynamixel communication path, AX-series Dynamixel actuators, calibrated ready pose, battery power system, and physical test surface.

This capsule supports software reproducibility and procedure verification. Exact walking distance, actuator temperature, and timing may vary depending on battery voltage, actuator condition, floor contact, mechanical alignment, load distribution, and manual timing latency.

## Main folders

- `control/` — current runnable control scripts.
- `experiments/` — selected experimental and legacy code milestones kept for comparison.
- `legacy/` — older stable baseline/reference files.
- `hardware/` — CAD/STL files and original ROBOTIS motion/task references.
- `docs/` — current full documentation files in DOCX and PDF format.
- `model/` — reusable robot model and kinematics files.
- `tools/` — calibration, diagnostics, and probing utilities.

## Main runnable files

- `control/main.py` — current hexapod launcher for terminal/web mode, local web control, and safe serial/COM port selection or switching.
- `control/IKControl.py` — main current controller with fixed gait fallback, IK movement, Bézier testing, Web UI support, terminal control, and Body IK experiment support.
- `control/SControl.py` — stable hardcoded/original gait controller kept as a fallback and baseline comparison.
- `control/ControllerClient.py` — controller/gamepad client used to send movement commands to the server.
- `control/StatsPos.py` and `control/CalibrationPose.py` — supporting status and calibration helpers.

## Recommended normal workflow

1. Install dependencies from the root folder:

   ```bash
   pip install -r requirements.txt
   ```

2. Use `control/main.py` for the current Raspberry Pi / web-control launcher:

   ```bash
   cd control
   python main.py --mode web --com <PORT>
   ```

   Example ports are `COM6` on Windows or `/dev/ttyUSB0` / `/dev/serial/by-id/<device>` on Linux/Raspberry Pi.

3. Use `control/IKControl.py` when directly testing the main controller logic.
4. Use `control/SControl.py` when you need the older stable hardcoded gait baseline.
5. Use `control/ControllerClient.py` when testing external controller input.

## Documentation

The current full documentation is stored in:

- `docs/Hexapod-Full-Doc.docx`
- `docs/Hexapod-Full-Doc.pdf`

The documentation describes the web-based joint-level control bridge, Dynamixel motor mapping, ready-pose calibration, flat-pose horn alignment method, Raspberry Pi onboard power integration, diagnostic tools, and future ROS / reinforcement learning direction.

## Release note

This package is intended for GitHub and Zenodo archiving as a clean public project snapshot. It does not include virtual environments, Python cache folders, personal planning notes, copied terminal logs, passwords, API keys, or machine-specific dependency folders.
