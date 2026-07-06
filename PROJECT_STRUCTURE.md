# Project Structure

```text
Hexapod_Research/
├── control/
│   ├── main.py
│   ├── IKControl.py
│   ├── SControl.py
│   ├── ControllerClient.py
│   ├── StatsPos.py
│   ├── CalibrationPose.py
│   ├── IKControl - backup.py
│   ├── IKControl w0.py
│   ├── README.md
│   ├── requirements.txt
│   ├── SYNTAX_CHECK_CURRENT.md
│   ├── notes/
│   │   └── README.md
│   └── legacy/
│       ├── README.md
│       ├── IKControl 0.py
│       ├── IKControlOG.py
│       └── WebLegacy_SControlX2Web.py
├── docs/
│   ├── Hexapod-Full-Doc.docx
│   └── Hexapod-Full-Doc.pdf
├── experiments/
│   ├── README.md
│   ├── controller_client_iterations/
│   ├── current_research_experimental/
│   ├── ik_development_versions/
│   ├── legacy_control_iterations/
│   └── legacy_server_control_iterations/
├── hardware/
│   ├── README.md
│   ├── cad_models/
│   └── legacy_robotis_motion_task_files/
├── legacy/
│   ├── README.md
│   ├── previous_attempts/
│   └── stable_baseline_2026_05_25/
├── model/
│   ├── README.md
│   ├── __init__.py
│   ├── hexapod_kinematics.py
│   └── robot_model.py
├── tools/
│   ├── README.md
│   ├── calibration/
│   ├── diagnostics/
│   └── probes/
├── .gitignore
├── CLEAN_FILE_LIST.md
├── PROJECT_STRUCTURE.md
├── README.md
└── requirements.txt
```

Removed from this clean package:

- virtual environments and installed dependency folders
- Python cache files and `__pycache__` folders
- personal future-task notes and temporary planning reminders
- copied terminal logs and temporary console outputs
- controller CSV logs and runtime logs
- old package notes and redundant cleanup scratch files
- machine-specific secrets or credential files
```
