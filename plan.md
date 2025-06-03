# Refactor Plan: Unified Teleoperation Control System for SO100 Robot

## 1. Objectives

1. Provide **one consolidated entry-point** that:
   * launches the HTTPS content server (to host `index.html`, `app.js`, assets …)
   * starts the secure WebSocket bridge used by the Quest VR controllers
   * initializes the robot interface (connection, joint-limit discovery, PyBullet viz)
   * optionally starts the keyboard tele-operation loop
2. **Decouple input devices** (VR controllers, keyboard, future game-pads) from core robot logic so they plug into the same command pipeline.
3. **Share IK / FK / safety logic** in one module that both VR and keyboard layers call.
4. Keep the system **async, non-blocking and testable**.
5. Allow headless (no PyBullet) and simulation modes via flags.

---

## 2. Proposed High-Level File/Package Layout

```
vr_so100_teleoperation/
├── teleop/                       # NEW top-level Python package
│   ├── __init__.py
│   ├── config.py                 # Ports, default SSL paths, scaling factors …
│   ├── core/
│   │   ├── robot_interface.py    # Thin wrapper around ManipulatorRobot + IK helpers
│   │   ├── kinematics.py         # FK / IK utilities (could import existing modules)
│   │   ├── visualizer.py         # PyBullet setup & marker helpers
│   │   └── ...
│   ├── inputs/
│   │   ├── __init__.py
│   │   ├── vr_ws_server.py       # WebSocket ↔ queue bridge (asyncio)
│   │   ├── keyboard_listener.py  # pynput listener ↔ queue bridge (thread / asyncio)
│   │   └── ...
│   ├── control_loop.py           # Consumes queue, produces joint commands
│   └── main.py                   # One CLI entry: `python -m teleop` or `teleop`
├── webapp/                       # Static files served over HTTPS
│   ├── index.html
│   ├── app.js
│   └── assets/
└── plan.md                       # ← you are here
```

### Notes
* **No code moved yet** – current scripts (`vr_robot_teleop.py`, `lerobot_keyboard_ik.py`, `serve_https.py`) remain as reference during migration.
* `teleop.config` centralises constants now duplicated across scripts (ports, joint names, index mappings, SSL cert paths…).
* `teleop.core.robot_interface.RobotInterface` encapsulates:
  * connection / disconnection
  * joint-state acquisition
  * `send_action()` with safety clamps
  * FK/IK helpers via `teleop.core.kinematics`
* `teleop.core.visualizer.PyBulletViz` handles optional PyBullet visualisation.
* Each *input* provider pushes high-level "goal" messages into an **`asyncio.Queue`**:

```python
@dataclass
class ControlGoal:
    arm: Literal["left", "right"]
    pos: np.ndarray           # 3-vector world frame
    wrist_roll_deg: float
    gripper_cmd: Optional[float]  # None = keep, value in deg
```

The **control loop** pulls from the queue at ~20 Hz, translates into joint angles via IK, clamps and calls the robot interface.

---

## 3. Execution Flow (simplified)

```mermaid
graph TD
    A(Start main.py) --> B(Init config & logger)
    B --> C(Launch HTTPS static server) & D(Init RobotInterface) & E(Init PyBullet (opt))
    C & D & E --> F(Spawn input tasks)
    F -->|VR| G(vr_ws_server.run())
    F -->|Keyboard| H(keyboard_listener.run())
    F --> I(Start control_loop)
```

* All tasks share one `asyncio` event loop.
* Graceful shutdown via `asyncio.CancelledError` – server sockets closed, robot torque disabled, PyBullet disconnected.

---

## 4. Migration Road-Map

| Stage | Goal | Deliverables |
|-------|------|--------------|
| 0 | Planning (you are here) | `plan.md` |
| 1 | Skeleton package | Empty modules + `main.py` printing banner |
| 2 | Move **robot interface** code | `robot_interface.py`, reuse logic from `vr_robot_teleop.py` / `lerobot_keyboard_ik.py` |
| 3 | Add **PyBullet visualiser** | Port visual-setup & markers |
| 4 | Implement **VR WebSocket server** task | Adapt `websocket_handler` but push to queue |
| 5 | Implement **keyboard listener** task | Reuse key mapping, push deltas to queue |
| 6 | Build **control loop** | Unified IK, safety clamps, robot command send |
| 7 | CLI & configuration | Flags for `--no-viz`, `--no-robot`, `--port`, etc. |
| 8 | Testing & docs | Unit tests for kinematics, integration test with `--no-robot` |
| 9 | Remove legacy scripts (optional) | After validation |

---

## 5. Key Refactor Considerations

1. **Thread vs. Async**: Keep everything in one `asyncio` loop; keyboard listener may still need a thread → queue.
2. **Safety**: Always clamp joint commands using URDF limits retrieved at startup.
3. **Extensibility**: New input methods (e.g., joystick) just implement `BaseInputProvider` and push to the queue.
4. **Logging**: Central `logging` config with per-module loggers; throttle repeated messages.
5. **Environment**: Allow running without a physical robot (`--dry-run`) for development.
6. **Packaging**: Add `setup.py` or `pyproject.toml` for installable CLI.

---

## 6. Next Steps for Implementation

1. Create directory `teleop/` with `__init__.py` and the module scaffolding shown above.
2. Move shared constants (joint names, indices, ranges) into `teleop.config`.
3. Port and unit-test FK/IK helpers into `teleop.core.kinematics`.
4. Incrementally adapt `vr_robot_teleop.py` logic into the new modular structure.
5. Continually run both old and new scripts side-by-side until feature parity is reached.

---

*End of plan.* 