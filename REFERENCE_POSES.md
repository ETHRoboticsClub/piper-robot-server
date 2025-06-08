# Reference Poses for IK Improvement

This feature helps fix IK (Inverse Kinematics) issues where the robot arms get locked in local minima and can't find good solutions to reach target positions.

## How it works

The enhanced IK solver tries multiple starting configurations when solving for a target position:

1. **Current pose** - The robot's current joint configuration (most likely to be close)
2. **Reference poses** - Pre-recorded "good" configurations that you manually set

The solver evaluates all solutions and picks the one with the lowest position error, but applies hysteresis to prevent disruptive switching to reference poses unless they provide significant improvement.

## Recording Reference Poses

1. **Run the recording script:**
   ```bash
   conda activate teleop
   python read_pose.py
   ```

2. **Motors are automatically disengaged:**
   - The script disables motor torque for safe manual positioning
   - You can now freely move the robot arms by hand
   - No risk of fighting against motor resistance or damage

3. **Position the robot arms:**
   - Manually move the robot arms to diverse, well-positioned configurations
   - Try different shoulder, elbow, and arm positions
   - Avoid extreme joint limits or awkward poses

4. **Record poses:**
   - Press Enter to record each good configuration
   - Record 2-3 different poses for variety
   - Type 'done' when finished

5. **Poses are saved** to `.cache/reference_poses.json`

## Configuration

In `config.yaml`:

```yaml
ik:
  use_reference_poses: true                    # Enable/disable the feature
  reference_poses_file: .cache/reference_poses.json  # Where poses are stored
  position_error_threshold: 0.001             # Error threshold in meters (1mm)
  hysteresis_threshold: 0.05                  # Required improvement to switch solutions (5cm)
  movement_penalty_weight: 0.01               # Penalty for large joint movements
```

### Advanced Settings

- **`hysteresis_threshold`**: Prevents disruptive switching by requiring a significant improvement (default 5cm) before using a **reference pose** solution instead of the current one. Current pose is always allowed.
- **`movement_penalty_weight`**: Penalizes solutions that require large joint movements, preferring smoother transitions. Reference poses get movement penalty, current pose doesn't.
- **`position_error_threshold`**: Threshold for "excellent" solutions that bypass reference pose checking

## Benefits

- **Prevents getting stuck:** If the current pose leads to a poor IK solution, reference poses provide alternative starting points
- **Better reachability:** Well-chosen reference poses can help reach difficult positions
- **Smooth operation:** Hysteresis prevents disruptive switching unless there's significant improvement
- **Movement-aware:** Solutions that require large joint movements are penalized to prefer smoother paths
- **Configurable:** Can be enabled/disabled and tuned via configuration

## Tips for Good Reference Poses

1. **Diversity:** Record poses with different elbow positions (bent vs straight)
2. **Workspace coverage:** Include poses that reach different areas of the workspace
3. **Avoid limits:** Don't record poses at extreme joint limits
4. **Functional positions:** Record poses that represent common working configurations

## Troubleshooting

- **No reference poses loaded:** Run `python read_pose.py` to record some
- **Feature disabled:** Check `use_reference_poses: true` in config.yaml
- **File not found:** Ensure the robot is connected when recording poses
- **Poor performance:** Try recording more diverse reference poses

The system will log when it uses reference poses:
```
DEBUG: IK: Used reference_1 rest pose (error: 0.0023m)
``` 