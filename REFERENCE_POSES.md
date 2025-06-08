# Reference Poses for IK Solutions

## Overview

The teleoperation system uses **reference poses** to help the IK solver escape local minima and find better solutions when the robot gets stuck. These are pre-recorded joint configurations that represent good, reachable poses for common workspace areas.

## How It Works

When the IK solver tries to reach a target position:

1. **Current pose**: Uses the robot's current joint angles as the starting point
2. **Reference poses**: Also tries IK starting from each recorded reference pose
3. **Best solution**: Selects the solution that gets closest to the target
4. **Hysteresis**: Only switches to a reference pose if it's significantly better (>5cm improvement)

This prevents the robot from getting stuck in local minima where small joint movements can't reach the target, but a different arm configuration could easily reach it.

## Recording Reference Poses

### Using the Recording Script

1. **Run the pose recorder**:
   ```bash
   python read_pose.py
   ```

2. **For each arm** (left/right):
   - **Manually position** the robot arm to a good pose in your workspace
   - **Press Enter** to record that pose
   - **Repeat** for 3-4 different poses covering your workspace

3. **Choose different configurations** like:
   - Arm extended forward
   - Arm to the side  
   - Arm in different elbow configurations
   - Poses that cover your typical work area

4. **Poses are saved** to `reference_poses.json`

### Configuration

In `config.yaml`:
```yaml
ik:
  use_reference_poses: true
  reference_poses_file: reference_poses.json  # Where poses are stored
  hysteresis_threshold: 0.05  # 5cm improvement needed to switch
  movement_penalty_weight: 0.01  # Prefer smaller joint movements
```

## File Format

The `reference_poses.json` file stores poses for each arm:

```json
{
  "left": [
    [10.5, 85.2, 95.1, 45.0, 0.0, 0.0],
    [15.7, 70.3, 110.8, 60.0, 5.0, 0.0],
    ...
  ],
  "right": [
    [12.1, 88.5, 92.4, 50.0, -2.0, 0.0],
    [18.3, 75.1, 105.2, 55.0, -8.0, 0.0],
    ...
  ]
}
```

Each pose is `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]` in degrees.

## Tips for Good Reference Poses

1. **Diverse configurations**: Record poses with different elbow positions (up vs down)
2. **Workspace coverage**: Include poses that reach different areas you commonly work in  
3. **Avoid limits**: Don't record poses near joint limits where the robot might get stuck
4. **Test reachability**: Make sure each pose can actually reach useful workspace areas

## Troubleshooting

- **Robot not moving to better solutions**: Check that hysteresis threshold isn't too high
- **Jumpy movements**: Reduce hysteresis threshold or increase movement penalty
- **Getting stuck**: Record more diverse reference poses covering your workspace
- **File not found**: Make sure `reference_poses.json` exists and the path in config is correct

The system will log when it uses reference poses:
```
DEBUG: IK: Used reference_1 rest pose (error: 0.0023m)
``` 