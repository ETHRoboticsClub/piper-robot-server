# URDF Joint Comparison - Old vs New

## Joint 1: Shoulder Pan (Rotation)

**Old URDF (Rotation):**
- Origin: `xyz="0 -0.0452 0.0165" rpy="1.5708 0 0"`
- Axis: `xyz="0 -1 0"`
- Limits: `lower="-2.1" upper="2.1"`

**New URDF (1):**
- Origin: `xyz="0 -0.0452 0.0165" rpy="1.57079 0 0"`
- Axis: `xyz="0 1 0"`
- Limits: `lower="-2" upper="2"`

**⚠️ DIFFERENCE: Axis direction is inverted! Old: `0 -1 0`, New: `0 1 0`**

## Joint 2: Shoulder Lift (Pitch)

**Old URDF (Pitch):**
- Origin: `xyz="0 0.1025 0.0306" rpy="1.5708 0 0"`
- Axis: `xyz="-1 0 0"`
- Limits: `lower="-0.1" upper="3.45"`

**New URDF (2):**
- Origin: `xyz="0 0.1025 0.0306" rpy="-1.8 0 0"`
- Axis: `xyz="1 0 0"`
- Limits: `lower="0" upper="3.5"`

**⚠️ DIFFERENCE: Axis direction inverted AND different RPY! Old: `1.5708 0 0` vs New: `-1.8 0 0`**

## Joint 3: Elbow Flex (Elbow)

**Old URDF (Elbow):**
- Origin: `xyz="0 0.11257 0.028" rpy="-1.5708 0 0"`
- Axis: `xyz="1 0 0"`
- Limits: `lower="-0.2" upper="3.14159"`

**New URDF (3):**
- Origin: `xyz="0 0.11257 0.028" rpy="1.57079 0 0"`
- Axis: `xyz="1 0 0"`
- Limits: `lower="-3.14158" upper="0"`

**⚠️ DIFFERENCE: Different RPY sign AND inverted limits! Old: `-1.5708` vs New: `1.57079`**

## Joint 4: Wrist Flex (Wrist_Pitch)

**Old URDF (Wrist_Pitch):**
- Origin: `xyz="0 0.0052 0.1349" rpy="-1.5708 0 0"`
- Axis: `xyz="1 0 0"`
- Limits: `lower="-1.8" upper="1.8"`

**New URDF (4):**
- Origin: `xyz="0 0.0052 0.1349" rpy="-1 0 0"`
- Axis: `xyz="1 0 0"`
- Limits: `lower="-2.5" upper="1.2"`

**⚠️ DIFFERENCE: Different RPY value! Old: `-1.5708` vs New: `-1`**

## Joint 5: Wrist Roll (Wrist_Roll)

**Old URDF (Wrist_Roll):**
- Origin: `xyz="0 -0.0601 0" rpy="0 1.5708 0"`
- Axis: `xyz="0 -1 0"`
- Limits: `lower="-3.14159" upper="3.14159"`

**New URDF (5):**
- Origin: `xyz="0 -0.0601 0" rpy="0 1.57079 0"`
- Axis: `xyz="0 1 0"`
- Limits: `lower="-3.14158" upper="3.14158"`

**⚠️ DIFFERENCE: Axis direction inverted! Old: `0 -1 0` vs New: `0 1 0`**

## Joint 6: Gripper (Jaw)

**Old URDF (Jaw):**
- Origin: `xyz="-0.0202 -0.0244 0" rpy="3.1416 0 3.33"`
- Axis: `xyz="0 0 1"`
- Limits: `lower="0" upper="1.7"`

**New URDF (6):**
- Origin: `xyz="-0.0202 -0.0244 0" rpy="0 3.14158 0"`
- Axis: `xyz="0 0 1"`
- Limits: `lower="-0.2" upper="2.0"`

**⚠️ DIFFERENCE: Completely different RPY! Old: `3.1416 0 3.33` vs New: `0 3.14158 0`**

## Summary of Issues

1. **Joint Axis Directions**: Multiple joints have inverted axis directions
2. **Joint Origins (RPY)**: Several joints have different orientation values
3. **Joint Limits**: Some joints have different or inverted limit ranges

These differences explain why the physical robot angles don't match the PyBullet visualization! 