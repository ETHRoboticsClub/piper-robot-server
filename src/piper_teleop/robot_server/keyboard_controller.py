import threading
from dataclasses import dataclass

import numpy as np
from pynput import keyboard
from tactile_teleop_sdk.inputs.base import ArmGoal


@dataclass
class KeyboardState:
    """Represents the current state of an arm's keyboard controls."""

    gripper_closed: bool = True
    reset_to_init: bool = False


class KeyboardController:
    """
    A controller that translates keyboard inputs into robot arm goals.

    This class listens for keyboard events and allows for simultaneous control
    of two separate robot arms using different sets of keys. It internally
    tracks the target position for each arm to allow for continuous movement.

    Key Mappings:
    ----------------------------------------------------
    | Action              | Left Arm      | Right Arm    |
    |---------------------|---------------|--------------|
    | Move Forward (+X)   | w             | t            |
    | Move Backward (-X)  | s             | g            |
    | Move Left (+Y)      | a             | f            |
    | Move Right (-Y)     | d             | h            |
    | Move Up (+Z)        | q             | r            |
    | Move Down (-Z)      | e             | z            |
    | Toggle Gripper      | space         | enter        |
    | Reset Position      | x             | b            |
    ----------------------------------------------------
    """

    def __init__(self, trans_step=0.01):
        """
        Initializes the KeyboardController.

        Args:
            trans_step (float): The distance in meters to move in one step.
        """
        self.trans_step = trans_step
        self.states = {
            "left": KeyboardState(),
            "right": KeyboardState(),
        }
        # Dictionaries to store the current target and initial origin for each arm
        self.target_transforms = {"left": None, "right": None}
        self.origin_transforms = {"left": None, "right": None}

        self._pressed_keys = set()
        self.listener_thread = threading.Thread(target=self._listen, daemon=True)
        self.listener_thread.start()

    def _listen(self):
        """Initializes and runs the keyboard listener."""
        with keyboard.Listener(on_press=self._on_press, on_release=self._on_release) as listener:
            listener.join()

    def _on_press(self, key):
        """Callback function for key press events."""
        self._pressed_keys.add(key)
        try:
            # Handle state changes that only happen once per press (toggles)
            if key == keyboard.Key.space:
                self.states["left"].gripper_closed = not self.states["left"].gripper_closed
                status = "Closed" if self.states["left"].gripper_closed else "Open"
                print(f"Left gripper: {status}")
            elif key == keyboard.Key.enter:
                self.states["right"].gripper_closed = not self.states["right"].gripper_closed
                status = "Closed" if self.states["right"].gripper_closed else "Open"
                print(f"Right gripper: {status}")
            elif key.char == "x":
                self.states["left"].reset_to_init = True
                print("--- Resetting Left arm ---")
            elif key.char == "b":
                self.states["right"].reset_to_init = True
                print("--- Resetting Right arm ---")
        except AttributeError:
            pass

    def _on_release(self, key):
        """Callback function for key release events."""
        if key in self._pressed_keys:
            self._pressed_keys.remove(key)
        try:
            # Reset the reset flag on key release
            if key.char == "x":
                self.states["left"].reset_to_init = False
            elif key.char == "b":
                self.states["right"].reset_to_init = False
        except AttributeError:
            pass

    def get_goal(self, arm_name: str, current_transform: np.ndarray) -> ArmGoal:
        """
        Generates an ArmGoal based on the current keyboard state for a specific arm.

        Args:
            arm_name: The name of the arm ("left" or "right") to get the goal for.
            current_transform: The current transform of the arm's end-effector.

        Returns:
            An ArmGoal object representing the desired action.
        """
        # On the first run, initialize the internal transforms from the robot's state
        if self.target_transforms[arm_name] is None:
            self.target_transforms[arm_name] = current_transform.copy()
            self.origin_transforms[arm_name] = current_transform.copy()

        state = self.states[arm_name]
        arm_goal = ArmGoal(arm=arm_name, gripper_closed=state.gripper_closed)

        # Handle reset command
        if state.reset_to_init:
            self.target_transforms[arm_name] = self.origin_transforms[arm_name].copy()
            arm_goal.reset_to_init = True
            return arm_goal

        # Determine translation vector based on pressed keys
        translation = np.array([0.0, 0.0, 0.0])
        keys = self._pressed_keys

        if arm_name == "left":
            if keyboard.KeyCode.from_char("w") in keys:
                translation[0] += self.trans_step
            if keyboard.KeyCode.from_char("s") in keys:
                translation[0] -= self.trans_step
            if keyboard.KeyCode.from_char("a") in keys:
                translation[1] += self.trans_step
            if keyboard.KeyCode.from_char("d") in keys:
                translation[1] -= self.trans_step
            if keyboard.KeyCode.from_char("q") in keys:
                translation[2] += self.trans_step
            if keyboard.KeyCode.from_char("e") in keys:
                translation[2] -= self.trans_step
        elif arm_name == "right":
            if keyboard.KeyCode.from_char("t") in keys:
                translation[0] += self.trans_step
            if keyboard.KeyCode.from_char("g") in keys:
                translation[0] -= self.trans_step
            if keyboard.KeyCode.from_char("f") in keys:
                translation[1] += self.trans_step
            if keyboard.KeyCode.from_char("h") in keys:
                translation[1] -= self.trans_step
            if keyboard.KeyCode.from_char("r") in keys:
                translation[2] += self.trans_step
            if keyboard.KeyCode.from_char("z") in keys:
                translation[2] -= self.trans_step

        # If there is movement, update the internal target transform
        if np.any(translation):
            # Create a small transformation step
            step_transform = np.eye(4)
            step_transform[:3, 3] = translation
            # Apply the step to the current target to get the new target.
            # This makes the movement relative to the arm's current position and orientation.
            self.target_transforms[arm_name] = self.target_transforms[arm_name] @ step_transform

        # Calculate the transform relative to the origin, which the control loop expects
        origin = self.origin_transforms[arm_name]
        target = self.target_transforms[arm_name]
        relative_transform = np.linalg.inv(origin) @ target
        arm_goal.relative_transform = relative_transform

        return arm_goal

    def stop(self):
        """Stops the keyboard listener."""
        # The listener runs in a daemon thread and will exit with the main program.
        print("Keyboard controller stopped.")
