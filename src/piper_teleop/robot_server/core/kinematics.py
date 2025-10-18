"""
Kinematics utilities for the SO100 robot.
Contains forward and inverse kinematics solvers using PyBullet.
"""

import logging
import math
import os
import tempfile

import casadi
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer

logger = logging.getLogger(__name__)


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


def quaternion_from_matrix(matrix):
    qw = math.sqrt(1 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2]) / 2
    qx = (matrix[2, 1] - matrix[1, 2]) / (4 * qw)
    qy = (matrix[0, 2] - matrix[2, 0]) / (4 * qw)
    qz = (matrix[1, 0] - matrix[0, 1]) / (4 * qw)
    return np.array([qx, qy, qz, qw])


def _make_mesh_paths_absolute(urdf_path: str) -> str:
    """
    Convert relative mesh paths in URDF to absolute paths.
    Returns path to a temporary URDF file with absolute paths.
    """
    urdf_abs_path = os.path.abspath(urdf_path)
    urdf_dir = os.path.dirname(urdf_abs_path)

    # Read the original URDF
    with open(urdf_abs_path, "r") as f:
        urdf_content = f.read()

    # Replace relative mesh paths with absolute paths
    # Look for patterns like: filename="assets/something.STL"
    import re

    def replace_mesh_path(match):
        relative_path = match.group(1)
        if not os.path.isabs(relative_path):
            # Convert to absolute path
            absolute_path = os.path.join(urdf_dir, relative_path)
            return f'filename="{absolute_path}"'
        return match.group(0)  # Keep absolute paths as-is

    # Replace all mesh filename attributes
    urdf_content = re.sub(r'filename="([^"]+\.STL)"', replace_mesh_path, urdf_content, flags=re.IGNORECASE)

    # Create temporary file with absolute paths
    temp_fd, temp_path = tempfile.mkstemp(suffix=".urdf", text=True)
    try:
        with os.fdopen(temp_fd, "w") as f:
            f.write(urdf_content)
        return temp_path
    except Exception:
        os.close(temp_fd)
        raise


class Arm_IK:
    def __init__(self, urdf_path: str, ground_height: float = 0.0):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        # Create temporary URDF with absolute mesh paths
        temp_urdf_path = _make_mesh_paths_absolute(urdf_path)

        try:
            # Load URDF with absolute paths
            self.robot = pin.RobotWrapper.BuildFromURDF(temp_urdf_path)

            # Build geometry model
            self.geom_model = pin.buildGeomFromUrdf(self.robot.model, temp_urdf_path, pin.GeometryType.COLLISION)

        finally:
            # Clean up temporary file
            if os.path.exists(temp_urdf_path):
                os.unlink(temp_urdf_path)

        for joint in self.robot.model.names:
            print("Joint name:", joint)

        for link in self.robot.model.frames:
            print("Link name:", link.name)

        self.mixed_jointsToLockIDs = ["joint7", "joint8", "arm2_joint7", "arm2_joint8"]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # Add ground plane geometry to collision model
        self.ground_height = ground_height
        self.exclude_from_ground_collision = ["base_link_0", "arm2_base_link_0"]
        self._add_ground_plane()
        self._init_collision_pairs()

        self.first_matrix = create_transformation_matrix(0, 0, 0, 0, -1.57, 0)
        # Transform from joint6 to end-effector gripper coordinates
        self.second_matrix = create_transformation_matrix(0.13, 0.0, 0.0, 0, 0, 0)
        self.last_matrix = np.dot(self.first_matrix, self.second_matrix)
        q = quaternion_from_matrix(self.last_matrix)
        self.reduced_robot.model.addFrame(
            pin.Frame(
                "ee",
                self.reduced_robot.model.getJointId("joint6"),
                pin.SE3(
                    pin.Quaternion(q[3], q[0], q[1], q[2]),
                    np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self.reduced_robot.model.addFrame(
            pin.Frame(
                "arm2_ee",
                self.reduced_robot.model.getJointId("arm2_joint6"),
                pin.SE3(
                    pin.Quaternion(q[3], q[0], q[1], q[2]),
                    np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )

        self.geometry_data = pin.GeometryData(self.geom_model)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.history_data = np.zeros(self.reduced_robot.model.nq)

        # Initialize the Meshcat visualizer for visualization
        self.vis = MeshcatVisualizer(
            self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model
        )
        self.vis.initViewer(open=True)
        self.vis.loadViewerModel("pinocchio")
        self.vis.displayFrames(True, frame_ids=[113, 114], axis_length=0.15, axis_width=5)
        self.vis.display(pin.neutral(self.reduced_robot.model))

        # Enable display of end effector target frames with short axis lengths
        frame_viz_names = ["ee_target_1", "ee_target_2"]
        FRAME_AXIS_POSITIONS = (
            np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
        )
        FRAME_AXIS_COLORS = (
            np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
        )
        axis_length = 0.1
        axis_width = 10
        for frame_viz_name in frame_viz_names:
            self.vis.viewer[frame_viz_name].set_object(
                mg.LineSegments(
                    mg.PointsGeometry(
                        position=axis_length * FRAME_AXIS_POSITIONS,
                        color=FRAME_AXIS_COLORS,
                    ),
                    mg.LineBasicMaterial(
                        linewidth=axis_width,
                        vertexColors=True,
                    ),
                )
            )

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Create generic symbolic variables
        cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, cq)

        # Create a unified solver for both arms
        self.solver = self._create_dual_ik_solver(cq)

    def _create_dual_ik_solver(self, cq):
        """Creates and configures a unified IK solver for both end-effectors."""
        # Define symbolic variables for the target poses first
        cTf1 = casadi.SX.sym("tf1", 4, 4)
        cTf2 = casadi.SX.sym("tf2", 4, 4)

        # Define the error function for the first arm using the pre-defined cTf1
        gripper_id_1 = self.reduced_robot.model.getFrameId("ee")
        error_expr1 = casadi.vertcat(cpin.log6(self.cdata.oMf[gripper_id_1].inverse() * cpin.SE3(cTf1)).vector)
        error1 = casadi.Function("error1", [cq, cTf1], [error_expr1])

        # Define the error function for the second arm using the pre-defined cTf2
        gripper_id_2 = self.reduced_robot.model.getFrameId("arm2_ee")
        error_expr2 = casadi.vertcat(cpin.log6(self.cdata.oMf[gripper_id_2].inverse() * cpin.SE3(cTf2)).vector)
        error2 = casadi.Function("error2", [cq, cTf2], [error_expr2])

        # Define the optimization problem
        opti = casadi.Opti()
        var_q = opti.variable(self.reduced_robot.model.nq)
        param_tf1 = opti.parameter(4, 4)
        param_tf2 = opti.parameter(4, 4)

        # Define the cost function for both arms
        error_vec1 = error1(var_q, param_tf1)
        pos_error1 = error_vec1[:3]
        ori_error1 = error_vec1[3:]

        error_vec2 = error2(var_q, param_tf2)
        pos_error2 = error_vec2[:3]
        ori_error2 = error_vec2[3:]

        weight_position = 1.0
        weight_orientation = 0.1

        cost_arm1 = casadi.sumsqr(weight_position * pos_error1) + casadi.sumsqr(weight_orientation * ori_error1)
        cost_arm2 = casadi.sumsqr(weight_position * pos_error2) + casadi.sumsqr(weight_orientation * ori_error2)

        total_cost = cost_arm1 + cost_arm2
        regularization = casadi.sumsqr(var_q)

        # Add constraints
        opti.subject_to(
            opti.bounded(
                self.reduced_robot.model.lowerPositionLimit, var_q, self.reduced_robot.model.upperPositionLimit
            )
        )

        # Set the objective
        opti.minimize(20 * total_cost + 0.01 * regularization)

        # Configure the solver
        opts = {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4}, "print_time": False}
        opti.solver("ipopt", opts)

        # Return a dictionary containing all necessary components for this solver
        return {"opti": opti, "var_q": var_q, "param_tf1": param_tf1, "param_tf2": param_tf2}

    def _init_collision_pairs(self):
        for i in range(self.geom_model.ngeoms):
            print("Geometry object:", self.geom_model.geometryObjects[i].name)

        arm1_indices = [0] + list(range(11, 20))
        arm2_indices = list(range(1, 11))

        for i in range(0, 3):
            for j in range(4, 9):
                geom1_idx = arm1_indices[i]
                geom2_idx = arm1_indices[j]
                # Avoid checking adjacent links
                if abs(geom1_idx - geom2_idx) > 1:
                    self.geom_model.addCollisionPair(pin.CollisionPair(geom1_idx, geom2_idx))

        for i in range(0, 3):
            for j in range(4, 9):
                geom1_idx = arm2_indices[i]
                geom2_idx = arm2_indices[j]
                # Avoid checking adjacent links
                if abs(geom1_idx - geom2_idx) > 1:
                    self.geom_model.addCollisionPair(pin.CollisionPair(geom1_idx, geom2_idx))

        for geom1_idx in arm1_indices:
            for geom2_idx in arm2_indices:
                # Exclude collision check between the two base links
                if geom1_idx == 0 and geom2_idx == 1:
                    continue
                self.geom_model.addCollisionPair(pin.CollisionPair(geom1_idx, geom2_idx))

        # Add collision pairs between robot links and ground plane
        ground_plane_idx = self.geom_model.ngeoms - 1
        for i in range(self.geom_model.ngeoms - 1):  # All robot links
            geom_name = self.geom_model.geometryObjects[i].name

            if geom_name not in self.exclude_from_ground_collision:
                self.geom_model.addCollisionPair(pin.CollisionPair(i, ground_plane_idx))
            else:
                logger.debug(f"Excluding {geom_name} from ground collision detection")

    def _add_ground_plane(self):
        """Add a ground plane geometry to the collision model for ground collision detection."""
        ground_size = [10.0, 10.0, 0.1]
        ground_pose = pin.SE3.Identity()
        ground_pose.translation = np.array([0.0, 0.0, self.ground_height - 0.05])

        ground_geometry = pin.GeometryObject("ground_plane", 0, pin.hppfcl.Box(*ground_size), ground_pose)
        self.geom_model.addGeometryObject(ground_geometry)
        logger.info(f"Added ground plane at height {self.ground_height}")

    def ik_fun(self, target_pose_1, target_pose_2, gripper=0, motorstate=None, motorV=None, visualize=True):
        opti = self.solver["opti"]
        var_q = self.solver["var_q"]
        param_tf1 = self.solver["param_tf1"]
        param_tf2 = self.solver["param_tf2"]

        gripper = np.array([gripper / 2.0, -gripper / 2.0])
        if motorstate is not None:
            self.init_data = motorstate
        opti.set_initial(var_q, self.init_data)

        if visualize:
            self.vis.viewer["ee_target_1"].set_transform(target_pose_1)
            self.vis.viewer["ee_target_2"].set_transform(target_pose_2)

        opti.set_value(param_tf1, target_pose_1)
        opti.set_value(param_tf2, target_pose_2)

        try:
            opti.solve_limited()
            sol_q = opti.value(var_q)

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                self.init_data = sol_q
                if max_diff > 30.0 / 180.0 * 3.1415:
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            if visualize:
                # print("sol_q:", sol_q)
                self.vis.display(sol_q)

            is_collision = self.check_collision(sol_q, gripper_1=gripper, gripper_2=gripper)
            return sol_q, is_collision

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            return None, False

    def check_collision(self, q, gripper_1=np.array([0, 0]), gripper_2=np.array([0, 0])):
        """Check for collisions including self-collision and ground plane collision."""
        pin.forwardKinematics(
            self.robot.model, self.robot.data, np.concatenate([q[0:6], gripper_1, q[6:12], gripper_2], axis=0)
        )
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        if collision:
            logger.error("❌ Collision detected")
        return collision

    def get_dist(self, q, xyz):
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        dist = math.sqrt(
            pow((xyz[0] - self.reduced_robot.data.oMi[6].translation[0]), 2)
            + pow((xyz[1] - self.reduced_robot.data.oMi[6].translation[1]), 2)
            + pow((xyz[2] - self.reduced_robot.data.oMi[6].translation[2]), 2)
        )
        return dist

    def get_pose(self, q):
        index = 6
        pin.forwardKinematics(self.reduced_robot.model, self.reduced_robot.data, np.concatenate([q], axis=0))
        end_pose = create_transformation_matrix(
            self.reduced_robot.data.oMi[index].translation[0],
            self.reduced_robot.data.oMi[index].translation[1],
            self.reduced_robot.data.oMi[index].translation[2],
            math.atan2(
                self.reduced_robot.data.oMi[index].rotation[2, 1], self.reduced_robot.data.oMi[index].rotation[2, 2]
            ),
            math.asin(-self.reduced_robot.data.oMi[index].rotation[2, 0]),
            math.atan2(
                self.reduced_robot.data.oMi[index].rotation[1, 0], self.reduced_robot.data.oMi[index].rotation[0, 0]
            ),
        )
        end_pose = np.dot(end_pose, self.last_matrix)
        return matrix_to_xyzrpy(end_pose)
