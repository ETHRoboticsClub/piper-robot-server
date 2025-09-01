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


def calc_pose_incre(base_pose, pose_data):
    begin_matrix = create_transformation_matrix(
        base_pose[0], base_pose[1], base_pose[2], base_pose[3], base_pose[4], base_pose[5]
    )
    zero_matrix = create_transformation_matrix(0.19, 0.0, 0.2, 0, 0, 0)
    end_matrix = create_transformation_matrix(
        pose_data[0], pose_data[1], pose_data[2], pose_data[3], pose_data[4], pose_data[5]
    )
    result_matrix = np.dot(zero_matrix, np.dot(np.linalg.inv(begin_matrix), end_matrix))
    xyzrpy = matrix_to_xyzrpy(result_matrix)
    return xyzrpy


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

        self.mixed_jointsToLockIDs = ["joint7", "joint8"]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0] * self.robot.model.nq),
        )

        # Add ground plane geometry to collision model
        self.ground_height = ground_height
        self.exclude_from_ground_collision = ["base_link_0"]
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
                    # pin.Quaternion(1, 0, 0, 0),
                    pin.Quaternion(q[3], q[0], q[1], q[2]),
                    np.array([self.last_matrix[0, 3], self.last_matrix[1, 3], self.last_matrix[2, 3]]),  # -y
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
        frame_viz_names = ["ee_target"]
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

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf = casadi.SX.sym("tf", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.gripper_id = self.reduced_robot.model.getFrameId("ee")
        self.error = casadi.Function(
            "error",
            [self.cq, self.cTf],
            [
                casadi.vertcat(
                    cpin.log6(self.cdata.oMf[self.gripper_id].inverse() * cpin.SE3(self.cTf)).vector,
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        # For smooth motion (commented out)
        # self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)
        self.param_tf = self.opti.parameter(4, 4)

        # self.totalcost = casadi.sumsqr(self.error(self.var_q, self.param_tf))
        # self.regularization = casadi.sumsqr(self.var_q)

        error_vec = self.error(self.var_q, self.param_tf)
        pos_error = error_vec[:3]
        ori_error = error_vec[3:]
        weight_position = 1.0
        weight_orientation = 0.1
        self.totalcost = casadi.sumsqr(weight_position * pos_error) + casadi.sumsqr(weight_orientation * ori_error)
        self.regularization = casadi.sumsqr(self.var_q)
        self.opti.subject_to(
            self.opti.bounded(
                self.reduced_robot.model.lowerPositionLimit, self.var_q, self.reduced_robot.model.upperPositionLimit
            )
        )
        # Debug prints (commented out)
        # print("lowerPositionLimit:", self.reduced_robot.model.lowerPositionLimit)
        # print("upperPositionLimit:", self.reduced_robot.model.upperPositionLimit)
        self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization)
        # For smooth motion (commented out)
        # self.opti.minimize(20 * self.totalcost + 0.01 * self.regularization + 0.1 * self.smooth_cost)

        opts = {"ipopt": {"print_level": 0, "max_iter": 50, "tol": 1e-4}, "print_time": False}
        self.opti.solver("ipopt", opts)

    def _init_collision_pairs(self):
        # Add collision pairs for self-collision detection
        for i in range(4, 9):
            for j in range(0, 3):
                self.geom_model.addCollisionPair(pin.CollisionPair(i, j))

        # Add collision pairs between robot links and ground plane
        # Ground plane is the last geometry object added
        ground_plane_idx = self.geom_model.ngeoms - 1
        for i in range(self.geom_model.ngeoms - 1):  # All robot links
            # Check if this geometry should be excluded from ground collision
            geom_name = self.geom_model.geometryObjects[i].name

            if geom_name not in self.exclude_from_ground_collision:
                self.geom_model.addCollisionPair(pin.CollisionPair(i, ground_plane_idx))
            else:
                logger.debug(f"Excluding {geom_name} from ground collision detection")

    def _add_ground_plane(self):
        """Add a ground plane geometry to the collision model for ground collision detection."""
        # Create a large box representing the ground plane at the specified height
        # The box extends from ground_height-0.05 to ground_height+0.05 to create a thin ground plane
        ground_size = [10.0, 10.0, 0.1]  # Large XY plane, thin in Z
        ground_pose = pin.SE3.Identity()
        ground_pose.translation = np.array([0.0, 0.0, self.ground_height - 0.05])

        # Create ground plane geometry
        ground_geometry = pin.GeometryObject(
            "ground_plane", 0, pin.hppfcl.Box(*ground_size), ground_pose  # Attach to world frame (frame 0)
        )

        # Add to geometry model
        self.geom_model.addGeometryObject(ground_geometry)
        logger.info(f"Added ground plane at height {self.ground_height}")

    def ik_fun(self, target_pose, gripper=0, motorstate=None, motorV=None, visualize=True):
        gripper = np.array([gripper / 2.0, -gripper / 2.0])
        if motorstate is not None:
            self.init_data = motorstate
        self.opti.set_initial(self.var_q, self.init_data)

        if visualize:
            # For visualization
            self.vis.viewer["ee_target"].set_transform(target_pose)

        self.opti.set_value(self.param_tf, target_pose)
        # For smooth motion (commented out)
        # self.opti.set_value(self.var_q_last, self.init_data)

        try:
            # sol = self.opti.solve()
            self.opti.solve_limited()
            sol_q = self.opti.value(self.var_q)

            if self.init_data is not None:
                max_diff = max(abs(self.history_data - sol_q))
                # print("max_diff:", max_diff)
                self.init_data = sol_q
                if max_diff > 30.0 / 180.0 * 3.1415:
                    # print("Excessive changes in joint angle:", max_diff)
                    self.init_data = np.zeros(self.reduced_robot.model.nq)
            else:
                self.init_data = sol_q
            self.history_data = sol_q

            if visualize:
                self.vis.display(sol_q)  # for visualization

            is_collision = self.check_collision(sol_q, gripper)
            return sol_q, is_collision

        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")
            # return original value (commented out)
            # sol_q = self.opti.debug.value(self.var_q)
            return None, False

    def check_collision(self, q, gripper=np.array([0, 0])):
        """Check for collisions including self-collision and ground plane collision."""
        pin.forwardKinematics(self.robot.model, self.robot.data, np.concatenate([q, gripper], axis=0))
        pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.geom_model, self.geometry_data)
        collision = pin.computeCollisions(self.geom_model, self.geometry_data, False)
        if collision:
            logger.error("❌ Collision detected")
        return collision

    def get_dist(self, q, xyz):
        # print("q:", q)
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
