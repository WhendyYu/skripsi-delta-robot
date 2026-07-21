
"""
Delta Robot Controller — Dynamic trajectory version

Upgrades:
- Dynamic joint trajectory timing
- Trapezoidal motion constraints
- Physically scaled segment durations
- Better stepper compatibility
- Ready for FastAccelStepper-based ESP32 execution
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point
from std_msgs.msg import Bool, UInt8, Empty, UInt16
from std_srvs.srv import Trigger

from msgs.msg import TrajectoryCommand, TrajectoryPoint, RobotStatus
from msgs.srv import SolveIK, CaptureWorkspace, MoveTo
from msgs.action import PickAndPlace


class DeltaController(Node):

    def __init__(self):
        super().__init__("delta_controller")

        # ================================================================
        # Parameters
        # ================================================================

        self.declare_parameter("max_velocity", 300.0)
        self.declare_parameter("clearance_height", -250.0)
        self.declare_parameter("approach_height", 10.0)
        self.declare_parameter("auto_initialize", True)
        self.declare_parameter("init_timeout", 20.0)
        self.declare_parameter("trajectory_timeout", 10.0)
        self.declare_parameter("reinitialize_after_action", True)
        self.declare_parameter("home_position", [0.0, 0.0, -250.0])
        self.declare_parameter("home_move_duration", 2.0)
        self.declare_parameter("joint_max_velocity", 180.0)
        self.declare_parameter("joint_max_acceleration", 720.0)
        self.declare_parameter("grip_settle_time", 0.3)
        self.declare_parameter('vision_enabled',False)
        self.declare_parameter('home_after_action',True)

        self.v_max = self.get_parameter("max_velocity").value
        self.z_safe = self.get_parameter("clearance_height").value
        self.z_approach = self.get_parameter("approach_height").value
        self.traj_timeout = self.get_parameter("trajectory_timeout").value
        self.home_position = np.array(self.get_parameter("home_position").value)
        self.home_move_duration = self.get_parameter("home_move_duration").value
        self.joint_amax = self.get_parameter("joint_max_acceleration").value
        self.grip_settle_time = self.get_parameter("grip_settle_time").value
        self.joint_vmax = self.get_parameter("joint_max_velocity").value
        self.joint_amax = self.get_parameter("joint_max_acceleration").value
        self.vision_enabled = (self.get_parameter('vision_enabled').get_parameter_value().bool_value)
        self.home_after_action = (self.get_parameter('home_after_action').get_parameter_value().bool_value)

        # ================================================================
        # State
        # ================================================================

        self.lock = threading.Lock()
        self.init_lock = threading.Lock()

        self.pos = self.home_position.copy()

        self.is_busy = False
        self.is_initialized = False

        self.current_traj = 0
        self.last_status = None
        self.home_joint_angles = None


        self.current_vacuum = 0
        self.current_servo_angle = 0

        self.traj_event = threading.Event()
        self.traj_done_event = threading.Event()
        self.status_event = threading.Event()

        # ================================================================
        # Server client
        # ================================================================

        self.ik_client = self.create_client(
                                        SolveIK, 
                                        "/solve_ik",
                                        callback_group=ReentrantCallbackGroup())

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for IK service...")

        # ================================================================
        # Publishers
        # ================================================================

        self.cmd_pub = self.create_publisher(
            TrajectoryCommand,
            "/robot/commands",
            10
        )

        self.grip_pub = self.create_publisher(
            Bool,
            "/gripper/command",
            10
        )

        self.servo_pub = self.create_publisher(
            UInt8,
            "/servo/command",
            10
        )

        self.enable_pub = self.create_publisher(
            Bool,
            "/robot/enable_motors",
            10
        )

        self.status_req_pub = self.create_publisher(
            Empty,
            "/robot/get_status",
            10
        )

        self.init_pub = self.create_publisher(
            Empty,
            "/robot/init",
            10
        )

        # ================================================================
        # Subscribers
        # ================================================================

        self.create_subscription(
            RobotStatus,
            "/robot/status",
            self.on_robot_status,
            10
        )

        self.create_subscription(
            UInt16,
            "/robot/trajectory_complete",
            self.on_trajectory_complete,
            10
        )

        # ================================================================
        # Action server
        # ================================================================

        self.action = ActionServer(
            self,
            PickAndPlace,
            "/pick_and_place",
            self.handle_action,
            goal_callback=self.goal_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # ================================================================
        # Services
        # ================================================================

        self.create_service(
            Trigger,
            "/initialize_robot",
            self.init_service_callback
        )

        self.create_service(
        MoveTo,
        "/move_to",
        self.move_to_callback
    )

        # ================================================================
        # Auto init
        # ================================================================

        self.init_timer = None

        if self.get_parameter("auto_initialize").value:
            self.init_timer = self.create_timer(
                1.0,
                self.auto_init_callback,
                callback_group=ReentrantCallbackGroup()
            )

        self.get_logger().info("Delta Controller initialized")

    
    # ================================================================
    # TRAJECTORY TIMING
    # ================================================================

    def compute_segment_time(self, q0, q1):
        """
        Compute trajectory duration using trapezoidal profile.

        q0/q1:
            joint angle vectors in degrees
        """

        delta = np.max(np.abs(q1 - q0))

        if delta < 1e-3:
            return 0.05

        vmax = self.joint_vmax
        amax = self.joint_amax

        # acceleration time
        t_accel = vmax / amax

        # accel distance
        d_accel = 0.5 * amax * t_accel**2

        # triangular profile
        if delta < 2.0 * d_accel:

            t_accel = np.sqrt(delta / amax)
            total = 2.0 * t_accel

        # trapezoidal profile
        else:

            d_cruise = delta - 2.0 * d_accel
            t_cruise = d_cruise / vmax

            total = 2.0 * t_accel + t_cruise

        # safety factor
        return total * 1.15

    # ================================================================
    # INITIALIZATION
    # ================================================================

    def auto_init_callback(self):

        with self.lock:
            if self.is_initialized or self.is_busy:
                return

        self.initialize_robot()


    def init_service_callback(self, request, response):

        self.get_logger().info("Forced reinitialization requested")

        with self.lock:
            self.is_initialized = False

        success = self.initialize_robot()

        response.success = success
        response.message = (
            "Reinitialized"
            if success
            else "Reinitialization failed"
        )

        return response


    def _solve_home_ik(self):

        if self.home_joint_angles is not None:
            return True

        self.get_logger().info("Solving IK for home position...")

        ik = self.solve_ik(self.home_position)

        if ik is None:
            self.get_logger().error("IK failed for home position")
            return False

        self.home_joint_angles = np.array(ik, dtype=float)

        self.get_logger().info(
            f"Home joint angles: {self.home_joint_angles}"
        )

        return True


    def _wait_for_status(self, timeout_sec):

        self.status_event.clear()

        self.status_req_pub.publish(Empty())

        return self.status_event.wait(timeout=timeout_sec)


    def _wait_for_motors_enabled(self, timeout_sec):

        deadline = time.time() + timeout_sec

        while time.time() < deadline:

            if self._wait_for_status(timeout_sec=1.0):

                with self.lock:
                    status = self.last_status

                if (
                    status is not None
                    and status.motors_enabled
                ):
                    return True

            time.sleep(0.05)

        return False

    def move_to_callback(
        self,
        request,
        response
    ):

        with self.lock:

            if self.is_busy:

                response.success = False
                response.message = "Robot busy"

                return response

            self.is_busy = True

        try:

            target = np.array([
                request.position.x,
                request.position.y,
                request.position.z
            ])

            self.current_servo_angle = int(
                request.orientation
            )

            self.current_vacuum = int(
                request.vacuum
            )

            self.move(target)

            response.success = True
            response.message = "Move complete"

        except Exception as e:

            response.success = False
            response.message = str(e)

            self.get_logger().error(
                f"MoveTo failed: {e}"
            )

        finally:

            with self.lock:
                self.is_busy = False

        return response

    def initialize_robot(self):
        """
        Deterministic initialization flow.

        MCU responsibilities:
            - read encoders
            - synchronize step counters
            - clear queues
            - publish synchronized=True

        ROS responsibilities:
            - verify synchronization
            - enable motors
            - command explicit move-to-home trajectory
        """

        with self.init_lock:

            if not self._solve_home_ik():
                return False

            with self.lock:
                if self.is_initialized:
                    return True

            self.get_logger().info(
                "INITIALIZATION START"
            )

            # ----------------------------------------------------------
            # Step 1: request current status
            # ----------------------------------------------------------

            if not self._wait_for_status(timeout_sec=3.0):

                self.get_logger().error(
                    "No status response from robot"
                )

                return False

            with self.lock:
                status = self.last_status

            if status is None:

                self.get_logger().error(
                    "Robot status unavailable"
                )

                return False

            # ----------------------------------------------------------
            # Step 2: send init command to MCU
            # ----------------------------------------------------------

            self.get_logger().info(
                "Sending MCU INIT..."
            )

           # Clear old cached status
            with self.lock:
                self.last_status = None

            self.status_event.clear()

            # Send INIT
            self.init_pub.publish(Empty())

            # Wait for NEW status packet
            if not self.status_event.wait(timeout=3.0):

                self.get_logger().error(
                    "No fresh status after INIT"
                )

                return False

            with self.lock:
                status = self.last_status

            if status is None:

                self.get_logger().error(
                    "Fresh status missing after INIT"
                )

                return False
            # ----------------------------------------------------------
            # Step 3: enable motors
            # ----------------------------------------------------------

            if not status.motors_enabled:

                self.get_logger().info(
                    "Enabling motors..."
                )

                self.enable_pub.publish(
                    Bool(data=True)
                )

                if not self._wait_for_motors_enabled(
                    timeout_sec=5.0
                ):

                    self.get_logger().error(
                        "Motors failed to enable"
                    )

                    return False

            # ----------------------------------------------------------
            # Step 4: request fresh status after enable
            # ----------------------------------------------------------

            if not self._wait_for_status(timeout_sec=2.0):

                self.get_logger().error(
                    "No status after enabling motors"
                )

                return False

            with self.lock:
                status = self.last_status

            if status is None:

                self.get_logger().error(
                    "Status lost after motor enable"
                )

                return False

            self.get_logger().info(
                f"[STATUS CACHE READ] "
                f"angles={status.current_angles} "
                f"traj={status.trajectory_id}"
            )

            current_angles = np.array(
                status.current_angles,
                dtype=float
            )

            self.get_logger().info(
                f"Current synchronized angles: "
                f"{current_angles}"
            )

            # ----------------------------------------------------------
            # Step 5: explicit move to home
            # ----------------------------------------------------------

            self.get_logger().info(
                "Moving robot to home position..."
            )

            try:

                self._move_joints(
                    target_angles=self.home_joint_angles,
                    start_angles=current_angles
                )

            except Exception as e:

                self.get_logger().error(
                    f"Home move failed: {e}"
                )

                return False

            # ----------------------------------------------------------
            # Step 6: verify robot idle
            # ----------------------------------------------------------

            if not self._wait_for_status(timeout_sec=2.0):

                self.get_logger().error(
                    "No status after home move"
                )

                return False

            with self.lock:
                status = self.last_status

            if status is None:

                self.get_logger().error(
                    "Robot status unavailable after home move"
                )

                return False

            if status.state != 0:

                self.get_logger().error(
                    f"Robot not idle after init "
                    f"(state={status.state})"
                )

                return False

            # ----------------------------------------------------------
            # Step 7: initialization success
            # ----------------------------------------------------------

            with self.lock:

                self.is_initialized = True

                self.pos = self.home_position.copy()

            self.get_logger().info(
                "=== INITIALIZATION SUCCESS ==="
            )

            return True

    # ================================================================
    # JOINT MOTION
    # ================================================================

    def _move_joints(
        self,
        target_angles,
        start_angles=None
    ):

        if start_angles is None:

            with self.lock:

                if self.last_status is None:
                    raise RuntimeError(
                        "No status available"
                    )

                start_angles = (
                    self.last_status.current_angles
                )

        q0 = np.array(
            start_angles,
            dtype=float
        )

        q1 = np.array(
            target_angles,
            dtype=float
        )

        duration = self.compute_segment_time(
            q0,
            q1
        )

        cmd = TrajectoryCommand()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # ============================================================
        # Point 0
        # ============================================================

        pt0 = TrajectoryPoint()

        pt0.joint_angles = q0.tolist()

        pt0.time_ms = 0

        pt0.vacuum = (
            self.current_vacuum
        )

        pt0.servo_angle = (
            self.current_servo_angle
        )

        pt0.reserved = 0

        # ============================================================
        # Point 1
        # ============================================================

        pt1 = TrajectoryPoint()

        pt1.joint_angles = q1.tolist()

        pt1.time_ms = int(
            duration * 1000.0
        )

        pt1.vacuum = (
            self.current_vacuum
        )

        pt1.servo_angle = (
            self.current_servo_angle
        )

        pt1.reserved = 0

        cmd.points.append(pt0)

        cmd.points.append(pt1)

        with self.lock:

            self.current_traj = (
                self.current_traj + 1
            ) % 65535

            current_id = (
                self.current_traj
            )

        cmd.trajectory_id = current_id

        self.traj_done_event.clear()

        self.cmd_pub.publish(cmd)

        if not self.traj_done_event.wait(
            timeout=self.traj_timeout
        ):
            raise RuntimeError(
                f"Joint trajectory "
                f"{current_id} timeout"
            )

    # ================================================================
    # ACTION SERVER
    # ================================================================

    def goal_callback(self, goal):

        with self.lock:

            if self.is_busy:
                self.get_logger().warn("Rejecting goal: busy")
                return GoalResponse.REJECT

            if not self.is_initialized:
                self.get_logger().warn("Rejecting goal: not initialized")
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def handle_action(self, goal_handle):

        with self.lock:
            self.is_busy = True

        start = time.time()

        result = PickAndPlace.Result()

        try:

            pick = self._p(
                goal_handle.request.pick_position
            )

            place = self._p(
                goal_handle.request.place_position
            )

            # ====================================================
            # PICK
            # ====================================================

            self.get_logger().info(
                "--- Pick sequence ---"
            )

            self.current_servo_angle = int(
                goal_handle.request.pick_orientation
            )

            self.move(pick)

            # close gripper
            self.gripper(True)

            # vacuum settle delay
            time.sleep(
                self.grip_settle_time
            )

            self.lift()

            # ====================================================
            # PLACE
            # ====================================================

            self.get_logger().info(
                "--- Place sequence ---"
            )

            self.current_servo_angle = int(
                goal_handle.request.place_orientation
            )

            self.move(place)

            # open gripper
            self.gripper(False)

            self.retract()

            # ====================================================
            # RETURN HOME
            # ====================================================

            if self.home_after_action:

                self.current_servo_angle = 0

                self.return_home()

            result.success = True

            result.execution_time = (
                time.time() - start
            )

            goal_handle.succeed()

        except Exception as e:

            self.get_logger().error(
                f"Action failed: {e}"
            )

            result.success = False

            result.error_message = str(e)

            goal_handle.abort()

        finally:

            with self.lock:
                self.is_busy = False

        return result

    # ================================================================
    # CARTESIAN MOTION
    # ================================================================

    def move(self, target):

        waypoints = self.plan(
            self.pos,
            target
        )

        traj = self.build_trajectory(
            waypoints
        )

        with self.lock:

            self.current_traj = (
                self.current_traj + 1
            ) % 65535

            current_id = (
                self.current_traj
            )

        traj.trajectory_id = current_id

        self.traj_done_event.clear()

        self.cmd_pub.publish(traj)

        if not self.traj_done_event.wait(
            timeout=self.traj_timeout
        ):
            raise RuntimeError(
                f"Trajectory "
                f"{current_id} timeout"
            )

        with self.lock:

            self.pos = target

    def lift(self):

        p = self.pos.copy()
        p[2] = self.z_safe

        self.move(p)

    def retract(self):

        p = self.pos.copy()
        p[2] += self.z_approach + 5

        self.move(p)

    def return_home(self):

        self.get_logger().info("Returning to home position")

        self.move(self.home_position.copy())

    def plan(self, start, end):
        """
        Cartesian waypoint generation only.
        Timing is now computed dynamically later.
        """

        pts = []

        pts.append(start.copy())

        above = np.array([
            end[0],
            end[1],
            self.z_safe
        ])

        pts.append(above)

        approach = np.array([
            end[0],
            end[1],
            end[2] + self.z_approach
        ])

        pts.append(approach)

        pts.append(end.copy())

        return pts

    # ================================================================
    # TRAJECTORY BUILDING
    # ================================================================

    def build_trajectory(
        self,
        waypoints
    ):

        cmd = TrajectoryCommand()

        cmd.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        t = 0.0

        prev_q = None

        if len(waypoints) > 10:

            raise RuntimeError(
                "Trajectory exceeds "
                "10 point limit"
            )

        for pos in waypoints:

            ik = self.solve_ik(pos)

            if ik is None:

                raise RuntimeError(
                    "IK failure"
                )

            q = np.array(
                ik,
                dtype=float
            )

            with self.lock:

                if self.last_status is not None:

                    current = np.array(
                        self.last_status.current_angles,
                        dtype=float
                    )

                    for i in range(3):

                        q[i] = (
                            self.nearest_equivalent(
                                q[i],
                                current[i]
                            )
                        )

            # ========================================================
            # Timing
            # ========================================================

            if prev_q is None:

                dt = 0.0

            else:

                dt = (
                    self.compute_segment_time(
                        prev_q,
                        q
                    )
                )

            t += dt

            # ========================================================
            # Build Point
            # ========================================================

            pt = TrajectoryPoint()

            pt.joint_angles = q.tolist()

            pt.time_ms = int(
                t * 1000.0
            )

            pt.vacuum = (
                self.current_vacuum
            )

            pt.servo_angle = (
                self.current_servo_angle
            )

            pt.reserved = 0

            cmd.points.append(pt)

            prev_q = q

        return cmd

    # ================================================================
    # IK
    # ================================================================

    def normalize_angle(self, angle):

        return ((angle + 180.0) % 360.0) - 180.0

    def solve_ik(self, p):

        req = SolveIK.Request()

        req.positions = [
            Point(
                x=float(p[0]),
                y=float(p[1]),
                z=float(p[2])
            )
        ]

        future = self.ik_client.call_async(req)

        done_event = threading.Event()

        future.add_done_callback(lambda _: done_event.set())

        if not done_event.wait(timeout=2.0):
            self.get_logger().warn("IK timeout")
            return None

        result = future.result()

        if result is None:
            return None

        if hasattr(result, "results") and len(result.results) > 0:

            r = result.results[0]

            if getattr(r, "success", False):
                return r.joint_angles[:3]

        if getattr(result, "success", False):
            return result.joint_angles[:3]

        return None

    def nearest_equivalent(self, target, current):

        best = target
        best_error = abs(target - current)

        for k in range(-5, 6):

            candidate = target + (360.0 * k)

            error = abs(candidate - current)

            if error < best_error:

                best = candidate
                best_error = error

        return best

    # ================================================================
    # EVENTS
    # ================================================================

    def on_robot_status(self, msg):

        with self.lock:
            self.last_status = msg

        self.status_event.set()

    def on_trajectory_complete(self, msg):

        with self.lock:
            current = self.current_traj

        if msg.data == current:
            self.traj_done_event.set()
            self.traj_event.set()

    # ================================================================
    # I/O
    # ================================================================

    def gripper(self, close):

        self.current_vacuum = int(close)

        self.grip_pub.publish(
            Bool(data=close)
        )

    def servo(self, angle):

        angle = int(angle)

        self.current_servo_angle = angle

        self.servo_pub.publish(
            UInt8(data=angle)
        )


    def _p(self, point):

        return np.array([
            point.x,
            point.y,
            point.z
        ])


# ================================================================
# MAIN
# ================================================================


def main():

    rclpy.init()

    node = DeltaController()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=6)

    executor.add_node(node)

    try:
        executor.spin()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()