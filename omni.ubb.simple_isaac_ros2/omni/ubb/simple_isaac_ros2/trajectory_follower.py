import math
import time
import os, sys
print(os.environ["LD_LIBRARY_PATH"])
# exec(open("/opt/ros/foxy/setup.sh").read())
# from importlib.machinery import SourceFileLoader
# module = SourceFileLoader("control_msgs", "/opt/ros/foxy/lib/python3.8/site-packages/control_msgs/__init__.py").load_module()        
# print(dir(module))                            
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.duration import Duration
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
import asyncio
from omni.isaac.dynamic_control import _dynamic_control
TIME_STEP = 0.05


def to_s(duration):
    return Duration.from_msg(duration).nanoseconds / 1e9


class TrajectoryFollower(Node):
    """Create and handle the action 'follow_joint_trajectory' server."""

    def __init__(self, joints:dict, controller_name: str):
        super().__init__('trajectory_server')
        self.__dc = _dynamic_control.acquire_dynamic_control_interface()
        self.__art = self.__dc.get_articulation("/panda")
        # Config
        self.__default_tolerance = 0.05

        # Parse motor and position sensors
        self.__joints = joints

        # Initialize trajectory list and action server
        self.__current_point_index = 1
        self.__start_time = None
        self.__goal = None
        self.__mode = None
        self.__tolerances = {}
        self.__server = ActionServer(
            self,
            FollowJointTrajectory,
            controller_name + '/follow_joint_trajectory',
            execute_callback=self.__on_update,
            goal_callback=self.__on_goal,
            cancel_callback=self.__on_cancel,
        )

    def log(self, *args):
        self.get_logger().warn(' '.join([str(arg) for arg in args]))

    def __on_goal(self, goal_handle):
        """Handle a new goal trajectory command."""
        # Reject if joints don't match
        for name in goal_handle.trajectory.joint_names:
            if name not in self.__joints.keys():
                joint_names = ', '.join(goal_handle.trajectory.joint_names)
                self.get_logger().error(f'Received a goal with incorrect joint names: ({joint_names})')
                return GoalResponse.REJECT

        # Reject if infinity or NaN
        for point in goal_handle.trajectory.points:
            for position, velocity in zip(point.positions, point.velocities):
                if math.isinf(position) or math.isnan(position) or math.isinf(velocity) or math.isnan(velocity):
                    self.get_logger().error('Received a goal with infinites or NaNs')
                    return GoalResponse.REJECT

        # Reject if joints are already controlled
        if self.__goal is not None:
            self.get_logger().error('Cannot accept multiple goals')
            return GoalResponse.REJECT

        # Store goal data
        self.__goal = goal_handle
        self.__current_point_index = 1
        self.__start_time = self.get_clock().now().nanoseconds / 1e9

        for tolerance in self.__goal.goal_tolerance:
            self.__tolerances[tolerance.name] = tolerance.position
        for name in self.__goal.trajectory.joint_names:
            if name not in self.__tolerances.keys():
                self.__tolerances[name] = self.__default_tolerance

        self.__mode = 'velocity'
        for point in self.__goal.trajectory.points:
            if to_s(point.time_from_start) != 0:
                self.__mode = 'time'
                break

        # If a user forget the initial position
        if to_s(self.__goal.trajectory.points[0].time_from_start) != 0:
            initial_point = JointTrajectoryPoint(
                positions=[self.__get_joint_position(name) for name in self.__goal.trajectory.joint_names],
                time_from_start=Duration().to_msg()
            )
            self.__goal.trajectory.points.insert(0, initial_point)
        # Accept the trajectory
        self.get_logger().info('Goal Accepted')
        return GoalResponse.ACCEPT

    def __on_cancel(self, goal_handle):
        """Handle a trajectory cancel command."""
        if self.__goal is not None:
            # Stop motors
            for name in self.__goal.trajectory.joint_names:
                # joint_id = self.__joints[name]
                self.__set_joint_position(name,self.__get_joint_position(name))
            self.__goal = None
            self.get_logger().info('Goal Canceled')
            goal_handle.destroy()
            return CancelResponse.ACCEPT
        return CancelResponse.REJECT

    def __regulate_velocity_mode(self):
        curr_point = self.__goal.trajectory.points[self.__current_point_index]

        for index, name in enumerate(self.__goal.trajectory.joint_names):
            self.__set_joint_position(name, curr_point.positions[index])

        done = TrajectoryFollower.__is_within_tolerance(
            curr_point.positions,
            [self.__get_joint_position(name) for name in self.__goal.trajectory.joint_names],
            [self.__tolerances[name] for name in self.__goal.trajectory.joint_names],
        )
        if done:
            self.__current_point_index += 1
            if self.__current_point_index >= len(self.__goal.trajectory.points):
                return True
        return False

    def __regulate_time_mode(self):
        now = self.get_clock().now().nanoseconds / 1e9
        prev_point = self.__goal.trajectory.points[self.__current_point_index - 1]
        curr_point = self.__goal.trajectory.points[self.__current_point_index]
        time_passed = now - self.__start_time

        if time_passed <= to_s(curr_point.time_from_start):
            # Linear interpolation
            ratio = (time_passed - to_s(prev_point.time_from_start)) /\
                (to_s(curr_point.time_from_start) - to_s(prev_point.time_from_start))
            for index, name in enumerate(self.__goal.trajectory.joint_names):
                side = -1 if curr_point.positions[index] < prev_point.positions[index] else 1
                target_position = prev_point.positions[index] + \
                    side * ratio * abs(curr_point.positions[index] - prev_point.positions[index])
                self.__set_joint_position(name, target_position)
        else:
            self.__current_point_index += 1
            if self.__current_point_index >= len(self.__goal.trajectory.points):
                return True
        return False

    def __set_joint_position(self, name, target_position):
        # joint_id = self.__joints[name]
        dof_ptr = self.__dc.find_articulation_dof(self.__art, name)
        # This should be called each frame of simulation if state on the articulation is being changed.
        self.__dc.wake_up_articulation(self.__art)
        # Set joint position target
        self.__dc.set_dof_position_target(dof_ptr, target_position)

    def __get_joint_position(self, name):

        # Get state for a specific degree of freedom
        dof_ptr = self.__dc.find_articulation_dof(self.__art, name)
        dof_state = self.__dc.get_dof_state(dof_ptr)
        return dof_state.pos


    async def __on_update(self, goal_handle):
        feedback_message = FollowJointTrajectory.Feedback()
        feedback_message.joint_names = list(self.__goal.trajectory.joint_names)
        while self.__goal:
            done = False

            # Regulate
            if self.__mode == 'time':
                done = self.__regulate_time_mode()
            else:
                done = self.__regulate_velocity_mode()

            # Finalize
            if done:
                self.get_logger().info('Goal Succeeded')
                self.__goal = None
                goal_handle.succeed()
                result = FollowJointTrajectory.Result()
                result.error_code = result.SUCCESSFUL
                return result

            # Publish state
            time_passed = self.get_clock().now().nanoseconds / 1e9 - self.__start_time
            feedback_message.actual.positions = [self.__get_joint_position(name)
                                                 for name in self.__goal.trajectory.joint_names]
            feedback_message.actual.time_from_start = Duration(seconds=time_passed).to_msg()
            goal_handle.publish_feedback(feedback_message)

            time.sleep(TIME_STEP)

        result = FollowJointTrajectory.Result()
        result.error_code = result.PATH_TOLERANCE_VIOLATED
        return result

    @staticmethod
    def __is_within_tolerance(a_vec, b_vec, tol_vec):
        """Check if two vectors are equals with a given tolerance."""
        for a, b, tol in zip(a_vec, b_vec, tol_vec):
            if abs(a - b) > tol:
                return False
        return True