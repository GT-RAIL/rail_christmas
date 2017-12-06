#!/usr/bin/env python
# This defines the robot states

from __future__ import print_function, division

import requests
import smach
import numpy as np

import rospy
import actionlib
import tf
import tf2_ros
import tf2_geometry_msgs

from sound_play.libsoundplay import SoundClient

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt as Arm
from hlpr_manipulation_utils.manipulator import Gripper

from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, Float64
from vector_msgs.msg import GripperCmd

from octomap_msgs.srv import BoundingBoxQuery
from std_srvs.srv import Trigger, TriggerResponse

# Define the states used in the study

class HelpState(smach.State):
    """State where the human can help the robot during nav fail"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['accept', 'find', 'place'],
            input_keys=['target_pose', 'prev_state'],
            output_keys=['target_pose']
        )

        # Service server to transition the robot back to the correct state
        self.helped_server = rospy.Service('~helped', Trigger, self._handle_helped)
        self._helped = False

    def _handle_helped(self, req):
        if not self._helped:
            self._helped = True
        return TriggerResponse(success=True)

    def execute(self, userdata):
        """Wait until the service call proceeds to transition the robot onto the
        resume state"""
        self._helped = False
        rospy.loginfo("Awaiting assistance")

        while not self._helped:
            rospy.sleep(rospy.Duration(nsecs=1e8))

        return userdata.prev_state


class AcceptCandyState(smach.State):
    """State for when the robot is waiting to initiate an interruption trial"""

    def __init__(
        self,
        tuck_pose,
        accept_candy_location
    ):
        smach.State.__init__(
            self,
            outcomes=['done', 'help', 'end'],
            output_keys=['prev_state']
        )

        self.tuck_pose = tuck_pose
        self.arm = Arm()

        # Move Base
        self.accept_candy_location = accept_candy_location
        self.base_client = None

        # Local vars to block until grasp
        self._candy_in_gripper = False
        self.candy_in_gripper_server = rospy.Service('~candy_in_gripper', Trigger, self._handle_candy_in_gripper)
        self._decked_the_tree = False
        self.completed_server = rospy.Service('~decked_the_tree', Trigger, self._handle_completed)

        # Gripper
        self.gripper = Gripper()

        # Pan Tilt
        self.tilt_publisher = rospy.Publisher(
            '/tilt_controller/command', Float64, queue_size=1
        )
        self.pan_publisher = rospy.Publisher(
            '/pan_controller/command', Float64, queue_size=1
        )


    def _handle_candy_in_gripper(self, req):
        """Indicate that there is candy in the gripper"""
        if not self._candy_in_gripper:
            self._candy_in_gripper = True
            self.gripper.close(force=10)
        return TriggerResponse(success=True)

    def _handle_completed(self, req):
        """Indicate that all candies are done"""
        if not self._decked_the_tree:
            self._decked_the_tree = True
        return TriggerResponse(success=True)

    def execute(self, userdata):
        """Move to wait location and poll for time"""
        self.tilt_publisher.publish(Float64(0))
        self.pan_publisher.publish(Float64(0))

        try:
            plan = self.arm.plan_ee_pos(self.tuck_pose)
            execution = self.arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution returned False")
        except Exception as e:
            rospy.logerror("{}: IGNORING".format(e))
        self.gripper.open()

        rospy.loginfo("Arm is ready to accept candy")

        # Initialize the action client if that is not already true
        if self.base_client is None:
            self.base_client = actionlib.SimpleActionClient(
                'vector_move_base',
                MoveBaseAction
            )
            self.base_client.wait_for_server()

        # Create the goal for the robot to move towards
        accept_candy_loc_goal = MoveBaseGoal(target_pose=self.accept_candy_location)
        accept_candy_loc_goal.target_pose.header.stamp = rospy.Time.now()

        # Move the robot to that goal
        self.base_client.send_goal(accept_candy_loc_goal)
        self.base_client.wait_for_result()
        if self.base_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("wait: navigate to location failed")
            userdata.prev_state = 'accept'
            return 'help'

        # Wait until the candy is in the gripper
        rospy.loginfo("Waiting for candy")
        while not self._candy_in_gripper and not self._decked_the_tree:
            rospy.sleep(rospy.Duration(nsecs=1e8))

        if self._decked_the_tree:
            rospy.loginfo("Decked the tree with stalks of candy!")
            return 'end'

        rospy.sleep(rospy.Duration(secs=3))
        self._candy_in_gripper = False
        return 'done'


class FindGraspState(smach.State):
    def __init__(
        self,
        find_grasp_location,
        find_grasp_pan,
        find_grasp_tilt
    ):
        smach.State.__init__(
            self,
            outcomes=['done', 'help'],
            input_keys=[],
            output_keys=['target_pose', 'prev_state']
        )

        self.find_grasp_location = find_grasp_location
        self.find_grasp_pan = find_grasp_pan
        self.find_grasp_tilt = find_grasp_tilt
        self.grasp_pose = None
        self.tilt_publisher = rospy.Publisher(
            '/tilt_controller/command', Float64, queue_size=1
        )
        self.pan_publisher = rospy.Publisher(
            '/pan_controller/command', Float64, queue_size=1
        )

        self.grasp_subscriber = rospy.Subscriber('/generator/grasp_topic', PoseStamped, self.set_grasp)

        self.base_client = None

    def set_grasp(self, input_grasp_data):
        self.grasp_pose = input_grasp_data

    def execute(self, userdata):
        if self.base_client is None:
            self.base_client = actionlib.SimpleActionClient(
                'vector_move_base',
                MoveBaseAction
            )
            self.base_client.wait_for_server()
        find_grasp_location = MoveBaseGoal(target_pose=self.find_grasp_location)
        find_grasp_location.target_pose.header.stamp = rospy.Time.now()
        self.base_client.send_goal(find_grasp_location)
        self.base_client.wait_for_result()

        if self.base_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            userdata.prev_state = 'find'
            return 'help'

        self.tilt_publisher.publish(Float64(self.find_grasp_tilt))
        self.pan_publisher.publish(Float64(self.find_grasp_pan))

        rospy.loginfo("Waiting for grasp pose")
        while self.grasp_pose is None:
            rospy.sleep(rospy.Duration(nsecs=1e8))
        userdata.target_pose = self.grasp_pose
        self.grasp_pose = None
        return 'done'


class PlaceCandyState(smach.State):
    def __init__(
        self,
        retreat_location
    ):
        smach.State.__init__(
            self,
            outcomes=['done', 'help'],
            input_keys=['target_pose'],
            output_keys=['prev_state']
        )

        # Arm
        self.arm = Arm()
        self.gripper = Gripper()

        # Base
        self.retreat_location = retreat_location
        self.base_client = None

        # TF
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publisher1 = rospy.Publisher('~debug1', PoseStamped, queue_size=1)
        self.publisher2 = rospy.Publisher('~debug2', PoseStamped, queue_size=1)

    def execute(self, userdata):
        """
        execute grasp
        :param userdata: smach data passed between states
        :return:
        """

        # Only needs to be a pose and not a pose stamped
        # Get header / transform frame data from userdata.target_pose
        transform = self.tf_buffer.lookup_transform(
            "base_link",
            userdata.target_pose.header.frame_id,
            rospy.Time(0),
            rospy.Duration(1.0)
        )

        transformed_pose = tf2_geometry_msgs.do_transform_pose(
            userdata.target_pose,
            transform
        )

        # Create transformation_matrix and transform for candy cane
        rot = tf.transformations.quaternion_matrix(
            np.array([
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w,
            ])
        )[0:3,0:3]
        trans = np.array([
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z,

        ])
        transformation_matrix = np.vstack((
            np.concatenate((rot, trans[:,None]), axis=1),
            np.array([0,0,0,1])
        ))
        transformation_matrix = np.dot(
            transformation_matrix,\
            np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]])
            #np.array([[0,0,-1,0.115], [0,1,0,0], [1,0,0,0], [0,0,0,1]])
        )

        # Transform and then rotate
        trans = np.zeros((3,))
        trans[:] = transformation_matrix[0:3,3]
        rot = transformation_matrix
        rot[0:3,3] = 0
        rot[3,0:3] = 0
        rot[3,3] = 1

        # Go back to the tf transforms
        q = tf.transformations.quaternion_from_matrix(rot)
        self.publisher2.publish(transformed_pose)
        target = transformed_pose
        target.pose.position = Point(*list(trans))
        target.pose.orientation = Quaternion(*list(q))
        self.publisher1.publish(target)

        # Move the arm down to place the candy on the tree
        try:
            target.pose.position.z += 0.05
            target.pose.position.x -= 0.15
            plan = self.arm.plan_ee_pos(target.pose)
            execution = self.arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution1 returned False")

            target.pose.position.x += 0.15
            plan = self.arm.plan_ee_pos(target.pose)
            execution = self.arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution1 returned False")

            # Go Down
            target.pose.position.z -= 0.05
            plan = self.arm.plan_ee_pos(target.pose)
            execution = self.arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution1 returned False")
        except Exception as e:
            rospy.logerror("{}".format(e))
            userdata.prev_state = 'place'
            return 'help'

        # Open the gripper
        self.gripper.open()

        try:
            target.pose.position.x -= 0.15
            plan = self.arm.plan_ee_pos(target.pose)
            execution = self.arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution1 returned False")
        except Exception as e:
            rospy.logerror("{}. IGNORING".format(e))

        # Move backwards
        if self.base_client is None:
            self.base_client = actionlib.SimpleActionClient(
                'vector_move_base',
                MoveBaseAction
            )
            self.base_client.wait_for_server()

        # Create the goal for the robot to move towards
        retreat_loc_goal = MoveBaseGoal(target_pose=self.retreat_location)
        retreat_loc_goal.target_pose.header.stamp = rospy.Time.now()

        # Move the robot to that goal
        self.base_client.send_goal(retreat_loc_goal)
        self.base_client.wait_for_result()
        if self.base_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("retreat: navigate to location failed. IGNORING")

        return 'done'
