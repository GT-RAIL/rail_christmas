#!/usr/bin/env python
# This defines the robot states

from __future__ import print_function, division

import requests
import smach
import numpy as np

import rospy
import actionlib
import tf

from sound_play.libsoundplay import SoundClient

from hlpr_manipulation_utils.arm_moveit2 import ArmMoveIt as Arm
from hlpr_manipulation_utils.manipulator import Gripper

from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, Float64
from vector_msgs.msg import GripperCmd

from std_srvs.srv import Trigger, TriggerResponse

# Define the states used in the study

class HelpState(smach.State):
    """State where the human can help the robot during nav fail"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
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

        return 'done'


class AcceptCandyState(smach.State):
    """State for when the robot is waiting to initiate an interruption trial"""

    def __init__(
        self,
        accept_candy_location
    ):
        smach.State.__init__(
            self,
            outcomes=['done']
        )

        # Move Base
        self.accept_candy_location = accept_candy_location
        self.base_client = None

        # Local vars to block until grasp
        self._candy_in_gripper = False
        self.candy_in_gripper_server = rospy.Service('~candy_in_gripper', Trigger, self._handle_candy_in_gripper)

        # Gripper
        self.gripper = Gripper()

    def _handle_candy_in_gripper(self, req):
        """Indicate that there is candy in the gripper"""
        if not self._candy_in_gripper:
            self._candy_in_gripper = True
            self.gripper.close(force=10)
        return TriggerResponse(success=True)

    def execute(self, userdata):
        """Move to wait location and poll for time"""

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
            return 'help'

        # Wait until the candy is in the gripper
        while not self._candy_in_gripper:
            rospy.sleep(rospy.Duration(nsecs=1e8))

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
            outcomes=['place_candy'],
            input_keys=[],
            output_keys=['target_pose']
        )

        self.find_grasp_location = find_grasp_location
        self.find_grasp_pan = find_grasp_pan
        self.find_grasp_tilt = find_grasp_tilt
        self.tilt_publisher = rospy.Publisher(
            '/tilt_controller/command', Float64, queue_size=1
        )
        self.pan_publisher = rospy.Publisher(
            '/pan_controller/command', Float64, queue_size=1
        )

        self.tf_listener = tf.TransformListener()

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
            return 'help'

        self.tilt_publisher.publish(Float64(self.find_grasp_tilt))
        self.pan_publisher.publish(Float64(self.find_grasp_pan))

        # TODO call grasp finder (below is commented skeleton of what to do maybe?)
        # self.have_new_grasp_server = rospy.Service('~have_new_grasp', Trigger, self._handle_new_grasp)
    # def _handle_new_grasp(self, req):
    #     """Indicate that there is candy in the gripper"""
    #     if not self._have_new_grasp:
    #         self._have_new_grasp = True
    #         self.grasp = req.grasp_data
    #     return TriggerResponse(success=True)

        # while not self._have_new_grasp:
        #     rospy.sleep(rospy.Duration(nsecs=1e8))
        # userdata.target_pose = self.grasp
        # return 'done'

        return 'done'


class PlaceCandyState(smach.State):
    def __init__(
        self,
        tuck_pose
    ):
        smach.State.__init__(
            self,
            outcomes=['done', 'help'],
            input_keys=['target_pose'],
            output_keys=[]
        )

        # Arm
        self.arm = Arm()
        self.gripper = Gripper()

    def execute(self, userdata):
        """
        execute grasp
        :param userdata: smach data passed between states
        :return:
        """

        # Only needs to be a pose and not a pose stamped
        target = userdata.target_pose

        # TODO: Transform pose from grasp to orientation of gripper
        # TODO: Include an offset of positive Z so that the cane is above the
        # grasp location
        try:
            plan = arm.plan_ee_pos(target)
            execution = arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution returned False")
        except Exception as e:
            rospy.logerror("{}".format(e))
            return 'help'

        # TODO: Move the arm down to place the candy on the tree

        # Open the gripper
        self.gripper.open()

        # TODO: Might need a move in the negative X direction with the base?

        try:
            plan = arm.plan_ee_pos(tuck_pose)
            execution = arm.move_robot(plan)
            if not execution:
                rospy.logwarn("Execution returned False")
        except Exception as e:
            rospy.logerror("{}: IGNORING".format(e))

        return 'done'
