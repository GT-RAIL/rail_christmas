#!/usr/bin/env python
# This defines the robot states

import requests
import smach
import numpy as np

import rospy
import actionlib
import tf

from sound_play.libsoundplay import SoundClient

from geometry_msgs.msg import PointStamped, Point
from interruptibility_msgs.msg import Interruptibility
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty, Float64

from christmas_state_machine.srv import EndHelp, EndHelpResponse, \
                                    EndManual, EndManualResponse, EndManualRequest
from std_srvs.srv import Trigger, TriggerResponse

# Define the states used in the study

class HelpState(smach.State):
    """State where the human can help the robot and transition it out of
    whatever situation it is stuck at"""

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['wait', 'accept_candy', 'place_candy', 'manual', 'post'],
            input_keys=['location', 'choice'],
            output_keys=['location', 'choice']
        )

        # Service server to transition the robot back to the correct state
        self.service_server = rospy.Service(
            '~end_help',
            EndHelp,
            self._handle_end_help
        )
        self.help_needed = False
        self.next_state = None

    def _handle_end_help(self, req):
        if self.help_needed:
            self.next_state = req.next_state
        return EndHelpResponse(success=True)

    def execute(self, userdata):
        """Wait until the service call proceeds to transition the robot onto the
        resume state"""

        # Enable the service and wait for a response
        self.help_needed = True
        self.next_state = None
        while self.next_state is None:
            rospy.sleep(rospy.Duration(nsecs=1e3))

        rospy.logdebug("help: next_state - %s" % self.next_state)
        self.help_needed = False
        return self.next_state


class ManualState(smach.State):
    """State for explicit manual control of the robot between interruption
    cycles"""

    def __init__(self, build_sound_name, verify_sound_name, complete_sound_name):
        smach.State.__init__(
            self,
            outcomes=['done'],
            input_keys=['choice'],
            output_keys=['choice']
        )

        # Service server to end manual control
        self.end_service_server = rospy.Service(
            '~end_manual',
            EndManual,
            self._handle_end_manual
        )

        self.verify_service_server = rospy.Service(
            '~verify',
            Trigger,
            self._handle_verify_prompt
        )

        self.complete_service_server = rospy.Service(
            '~complete',
            Trigger,
            self._handle_complete_prompt
        )

        self.manual_choice = None
        self.manual_control = False
        self.build_sound_name = build_sound_name
        self.verify_sound_name = verify_sound_name
        self.complete_sound_name = complete_sound_name
        self.sound_client = None

    def _handle_end_manual(self, req):
        """Completes the transition out of the manual state"""
        if self.manual_control:
            self.manual_choice = req.choice
            self.manual_control = False
        return EndManualResponse(success=True)

    def _handle_verify_prompt(self, req):
        """Forces the robot to say the verify sound"""
        if self.sound_client is None:
            return TriggerResponse(success=False)
        self.sound_client.waveSound(self.verify_sound_name).play()
        return TriggerResponse(success=True)

    def _handle_complete_prompt(self, req):
        """Forces the robot to say the complete sound"""
        if self.sound_client is None:
            return TriggerResponse(success=False)
        self.sound_client.waveSound(self.complete_sound_name).play()
        return TriggerResponse(success=True)

    def execute(self, userdata):
        """Hand over manual control to the human"""
        if self.sound_client is None:
            self.sound_client = SoundClient(blocking=False)

        # Check to see if we need to follow the ignored/not found dialog or the
        # build complete dialog
        if len(userdata.keys()) == 0:
            userdata.choice = ''

        if userdata.choice == EndManualRequest.IGNORE \
        or userdata.choice == EndManualRequest.POST:
            self.sound_client.waveSound(self.build_sound_name).play()
        else: # choice == 'interrupt' || ''
            self.sound_client.waveSound(self.verify_sound_name).play()

        self.manual_control = True
        rospy.loginfo("Control is now yours")

        while self.manual_control:
            rospy.sleep(rospy.Duration(nsecs=1e8))

        rospy.loginfo("Taking back autonomous control. Choice: {}".format(self.manual_choice))
        userdata.choice = self.manual_choice
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

        self.accept_candy_location = accept_candy_location
        self.move_client = None

    def execute(self, userdata):
        """Move to wait location and poll for time"""

        # Initialize the action client if that is not already true
        if self.move_client is None:
            self.move_client = actionlib.SimpleActionClient(
                'vector_move_base',
                MoveBaseAction
            )
            self.move_client.wait_for_server()

        # Create the goal for the robot to move towards
        accept_candy_loc_goal = MoveBaseGoal(target_pose=self.accept_candy_location)
        accept_candy_loc_goal.target_pose.header.stamp = rospy.Time.now()

        # Move the robot to that goal
        self.move_client.send_goal(accept_candy_loc_goal)
        self.move_client.wait_for_result()
        if self.move_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn("wait: navigate to location failed")
            return 'help'

        return 'done'


class FindGraspState(smach.State):
    def __init__(
        self,
        place_candy_location,
        place_candy_angle,
        pan_tilt_topic
    ):
        smach.State.__init__(
            self,
            outcomes=['place_candy'],
            input_keys=['location'],
            output_keys=['location', 'location_name', 'choice']
        )

        self.place_candy_location = place_candy_location
        self.place_candy_angle = place_candy_angle
        self.pan_tilt_publisher = rospy.Publisher(pan_tilt_topic, Float64,
                                                  queue_size=1)

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        if self.move_client is None:
            self.move_client = actionlib.SimpleActionClient(
                'vector_move_base',
                MoveBaseAction
            )
            self.move_client.wait_for_server()
        place_candy_location = MoveBaseGoal(target_pose=self.place_candy_location)
        place_candy_location.target_pose.header.stamp = rospy.Time.now()
        self.pan_tilt_publisher.publish(Float64(self.place_candy_angle))
        self.move_client.send_goal(place_candy_location)
        self.move_client.wait_for_result()
        if self.move_client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            return 'help'
        return 'done'


class PlaceCandyState(smach.State):
    def __init__(
        self,
    ):
        smach.State.__init__(
            self
        )

    def execute(self, grasp_data):
        """
        execute grasp
        :param grasp_data:
        :return:
        """
        return 'done'
