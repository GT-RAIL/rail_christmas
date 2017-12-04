#!/usr/bin/env python
# This defines the robot states

import sys
import rospy
import rospkg
import smach

import moveit_commander

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from christmas_state_machine.states import AcceptCandyState, FindGraspState, PlaceCandyState

# Define the state machine here

class ChristmasStateMachine(object):
    """The state machine for the study"""

    state_machine = None

    def __init__(self):
        """Use ROS Params to set up a state machine"""

        # Setup the state machine and start the moveit_commander
        self.state_machine = smach.StateMachine(outcomes=['end'])
        moveit_commander.roscpp_initialize(sys.argv)

        # Location specific parameters
        loc_header = Header(frame_id='/map')

        wait_location_param = rospy.get_param(
            '~wait_location', { 'pos': [], 'ori': [], 'ang': 0.0 }
        )
        wait_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*wait_location_param.get('pos')),
                orientation=Quaternion(*wait_location_param.get('ori'))
            )
        )
        wait_angle = wait_location_param.get('ang')

        pan_tilt_topic_name = rospy.get_param(
            '~pan_tilt_topic_name',
            '/tilt_controller/command'
        )

        # Setup the state machine
        with self.state_machine:
            smach.StateMachine.add(
                'ACCEPT_CANDY',
                AcceptCandyState(),
                transitions={'done': 'FIND_GRASP',
                             'help': 'HELP'}
            )

            smach.StateMachine.add(
                'FIND_GRASP',
                FindGraspState(
                    wait_location, wait_angle, pan_tilt_topic_name
                ),
                transitions={
                    'done': 'PLACE_CANDY',
                    'help': 'HELP',
                },
                remapping={'target_pose': 'target_pose'}
            )
            smach.StateMachine.add(
                'PLACE_CANDY',
                PlaceCandyState(
                    observe_location, observe_angle, pan_tilt_topic_name
                ),
                transitions={
                    'done': 'ACCEPT_CANDY',
                    'help': 'HELP'
                },
                remapping={'target_pose': 'target_pose'}
            )

            smach.StateMachine.add(
                'HELP',
                HelpState(),
                transitions={
                    'accept': 'ACCEPT_CANDY',
                    'find': 'FIND_GRASP',
                    'place': 'PLACE_CANDY'
                },
                remapping={'target_pose': 'target_pose'}
            )

    def execute(self):
        return self.state_machine.execute()
