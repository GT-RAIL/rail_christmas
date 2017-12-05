#!/usr/bin/env python
# This defines the robot states

import sys
import rospy
import rospkg
import smach

import moveit_commander

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from christmas_state_machine.states import AcceptCandyState, FindGraspState, PlaceCandyState, HelpState

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

        accept_candy_location_param = rospy.get_param(
            '~accept_candy_location', {'pos': [], 'ori': [], 'pan': 0.0, 'tilt': 0.0}
        )
        accept_candy_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*accept_candy_location_param.get('pos')),
                orientation=Quaternion(*accept_candy_location_param.get('ori'))
            )
        )

        find_grasp_location_param = rospy.get_param(
            '~find_grasp_location', {'pos': [], 'ori': [], 'pan': 0.0, 'tilt': 0.0}
        )
        find_grasp_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*find_grasp_location_param.get('pos')),
                orientation=Quaternion(*find_grasp_location_param.get('ori'))
            )
        )
        find_grasp_pan = find_grasp_location_param.get('pan')
        find_grasp_tilt = find_grasp_location_param.get('tilt')

        retreat_location_param = rospy.get_param(
            '~retreat_location', {'pos': [], 'ori': [], 'pan': 0.0, 'tilt': 0.0}
        )
        retreat_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*retreat_location_param.get('pos')),
                orientation=Quaternion(*retreat_location_param.get('ori'))
            )
        )
        # set tuck pose to arm configuration for candy reception
        tuck_pose_param = rospy.get_param(
            '~accept_candy_arm_pose', {'pos': [], 'ori': []}
        )
        tuck_pose = Pose(
            position=Point(*tuck_pose_param.get('pos')),
            orientation=Quaternion(*tuck_pose_param.get('ori'))
        )

        # Setup the state machine
        with self.state_machine:
            smach.StateMachine.add(
                'ACCEPT_CANDY',
                AcceptCandyState(
                    tuck_pose,
                    accept_candy_location
                ),
                transitions={
                    'done': 'FIND_GRASP',
                    'help': 'HELP'
                },
                remapping={
                    'target_pose': 'target_pose',
                    'prev_state': 'prev_state'
                }
            )

            smach.StateMachine.add(
                'FIND_GRASP',
                FindGraspState(
                    find_grasp_location, find_grasp_pan, find_grasp_tilt
                ),
                transitions={
                    'done': 'PLACE_CANDY',
                    'help': 'HELP',
                },
                remapping={
                    'target_pose': 'target_pose',
                    'prev_state': 'prev_state'
                }
            )
            smach.StateMachine.add(
                'PLACE_CANDY',
                PlaceCandyState(
                    retreat_location
                ),
                transitions={
                    'done': 'ACCEPT_CANDY',
                    'help': 'HELP'
                },
                remapping={
                    'target_pose': 'target_pose',
                    'prev_state': 'prev_state'
                }
            )

            smach.StateMachine.add(
                'HELP',
                HelpState(),
                transitions={
                    'accept': 'ACCEPT_CANDY',
                    'find': 'FIND_GRASP',
                    'place': 'PLACE_CANDY'
                },
                remapping={
                    'target_pose': 'target_pose',
                    'prev_state': 'prev_state'
                }
            )

    def execute(self):
        return self.state_machine.execute()
