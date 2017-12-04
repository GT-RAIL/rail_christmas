#!/usr/bin/env python
# This defines the robot states

import rospy
import rospkg
import smach

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

from christmas_state_machine.states import WaitState, ObserveState, \
                                       InterruptState, BuildWaitState, \
                                       HelpState, ManualState, PostState

# Define the state machine here

class ChristmasStateMachine(object):
    """The state machine for the study"""

    state_machine = None

    def __init__(self):
        """Use ROS Params to set up a state machine"""
        self.state_machine = smach.StateMachine(outcomes=['end'])

        # Get the parameters for the states. The subheadings show the groupings

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

        post_location_param = rospy.get_param(
            '~post_location', { 'pos': [], 'ori': [], 'ang': 0.0 }
        )
        post_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*post_location_param.get('pos')),
                orientation=Quaternion(*post_location_param.get('ori'))
            )
        )
        post_angle = post_location_param.get('ang')

        observe_location_param = rospy.get_param(
            '~observe_location', { 'pos': [], 'ori': [], 'ang': 0.0 }
        )
        observe_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*observe_location_param.get('pos')),
                orientation=Quaternion(*observe_location_param.get('ori'))
            )
        )
        observe_angle = observe_location_param.get('ang')

        build_location_param = rospy.get_param(
            '~build_location',
            { 'pos': [], 'ori': [], 'ang': 0.0, 'wait': 0.0, 'check': [] }
        )
        build_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*build_location_param.get('pos')),
                orientation=Quaternion(*build_location_param.get('ori'))
            )
        )
        build_check_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*build_location_param.get('check'))
            )
        )
        build_ask_angle = build_location_param.get('ang')
        build_wait_angle = build_location_param.get('wait')

        couch_location_param = rospy.get_param(
            '~couch_location',
            { 'pos': [], 'ori': [], 'ang': 0.0, 'wait': 0.0, 'check': [] }
        )
        couch_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*couch_location_param.get('pos')),
                orientation=Quaternion(*couch_location_param.get('ori'))
            )
        )
        couch_check_location = PoseStamped(
            header=loc_header,
            pose=Pose(
                position=Point(*couch_location_param.get('check'))
            )
        )
        couch_ask_angle = couch_location_param.get('ang')
        couch_wait_angle = couch_location_param.get('wait')

        # Service and Topic names
        int_start_trigger = rospy.get_param(
            '~int_start_trigger',
            '/interruptibility_estimator/start_buffer'
        )
        int_stop_trigger = rospy.get_param(
            '~int_stop_trigger',
            '/interruptibility_estimator/stop_buffer'
        )
        filter_start_trigger = rospy.get_param(
            '~filter_start_trigger',
            '/interruptibility_filter/start_filter'
        )
        filter_stop_trigger = rospy.get_param(
            '~filter_stop_trigger',
            '/interruptibility_filter/stop_filter'
        )
        pan_tilt_topic_name = rospy.get_param(
            '~pan_tilt_topic_name',
            '/tilt_controller/command'
        )
        log_topics = rospy.get_param('~log_topics', {})
        log_start_topic = log_topics.get('start')
        log_stop_topic = log_topics.get('stop')

        # Study condition parameters
        condition = rospy.get_param('/study/condition', 'RND')
        interrupt_wait_threshold = rospy.get_param(
            '/study/interrupt_wait_threshold', 2
        )

        # Setup the state machine
        with self.state_machine:
            smach.StateMachine.add(
                'MANUAL',
                ManualState(),
                transitions={ 'done': 'WAIT' },
                remapping={ 'choice': 'choice' }
            )

            smach.StateMachine.add(
                'WAIT',
                WaitState(
                    wait_location, wait_angle, pan_tilt_topic_name
                ),
                transitions={
                    'observe': 'OBSERVE',
                    'help': 'HELP',
                    'complete': 'end'
                },
                remapping={
                    'choice': 'choice'
                }
            )

            smach.StateMachine.add(
                'OBSERVE',
                ObserveState(
                    observe_location, observe_angle,
                    condition, interrupt_wait_threshold,
                    int_start_trigger, int_stop_trigger,
                    filter_start_trigger, filter_stop_trigger,
                    log_start_topic, log_stop_topic, pan_tilt_topic_name
                ),
                transitions={
                    'interrupt': 'INTERRUPT',
                    'post': 'POST',
                    'help': 'HELP'
                },
                remapping={
                    'location': 'location',
                    'choice': 'choice'
                }
            )

            smach.StateMachine.add(
                'INTERRUPT',
                InterruptState(
                    build_location, build_ask_angle, build_check_location,
                    couch_location, couch_ask_angle, couch_check_location,
                    pan_tilt_topic_name
                ),
                transitions={
                    'grabbed': 'BUILD_WAIT',
                    'ignored': 'POST',
                    'help': 'HELP'
                },
                remapping={
                    'location': 'location',
                    'location_name': 'location_name',
                    'choice': 'choice',
                }
            )

            smach.StateMachine.add(
                'BUILD_WAIT',
                BuildWaitState(build_wait_angle, couch_wait_angle, wait_angle,
                               pan_tilt_topic_name),
                transitions={
                    'done': 'POST',
                    'help': 'HELP',
                },
                remapping={
                    'location_name': 'location_name'
                }
            )

            smach.StateMachine.add(
                'POST',
                PostState(post_location, post_angle, pan_tilt_topic_name),
                transitions={
                    'done': 'MANUAL',
                    'help': 'HELP'
                },
                remapping={
                    'choice': 'choice'
                }
            )

            smach.StateMachine.add(
                'HELP',
                HelpState(),
                transitions={
                    'wait': 'WAIT',
                    'observe': 'OBSERVE',
                    'interrupt': 'INTERRUPT',
                    'manual': 'MANUAL',
                    'post': 'POST'
                },
                remapping={
                    'location': 'location',
                    'choice': 'choice',
                    'location_name': 'location_name',
                }
            )

    def execute(self):
        return self.state_machine.execute()
