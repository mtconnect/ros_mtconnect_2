from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

ACTIONS = [
    'move',
    'grab',
    'release',
]

class Bridge:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient("work", DeviceWorkAction)
        self.work_timeout = rospy.get_param("~work_timeout", 120.0)
        if (not self.action_client.wait_for_server(rospy.Duration(self.work_timeout))):
            rospy.logerr("DeviceWork server not available")

    def do_work(self, action_type, work_info):
        rospy.loginfo("Sending '{}' work, data: {}".format(action_type, work_info))
        action_goal = DeviceWorkGoal()
        action_goal.type = action_type
        action_goal.data = work_info
        self.action_client.send_goal(action_goal)
        self.action_client.wait_for_result(rospy.Duration.from_sec(self.work_timeout))

        goal_state = self.action_client.get_state()
        if goal_state == GoalStatus.PENDING:
            rospy.logerr("  timed out waiting for work to complete")
            self.action_client.cancel_goal()
        elif goal_state == GoalStatus.ABORTED:
            rospy.logerr("  work was aborted!")
        elif goal_state == GoalStatus.SUCCEEDED:
            rospy.loginfo("  work completed successfully")

    def spin(self):
        rospy.spin()
