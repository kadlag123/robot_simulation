#!/usr/bin/env python
import rospkg
import argparse
import rospy
from hector_uav_msgs.msg import PoseActionGoal, PoseActionResult
from hector_uav_msgs.srv import EnableMotors

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
import csv
import numpy as np

class PillionTrajectory():
    def __init__(self):
        self.pub = rospy.Publisher('/action/pose/goal', PoseActionGoal, queue_size=1)
        self.sub = rospy.Subscriber('/action/pose/result', PoseActionResult, self.callback)
        self.rate = rospy.Rate(0.8)
        rospack = rospkg.RosPack()
        self.path = rospack.get_path('iarc')
        self.traj = [(5 * 3 ** 0.5, -6, 3, 0, 0, 0, 1),
                     (400 + 5 * 3 ** 0.5, -6, 3, 0, 0, 0, 1),
                     (400 + 5 * 3 ** 0.5, -4, 3, 0, 0, 1, 0),
                     (5 * 3 ** 0.5, -4, 3, 0, 0, 1, 0)]
        self.execute()

    def enable_motors(self):
		rospy.wait_for_service('enable_motors')
		try:
			client = rospy.ServiceProxy('enable_motors', EnableMotors)
			result = client(True)
			return result
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

    def set_twist_limit(self):
		pub = rospy.Publisher('/command/twist_limit', Twist, queue_size=10)
		twist = Twist()
		twist.linear.x = 10
		twist.linear.y = 10
		twist.linear.z = 10
		while pub.get_num_connections() < 1:
			pass
		pub.publish(twist)

    def pose_gen(self, target_pose):
		pose = Pose()
		pose.position.x = target_pose[0]
		pose.position.y = target_pose[1]
		pose.position.z = target_pose[2]
		pose.orientation.x = target_pose[3]
		pose.orientation.y = target_pose[4]
		pose.orientation.z = target_pose[5]
		pose.orientation.w = target_pose[6]
		return pose

    def callback(self, result):
        id = int(result.status.goal_id.id)
        status = int(result.status.status)
        print(id, status)

        if status != 3:
            self.goal_gen(self.traj[(id - 1) % 4], id + 4)
        else:
            row = self.traj[id % 4]
            self.goal_gen(row, id + 1)
            self.rate.sleep()

    def goal_gen(self, col, id = 0):
		stamp = rospy.Time.now()
		header = Header()
		header.seq = id
		header.frame_id = 'world'
		header.stamp = stamp

		pose = self.pose_gen(col)

		goal = PoseActionGoal()
		goal.header = header
		goal.goal_id.stamp = stamp
		goal.goal_id.id = str(id)
		goal.goal.target_pose.header = header
		goal.goal.target_pose.pose = pose
		while self.pub.get_num_connections() < 1:
			pass
		self.pub.publish(goal)

    def execute(self):
        print(self.enable_motors())
        #self.set_twist_limit()
        self.goal_gen((0, 0, 3, 1, 0, 0, 0))

if __name__ == '__main__':
	rospy.init_node('iarc_drone')
	pillionTrajectory = PillionTrajectory()
	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
