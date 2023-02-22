#!/usr/bin/python3
# ROS packages
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point


rospy.init_node(name='uav_att_pid', anonymous=False)
marker_end_pub = rospy.Publisher('/rviz_end_marker', Marker, queue_size=10)

rate = rospy.Rate(100)

position = Pose()
uav_end = Marker()
uav_end.header.frame_id = 'yyf_uav'
uav_end.type = Marker.SPHERE_LIST		# 4个球
uav_end.colors = [ColorRGBA(r=1, g=0, b=0, a=1),			# 1
				  ColorRGBA(r=0, g=1, b=0, a=1),			# 2
				  ColorRGBA(r=0, g=0, b=1, a=1),			# 3
				  ColorRGBA(r=1, g=0.6, b=0, a=1)			# 4
				  ]
uav_end.scale.x = 0.4
uav_end.scale.y = 0.4
uav_end.scale.z = 0.4


if __name__ == '__main__':
	while not rospy.is_shutdown():

		'''RVIZ'''
		pt = [Point() for _ in range(4)]
		pt[0] = Point(x=-1, y=-1, z=0)
		pt[1] = Point(x=-1, y=1, z=0)
		pt[2] = Point(x=1, y=-1, z=0)
		pt[3] = Point(x=1, y=1, z=0)

		poses = [Pose() for _ in range(4)]
		# orientation = Quaternion(x=0, y=0, z=0, w=1)
		# poses[0].orientation = orientation
		poses[0].position.x = pt[0].x
		poses[0].position.y = pt[0].y
		poses[0].position.z = pt[0].z
		poses[0].orientation.x = 0
		poses[0].orientation.y = 0
		poses[0].orientation.z = 0
		poses[0].orientation.w = 1
		# poses[1].orientation = orientation
		# poses[1].position = pt[1]
		# poses[2].orientation = orientation
		# poses[2].position = pt[2]
		# poses[3].orientation = orientation
		# poses[3].position = pt[3]
		# uav_end.points = [poses[0], poses[0], poses[0], poses[0]]
		uav_end.points = pt
		marker_end_pub.publish(uav_end)
		'''publish'''

		rate.sleep()
