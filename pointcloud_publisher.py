from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


import numpy as np
import os
import rospy

from sensor_msgs.msg import PointCloud2,PointField
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point,Pose,Quaternion
from std_msgs.msg import Header,ColorRGBA



def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
	'''
	Create a sensor_msgs.PointCloud2 from an array
	of points.
	'''
	msg = PointCloud2()
	# print(points.shape,colors.shape)
	assert(points.shape == colors.shape)

	buf = []

	if stamp:
		msg.header.stamp = stamp
	if frame_id:
		msg.header.frame_id = frame_id
	if seq: 
		msg.header.seq = seq
	if len(points.shape) == 3:
		msg.height = points.shape[1]
		msg.width = points.shape[0]
	else:
		N = len(points)
		xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
		msg.height = 1
		msg.width = N

	msg.fields = [
		PointField('x', 0, PointField.FLOAT32, 1),
		PointField('y', 4, PointField.FLOAT32, 1),
		PointField('z', 8, PointField.FLOAT32, 1),
		PointField('r', 12, PointField.FLOAT32, 1),
		PointField('g', 16, PointField.FLOAT32, 1),
		PointField('b', 20, PointField.FLOAT32, 1)
		]
	msg.is_bigendian = False
	msg.point_step = 24
	msg.row_step = msg.point_step * N
	msg.is_dense = True; 
	msg.data = xyzrgb.tostring()

	return msg

inp = np.loadtxt("./out.ply",delimiter=' ')

pubc = rospy.Publisher('/cloud_in', PointCloud2, queue_size=10)
rospy.init_node('reg_points', anonymous=True)
rate = rospy.Rate(10)

cmsg = xyzrgb_array_to_pointcloud2(inp[:,0:3],inp[:,3:6],frame_id='camera')

while not rospy.is_shutdown():
	print('Hi')
	pubc.publish(cmsg)
	rate.sleep()
	if rospy.is_shutdown():
		break
		