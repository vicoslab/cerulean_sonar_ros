#!/usr/bin/env python3
import socket
import struct
import time
import rospy
import numpy as np

from std_msgs.msg import Float32
from cerulean_sonar_ros.msg import OmniscanRaw
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

class O450CParser:
	def __init__(self):
		rospy.init_node('o450c_parser_node')

		self.param_frame_id = rospy.get_param('~frame_id', 'omniscan_link')

		self.raw_sub = rospy.Subscriber("/omniscan/raw", OmniscanRaw, self.raw_callback)

		self.heading_pub = rospy.Publisher('/omniscan/heading', Float32, queue_size=10)
		self.profile_pub = rospy.Publisher('/omniscan/tof_profile', OccupancyGrid, queue_size=10)

	def raw_callback(self, msg):

		heading = msg.transducer_heading_deg + msg.vehicle_heading_deg
		self.heading_pub.publish(Float32(data=heading))

		#mapping uint16 to 0-1 float
		data_norm = np.asarray(msg.pwr_results, dtype=np.float64) / 65535.0

		grid = OccupancyGrid()
		grid.header.stamp = rospy.Time.now()
		grid.header.frame_id = self.param_frame_id
		grid.info.resolution = (msg.length_mm/1000.0) / max(1, msg.num_results)
		grid.info.width = msg.num_results
		grid.info.height = 1
		grid.info.origin.position = Point(msg.start_mm/1000.0, 0.0, 0.0)
		grid.info.origin.orientation.w = 1.0
		grid.data = np.array(255  - data_norm * 255, dtype=np.int8).tolist()
		self.profile_pub.publish(grid)

def main():
	driver = O450CParser()
	rospy.spin()

if __name__ == '__main__':
	main()