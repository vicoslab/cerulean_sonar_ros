#!/usr/bin/env python3
import rospy
import numpy as np

from std_msgs.msg import Float32
from cerulean_sonar_ros.msg import OmniscanRaw
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point

class O450CParser:
	def __init__(self):
		rospy.init_node('o450c_parser_node')

		# Frame id
		self.param_frame_id = rospy.get_param('~frame_id', 'omniscan_link')

		# Display scaling parameters
		self.param_min_db = rospy.get_param('~min_db', -20.0)   # lower display bound in dB
		self.param_max_db = rospy.get_param('~max_db', 67.0)     # upper display bound in dB
		self.param_gamma  = rospy.get_param('~gamma', 1.5)      # gamma correction (>1 boosts midtones)

		self.raw_sub = rospy.Subscriber("/omniscan/raw", OmniscanRaw, self.raw_callback)

		self.heading_pub = rospy.Publisher('/omniscan/heading', Float32, queue_size=10)
		self.profile_pub = rospy.Publisher('/omniscan/tof_profile', OccupancyGrid, queue_size=10)

	def scale_power(self, msg):
		"""Scale raw power results into dB values as per BR ping-python mapping."""
		raw = np.array(msg.pwr_results, dtype=np.float64)
		scaled_power = msg.min_pwr_db + (raw / 65535.0) * (msg.max_pwr_db - msg.min_pwr_db)
		return scaled_power

	def apply_display_mapping(self, db_array):
		"""Map dB values into 8-bit grayscale with gamma correction."""
		# Clip to display window
		db_clipped = np.clip(db_array, self.param_min_db, self.param_max_db)

		# Normalize to [0,1]
		norm = (db_clipped - self.param_min_db) / (self.param_max_db - self.param_min_db + 1e-9)

		# Gamma correction
		if self.param_gamma != 1.0:
			norm = np.power(norm, 1.0 / self.param_gamma)

		# Scale to 0–255
		img = (norm * 255.0).astype(np.uint8)
		img = 255 - img

		return img

	def raw_callback(self, msg):
		heading = msg.transducer_heading_deg + msg.vehicle_heading_deg
		self.heading_pub.publish(Float32(data=heading))

		# Convert to dB and apply mapping
		db_vals = self.scale_power(msg)
		img_vals = self.apply_display_mapping(db_vals)

		# Fill OccupancyGrid
		grid = OccupancyGrid()
		grid.header.stamp = rospy.Time.now()
		grid.header.frame_id = self.param_frame_id
		grid.info.resolution = (msg.length_mm / 1000.0) / max(1, msg.num_results)
		grid.info.width = msg.num_results
		grid.info.height = 1
		grid.info.origin.position = Point(msg.start_mm / 1000.0, 0.0, 0.0)
		grid.info.origin.orientation.w = 1.0

		grid.data = img_vals.astype(np.int8).tolist()
		self.profile_pub.publish(grid)


def main():
	driver = O450CParser()
	rospy.spin()

if __name__ == '__main__':
	main()
