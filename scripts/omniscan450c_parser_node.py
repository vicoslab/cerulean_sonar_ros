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
		self.param_sampling = rospy.get_param('~sampling', 1.0) # Up or downsample scan
		self.param_min_db = rospy.get_param('~min_db', 5.0)   # lower display bound in dB
		self.param_max_db = rospy.get_param('~max_db', 100.0)     # upper display bound in dB
		self.param_gamma  = rospy.get_param('~gamma', 1.5)      # gamma correction (>1 boosts midtones)

		self.raw_sub = rospy.Subscriber("/omniscan/raw", OmniscanRaw, self.raw_callback)

		self.heading_pub = rospy.Publisher('/omniscan/heading', Float32, queue_size=10)
		self.profile_pub = rospy.Publisher('/omniscan/tof_profile', OccupancyGrid, queue_size=10)

	def scale_power(self, msg, gain_start=0.25, gain_end=1.6):
		"""
		Scale raw power results into dB values as per BR ping-python mapping,
		with an additional index-based gain filter (linear ramp).
		
		Parameters
		----------
		msg : object
			Must have attributes `pwr_results`, `min_pwr_db`, `max_pwr_db`.
		gain_start : float
			Gain multiplier at the first sample (default 0.1).
		gain_end : float
			Gain multiplier at the last sample (default 1.0).
		
		Returns
		-------
		scaled_power : ndarray
			Array of scaled power values with applied gain ramp.
		"""
		raw = np.array(msg.pwr_results, dtype=np.float64)

		# Map raw power to dB
		scaled_power = msg.min_pwr_db + (raw / 65535.0) * (msg.max_pwr_db - msg.min_pwr_db)

		# Build linear gain ramp from gain_start to gain_end
		n = len(raw)
		if n > 1:
			gain = np.linspace(gain_start, gain_end, n)
		else:  # single-sample edge case
			gain = np.array([gain_start])

		# Apply gain filter
		scaled_power *= gain

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
		img = (norm * 1.4 * 255.0).astype(np.uint8)
		#img = 255 - img

		return img

	def resample(self, arr):		
		n = len(arr)
		new_n = int(np.round(n * self.param_sampling))
		if new_n < 2:
			return arr[:1].copy()  # edge case: collapse to single element
		
		# Original and new index positions
		orig_idx = np.linspace(0, 1, n)
		new_idx = np.linspace(0, 1, new_n)
		
		return np.interp(new_idx, orig_idx, arr)

	def raw_callback(self, msg):
		heading = msg.transducer_heading_deg + msg.vehicle_heading_deg
		self.heading_pub.publish(Float32(data=heading))

		# Convert to dB and apply mapping
		db_vals = self.scale_power(msg)
		img_vals = self.apply_display_mapping(db_vals)
		num = msg.num_results

		# Resample if neededž
		if self.param_sampling != 1.0:
			img_resampled = self.resample(img_vals)
			num = len(img_resampled)
		else:
			img_resampled = img_vals

		# Fill OccupancyGrid
		grid = OccupancyGrid()
		grid.header.stamp = rospy.Time.now()
		grid.header.frame_id = self.param_frame_id
		grid.info.resolution = (msg.length_mm / 1000.0) / max(1, num)
		grid.info.width = num
		grid.info.height = 1
		grid.info.origin.position = Point(msg.start_mm / 1000.0, 0.0, 0.0)
		grid.info.origin.orientation.w = 1.0

		grid.data = img_resampled.astype(np.int8).tolist()
		self.profile_pub.publish(grid)


def main():
	driver = O450CParser()
	rospy.spin()

if __name__ == '__main__':
	main()
