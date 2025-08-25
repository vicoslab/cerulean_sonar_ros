#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Bool

def bresenham_line(x0, y0, x1, y1):
	"""Return list of (x, y) pixels along a line from (x0,y0) to (x1,y1)."""
	points = []
	dx = abs(x1 - x0)
	dy = abs(y1 - y0)
	sx = 1 if x0 < x1 else -1
	sy = 1 if y0 < y1 else -1
	err = dx - dy

	while True:
		points.append((x0, y0))
		if x0 == x1 and y0 == y1:
			break
		e2 = 2 * err
		if e2 > -dy:
			err -= dy
			x0 += sx
		if e2 < dx:
			err += dx
			y0 += sy
	return points


class SideScanStitcher:
	def __init__(self):
		rospy.init_node("sonar_map_stitcher")

		self.nadir_range = rospy.get_param("~nadir_removal_range", 3.0)
		self.world_frame = rospy.get_param("~world_frame_id", "local")
		self.resolution = rospy.get_param("~resolution", 0.2)  # meters per pixel
		self.publish_rate = rospy.get_param("~publish_rate_hz", 1.0)  # Hz

		self.grid = None
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.width = 0
		self.height = 0
		self.updated = False
		self.mapping_enabled = True
		self.last_scan = None 

		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		self.sub = rospy.Subscriber("/omniscan/tof_profile", OccupancyGrid, self.sonar_data_callback, queue_size=10)
		self.pub = rospy.Publisher("/omniscan/world_grid", OccupancyGrid, queue_size=1)

		self.reset_sub = rospy.Subscriber("/omniscan/reset_map", Empty, self.reset_callback, queue_size=1)
		self.enable_sub = rospy.Subscriber("/omniscan/mapping_enabled", Bool, self.enabled_callback, queue_size=1)
		self.enable_pub = rospy.Publisher("/omniscan/mapping_enabled", Bool, queue_size=1, latch=True)
		self.enable_pub.publish(True)

		rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.update)

	def reset_callback(self, msg):
		rospy.loginfo("Resetting sonar map")
		self.grid = None
		self.width = 0
		self.height = 0
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.updated = True

	def enabled_callback(self, msg):
		if msg.data != self.mapping_enabled:
			self.mapping_enabled = msg.data
			self.enable_pub.publish(self.mapping_enabled)
			rospy.loginfo("Mapping %s", "ENABLED" if self.mapping_enabled else "DISABLED")

	def expand_grid_to_include(self, x_min, x_max, y_min, y_max):
		"""Expand global grid if needed to cover new bounds in world frame."""

		# Convert requested bounds into grid indices w.r.t. current origin
		gx_min_req = int(np.floor((x_min - self.origin_x) / self.resolution))
		gx_max_req = int(np.ceil((x_max - self.origin_x) / self.resolution))
		gy_min_req = int(np.floor((y_min - self.origin_y) / self.resolution))
		gy_max_req = int(np.ceil((y_max - self.origin_y) / self.resolution))

		if self.grid is None:
			# Initialize grid with requested size
			self.origin_x = np.floor(x_min / self.resolution) * self.resolution
			self.origin_y = np.floor(y_min / self.resolution) * self.resolution
			self.width = int(np.ceil((x_max - self.origin_x) / self.resolution))
			self.height = int(np.ceil((y_max - self.origin_y) / self.resolution))
			self.grid = np.zeros((self.height, self.width), dtype=np.int16)
			return

		# Current bounds in index space
		cur_gx_min = 0
		cur_gx_max = self.width
		cur_gy_min = 0
		cur_gy_max = self.height

		# New bounds relative to current origin
		new_gx_min = min(cur_gx_min, gx_min_req)
		new_gx_max = max(cur_gx_max, gx_max_req)
		new_gy_min = min(cur_gy_min, gy_min_req)
		new_gy_max = max(cur_gy_max, gy_max_req)

		new_width = new_gx_max - new_gx_min
		new_height = new_gy_max - new_gy_min

		# Allocate new grid
		new_grid = np.zeros((new_height, new_width), dtype=np.int16)

		# Copy old grid into correct place
		off_x = -new_gx_min
		off_y = -new_gy_min
		new_grid[off_y:off_y+self.height, off_x:off_x+self.width] = self.grid

		# Update grid + origin
		self.grid = new_grid
		self.origin_x = self.origin_x + new_gx_min * self.resolution
		self.origin_y = self.origin_y + new_gy_min * self.resolution
		self.width = new_width
		self.height = new_height

	def paint_beam(self, x0, y0, x1, y1, val):
		"""Rasterize a sonar beam from (x0,y0) -> (x1,y1) into the grid."""
		gx0 = int(np.floor((x0 - self.origin_x) / self.resolution))
		gy0 = int(np.floor((y0 - self.origin_y) / self.resolution))
		gx1 = int(np.floor((x1 - self.origin_x) / self.resolution))
		gy1 = int(np.floor((y1 - self.origin_y) / self.resolution))

		points = bresenham_line(gx0, gy0, gx1, gy1)

		for gx, gy in points:
			if 0 <= gx < self.width and 0 <= gy < self.height:
				self.grid[gy, gx] = val


	def sonar_data_callback(self, msg):
		if not self.mapping_enabled:
			self.updated = True
			return

		try:
			# Transform sonar origin into world
			pose = PoseStamped()
			pose.header = msg.header
			pose.pose.orientation.w = 1.0
			transform = self.tf_buffer.lookup_transform(
				self.world_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5)
			)
			pose_world = tf2_geometry_msgs.do_transform_pose(pose, transform)
			x0 = pose_world.pose.position.x
			y0 = pose_world.pose.position.y

			q = transform.transform.rotation
			_, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

		except Exception as e:
			rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
			return

		# Scan parameters
		scan_len = msg.info.width * msg.info.resolution
		x_min = x0 - scan_len
		x_max = x0 + scan_len
		y_min = y0 - scan_len
		y_max = y0 + scan_len
		self.expand_grid_to_include(x_min, x_max, y_min, y_max)

		cos_yaw = math.cos(yaw)
		sin_yaw = math.sin(yaw)

		# Compute world coordinates of this ping line
		positions = []
		for i, val in enumerate(msg.data):
			r = i * msg.info.resolution

			if r > self.nadir_range:
				wx = x0 + r * cos_yaw
				wy = y0 + r * sin_yaw
				positions.append((wx, wy))

		positions = np.array(positions)
		
		#values = np.array(msg.data)
		values = np.array(msg.data, dtype=np.int16)
		values[values < 0] += 256 

		# If we have a previous scan, interpolate between them
		if self.last_scan is not None:
			prev_positions, prev_values = self.last_scan

			if len(prev_positions) != len(positions):
				#scan settings changed, can't interpolate
				return

			n = len(positions)
			for i in range(n):
				x1, y1 = prev_positions[i]
				x2, y2 = positions[i]
				v1, v2 = prev_values[i], values[i]

				gx1 = int(np.floor((x1 - self.origin_x) / self.resolution))
				gy1 = int(np.floor((y1 - self.origin_y) / self.resolution))
				gx2 = int(np.floor((x2 - self.origin_x) / self.resolution))
				gy2 = int(np.floor((y2 - self.origin_y) / self.resolution))

				points = bresenham_line(gx1, gy1, gx2, gy2)

				for j, (gx, gy) in enumerate(points):
					if 0 <= gx < self.width and 0 <= gy < self.height:
						t = j / max(1, len(points) - 1)
						#val = int(round(v1 + t * (v2 - v1)))
						#self.grid[gy, gx] = val
						val = int(round(v1 + t * (v2 - v1)))
						val = 0 if val < 0 else (255 if val > 255 else val)
						self.grid[gy, gx] = val

		# Save this scan for the next interpolation
		self.last_scan = (positions, values)

		self.updated = True
	def update(self, event):
		if not self.updated or self.grid is None:
			return
		self.updated = False

		out = OccupancyGrid()
		out.header.stamp = rospy.Time.now()
		out.header.frame_id = self.world_frame
		out.info.resolution = self.resolution
		out.info.width = self.width
		out.info.height = self.height
		out.info.origin.position.x = self.origin_x
		out.info.origin.position.y = self.origin_y
		out.info.origin.orientation.w = 1.0
		out.data = self.grid.astype(np.int8).flatten().tolist()

		self.pub.publish(out)


def main():
	node = SideScanStitcher()
	rospy.spin()

if __name__ == "__main__":
	main()
