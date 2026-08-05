#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import math
import cv2

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty, Bool

GRID_MARGIN = 16
STRIP_PAD_CELLS = 0.75
MIN_CURVATURE = 1e-12


class SideScanStitcher:
	def __init__(self):
		rospy.init_node("sonar_map_stitcher")

		self.nadir_range = rospy.get_param("~nadir_removal_range", 3.0)
		self.world_frame = rospy.get_param("~world_frame_id", "local")
		self.resolution = rospy.get_param("~resolution", 0.2)
		self.publish_rate = rospy.get_param("~publish_rate_hz", 1.0)
		self.beam_azimuth = math.radians(rospy.get_param("~beam_azimuth_deg", 0.0))
		self.max_yaw_step = math.radians(rospy.get_param("~max_yaw_step_deg", 20.0))

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

		self.sub = rospy.Subscriber("/omniscan/tof_profile", OccupancyGrid, self.sonar_data_callback, queue_size=1, buff_size=2**24)
		self.pub = rospy.Publisher("/omniscan/world_grid", OccupancyGrid, queue_size=1)

		self.reset_sub = rospy.Subscriber("/omniscan/reset_map", Empty, self.reset_callback, queue_size=1)
		self.enable_sub = rospy.Subscriber("/omniscan/mapping_enabled", Bool, self.enabled_callback, queue_size=1)
		self.enable_pub = rospy.Publisher("/omniscan/mapping_enabled", Bool, queue_size=1, latch=True)
		self.enable_pub.publish(True)

		rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.update)
		rospy.loginfo("Omniscan mapper ready.")

	def reset_callback(self, msg):
		rospy.loginfo("Resetting sonar map")
		self.grid = None
		self.width = 0
		self.height = 0
		self.origin_x = 0.0
		self.origin_y = 0.0
		self.last_scan = None
		self.updated = True

	def enabled_callback(self, msg):
		if msg.data != self.mapping_enabled:
			self.mapping_enabled = msg.data
			self.last_scan = None
			self.enable_pub.publish(self.mapping_enabled)
			rospy.loginfo("Mapping %s", "ENABLED" if self.mapping_enabled else "DISABLED")

	def expand_grid_to_include(self, x_min, x_max, y_min, y_max):
		if self.grid is None:
			self.origin_x = (math.floor(x_min / self.resolution) - GRID_MARGIN) * self.resolution
			self.origin_y = (math.floor(y_min / self.resolution) - GRID_MARGIN) * self.resolution
			self.width = int(math.ceil((x_max - self.origin_x) / self.resolution)) + GRID_MARGIN
			self.height = int(math.ceil((y_max - self.origin_y) / self.resolution)) + GRID_MARGIN
			self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
			return

		ix_min = int(math.floor((x_min - self.origin_x) / self.resolution))
		ix_max = int(math.ceil((x_max - self.origin_x) / self.resolution))
		iy_min = int(math.floor((y_min - self.origin_y) / self.resolution))
		iy_max = int(math.ceil((y_max - self.origin_y) / self.resolution))

		pad_left = GRID_MARGIN - ix_min if ix_min < 0 else 0
		pad_right = ix_max - (self.width - 1) + GRID_MARGIN if ix_max > self.width - 1 else 0
		pad_down = GRID_MARGIN - iy_min if iy_min < 0 else 0
		pad_up = iy_max - (self.height - 1) + GRID_MARGIN if iy_max > self.height - 1 else 0

		if not (pad_left or pad_right or pad_down or pad_up):
			return

		new_width = self.width + pad_left + pad_right
		new_height = self.height + pad_down + pad_up
		new_grid = np.zeros((new_height, new_width), dtype=np.uint8)
		new_grid[pad_down:pad_down+self.height, pad_left:pad_left+self.width] = self.grid

		self.grid = new_grid
		self.origin_x -= pad_left * self.resolution
		self.origin_y -= pad_down * self.resolution
		self.width = new_width
		self.height = new_height

	def sonar_data_callback(self, msg):
		if not self.mapping_enabled:
			return

		try:
			transform = self.tf_buffer.lookup_transform(self.world_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.5))
		except Exception as e:
			rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
			return

		t = transform.transform.translation
		q = transform.transform.rotation
		_, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

		bin_m = msg.info.resolution
		values = np.array(msg.data, dtype=np.int16)
		values[values < 0] += 256

		azimuth = yaw + self.beam_azimuth
		scan = (t.x, t.y, math.cos(azimuth), math.sin(azimuth), values, bin_m)

		if self.last_scan is not None:
			self.integrate(self.last_scan, scan)

		self.last_scan = scan
		self.updated = True

	def solve_sweep_fraction(self, ax, ay, vx, vy, ex, ey, pdx, pdy):
		a = -(vx * ey - vy * ex)
		b = (ax * ey - ay * ex) - (vx * pdy - vy * pdx)
		c = ax * pdy - ay * pdx

		with np.errstate(all="ignore"):
			safe_b = np.where(np.abs(b) > 0.0, b, 1.0)
			linear = np.where(np.abs(b) > 0.0, -c / safe_b, 0.0)

			if abs(a) > MIN_CURVATURE:
				disc = np.sqrt(np.maximum(b * b - 4.0 * a * c, 0.0))
				q = -0.5 * (b + np.where(b >= 0.0, 1.0, -1.0) * disc)
				safe_q = np.where(np.abs(q) > 0.0, q, 1.0)
				candidates = (q / a, np.where(np.abs(q) > 0.0, c / safe_q, linear), linear)
			else:
				candidates = (linear,)

		best = None
		best_err = None
		for s in candidates:
			s = np.nan_to_num(s, nan=1e6, posinf=1e6, neginf=-1e6)
			err = np.abs(np.clip(s, 0.0, 1.0) - s)
			if best is None:
				best, best_err = s, err
			else:
				take = err < best_err
				best = np.where(take, s, best)
				best_err = np.where(take, err, best_err)

		return np.clip(best, 0.0, 1.0)

	def integrate(self, prev, cur):
		px, py, pdx, pdy, prev_values, prev_bin = prev
		cx, cy, cdx, cdy, values, bin_m = cur

		if len(prev_values) != len(values) or prev_bin != bin_m:
			return

		if abs(math.atan2(pdx*cdy - pdy*cdx, pdx*cdx + pdy*cdy)) > self.max_yaw_step:
			return

		n = len(values)
		max_range = n * bin_m
		pad = STRIP_PAD_CELLS * self.resolution

		vx = cx - px
		vy = cy - py
		ex = cdx - pdx
		ey = cdy - pdy

		mdx = 0.5 * (pdx + cdx)
		mdy = 0.5 * (pdy + cdy)
		mx = px + 0.5 * vx
		my = py + 0.5 * vy

		corners_x = (px + self.nadir_range*pdx, px + max_range*pdx, cx + self.nadir_range*cdx, cx + max_range*cdx, mx + max_range*mdx)
		corners_y = (py + self.nadir_range*pdy, py + max_range*pdy, cy + self.nadir_range*cdy, cy + max_range*cdy, my + max_range*mdy)
		x_min, x_max = min(corners_x) - pad, max(corners_x) + pad
		y_min, y_max = min(corners_y) - pad, max(corners_y) + pad
		self.expand_grid_to_include(x_min, x_max, y_min, y_max)

		ix0 = int(math.floor((x_min - self.origin_x) / self.resolution))
		ix1 = int(math.ceil((x_max - self.origin_x) / self.resolution)) + 1
		iy0 = int(math.floor((y_min - self.origin_y) / self.resolution))
		iy1 = int(math.ceil((y_max - self.origin_y) / self.resolution)) + 1

		cell_x = self.origin_x + (np.arange(ix0, ix1) + 0.5) * self.resolution
		cell_y = self.origin_y + (np.arange(iy0, iy1) + 0.5) * self.resolution
		wx, wy = np.meshgrid(cell_x, cell_y)

		s = self.solve_sweep_fraction(wx - px, wy - py, vx, vy, ex, ey, pdx, pdy)

		dx = pdx + s * ex
		dy = pdy + s * ey
		norm = np.sqrt(dx * dx + dy * dy)
		dx /= norm
		dy /= norm

		ax = wx - (px + s * vx)
		ay = wy - (py + s * vy)

		r = ax * dx + ay * dy
		perp = np.abs(ax * dy - ay * dx)

		inside = (perp <= pad) & (r >= self.nadir_range) & (r < max_range)

		map_x = np.where(inside, r / bin_m, -10.0).astype(np.float32)
		map_y = np.where(inside, s, -10.0).astype(np.float32)

		profile = np.stack((prev_values, values)).astype(np.float32)
		sampled = cv2.remap(profile, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)

		region = self.grid[iy0:iy1, ix0:ix1]
		np.copyto(region, np.clip(sampled, 0.0, 255.0).astype(np.uint8), where=inside)

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
		out.data = self.grid.view(np.int8).ravel().tolist()

		self.pub.publish(out)


def main():
	node = SideScanStitcher()
	rospy.spin()

if __name__ == "__main__":
	main()