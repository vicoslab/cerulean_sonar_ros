#!/usr/bin/env python3
import rospy
import threading
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import GridCells
from std_msgs.msg import Empty
from sensor_msgs.msg import Range

class EchosounderMap:
    def __init__(self):
        rospy.init_node("echosounder_map")

        self.resolution = rospy.get_param("~resolution", 2.0)
        self.world_frame = rospy.get_param("~world_frame", "local")
        self.sonar_frame = rospy.get_param("~sonar_frame", "echosounder_link")

        self.cells = set()
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.grid_pub = rospy.Publisher("/echosounder_map/grid", GridCells, queue_size=1)
        self.range_sub = rospy.Subscriber("echosounder/range", Range, self.range_callback)
        self.reset_sub = rospy.Subscriber("/echosounder_map/reset", Empty, self.reset_callback)

        self.timer = rospy.Timer(rospy.Duration(15.0), self.publish_grid)

        self.updated = True

    def reset_callback(self, _msg):
        with self.lock:
            self.cells.clear()
        
        self.updated = True

    def range_callback(self, msg: Range):
        p = PointStamped()
        p.header = msg.header
        p.point.x = msg.range
        p.point.y = 0.0
        p.point.z = 0.0

        try:
            tf = self.tf_buffer.transform(p, self.world_frame, rospy.Duration(0.2))
        except Exception:
            return

        x = tf.point.x
        y = tf.point.y

        ix = int(x / self.resolution)
        iy = int(y / self.resolution)

        with self.lock:
            self.cells.add((ix, iy))

        self.updated = True

    def publish_grid(self, _event):
        if not self.updated:
            return

        grid = GridCells()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = self.world_frame
        grid.cell_width = self.resolution
        grid.cell_height = self.resolution

        with self.lock:
            for ix, iy in self.cells:
                p = Point()
                p.x = (ix + 0.5) * self.resolution
                p.y = (iy + 0.5) * self.resolution
                p.z = 0.0
                grid.cells.append(p)

        self.grid_pub.publish(grid)

if __name__ == "__main__":
    node = EchosounderMap()
    rospy.spin()
