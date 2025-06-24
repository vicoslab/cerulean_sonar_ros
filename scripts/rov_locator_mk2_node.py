#!/usr/bin/env python
import math
import rospy
import serial

from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from sensor_msgs.msg import Imu
#from tf.transformations import quaternion_from_euler

def quaternion_from_euler2(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

class LocatorNode:
    def __init__(self):
        rospy.init_node('rov_locator_mk2_node')
        
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.frame_id = rospy.get_param('~frame_id', 'locator')
        self.publish_raw = rospy.get_param('~publish_nmea_string', False)

        self.imu_pub = rospy.Publisher('/locator/imu/data', Imu, queue_size=10)

        self.receiverpose_pub = rospy.Publisher('/locator/receiver_pose', PoseStamped, queue_size=10)
        self.transmitter_apparent_pose_pub = rospy.Publisher('/locator/transmitter_apparent_pose', PoseStamped, queue_size=10)
        self.transmitter_true_pose_pub = rospy.Publisher('/locator/transmitter_true_pose', PoseStamped, queue_size=10)

        self.bearingmath_pub = rospy.Publisher('/locator/bearing_math', Float32, queue_size=10)
        self.bearingcompass_pub = rospy.Publisher('/locator/bearing_compass', Float32, queue_size=10)
        self.elevation_pub = rospy.Publisher('/locator/elevation', Float32, queue_size=10)
        self.slant_pub = rospy.Publisher('/locator/slant', Float32, queue_size=10)
        self.compass_pub = rospy.Publisher('/locator/compass_heading', Float32, queue_size=10)
        self.init_pub = rospy.Publisher('/locator/is_initialized', Bool, queue_size=10, latch=True)

        if self.publish_raw:
            self.string_pub = rospy.Publisher('/locator/nmea_string', String, queue_size=10)

    def calculate_nmea_checksum(self, sentence):
        """
        Calculate the NMEA checksum for the sentence.
        Input sentence should NOT include the starting '$' nor the '*' and checksum.
        """
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return checksum

    def parse_nmea_line(self, line):
        """
        Parses a single NMEA sentence and verifies the checksum.
        Returns a list of fields if valid, otherwise None.
        """
        # Remove CR/LF and any surrounding whitespace
        line = line.strip()
        if not line.startswith('$'):
            return None

        # Look for the asterisk separating data from checksum
        try:
            data_part, checksum_str = line.split('*')
        except ValueError:
            print("No checksum delimiter found in line:", line)
            return None

        # Remove the '$' from the beginning of the data part
        sentence = data_part[1:]
        # Calculate checksum from the sentence
        calc_checksum = self.calculate_nmea_checksum(sentence)
        try:
            recv_checksum = int(checksum_str, 16)
        except ValueError:
            print("Invalid checksum format:", checksum_str)
            return None

        if calc_checksum != recv_checksum:
            print(f"Checksum mismatch: calculated {calc_checksum:02X} but received {recv_checksum:02X}")
            return None

        # Split the sentence into fields (comma-separated)
        fields = sentence.split(',')
        return fields

    def process_usrth(self, fields):
        """
        Processes a $USRTH message by mapping known fields.
        Since new fields may be appended, this function only prints known ones.
        Field indices (after the message type field at index 0) follow the provided documentation:
        
        Format:  $USRTH,ab,ac,ae,sr,tb,cb,te,er,ep,ey,ch,db,ah,ag,ls,im,oc,idx,idq*hh
        
        labels = [
            "apparent bearing (math)",    # ab 1
            "apparent bearing (compass)", # ac 2
            "apparent elevation",         # ae 3
            "slant range (m)",            # sr 4
            "true bearing (math)",        # tb 5
            "true bearing (compass)",     # cb 6
            "true elevation",             # te 7
            "Euler roll",                 # er 8
            "Euler pitch",                # ep 9
            "Euler yaw",                  # ey 10
            "compass heading",            # ch 11
            "analog AGC gain",            # db 12
            "CPU autosync support",       # ah 13
            "GNSS autosync support",      # ag 14
            "seconds since last autosync",# ls 15
            "IMU status",                 # im 16 
            "operating channel",          # oc 17
            "transmitter/transponder ID decoded", # idx
            "transmitter/transponder ID queried"  # idq
        ]"""

        # fields[0] == "$USRTH"

        if fields[1] != '':
            bearing_math = Float32()
            bearing_math.data = float(fields[1])
            self.bearingmath_pub.publish(bearing_math)

        if fields[2] != '':
            bearing_compass = Float32()
            bearing_compass.data = float(fields[2])
            self.bearingcompass_pub.publish(bearing_compass)

        if fields[3] != '':
            elevation = Float32()
            elevation.data = float(fields[3])
            self.elevation_pub.publish(elevation)

        if fields[4] != '':
            slant = Float32()
            slant.data = float(fields[4])
            self.slant_pub.publish(slant)

        roll = float(fields[7]) if fields[7] else 0.0
        pitch = float(fields[8]) if fields[8] else 0.0
        yaw = float(fields[9]) if fields[9] else 0.0

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation = quaternion_from_euler2(roll, pitch, yaw)    
        self.imu_pub.publish(imu_msg)

        receiver_pose = PoseStamped()
        receiver_pose.header.stamp = rospy.Time.now()
        receiver_pose.header.frame_id = "locator"
        receiver_pose.pose.orientation = imu_msg.orientation
        self.receiverpose_pub.publish(receiver_pose)

        #apparent pose
        if fields[1] != '' and fields[3] != '' and fields[4] != '':
            apparent_bearing_rad = math.radians(float(fields[1]))
            apparent_elevation_rad = math.radians(float(fields[3]))
            slant_range = float(fields[4])

            #receiver reports negative values when invalid
            if slant_range > 0:            
                transmitter_pose = PoseStamped()
                transmitter_pose.header.stamp = rospy.Time.now()
                transmitter_pose.header.frame_id = "locator"
                transmitter_pose.pose.position = self.calculate_pos(apparent_bearing_rad, apparent_elevation_rad, slant_range)
                transmitter_pose.pose.orientation = quaternion_from_euler2(0, -1.5708, 0) #pointing up I guess
                self.transmitter_apparent_pose_pub.publish(transmitter_pose)

        #true pose
        if fields[5] != '' and fields[7] != '' and fields[4] != '':
            true_bearing_rad = math.radians(float(fields[5]))
            true_elevation_rad = math.radians(float(fields[7]))
            slant_range = float(fields[4])
            
            if slant_range > 0: 
                transmitter_pose = PoseStamped()
                transmitter_pose.header.stamp = rospy.Time.now()
                transmitter_pose.header.frame_id = "locator"
                transmitter_pose.pose.position = self.calculate_pos(true_bearing_rad, true_elevation_rad, slant_range)
                transmitter_pose.pose.orientation = quaternion_from_euler2(0, -1.5708, 0) #pointing up I guess
                self.transmitter_true_pose_pub.publish(transmitter_pose)

        #is intialized based on last autosync
        if fields[15] != '':
            msg = Bool()
            msg.data = int(fields[15]) >= 0
            self.init_pub.publish(msg)


    #https://docs.ceruleansonar.com/c/rov-locator/appendix-math-for-computing-remote-latitude-longitude/receiver-and-gps-at-topside-and-transmitter-deepside
    def calculate_pos(self, bearing, elevation, slant):
        map_radius = math.cos(elevation) * slant  # horizontal distance
        depth = math.sin(elevation) * slant       # vertical distance
        
        p = Point()
        p.x = map_radius * math.cos(bearing)
        p.y = map_radius * math.sin(bearing)
        p.z = abs(depth)
        return p

    def run(self):    
        rospy.loginfo("ROV Locator Mk2 setting up on %s", self.port)
        ser = serial.Serial(self.port, 115200, timeout=1)
        try:
            while not rospy.is_shutdown():
                raw_line = ser.readline()
                if not raw_line:
                    continue

                # Decode the raw bytes; ignore errors to handle any non-ascii gracefully
                try:
                    line = raw_line.decode('ascii', errors='ignore')
                except Exception as e:
                    print("Decode error:", e)
                    continue

                if not line.startswith('$'):
                    continue

                if self.publish_raw:
                    self.string_pub.publish(line)

                # Parse and validate the NMEA sentence
                fields = self.parse_nmea_line(line)
                if fields is None:
                    continue

                if fields[0].startswith("USRTH"):
                    self.process_usrth(fields)

        except KeyboardInterrupt:
            print("Exiting...")
        finally:
            ser.close()


if __name__ == "__main__":
    try:
        locator_node = LocatorNode()
        locator_node.run()
    except rospy.ROSInterruptException:
        pass
