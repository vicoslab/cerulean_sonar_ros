#!/usr/bin/env python3
import rospy
import math
import numpy as np

from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import Range
from std_msgs.msg import Header, String, Float32

class DVLVelocityMapper:
    def __init__(self):
        self.tilt_rad = 1.22173
        self.yaw_rad = 2.0944

        self.tilt_mult = 1.0 / math.cos(self.tilt_rad)

        self.beam_dirs = self._compute_beam_directions()
        self.H = np.array(self.beam_dirs)
        
    def _compute_beam_directions(self):
        def rot_matrix(roll, pitch, yaw):
            R_x = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])
            R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                           [0, 1, 0],
                           [-np.sin(pitch), 0, np.cos(pitch)]])
            R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                           [np.sin(yaw), np.cos(yaw), 0],
                           [0, 0, 1]])
            return R_z @ R_y @ R_x
        
        beam_a_R = rot_matrix(0, self.tilt_rad, -self.yaw_rad)
        beam_b_R = rot_matrix(0, self.tilt_rad, 0)
        beam_c_R = rot_matrix(0, self.tilt_rad, self.yaw_rad)
        
        # Beams point along their local +Z axis
        local_beam_dir = np.array([0, 0, 1])
        
        return [
            beam_a_R @ local_beam_dir,
            beam_b_R @ local_beam_dir,
            beam_c_R @ local_beam_dir
        ]
    
    def beam_to_dvl_velocity(self, v_beam_a, v_beam_b, v_beam_c):
        v_beams = np.array([v_beam_a, v_beam_b, v_beam_c])
        v_dvl = np.linalg.solve(self.H, v_beams)

        #X and Y correction, probably should do this in a more proper way
        v_dvl[0] = v_dvl[0] * self.tilt_mult
        v_dvl[1] = -v_dvl[1] * self.tilt_mult
        return v_dvl  # [v_x, v_y, v_z]

class DVLParserNode:
    def __init__(self):
        rospy.init_node('dvl_nmea_parser_node')
        
        self.frame_id = rospy.get_param('~frame_id', 'dvl')

        self.beam_center_pub = rospy.Publisher('/dvl/beam_center', Range, queue_size=10)
        self.beams_consolidated_pub = rospy.Publisher('/dvl/beams_all', Range, queue_size=10)
        self.beam_pubs = {
            'A': rospy.Publisher('/dvl/beam_A', Range, queue_size=10),
            'B': rospy.Publisher('/dvl/beam_B', Range, queue_size=10),
            'C': rospy.Publisher('/dvl/beam_C', Range, queue_size=10)
        }

        self.vel_pubs = {
            'A': rospy.Publisher('/dvl/raw_vel_A', Float32, queue_size=10),
            'B': rospy.Publisher('/dvl/raw_vel_B', Float32, queue_size=10),
            'C': rospy.Publisher('/dvl/raw_vel_C', Float32, queue_size=10)
        }

        self.velocity_pub = rospy.Publisher('/dvl/vel', TwistStamped, queue_size=10)
        self.velocity_ekf_pub = rospy.Publisher('/dvl/vel_dvkfc', TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('/dvl/position_delta', Vector3, queue_size=10)
        self.pitch_pub = rospy.Publisher('/dvl/pitch', Float32, queue_size=10)
        self.roll_pub = rospy.Publisher('/dvl/roll', Float32, queue_size=10)

        self.vel_sub = rospy.Subscriber("/dvl/nmea_string", String, self.nmea_callback)

        self.beam_mapper = DVLVelocityMapper()
    
        rospy.loginfo("DVL NMEA parser ready.")

    def nmea_callback(self, msg):
        time_stamp = rospy.Time.now()
        message = msg.data
        fields = message.split(',')

        if not fields:
            return
            
        msg_type = fields[0]
        
        if msg_type == '$DVKFC':
            parsed_data = self.parse_dvkfc(message)
            if parsed_data:
                for channel_name, channel_data in parsed_data['channels'].items():
                    self.publish_beam_data(time_stamp, channel_data, channel_name)
                self.publish_ekf_velocity(time_stamp, parsed_data)                
        elif msg_type == '$DVPDX':
            parsed_data = self.parse_dvpdx(fields)
            if parsed_data:                        
                self.publish_velocity_data(time_stamp, parsed_data)

    def publish_beam_data(self, time_stamp, channel_data, channel_name):
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = time_stamp
        range_msg.header.frame_id = f"{self.frame_id}_beam_{channel_name.lower()}"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.0872665 
        range_msg.min_range = 0.5
        range_msg.max_range = 50.0
        range_msg.range = channel_data['range_m']
        
        self.beam_pubs[channel_name].publish(range_msg)
        self.beams_consolidated_pub.publish(range_msg)

        if channel_data["velocity_confidence"] < 0.5:
            vel_msg = Float32()
            vel_msg.data = channel_data['velocity_ms']        
            self.vel_pubs[channel_name].publish(vel_msg)

    def publish_ekf_velocity(self, time_stamp, data):
        """
        Convert DVL beam velocities to orthogonal X, Y, Z velocities in DVL frame
        
        Args:
            va, vb, vc: Beam velocities in m/s
        
        Returns:
            vx, vy, vz: Velocities in DVL frame (m/s)
        """

        A = data["channels"]["A"]
        B = data["channels"]["B"]
        C = data["channels"]["C"]

        if A["velocity_confidence"]  > 0.5:
            return
        
        if B["velocity_confidence"]  > 0.5:
            return
        
        if C["velocity_confidence"]  > 0.5:
            return

        v_x, v_y, v_z = self.beam_mapper.beam_to_dvl_velocity(A["velocity_ms"],B["velocity_ms"],C["velocity_ms"])

        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.stamp = time_stamp
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = v_x
        twist_msg.twist.linear.y = v_y
        twist_msg.twist.linear.z = v_z
        self.velocity_ekf_pub.publish(twist_msg)

    def publish_velocity_data(self, time_stamp, data):

        if data["confidence"] < 30:
            #data is rubbish, ignore
            return
        
        deltasec = data["delta_time_usec"] / 1_000_000
        
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.stamp = time_stamp
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = data["position_delta"]["x"] / deltasec
        twist_msg.twist.linear.y = data["position_delta"]["y"] / deltasec
        twist_msg.twist.linear.z = data["position_delta"]["z"] / deltasec
        self.velocity_pub.publish(twist_msg)

        vec_msg = Vector3()
        vec_msg.x = data["position_delta"]["x"]
        vec_msg.y = data["position_delta"]["y"]
        vec_msg.z = data["position_delta"]["z"]
        self.pos_pub.publish(vec_msg)

        self.pitch_pub.publish(math.radians(data["pitch"]))
        self.roll_pub.publish(math.radians(data["roll"]))

        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = time_stamp
        range_msg.header.frame_id = f"{self.frame_id}_beam_center"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.174533 #total envelope
        range_msg.min_range = 0.2
        range_msg.max_range = 50.0
        range_msg.range = data["standoff"]
        self.beam_center_pub.publish(range_msg)
        self.beams_consolidated_pub.publish(range_msg)

    def parse_dvkfc(self, message):
        # Original parse_dvkfc function contents here
        message = message.split('*')[0]
        fields = message.split(',')
        
        if len(fields) < 25 or not fields[0] == '$DVKFC':
            return None
            
        try:
            data = {
                'version': int(fields[1]),
                'sequence': int(fields[2]),
                'delta_time': float(fields[3]),
                'system_time': float(fields[4])
            }
            
            channels = {}
            for i, channel in enumerate(['A', 'B', 'C']):
                marker_idx = fields.index(f'[{channel}]')
                base_idx = marker_idx + 1
                
                channels[channel] = {
                    'gain_db': int(fields[base_idx]),
                    'ping_cycles': int(fields[base_idx + 1]),
                    'range_m': float(fields[base_idx + 2]),
                    'range_confidence': float(fields[base_idx + 3]),
                    'velocity_ms': float(fields[base_idx + 4]),
                    'velocity_confidence': float(fields[base_idx + 5])
                }
            
            data['channels'] = channels
            return data
        except (ValueError, IndexError) as e:
            rospy.logerr(f"Error parsing DVKFC message: {e}")
            return None

    def parse_dvpdl(self, fields):
        try:
            # Remove the checksum part from the last field
            last_field = fields[-1].split('*')[0]
            
            return {
                'time_usec': int(fields[1]),
                'delta_time_usec': int(fields[2]),
                'angle_delta': { #this is always zero
                    'roll': float(fields[3]),
                    'pitch': float(fields[4]),
                    'yaw': float(fields[5])
                },
                'position_delta': {
                    'x': float(fields[6]),
                    'y': -float(fields[7]),
                    'z': -float(fields[8])
                },
                'confidence': int(last_field)
            }
        except (ValueError, IndexError) as e:
            print(f"Error parsing DVPDL message: {e}")
            return None

    #https://docs.ceruleansonar.com/c/tracker-650/expectations
    def parse_dvpdx(self, fields):
        # Remove the checksum part from the last field
        fields = fields[:-1] + [fields[-1].split('*')[0]]
        
        data = self.parse_dvpdl(fields[:10])  # Get the base DVPDL fields
        if not data:
            return None
            
        try:
            data.update({
                'mode': int(fields[10]),
                'pitch': float(fields[11]),
                'roll': float(fields[12]),
                'standoff': float(fields[13])
            })
            return data
        except (ValueError, IndexError) as e:
            print(f"Error parsing DVPDX additional fields: {e}")
            return None

if __name__ == "__main__":
    try:
        dvl_node = DVLParserNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass