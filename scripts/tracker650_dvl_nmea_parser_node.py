#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import TwistStamped, Vector3
from sensor_msgs.msg import Range
from std_msgs.msg import Header, String, Float32

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

        self.velocity_pub = rospy.Publisher('/dvl/vel', TwistStamped, queue_size=10)
        self.pos_pub = rospy.Publisher('/dvl/position_delta', Vector3, queue_size=10)
        self.pitch_pub = rospy.Publisher('/dvl/pitch', Float32, queue_size=10)
        self.roll_pub = rospy.Publisher('/dvl/roll', Float32, queue_size=10)

        self.vel_sub = rospy.Subscriber("/dvl/nmea_string", String, self.nmea_callback)

        rospy.loginfo("DVL NMEA parser ready.")

    def nmea_callback(self, msg):
        message = msg.data
        fields = message.split(',')

        if not fields:
            return
            
        msg_type = fields[0]
        
        if msg_type == '$DVKFC':
            parsed_data = self.parse_dvkfc(message)
            if parsed_data:
                for channel_name, channel_data in parsed_data['channels'].items():
                    self.publish_beam_data(channel_data, channel_name)
        elif msg_type == '$DVPDX':
            parsed_data = self.parse_dvpdx(fields)
            if parsed_data:                        
                self.publish_velocity_data(parsed_data)

    def publish_beam_data(self, channel_data, channel_name):
        range_msg = Range()
        range_msg.header = Header()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = f"{self.frame_id}_beam_{channel_name.lower()}"
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.0872665 
        range_msg.min_range = 0.5
        range_msg.max_range = 50.0
        range_msg.range = channel_data['range_m']
        
        self.beam_pubs[channel_name].publish(range_msg)
        self.beams_consolidated_pub.publish(range_msg)

    def publish_velocity_data(self, data):

        if data["confidence"] < 30:
            #data is rubbish, ignore
            return
        
        deltasec = data["delta_time_usec"] / 1_000_000
        
        twist_msg = TwistStamped()
        twist_msg.header = Header()
        twist_msg.header.stamp = rospy.Time.now()
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
        range_msg.header.stamp = rospy.Time.now()
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