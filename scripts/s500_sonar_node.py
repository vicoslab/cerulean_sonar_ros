#!/usr/bin/env python3
import socket
import struct
import time
import rospy

from sensor_msgs.msg import Range

class S500Driver:
    def __init__(self):
        rospy.init_node('s500_sonar_node')
        
        self.host = rospy.get_param('~host', '192.168.2.92')
        self.port = rospy.get_param('~port', 51200)
        self.frame_id = rospy.get_param('~frame_id', 'sonar_link')
        self.field_of_view = rospy.get_param('~field_of_view', 0.0872665) #5 deg
        
        # Create publisher
        self.range_pub = rospy.Publisher('sonar/range', Range, queue_size=10)
        
        # Initialize socket connection
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        
        # Initialize Range message
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.ULTRASOUND
        self.range_msg.field_of_view = self.field_of_view
        self.range_msg.min_range = 0.1  # 10cm
        self.range_msg.max_range = 50.0  # 50m
        self.range_msg.header.frame_id = self.frame_id

    def create_packet(self, packet_id, payload=b''):
        header = struct.pack('<2sHHBB', b'BR', len(payload), packet_id, 0, 0)
        packet = header + payload
        checksum = sum(packet) & 0xFFFF
        return packet + struct.pack('<H', checksum)

    def receive_packet(self):
        header = self.sock.recv(8)
        if len(header) < 8:
            return None
        
        start, payload_len, packet_id, _, _ = struct.unpack('<2sHHBB', header)
        if start != b'BR':
            return None

        remaining = self.sock.recv(payload_len + 2)
        payload = remaining[:-2]
        return packet_id, payload

    def start_pinging(self):
        # Configure continuous pinging
        params = struct.pack('<IIhhHHHBB', 
            0,      # start_mm
            50000,  # length_mm (50m range)
            -1,     # gain_index (auto)
            100,    # msec_per_ping (10Hz)
            0,      # pulse_len_usec (auto)
            1223,   # report_id (distance2)
            0,      # reserved
            0,      # chirp
            0       # decimation
        )
        self.sock.send(self.create_packet(1015, params))

    def stop_pinging(self):
        # Set transceiver to idle by sending a set_ping_params command with report_id=0
        params = struct.pack('<IIhhHHHBB', 
            0,      # start_mm
            0,      # length_mm 
            -1,     # gain_index (auto)
            -1,     # msec_per_ping (single ping)
            0,      # pulse_len_usec (auto)
            0,      # report_id (0 disables pinging)
            0,      # reserved
            0,      # chirp
            0       # decimation
        )
        self.sock.send(self.create_packet(1015, params))

    def run(self):
        self.start_pinging()
        
        while not rospy.is_shutdown():
            try:
                packet_id, payload = self.receive_packet()
                
                if packet_id == 1223:  # distance2 packet
                    dist, avg_dist, _, conf, avg_conf, timestamp = struct.unpack('<IIHBBI', payload)
                    
                    # Update Range message
                    self.range_msg.header.stamp = rospy.Time.now()
                    self.range_msg.range = dist / 1000.0  # Convert mm to meters
                    
                    # Publish message
                    self.range_pub.publish(self.range_msg)
                    
                    # Debug output (optional)
                    rospy.logdebug(f"Distance: {dist/1000:.2f}m (Confidence: {conf}%)")
                
            except Exception as e:
                rospy.logerr(f"Error reading from sonar: {e}")
                break
        

    def shutdown(self):
        self.stop_pinging()
        self.sock.close()

def main():
    driver = None
    try:
        driver = S500Driver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if driver:
            driver.shutdown()

if __name__ == '__main__':
    main()