#!/usr/bin/env python3

import rospy
import socket
import time
from std_msgs.msg import String

class DVLSocketNode:
    def __init__(self):
        rospy.init_node('dvl_socket_node')
        
        # Get parameters
        self.ip = rospy.get_param('~ip', '192.168.2.3')
        self.command_port = rospy.get_param('~command_port', 50000)
        self.listen_port = rospy.get_param('~listen_port', 27000)
        self.command_list = rospy.get_param('~command_list', ["SEND-DVKFC ON\n", "SEND-DVPDL OFF\n"])
        self.frame_id = rospy.get_param('~frame_id', 'dvl')

        self.string_pub = rospy.Publisher('/dvl/nmea_string', String, queue_size=10)

        # Initialize sockets
        self.setup_sockets()

    def setup_sockets(self):
        # Setup listening socket
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.listen_sock.bind(('255.255.255.255', self.listen_port))

        # Send initial commands
        command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for command in self.command_list:
                command_sock.sendto(command.encode('ascii'), (self.ip, self.command_port))
                rospy.loginfo(f"Sent command '{command.strip()}' to {self.ip}:{self.command_port}")
                time.sleep(0.1)
        finally:
            command_sock.close()

    def run(self):
        rospy.loginfo("Starting DVL socket node...")
        
        msg_received = False

        while not rospy.is_shutdown():
            try:
                data, _ = self.listen_sock.recvfrom(4096)
                message = data.decode('ascii').strip()
                fields = message.split(',')

                if not fields:
                    continue

                if not msg_received:
                    rospy.loginfo("Data received!")
                    msg_received = True

                self.string_pub.publish(message)
                        
            except Exception as e:
                rospy.logerr(f"Error in DVL node: {e}")
                continue

if __name__ == "__main__":
    try:
        dvl_node = DVLSocketNode()
        dvl_node.run()
    except rospy.ROSInterruptException:
        pass