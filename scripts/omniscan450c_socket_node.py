#!/usr/bin/env python3
import socket
import struct
import time
import rospy
import numpy as np

from std_msgs.msg import Header
from cerulean_sonar_ros.msg import OmniscanRaw

from dynamic_reconfigure.server import Server
from cerulean_sonar_ros.cfg import Omniscan450Config

class O450CDriver:
	def __init__(self):

		rospy.init_node('o450c_socker_driver_node')

		self.param_ip = rospy.get_param('~ip', '192.168.2.25')
		self.param_port = rospy.get_param('~port', 51200)
		self.param_frame_id = rospy.get_param('~frame_id', 'omniscan_link')

		self.param_min_range = rospy.get_param('~start_range_meters', 0)
		self.param_max_range = rospy.get_param('~end_range_meters', 50)
		self.param_num_data_points = rospy.get_param('~num_data_points', 200)

		self.param_speed_of_sound = rospy.get_param('~speed_of_sound', 1515)

		self.raw_pub = rospy.Publisher("/omniscan/raw", OmniscanRaw, queue_size=10)

		#https://docs.ceruleansonar.com/c/omniscan-450/application-programming-interface
		self.msec_per_ping = 0          # 0 for best rate
		self.pulse_len_percent = 0.002
		self.filter_duration_percent = 0.0015
		self.gain_index = -1            # -1 for auto gain

		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.settimeout(5.0)
		self.sock.connect((self.param_ip, self.param_port))

		self.set_speed_of_sound(self.param_speed_of_sound)

		self.dynamic_reconfigure_server = Server(Omniscan450Config, self.reconfig_callback)

	def reconfig_callback(self, config, level):
		rospy.loginfo(f"Reconfigure Request: min_range={config['min_range']}, "
					  f"max_range={config['max_range']}, n_points={config['n_points']}, "
					  f"speed_of_sound={config['speed_of_sound']}")

		self.param_min_range = config["min_range"]
		self.param_max_range = config["max_range"]
		self.param_num_data_points = config["n_points"]
		self.param_speed_of_sound = config["speed_of_sound"]

		self.set_speed_of_sound(self.param_speed_of_sound)
		self.stop_pinging()
		time.sleep(0.1)
		self.start_pinging()

		return config


	def create_packet(self, packet_id, payload=b''):
		# 'BR' + u16 payload_len + u16 packet_id + u8 rsvd + u8 rsvd + payload + u16 checksum
		header = struct.pack('<2sHHBB', b'BR', len(payload), packet_id, 0, 0)
		packet = header + payload
		checksum = sum(packet) & 0xFFFF
		return packet + struct.pack('<H', checksum)

	def receive_packet(self):
		try:
			header = self.sock.recv(8)
			if len(header) < 8:
				return None, None
			start, payload_len, packet_id, _, _ = struct.unpack('<2sHHBB', header)
			if start != b'BR':
				return None, None

			# read payload + checksum
			remaining = b''
			want = payload_len + 2
			while len(remaining) < want:
				chunk = self.sock.recv(want - len(remaining))
				if not chunk:
					return None, None
				remaining += chunk

			payload, rx_checksum_bytes = remaining[:-2], remaining[-2:]

			#checksum
			calc = (sum(header + payload) & 0xFFFF)
			if calc != struct.unpack('<H', rx_checksum_bytes)[0]:
				print("Checksum mismatch")
				return None, None
			
			return packet_id, payload
		except socket.timeout:
			return None, None

	def set_speed_of_sound(self, speed: int):
		payload = struct.pack('<I', speed)
		pkt = self.create_packet(116, payload)
		self.sock.sendall(pkt)
		rospy.loginfo("Set speed of sound to "+str(speed))

	def start_pinging(self):
		"""
		os_ping_params (ID 2197)
		<IIIffffhHBB>
		"""
		payload = struct.pack(
			'<IIIffffhHBB',
			int(self.param_min_range * 1000),       # u32 start_mm
			int(self.param_max_range * 1000),       # u32 length_mm
			self.msec_per_ping,                     # u32 msec_per_ping
			0.0,                                    # float reserved
			0.0,                                    # float reserved
			float(self.pulse_len_percent),          # float pulse_len_percent
			float(self.filter_duration_percent),    # float filter_duration_percent
			int(self.gain_index),                   # i16 gain_index
			int(self.param_num_data_points),        # u16 number of signal data points
			1,                                      # u8 enable
			0                                       # u8 reserved
		)
		pkt = self.create_packet(2197, payload)
		self.sock.sendall(pkt)
		rospy.loginfo("Omniscan started pinging!")

	def stop_pinging(self):
		payload = struct.pack(
			'<IIIffffhHBB',
			int(self.param_min_range * 1000),
			int(self.param_max_range * 1000),
			self.msec_per_ping,
			0.0,
			0.0,
			float(self.pulse_len_percent),
			float(self.filter_duration_percent),
			int(self.gain_index),
			int(self.param_num_data_points),
			0,   # disable
			0
		)
		pkt = self.create_packet(2197, payload)
		self.sock.sendall(pkt)
		rospy.loginfo("Omniscan stopped pinging!")

	def process_profile_packet(self, payload: bytes):

		time_stamp = rospy.Time.now()

		"""
		os_mono_profile (ID 2198)

		Header layout:
		<IIIIIHHHBBffffff>
		ping_number (u32)
		start_mm (u32)
		length_mm (u32)
		timestamp_ms (u32)
		ping_hz (u32)
		gain_index (u16)
		num_results (u16)
		sos_dmps (u16)
		channel_number (u8)
		reserved (u8)
		pulse_duration_sec (float)
		analog_gain (float)
		max_pwr_db (float)
		min_pwr_db (float)
		transducer_heading_deg (float)
		vehicle_heading_deg (float)
		Followed by: pwr_results[num_results] (u16)
		"""
		try:
			header_fmt = '<IIIIIHHHBBffffff'
			header_size = struct.calcsize(header_fmt)
			
			(ping_number, start_mm, length_mm, timestamp_ms, ping_hz,
			 gain_index, num_results, sos_dmps, channel_number, _resv,
			 pulse_duration_sec, analog_gain, max_pwr_db, min_pwr_db,
			 transducer_heading_deg, vehicle_heading_deg) = struct.unpack(header_fmt, payload[:header_size])

			data_needed = header_size + num_results * 2
			if len(payload) < data_needed:
				return
			pwr_results = np.frombuffer(payload[header_size:data_needed], dtype='<u2')

			msg = OmniscanRaw()
			msg.header.stamp = time_stamp
			msg.header.frame_id = self.param_frame_id
			msg.ping_number = ping_number
			msg.start_mm = start_mm
			msg.length_mm = length_mm
			msg.timestamp_ms = timestamp_ms
			msg.ping_hz = ping_hz
			msg.gain_index = gain_index
			msg.num_results = num_results
			msg.sos_dmps = sos_dmps
			msg.channel_number = channel_number
			msg.pulse_duration_sec = pulse_duration_sec
			msg.analog_gain = analog_gain
			msg.max_pwr_db = max_pwr_db
			msg.min_pwr_db = min_pwr_db
			msg.transducer_heading_deg = transducer_heading_deg
			msg.vehicle_heading_deg = vehicle_heading_deg

			msg.pwr_results = pwr_results
			self.raw_pub.publish(msg)


		except Exception as e:
			rospy.logwarn(f"Error processing profile packet: {e}")

	def run(self):
		rospy.loginfo("Starting Omniscan 450C driver")
		self.start_pinging()
		try:
			while not rospy.is_shutdown():
				packet_id, payload = self.receive_packet()
				if packet_id is None:
					continue
				if packet_id == 2198:
					self.process_profile_packet(payload)
				else:
					# Unknown or unhandled packet type; safely skip
					pass
		except KeyboardInterrupt:
			rospy.loginfo("Interrupted by user")

	def shutdown(self):
		rospy.loginfo("Shutting down Omniscan 450C driver...")
		try:
			self.stop_pinging()
			time.sleep(0.1)
			self.sock.close()
		except Exception as e:
			rospy.loginfo(f"Error during shutdown: {e}")

def main():
	driver = None
	try:
		driver = O450CDriver()
		driver.run()
	except rospy.ROSInterruptException:
		pass
	finally:
		if driver:
			driver.shutdown()

if __name__ == '__main__':
	main()