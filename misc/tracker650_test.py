import socket
from datetime import datetime

#just a python script as an API reference

def parse_dvkfc(message):
    # Split the message into fields, but remove the checksum part first
    message = message.split('*')[0]  # Remove checksum
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
        
        # Parse the three channels (A, B, C)
        channels = {}
        for i, channel in enumerate(['A', 'B', 'C']):
            # Find the channel marker index
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
        print(f"Error parsing DVKFC message: {e}")
        return None

def parse_dvpdl(fields):
    try:
        # Remove the checksum part from the last field
        last_field = fields[-1].split('*')[0]
        
        return {
            'type': 'DVPDL',
            'time_usec': int(fields[1]),
            'delta_time_usec': int(fields[2]),
            'angle_delta': {
                'roll': float(fields[3]),
                'pitch': float(fields[4]),
                'yaw': float(fields[5])
            },
            'position_delta': {
                'x': float(fields[6]),
                'y': float(fields[7]),
                'z': float(fields[8])
            },
            'confidence': int(last_field)
        }
    except (ValueError, IndexError) as e:
        print(f"Error parsing DVPDL message: {e}")
        return None

def parse_dvpdx(fields):
    # Remove the checksum part from the last field
    fields = fields[:-1] + [fields[-1].split('*')[0]]
    
    data = parse_dvpdl(fields[:10])  # Get the base DVPDL fields
    if not data:
        return None
        
    try:
        data['type'] = 'DVPDX'
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


def enable_dvkfc(ip='192.168.2.3', port=50000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Command - note the \n terminator
    command = "SEND-DVKFC ON\n"
    
    try:
        # Send the command
        sock.sendto(command.encode('ascii'), (ip, port))
        print(f"Sent command '{command.strip()}' to {ip}:{port}")
    except Exception as e:
        print(f"Error sending command: {e}")
    finally:
        sock.close()

def read_data():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # Bind to the broadcast port
    bind_address = ('255.255.255.255', 27000)
    sock.bind(bind_address)
    
    print("Listening for Tracker-650 data on port 27000...")
    
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            message = data.decode('ascii').strip()
            fields = message.split(',')

            if not fields:
                continue
                
            msg_type = fields[0]
            parsed_data = None
            
            if msg_type == '$DVPDL':
                parsed_data = parse_dvpdl(fields)
            elif msg_type == '$DVPDX':
                parsed_data = parse_dvpdx(fields)
            elif msg_type == '$DVKFC':
                parsed_data = parse_dvkfc(message)
                
            if parsed_data:
                print(parsed_data)
                
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
        except Exception as e:
            print(f"Error: {e}")
            continue

if __name__ == "__main__":
    enable_dvkfc()
    read_data()