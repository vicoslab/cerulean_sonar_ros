 
#!/usr/bin/env python3
import serial

def calculate_nmea_checksum(sentence):
    """
    Calculate the NMEA checksum for the sentence.
    Input sentence should NOT include the starting '$' nor the '*' and checksum.
    """
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return checksum

def parse_nmea_line(line):
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
    calc_checksum = calculate_nmea_checksum(sentence)
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

def process_usrth(fields):
    """
    Processes a $USRTH message by mapping known fields.
    Since new fields may be appended, this function only prints known ones.
    Field indices (after the message type field at index 0) follow the provided documentation:
    
    Format:  $USRTH,ab,ac,ae,sr,tb,cb,te,er,ep,ey,ch,db,ah,ag,ls,im,oc,idx,idq*hh
    """
    labels = [
        "apparent bearing (math)",    # ab
        "apparent bearing (compass)", # ac
        "apparent elevation",         # ae
        "slant range (m)",            # sr
        "true bearing (math)",        # tb
        "true bearing (compass)",     # cb
        "true elevation",             # te
        "Euler roll",                 # er
        "Euler pitch",                # ep
        "Euler yaw",                  # ey
        "compass heading",            # ch
        "analog AGC gain",            # db
        "CPU autosync support",       # ah
        "GNSS autosync support",      # ag
        "seconds since last autosync",# ls
        "IMU status",                 # im
        "operating channel",          # oc
        "transmitter/transponder ID decoded", # idx
        "transmitter/transponder ID queried"  # idq
    ]
    # fields[0] is the message type "USRTH", so fields[1] corresponds to the first data value.
    print("Parsed $USRTH message:")
    for i, label in enumerate(labels, start=1):
        # If the message has fewer fields than expected, skip missing ones
        if i < len(fields):
            print(f"  {label}: {fields[i]}")
    print("-" * 40)

def main():
    # Open the serial port. Change '/dev/ttyUSB0' if necessary.
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    print("Listening on /dev/ttyUSB0 at 115200 baud...")
    try:
        while True:
            # Read a line from the serial port (ends with CRLF)
            raw_line = ser.readline()
            if not raw_line:
                continue

            # Decode the raw bytes; ignore errors to handle any non-ascii gracefully
            try:
                line = raw_line.decode('ascii', errors='ignore')
            except Exception as e:
                print("Decode error:", e)
                continue

            # Only process lines starting with a '$'
            if not line.startswith('$'):
                continue

            # Parse and validate the NMEA sentence
            fields = parse_nmea_line(line)
            if fields is None:
                continue

            # Process only $USRTH messages (ignore others like $USINF, $USERR, etc.)
            if fields[0].startswith("USRTH"):
                process_usrth(fields)
            # Other message types can be added here as needed.
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
