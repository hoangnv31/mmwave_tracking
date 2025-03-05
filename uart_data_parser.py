import serial
import struct
import time
import json
import os
import math
import logging

# Configure logging
logging.basicConfig(level=logging.ERROR, format='%(asctime)s - %(levelname)s - %(message)s')
log = logging.getLogger(__name__)

# Constants from demo.pdf
UART_MAGIC_WORD = b'\x01\x02\x03\x04\x05\x06\x07\x08'
FRAME_HEADER_SIZE = 52
TLV_HEADER_SIZE = 8

POINTCLOUD_3D = 1020
TARGET_LIST_3D = 1010
TARGET_INDEX = 1011
PRESENCE_INDICATION = 1012
TARGET_HEIGHT = 1021

class SerialDataHandler:
    def __init__(self, port, baud_rate, parser_type="DoubleCOMPort", save_binary=0, frames_per_file=100, filepath="", replay = False):
        """
        Initializes the SerialDataHandler.

        Args:
            port (str): The serial port to use (e.g., 'COM3' or '/dev/ttyACM0').
            baud_rate (int): The baud rate for the serial connection.
            parser_type (str): The type of parser to use.
            save_binary (int): Flag to save binary data.
            frames_per_file (int): Number of frames per file when saving binary data.
            filepath (str): Filepath to store saved binary data.
            replay (bool): Flag to indicate if in replay mode.

        """
        self.port = port
        self.baud_rate = baud_rate
        self.parserType = parser_type
        self.saveBinary = save_binary
        self.framesPerFile = frames_per_file
        self.filepath = filepath
        self.replay = replay
        self.dataCom = None
        self.frames = []
        self.binData = bytearray(b'')
        self.uartCounter = 0
        self.first_file = True
        self.replay_history = []

    def connect(self):
        """
        Connects to the serial port.
        """
        try:
            self.dataCom = serial.Serial(self.port, self.baud_rate, timeout=2)
            log.info(f"Connected to {self.port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            log.error(f"Error connecting to serial port: {e}")
            self.dataCom = None

    def disconnect(self):
        """
        Disconnects from the serial port.
        """
        if self.dataCom and self.dataCom.is_open:
            self.dataCom.close()
            log.info("Disconnected from serial port")

    def replayHist(self):
        """
        Returns the replay history.
        """
        if len(self.replay_history) > 0:
            return self.replay_history.pop(0)
        else:
           return None


    def read_and_parse_uart_double_com_port(self):
            """
            Reads and parses data from the UART port, handling the magic word and TLV structures.

            Returns:
                dict or None: A dictionary containing parsed data or None if an error occurred.
            """
            self.fail = 0

            if self.replay:
                return self.replayHist()

            data = {'cfg': None, 'demo': None, 'device': None}
            index = 0
            magicByte = b''
            frameData = bytearray(b'')

            while True:
                # Read a single byte
                magicByte = self.dataCom.read(1)

                if not magicByte:
                   log.error("ERROR: No data detected on COM Port, read timed out")
                   log.error("\tBe sure that the device is in the proper mode, and that the cfg you are sending is valid")
                   continue

                if magicByte == UART_MAGIC_WORD[index]:
                    index += 1
                    frameData.append(magicByte)
                    if index == 8:
                        break
                else:
                   if index == 0:
                        index = 0
                        frameData = bytearray(b'')

            # Read in version from the header (4 bytes)
            versionBytes = self.dataCom.read(4)
            frameData += bytearray(versionBytes)

            # Read in length from header (4 bytes)
            lengthBytes = self.dataCom.read(4)
            frameData += bytearray(lengthBytes)

            # Convert length bytes to an integer (little-endian)
            frameLength = int.from_bytes(lengthBytes, byteorder='little')
            frameLength -= 16

            # Read the remaining frame data
            frameData += bytearray(self.dataCom.read(frameLength))

            # Parse the frame data
            if self.parserType == "DoubleCOMPort":
               outputDict = parse_standard_frame(frameData)
            else:
                log.error('FAILURE: Bad parserType')
                return None

            # If save binary is enabled, save binary data
            if self.saveBinary == 1:
                self.uartCounter += 1
                # Saving data here for replay
                frameJSON = {}
                frameJSON['frameData'] = outputDict
                frameJSON['timestamp'] = time.time() * 1000
                self.frames.append(frameJSON)
                data['data'] = self.frames

                if self.uartCounter % self.framesPerFile == 0:
                   if self.first_file:
                        if not os.path.exists('binData/'):
                            os.mkdir('binData/')
                        os.mkdir('binData/'+self.filepath)
                        self.first_file = False
                   with open(f'./binData/{self.filepath}/replay_{math.floor(self.uartCounter/self.framesPerFile)}.json', 'w') as fp:
                        json_object = json.dumps(data, indent=4)
                        fp.write(json_object)
                        #self.frames = [] #uncomment to put data into one file at a time in 100 frame chunks
            return outputDict

def parse_standard_frame(frame_data):
    """
    Parses a frame of data based on the TLV structure.

    Args:
        frame_data (bytes): The raw frame data, including the frame header.

    Returns:
        dict: A dictionary containing parsed data including the frame header and TLVs.
    """
    output_dict = {}
    # Extract frame header
    header = {}
    header['sync'] = struct.unpack('<Q', frame_data[0:8])
    header['version'] = struct.unpack('<I', frame_data[8:12])
    header['packetLength'] = struct.unpack('<I', frame_data[12:16])
    header['platform'] = struct.unpack('<I', frame_data[16:20])
    header['frameNumber'] = struct.unpack('<I', frame_data[20:24])
    header['subframeNumber'] = struct.unpack('<I', frame_data[24:28])
    header['chirpMargin'] = struct.unpack('<I', frame_data[28:32])
    header['frameProcTimeInUsec'] = struct.unpack('<I', frame_data[32:36])
    header['trackProcessTime'] = struct.unpack('<I', frame_data[36:40])
    header['uartSentTime'] = struct.unpack('<I', frame_data[40:44])
    header['numTLVs'] = struct.unpack('<H', frame_data[44:46])
    header['checksum'] = struct.unpack('<H', frame_data[46:48])
    output_dict['header'] = header

    # Initialize an empty list to store TLV data
    tlvs = []
    output_dict['tlvs'] = tlvs

    # Start reading TLVs after the header
    tlv_start = FRAME_HEADER_SIZE

    for _ in range(header['numTLVs']):
        # Check if there's enough data left for a TLV header
        if tlv_start + TLV_HEADER_SIZE > len(frame_data):
            log.error("Error: Not enough data for TLV header")
            break

        # Read TLV header
        tlv_header = {}
        tlv_header['type'] = struct.unpack('<I', frame_data[tlv_start:tlv_start+4])
        tlv_header['length'] = struct.unpack('<I', frame_data[tlv_start+4:tlv_start+8])

        # Check if there's enough data left for TLV payload
        if tlv_start + tlv_header['length'] > len(frame_data):
            log.error("Error: Not enough data for TLV payload")
            break

        # Read TLV payload
        tlv_payload_start = tlv_start + TLV_HEADER_SIZE
        tlv_payload_end = tlv_start + tlv_header['length']
        tlv_payload = frame_data[tlv_payload_start:tlv_payload_end]

        # Parse TLV payload based on type
        if tlv_header['type'] == POINTCLOUD_3D:
           parsed_tlv = parse_point_cloud_tlv(tlv_payload)
           tlvs.append(parsed_tlv)

        elif tlv_header['type'] == TARGET_LIST_3D:
            parsed_tlv = parse_target_list_tlv(tlv_payload)
            tlvs.append(parsed_tlv)

        elif tlv_header['type'] == TARGET_INDEX:
            parsed_tlv = parse_target_index_tlv(tlv_payload)
            tlvs.append(parsed_tlv)
        elif tlv_header['type'] == PRESENCE_INDICATION:
             parsed_tlv = parse_presence_indication_tlv(tlv_payload)
             tlvs.append(parsed_tlv)
        elif tlv_header['type'] == TARGET_HEIGHT:
             parsed_tlv = parse_target_height_tlv(tlv_payload)
             tlvs.append(parsed_tlv)
        else:
            log.warning(f"Warning: Unknown TLV type: {tlv_header['type']}")
            parsed_tlv = {'type': tlv_header['type'], 'data': tlv_payload.hex()}
            tlvs.append(parsed_tlv)

        # Move to the next TLV
        tlv_start += tlv_header['length']

    return output_dict

def parse_point_cloud_tlv(payload):
    """
    Parses a Point Cloud TLV payload.

    Args:
        payload (bytes): The payload of the Point Cloud TLV.

    Returns:
       dict: Parsed point cloud data.
    """
    points = []
    unit_struct_size = 20

    # Ensure that there is enough data for unit structure
    if len(payload) < unit_struct_size:
         log.error(f"Error: Not enough data for point cloud unit structure")
         return {'type': POINTCLOUD_3D, 'error': 'Insufficient data for unit struct'}

    unit_data = struct.unpack('<fffff', payload[0:unit_struct_size])
    point_data = payload[unit_struct_size:]

    point_size = 8  # Size of each point struct (1+1+2+2+2 bytes)
    num_points = len(point_data) // point_size

    for i in range(num_points):
       start = i * point_size
       end = start + point_size

       if end > len(point_data):
          log.error(f"Error: Insufficient data for point {i} in point cloud tlv")
          continue

       point = struct.unpack('<bbhHh', point_data[start:end])
       point_dict = {
          'elevationUnit' : unit_data,
          'azimuthUnit' : unit_data[1],
          'dopplerUnit' : unit_data[2],
          'rangeUnit' : unit_data[3],
          'snrUnit' : unit_data[4],
          'elevation': point,
          'azimuth': point[1],
          'doppler': point[2],
          'range': point[3],
          'snr': point[4]
          }
       points.append(point_dict)
    return {'type': POINTCLOUD_3D, 'points': points}

def parse_target_list_tlv(payload):
    """
    Parses a Target List TLV payload.

    Args:
        payload (bytes): The payload of the Target List TLV.

    Returns:
        dict: Parsed target list data.
    """
    targets = []
    target_size = 108 # Size of each target struct (4 + 4*10 + 16*4 +4 bytes)
    num_targets = len(payload) // target_size

    for i in range(num_targets):
        start = i * target_size
        end = start + target_size
        if end > len(payload):
           log.error(f"Error: Insufficient data for target {i} in target list tlv")
           continue
        target_data = struct.unpack('<Ifffffffffff16f', payload[start:end])
        target = {
            'tid': target_data,
            'posX': target_data[1],
            'posY': target_data[2],
             'velX': target_data[3],
            'velY': target_data[4],
             'accX': target_data[5],
             'accY': target_data[6],
             'posZ': target_data[7],
             'velZ': target_data[8],
             'accZ': target_data[9],
             'EC': list(target_data[10:26]),
             'g': target_data[10]
             }
        targets.append(target)

    return {'type': TARGET_LIST_3D, 'targets': targets}


def parse_target_index_tlv(payload):
    """
    Parses a Target Index TLV payload.

    Args:
        payload (bytes): The payload of the Target Index TLV.

    Returns:
        dict: Parsed target index data.
    """
    target_ids = list(struct.unpack(f'<{len(payload)}B', payload))
    return {'type': TARGET_INDEX, 'targetIDs': target_ids}

def parse_presence_indication_tlv(payload):
    """
    Parses a Presence Indication TLV payload.

    Args:
         payload (bytes): The payload of the Presence Indication TLV.

    Returns:
        dict: Parsed presence indication data.
    """
    if len(payload) != 4:
          log.error("Error: Presence indication payload must be 4 bytes")
          return {'type': PRESENCE_INDICATION, 'error': 'Invalid payload length'}
    presence = struct.unpack('<I', payload)
    return {'type': PRESENCE_INDICATION, 'presence': presence}

def parse_target_height_tlv(payload):
    """
    Parses a Target Height TLV payload.

    Args:
        payload (bytes): The payload of the Target Height TLV.

    Returns:
        dict: Parsed target height data.
    """
    target_heights = []
    target_height_size = 9 # Size of each target height struct (1+4+4 bytes)
    num_targets = len(payload) // target_height_size
    for i in range(num_targets):
         start = i* target_height_size
         end = start + target_height_size

         if end > len(payload):
           log.error(f"Error: Insufficient data for target height {i} in target height tlv")
           continue

         target_height_data = struct.unpack('<Bff',payload[start:end])
         target_height = {
              'targetID': target_height_data,
              'maxZ': target_height_data[1],
              'minZ': target_height_data[2]
         }
         target_heights.append(target_height)
    return {'type': TARGET_HEIGHT, 'targetHeights': target_heights}

#send cfg over uart
def sendCfg():
    fname ="AOP_6m_default.cfg"
    with open(fname, "r") as cfg_file:
        cfg = cfg_file.readlines()
    cliCom = serial.Serial("COM6", 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
    print("Open CLI COM port done")
    # Remove empty lines from the cfg
    cfg = [line for line in cfg if line != '\n']
    # Ensure \n at end of each line
    cfg = [line + '\n' if not line.endswith('\n') else line for line in cfg]
    # Remove commented lines
    cfg = [line for line in cfg if line[0] != '%']

    for line in cfg:
        time.sleep(.03) # Line delay

        if(cliCom.baudrate == 1250000):
            for char in [*line]:
                time.sleep(.001) # Character delay. Required for demos which are 1250000 baud by default else characters are skipped
                cliCom.write(char.encode())
        else:
            cliCom.write(line.encode())
            
        ack = cliCom.readline()
        print(ack, flush=True)
        ack = cliCom.readline()
        print(ack, flush=True)
        if (True):
            ack = cliCom.readline()
            print(ack, flush=True)
            ack = cliCom.readline()
            print(ack, flush=True)

        splitLine = line.split()
        if(splitLine[0] == "baudRate"): # The baudrate CLI line changes the CLI baud rate on the next cfg line to enable greater data streaming off the xWRL device.
            try:
                cliCom.baudrate = int(splitLine[1])
            except:
                log.error("Error - Invalid baud rate")
                exit(1)
    # Give a short amount of time for the buffer to clear
    time.sleep(0.03)
    cliCom.reset_input_buffer()
    # NOTE - Do NOT close the CLI port because 6432 will use it after configuration

def dump_com_port_to_hex_file(port, baudrate, filename, duration=None):
    """
    Dumps data received from a COM port to a file as hexadecimal code.

    Args:
        port (str): The COM port to read from (e.g., "COM3", "/dev/ttyUSB0").
        baudrate (int): The baud rate of the serial connection.
        filename (str): The name of the file to write the hex data to.
        duration (float, optional): The duration in seconds to capture data. 
                                  If None, it will capture indefinitely until manually stopped.
        timeout (float, optional): Read timeout in seconds, passed to the serial object.
    """

    try:
        ser = serial.Serial(port, baudrate, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return

    print(f"Opened {port} at {baudrate} baud. Dumping data to {filename}...")

    try:
        with open(filename, "w") as f:
            start_time = time.time()
            while True:
                if duration and (time.time() - start_time) >= duration:
                    break

                data = ser.read(ser.in_waiting)  # Read available data or at least 1 byte
                if data:
                    hex_data = ' '.join(f'{byte:02x}' for byte in data)  # Format as hex with spaces
                    f.write(hex_data)
                    print(hex_data)  # Optionally print to console as well

    except KeyboardInterrupt:
        print("\nCapture stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == '__main__':
    sendCfg()
    print("Send config done!")
    
    com_port = "COM5"  # Replace with your COM port
    baud_rate = 921600  # Replace with your baud rate
    output_file = "com_data.hex"
    capture_duration = 10  # Capture for 10 seconds (or None for indefinite capture)
    dump_com_port_to_hex_file(com_port, baud_rate, output_file, capture_duration)