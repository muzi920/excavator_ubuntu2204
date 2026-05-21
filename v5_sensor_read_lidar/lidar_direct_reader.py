import socket
import struct
import math
import sys
import time

# Constants matching the C++ SDK
UDP_PORT = 6668
IMU_UDP_PORT = 6543 # sometimes IMU data uses a different port, but usually mixed or broadcasted
# Try to bind to the listen port 6668 as specified in the yaml
LISTEN_PORT = 6668
LIDAR_IP = "192.168.158.98"
LIDAR_PORT = 6543

# Protocol flags
LIDARIMUDATA = 0x00
LIDARPOINTCLOUD = 0x01

# The structure of BlueSeaLidarEthernetPacket is roughly:
# uint8_t version
# uint16_t length
# uint16_t time_interval
# uint16_t dot_num
# uint16_t udp_cnt
# uint8_t frame_cnt
# uint8_t data_type
# uint8_t time_type
# uint8_t rsvd[12] (or RuntimeInfoV1)
# uint32_t crc32
# uint64_t timestamp
# uint8_t data[] (points)
#
# Size before data: 1+2+2+2+2+1+1+1+12+4+8 = 36 bytes

def parse_point_cloud(packet_data):
    if len(packet_data) < 36:
        return

    # Unpack header
    header_fmt = '<B H H H H B B B 12s I Q'
    header = struct.unpack_from(header_fmt, packet_data, 0)
    
    version = header[0]
    length = header[1]
    dot_num = header[3]
    data_type = header[6]
    timestamp = header[10]

    if data_type != LIDARPOINTCLOUD:
        return

    # To avoid flooding the console, only print 1 in every 10 pointcloud packets
    global pc_count
    if 'pc_count' not in globals():
        pc_count = 0
    pc_count += 1
    
    if pc_count % 10 == 0:
        print(f"[PointCloud] Received packet: Version={version}, DotNum={dot_num}, Timestamp={timestamp}")
        
        # Unpack points (BlueSeaLidarSpherPoint - 8 bytes each)
    # uint32_t depth : 24;
    # uint32_t theta_hi : 8;
    # uint32_t theta_lo : 12;
    # uint32_t phi : 20;
    # uint8_t reflectivity;
    # uint8_t tag;
    
    offset = 36
    points_parsed = 0
    for i in range(dot_num):
        if offset + 8 > len(packet_data):
            break
            
        point_data = struct.unpack_from('<I I B B', packet_data, offset)
        word1 = point_data[0]
        word2 = point_data[1]
        reflectivity = point_data[2]
        tag = point_data[3]
        
        depth = word1 & 0xFFFFFF
        theta_hi = (word1 >> 24) & 0xFF
        theta_lo = word2 & 0xFFF
        phi = (word2 >> 12) & 0xFFFFF
        
        theta = (theta_hi << 12) | theta_lo
        
        # Convert to cartesian as per C++ SDK
        ang = (90000 - theta) * math.pi / 180000.0
        depth_m = depth / 1000.0
        
        r = depth_m * math.cos(ang)
        z = depth_m * math.sin(ang)
        
        phi_ang = phi * math.pi / 180000.0
        x = math.cos(phi_ang) * r
        y = math.sin(phi_ang) * r
        
        if i == 0: # Just print the first point as a sample
            print(f"  -> Sample Point 0: x={x:.3f}, y={y:.3f}, z={z:.3f}, depth={depth_m:.3f}m, ref={reflectivity}")
            
        offset += 8
        points_parsed += 1

def parse_imu_data(packet_data):
    # IMU data has a different header: 0xfa 0x88
    if len(packet_data) < 27:
        return
        
    if packet_data[0] == 0xfa and packet_data[1] == 0x88:
        # TransBuf header is 8 bytes: code(2), len(2), idx(2), pad(2)
        # Then IIM42652_FIFO_PACKET_16_ST (25 bytes)
        imu_fmt = '<B h h h h h h b H Q'
        try:
            imu_data = struct.unpack_from(imu_fmt, packet_data, 8 + 1)
            
            accel_x = imu_data[1] * 4.0 / 0x10000
            accel_y = imu_data[2] * 4.0 / 0x10000
            accel_z = imu_data[3] * 4.0 / 0x10000
            
            gyro_x = imu_data[4] * 4000.0 / 0x10000 * math.pi / 180
            gyro_y = imu_data[5] * 4000.0 / 0x10000 * math.pi / 180
            gyro_z = imu_data[6] * 4000.0 / 0x10000 * math.pi / 180
            
            timestamp = imu_data[9]
            
            # Reduce IMU printing frequency
            global imu_count
            if 'imu_count' not in globals():
                imu_count = 0
            imu_count += 1
            if imu_count % 100 == 0:
                print(f"[IMU] Accel: ({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}), Gyro: ({gyro_x:.3f}, {gyro_y:.3f}, {gyro_z:.3f}), TS: {timestamp}")
        except struct.error:
            pass

def crc32_stm32(data):
    """
    Calculate STM32 hardware CRC-32 (Ethernet standard CRC32 polynomial 0x04C11DB7)
    This is equivalent to the C++ stm32crc function
    """
    crc = 0xFFFFFFFF
    for i in range(0, len(data), 4):
        # Read 32-bit word, big endian (STM32 hardware CRC processes words)
        word = struct.unpack_from('>I', data, i)[0] if i + 4 <= len(data) else 0
        crc ^= word
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc

def pack_net_cmd(cmd_type, payload):
    """
    Create a UDP command packet identical to PackNetCmd in pacecatlidarsdk.cpp
    """
    import random
    sn = random.randint(0, 65535)
    
    # Header format: uint16_t sign, uint16_t cmd, uint16_t sn, uint16_t len
    # PACK_PREAMLE is usually 0x484C (or 0x4C48 depending on endianness). 
    # Let's use little endian '<' for struct packing
    sign = 0x484C
    length = len(payload)
    
    # The payload length must be padded to a multiple of 4
    len4 = ((length + 3) >> 2) * 4
    padded_payload = payload.encode('ascii') + b'\x00' * (len4 - length)
    
    # Pack header + padded payload
    header = struct.pack('<H H H H', sign, cmd_type, sn, length)
    packet_without_crc = header + padded_payload
    
    # Calculate CRC32 using STM32 logic
    # In C++: uint32_t chk = stm32crc((uint32_t*)buf, len4/4 + 2);
    crc = crc32_stm32(packet_without_crc)
    
    # Append CRC
    packet = packet_without_crc + struct.pack('<I', crc)
    return packet

def send_start_command(sock):
    """
    Send the "LSTARH" command to start the Lidar
    """
    cmd = "LSTARH"
    cmd_type = 0x0043 # C_PACK (Command Packet)
    packet = pack_net_cmd(cmd_type, cmd)
    
    # 模拟 C++ 驱动，多次尝试发送
    for i in range(5):
        print(f"Sending Start Command ({cmd}) to {LIDAR_IP}:{LIDAR_PORT}... (Try {i+1}/5)")
        sock.sendto(packet, (LIDAR_IP, LIDAR_PORT))
        time.sleep(0.1)

def main():
    print(f"Starting UDP Lidar listener on port {LISTEN_PORT}...")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Try binding
    try:
        sock.bind(('0.0.0.0', LISTEN_PORT))
        print("Socket bound successfully.")
    except Exception as e:
        print(f"Failed to bind socket: {e}")
        sys.exit(1)

    # Send the START command to the Lidar
    send_start_command(sock)

    print("Waiting for data (Press Ctrl+C to stop)...")
    
    try:
        while True:
            data, addr = sock.recvfrom(65536)
            
            if not data:
                continue
                
            # Determine packet type based on first few bytes
            if (data[0] == 0x00 or data[0] == 0x01) and len(data) >= 36:
                # Likely point cloud
                parse_point_cloud(data)
            elif data[0] == 0xfa and data[1] == 0x88:
                # Likely IMU
                parse_imu_data(data)
            elif data[0] == 0x4c and data[1] == 0x69 and data[2] == 0x44 and data[3] == 0x41:
                print(f"[Heartbeat] Received heartbeat from {addr}")
            # else:
            #     print(f"Unknown packet type from {addr}: {data[:4].hex()}")
                
    except KeyboardInterrupt:
        print("\nStopping listener.")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
