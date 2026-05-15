import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 6668

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(3.0)

print(f"Listening on UDP port {UDP_PORT}...")
try:
    while True:
        data, addr = sock.recvfrom(4096)
        print(f"Received packet from {addr}: size={len(data)}, first bytes={data[:4].hex()}")
except socket.timeout:
    print("Timeout! No data received.")
