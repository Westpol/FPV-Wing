import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on UDP {UDP_IP}:{UDP_PORT}")

while True:
    packet, addr = sock.recvfrom(1024)  # buffer size 1024 bytes
    print(f"From {addr}: {packet.hex(' ')}")
    if packet[0] == 0xC8:
        if packet[2] == 0x08:      # Battery Sensor
            payload = packet[3:11]
            voltage, current = struct.unpack(">hh", payload[0:4])
            capacity_used = (payload[4] << 16) | (payload[5] << 8) | payload[6]
            remaining = payload[7]