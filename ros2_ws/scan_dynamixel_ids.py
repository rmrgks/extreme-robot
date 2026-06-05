from dynamixel_sdk import PortHandler, PacketHandler

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

port = PortHandler(DEVICENAME)
packet = PacketHandler(PROTOCOL_VERSION)

port.openPort()
port.setBaudRate(BAUDRATE)

found = []

for dxl_id in range(253):
    model, result, error = packet.ping(port, dxl_id)
    if result == 0:
        print(f"Found ID: {dxl_id}, Model: {model}")
        found.append(dxl_id)

print("Found IDs:", found)
port.closePort()
