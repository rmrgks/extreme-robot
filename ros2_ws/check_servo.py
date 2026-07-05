from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler('/dev/ttyUSB0')
p.openPort()
p.setBaudRate(1000000)
ph = PacketHandler(2.0)

mode, r, e = ph.read1ByteTxRx(p, 0, 11)
print('Operating Mode:', mode)

min_pos, r, e = ph.read4ByteTxRx(p, 0, 48)
max_pos, r, e = ph.read4ByteTxRx(p, 0, 52)
print('Min Pos:', min_pos)
print('Max Pos:', max_pos)

cur_pos, r, e = ph.read4ByteTxRx(p, 0, 132)
print('Current tick:', cur_pos)

p.closePort()
