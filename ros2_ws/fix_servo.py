from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler('/dev/ttyUSB0')
p.openPort()
p.setBaudRate(1000000)
ph = PacketHandler(2.0)

# Operating Mode 변경은 토크 OFF 상태에서만 가능
ph.write1ByteTxRx(p, 0, 64, 0)  # Torque OFF
print('Torque OFF')

# Operating Mode 3 = Position Control
ph.write1ByteTxRx(p, 0, 11, 3)
mode, r, e = ph.read1ByteTxRx(p, 0, 11)
print('Operating Mode set to:', mode)

# Position Limit 정상화
ph.write4ByteTxRx(p, 0, 48, 0)     # Min Position Limit = 0
ph.write4ByteTxRx(p, 0, 52, 4095)  # Max Position Limit = 4095
min_pos, r, e = ph.read4ByteTxRx(p, 0, 48)
max_pos, r, e = ph.read4ByteTxRx(p, 0, 52)
print('Min Pos:', min_pos, '/ Max Pos:', max_pos)

# Torque 다시 ON
ph.write1ByteTxRx(p, 0, 64, 1)
print('Torque ON')

p.closePort()
print('Done')
