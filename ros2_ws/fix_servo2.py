from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler('/dev/ttyUSB0')
p.openPort()
p.setBaudRate(1000000)
ph = PacketHandler(2.0)

ph.write1ByteTxRx(p, 0, 64, 0)  # Torque OFF (EEPROM 쓰기 전 필수)

# XL430 주소 48 = Max Position Limit, 주소 52 = Min Position Limit
ph.write4ByteTxRx(p, 0, 48, 4095)  # Max = 4095
ph.write4ByteTxRx(p, 0, 52, 0)     # Min = 0

max_pos, r, e = ph.read4ByteTxRx(p, 0, 48)
min_pos, r, e = ph.read4ByteTxRx(p, 0, 52)
print(f'Max Position Limit (addr 48): {max_pos}')
print(f'Min Position Limit (addr 52): {min_pos}')

ph.write1ByteTxRx(p, 0, 64, 1)  # Torque ON

import time; time.sleep(0.5)

result, error = ph.write4ByteTxRx(p, 0, 116, 2048)
print(f'Write goal 2048 -> result={result}, error={error}')

time.sleep(2.0)
cur, r, e = ph.read4ByteTxRx(p, 0, 132)
print(f'Current tick: {cur}')

p.closePort()
