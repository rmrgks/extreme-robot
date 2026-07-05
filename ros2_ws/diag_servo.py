from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler('/dev/ttyUSB0')
p.openPort()
p.setBaudRate(1000000)
ph = PacketHandler(2.0)

model, r, e = ph.read2ByteTxRx(p, 0, 0)
print('Model Number:', model)

mode, r, e = ph.read1ByteTxRx(p, 0, 11)
print('Operating Mode:', mode)

drive, r, e = ph.read1ByteTxRx(p, 0, 10)
print('Drive Mode:', drive)

min_pos, r, e = ph.read4ByteTxRx(p, 0, 48)
max_pos, r, e = ph.read4ByteTxRx(p, 0, 52)
print('Min Position Limit:', min_pos)
print('Max Position Limit:', max_pos)

cur, r, e = ph.read4ByteTxRx(p, 0, 132)
print('Current tick:', cur)

torque, r, e = ph.read1ByteTxRx(p, 0, 64)
print('Torque Enable:', torque)

# 현재 위치에서 아주 조금만 이동 (184 -> 200)
ph.write1ByteTxRx(p, 0, 64, 1)
result, error = ph.write4ByteTxRx(p, 0, 116, 200)
print(f'Write goal 200 -> result={result}, error={error}')

import time; time.sleep(1.0)
cur2, r, e = ph.read4ByteTxRx(p, 0, 132)
print('Current tick after goal 200:', cur2)

p.closePort()
