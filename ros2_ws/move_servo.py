from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler('/dev/ttyUSB0')
p.openPort()
p.setBaudRate(1000000)
ph = PacketHandler(2.0)

# Hardware Error Status 확인
hw_err, r, e = ph.read1ByteTxRx(p, 0, 70)
print('Hardware Error Status:', hw_err)

# 서보 재부팅 (에러 플래그 초기화)
print('Rebooting servo...')
ph.reboot(p, 0)
import time; time.sleep(1.0)

# 재부팅 후 모드 재설정 (EEPROM에 저장됐으면 유지되나 확인)
mode, r, e = ph.read1ByteTxRx(p, 0, 11)
print('Operating Mode after reboot:', mode)

# 토크 ON
ph.write1ByteTxRx(p, 0, 64, 1)
print('Torque ON')

# 위치 2048로 직접 이동
result, error = ph.write4ByteTxRx(p, 0, 116, 2048)
print(f'Write goal 2048 -> result={result}, error={error}')
time.sleep(2.0)

cur, r, e = ph.read4ByteTxRx(p, 0, 132)
print('Current tick after command:', cur)

p.closePort()
