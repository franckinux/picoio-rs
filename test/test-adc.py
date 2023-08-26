import time

import crcmod
import serial

# 01234567890123456
# ^PWM 18,060,4444$

crc16 = crcmod.predefined.mkPredefinedCrcFun("modbus")
# this is a reference test
# print(hex(crc16(b"123456789")))

ser = serial.Serial("/dev/ttyACM0", timeout=0.3)

while True:
    try:
        ser.reset_input_buffer()

        # send command
        pin = 26
        cmd = f"^ADC {pin:02d}"
        crc = crc16(cmd[1:7].encode())
        cmd += f",{crc:04X}$"
        # print(cmd)
        ser.write(cmd.encode())

        # get response
        resp = ser.read(size=16).decode()

        assert resp[0] == '^' and resp[-1] == '$'
        if resp[1:].startswith("DATA "):
            crc = crc16(resp[1:10].encode())
            if resp[11:15] == f"{crc:04X}":
                print(resp[1:-6])
            else:
                print(resp)
                print("bad crc")
        if resp[1:].startswith("NOK "):
            crc = crc16(resp[1:9].encode())
            if resp[10:14] == f"{crc:04X}":
                print(resp[1:-6])
            else:
                print("bad crc")
        elif resp[1:].startswith("OK "):
            print("OK")

        # time.sleep(0.1)
    except KeyboardInterrupt:
        break

ser.close()
print()
