import crcmod
import serial

# 01234567890123456
# ^PWM 18,060,4444$

crc16 = crcmod.predefined.mkPredefinedCrcFun("modbus")
# this is a reference test
# print(hex(crc16(b"123456789")))

ser = serial.Serial("/dev/ttyACM0", timeout=0.3)

while True:
    print("1: ADC")
    print("2: GDI")
    print("3: GDIS")
    print("4: PWM")
    print("5: SDO")
    print("6: SDOS")
    try:
        action = int(input("action ? "))
    except KeyboardInterrupt:
        break
    except ValueError:
        continue
    if action not in (1, 2, 3, 4, 5, 6):
        continue

    if action == 1:
        try:
            pin = int(input("pin [26-29] ? "))
        except KeyboardInterrupt:
            break
        except ValueError:
            continue

        # send command
        cmd = f"^ADC {pin:02d}"
        crc = crc16(cmd[1:7].encode())

    elif action == 2:
        try:
            pin = int(input("pin [12-17] ? "))
        except KeyboardInterrupt:
            break
        except ValueError:
            continue

        # send command
        cmd = f"^GDI {pin:02d}"
        crc = crc16(cmd[1:7].encode())

    elif action == 3:
        # send command
        cmd = f"^GDIS _"
        crc = crc16(cmd[1:7].encode())

    elif action == 4:
        try:
            pin = int(input("pin [18-22] ? "))
            pulse = int(input("pulse duration [1000-1500] ? "))
        except KeyboardInterrupt:
            break
        except ValueError:
            continue

        # send command
        cmd = f"^PWM {pin:02d},{pulse:04d}"
        crc = crc16(cmd[1:12].encode())

    elif action == 5:
        try:
            pin = int(input("pin [0-11] ? "))
            state = int(input("state [0-1] ? "))
        except KeyboardInterrupt:
            break
        except ValueError:
            continue

        # send command
        cmd = f"^SDO {pin:02d},{state:1d}"
        crc = crc16(cmd[1:9].encode())

    elif action == 6:
        try:
            state = int(input("logical states in hex [0-0xFFF] ? "), 16)
            mask = int(input("mask in hex [0-0xFFF] ? "), 16)
        except KeyboardInterrupt:
            break
        except ValueError:
            continue

        # send command
        cmd = f"^SDOS {state:04X},{mask:04X}"
        crc = crc16(cmd[1:15].encode())
    else:
        continue

    ser.reset_input_buffer()
    cmd += f",{crc:04X}$"
    # print(cmd)
    ser.write(cmd.encode())

    # get response
    resp = ser.read(size=16).decode()
    # print(resp)

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

ser.close()
print()
