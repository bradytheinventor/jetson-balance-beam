import smbus
import time

#TODO: atmega328p supports fast mode @ 8MHz. how to change SCL?

# Arduino is 0x29 on Bus 1 (pins 3,5)
# i2cdetect -y -r 1
bus = smbus.SMBus(1)
addr = 0x29

def write(msg: str) -> None:
    for c in list(msg):
        bus.write_byte(addr, ord(c))

try:
    while(1):
        write("ON\n")
        time.sleep(0.05)
        write("OFF\n")
        time.sleep(0.05)
finally:
    bus.close()
    print("Quitting...")

print("Done.")

