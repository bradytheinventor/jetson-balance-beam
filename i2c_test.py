import smbus
import time

#TODO: atmega328p supports fast mode @ 8MHz. how to change SCL?

# Arduino is 0x29 on Bus 1 (pins 3,5)
# i2cdetect -y -r 1
bus = smbus.SMBus(1)
addr = 0x29

mid_us = 1590
min_us = mid_us-200
max_us = mid_us+200

def write(msg: str) -> None:
    for c in list(msg):
        bus.write_byte(addr, ord(c))

def write_uint16(i):
    bytes = [(i&0xFF00) >> 8, i&0x00FF]
    bus.write_i2c_block_data(addr, bytes[0], bytes[1:])

time.sleep(0.5)

try:
    while(1):
        write_uint16(min_us)
        time.sleep(1)
        write_uint16(max_us)
        time.sleep(1)

except KeyboardInterrupt:
    print("Quitting...")
    write_uint16(0)
    time.sleep(0.01)

finally:
    bus.close()

print("Done.")

