import smbus
import time

# Arduino is 0x29 on Bus 1 (pins 3,5)
bus = smbus.SMBus(1)
addr = 0x29

#on = list("ON\n".encode())
#off = list("OFF\n".encode())

time.sleep(1)

try:
    while(1):
        bus.write_byte(addr, ord('N'))
        #bus.write_i2c_block_data(addr, 0x0, on)
        time.sleep(0.01)
        bus.write_byte(addr, ord('F'))
        #bus.write_i2c_block_data(addr, 0x0, off)
        time.sleep(0.01)
finally:
    print("Quitting...")

print("Done.")

