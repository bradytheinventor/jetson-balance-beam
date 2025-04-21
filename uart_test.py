import time
import serial

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=1200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

time.sleep(1)

try:
    while(1):
        serial_port.write("ON\n".encode())
        #time.sleep(0.25)
        #while(serial_port.inWaiting() > 0):
        #    print(serial_port.read())
        time.sleep(1)

        serial_port.write("OFF\n".encode())
        #time.sleep(0.25)
        #while(serial_port.inWaiting() > 0):
        #    print(serial_port.read())
        time.sleep(1)

except KeyboardInterrupt:
    print("Quitting...")

finally:
    serial_port.close()
    print("Serial closed")

print("Done.")

