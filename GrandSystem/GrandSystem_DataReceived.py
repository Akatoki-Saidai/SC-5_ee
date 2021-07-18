import serial

ser = serial.Serial('/dev/tty.usbserial-0001',115200,timeout=None)

while True:
    line = ser.readline()
    print(line)