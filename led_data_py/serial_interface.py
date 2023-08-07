import serial as ser

ser = ser.Serial(port='/dev/ttyACM0', baudrate=115200, timeout = 0.2)

readDone = True

def read():
    while not readDone:
        line = ser.readline()
        print(line)

def close():
    ser.close()

