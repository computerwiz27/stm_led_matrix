import serial as ser
import subprocess as sub

import serial_interface as inter

con = ser.Serial()

def connect(devName:str):
    print("Connecting...")

    connected = False
    while not connected:
        devPath = sub.check_output(["./usb_dev_path.sh", devName]).decode().rstrip()
        
        if len(devPath) > 0:
            con.port = devPath
            con.baudrate = 115200
            con.open
            connected = con.is_open
    
    print("Connected to " + devPath + " - " + devName)

    

readDone = True

def read():
    while not readDone:
        line = ser.readline()
        print(line)

def close():
    ser.close()

