import serial as ser
import subprocess as sub
import threading as thr
import time

import serial_interface as inter

con = ser.Serial()

def waitCon():
    time.sleep(15)
    if not con.is_open:
        print("Make sure the device is plugged in")

def connect(devName:str):
    print("Connecting...")
    thr.Thread(target=waitCon, daemon=True).start()

    connected = False
    while not connected:
        devPath = sub.check_output(["./usb_dev_path.sh", devName]).decode().rstrip()
        if len(devPath) > 0:
            devPath = "/dev/" + devPath
            con.port = devPath
            con.baudrate = 115200
            con.open()
            connected = con.is_open
    res = sendCommand(inter.CONFIRM)
    if res[0] == inter.CONFIRM:
        print("Connected to " + devPath + " - " + devName)
    else:
        print("Failed connecting to " + devPath + " - " + devName)
    
def disconnect(devName:str = ""):
    port = con.port
    con.close()
    print("Disconnected from " + port, end=" ")
    if len(devName) > 0 : print("-", end=" ")
    print(devName)

def sendCommand(opc, led=0, r=0, g=0, b=0):
    led = led.to_bytes(2, "big")
    cmd = bytearray([opc, r, g, b, led[0], led[1]])
    
    con.write(cmd)
    return con.read(inter.COMMAND_SIZE)

def sendSpecs():
    res = sendCommand(inter.SEND)
    matrix_shape = [0, 0, 0]
    matrix_shape[0] = res[0]
    matrix_shape[1] = res[1]
    matrix_shape[2] = res[2]
    towers_shape = [0, 0]
    towers_shape[0] = res[3]
    towers_shape[1] = res[4]
    time.sleep(1)
