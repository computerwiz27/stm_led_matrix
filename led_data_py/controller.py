import serial_manager as ser

import threading as thr
import time

def controller(process, processName: str):
    while True:
        i = input()

        match i:
            case "s":
                ser.readDone = False
                t = thr.Thread(target=process, name=processName)
                t.start()
                print("Started {} process".format(t.getName()))

            case "q":
                ser.readDone = True
                print("Quit {} process".format(t.getName()))
                ser.close()

            case "e":
                print("Exiting")
                break

            case _:
                continue
                

def start():
    controller(ser.read, "read serial")