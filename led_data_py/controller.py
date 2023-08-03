import threading as thr
import time

done = False

def count():
    n = 0
    while not done:
        print(n)
        time.sleep(0.25)
        n += 1

def controller(process, processName: str):
    global done
    while True:
        i = input()

        match i:
            case "s":
                done = False
                t = thr.Thread(target=process, name=processName)
                t.start()
                print("Started {} process".format(t.getName()))

            case "q":
                done = True
                print("Quit {} process".format(t.getName()))

            case "e":
                print("Exiting")
                break

            case _:
                continue
                

def start():
    controller(count, "count")