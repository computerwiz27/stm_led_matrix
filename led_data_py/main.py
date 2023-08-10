#!/usr/bin/python

import led_matrix as led
import controller as ctrl
import serial_manager as ser

import serial_interface

#ctrl.start()

device = "STMicroelectronics STM32 STLink"

ser.connect(device)
ser.sendSpecs()

ser.disconnect(device)

