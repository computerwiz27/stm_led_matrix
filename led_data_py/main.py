#!/usr/bin/python

import led_matrix as led
import controller as ctrl
import serial_manager as ser

#ctrl.start()

device = "STMicroelectronics STM32 STLink"

ser.connect(device)
