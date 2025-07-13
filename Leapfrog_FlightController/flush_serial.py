# This function will reset the serial ports I think...

#!/usr/bin/python3
import serial
import RPi.GPIO as GPIO
import time

# Use ls /dev/ttyA* and make sure to update the main serial port to match
# 1: , 2: RFD modem, 3: ??
ports = [("/dev/ttyAMA0",115200),       # Main serial link
         ("/dev/ttyAMA3",9600),         # RFD Modem
         ("/dev/ttyAMA4",57600)         # From vehicle.launch.py :launch_ros.actions.Node
        ]

for port,baud in ports:
    print("Cleaning: ", port, " at ", baud)
    try:
        ser = serial.Serial(port, baud)
        ser.flushInput()
        ser.flushOutput()
        ser.close()
    except Exception as e:
        print(f"Skipping {port}: {e}")


# Actuators? TODO Remove! 
relay_pins = [25, 11, 18, 23, 24, 17, 27, 22, 16, 19]
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.output(25, GPIO.LOW)
GPIO.output(11, GPIO.HIGH)

# Do not do cleanup to maintain the gpio state