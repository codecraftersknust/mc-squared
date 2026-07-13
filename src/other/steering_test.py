import serial
import time

arduino = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)  # Wait for Arduino reset

while True:
    angle = input("Enter angle (45-135): ")

    arduino.write((angle + '\n').encode())
