"""
LED Flinger, an interactive RGB LED game - Created by Footleg

This code is for the player 2 unit at the far end of the LED strip.
It is not connected to the LED strip (apart from to obtain power).
It communicates the TOF sensor data to the main RP2040 unit over UART.

REQUIRED HARDWARE:
* Raspberry Pi Pico
* TOF vl53l1 sensor on the I2C bus.
* RS485 serial board (optional, but improves reliability of UART serial over longer cables)

Copyright (c) 2023 Paul 'Footleg' Fretwell  https://github.com/Footleg/circuit_python

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This code is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this software.  If not, see <http://www.gnu.org/licenses/>.

"""
import time
import random
import board
# from digitalio import DigitalInOut, Direction, Pull
# import microcontroller
# import neopixel
import busio
import adafruit_vl53l1x

# Set minimum time (ms) between message transmissions
speed = 100

# Initialize I2C bus and sensor.
i2c = busio.I2C(scl=board.GP5, sda=board.GP4)
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# OPTIONAL: can set non-default values
vl53.distance_mode = 1
vl53.timing_budget = 100

# Configure UART
uart = busio.UART(board.GP0, board.GP1, baudrate=9600)

#Globals for touchless sensing
state = 0 # 0=Waiting for start condition, 1=setting length, 2=charging, 3=launch
length = 0
lastLength = 0
lastDist = 0
chargeStart = time.monotonic_ns()
lastLenMessage = time.monotonic_ns()

vl53.start_ranging()

while True:
    if vl53.data_ready:
        distance = vl53.distance
        vl53.clear_interrupt()
        
        if distance != None:
            if state == 0:
                # Waiting for distance < 10
                #print("Waiting: Distance={} cm".format(distance))
                if distance < 10:
                    state = 1
                    lastDist = distance
                    length = 0
            elif state == 1:
                #print("Loading: Length= {}, Distance={} cm".format(length,distance))
                # Setting length, based on any distance > 15
                if distance > lastDist and distance > 15:
                    length = distance - 15
                elif length > 0 and distance < lastDist - 1:
                    state = 2
                    chargeStart = time.monotonic_ns()
                lastDist = distance
            elif state == 2:
                # Charge with continuous push from Length down to 10cm
                if distance > lastDist - 1:
                    #Reset charging
                    chargeStart = time.monotonic_ns()
                    lastDist = distance
                elif distance > length + 15:
                    #Revert to setting length again
                    state = 1
                    lastDist = distance
                elif distance < 10:
                    #Launch
                    charge = int( (time.monotonic_ns() - chargeStart)/10000000 )
                    darterLength = int(length)
                    if charge > 255:
                        charge = 255
                    if darterLength > 255:
                        darterLength = 255
                    state = 3
                    print(f"Launched: Length={darterLength} cm, Charge={charge}" )
                    uart.write(bytes([50,darterLength,charge,0]))
                    lastLenMessage = time.monotonic_ns()
            elif state == 3:
                if distance > 9:
                    state = 0
    
    # Send length to indicate loading darter
    if state == 1 and length > 0:
        #Setting darter length
        darterLength = int(length)
        if darterLength > 255:
            darterLength = 255
        # Only send new length if it has changed since last sent
        if (lastLength != darterLength) and (int( (time.monotonic_ns() - lastLenMessage) / 1000000) > speed):
            #print(f"Sending Length={darterLength}")
            uart.write(bytes([49,darterLength,0]))
            lastLength = darterLength
            lastLenMessage = time.monotonic_ns()
    
    time.sleep(0.001)
