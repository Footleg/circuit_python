"""
LED Flinger, an interactive RGB LED game - Created by Footleg

REQUIRED HARDWARE:
* Raspberry Pi Pico
* RGB NeoPixel LEDs connected to pin GP2.
* TOF vl53l1 sensor on the I2C bus.
* Optional second device connected over UART for 2 player games
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
from digitalio import DigitalInOut, Direction
import microcontroller
import neopixel
import busio
import adafruit_vl53l1x
from tof_decoder import distanceHandler

debugcount = 0

# Brightness of Neopixels (0.1 - 1.0) Set to lower end while powered off a USB port.
pixLevel = 0.8

# Winning score
winning_score = 5

# Update this to match the number of NeoPixel LEDs connected to your board.
num_pixels = 839

# Anti-aliasing: ratio of position coordinate space for animations to actual pixels
# e.g. Set to 4 means the animation needs to move 4 spaces to move to the next neopixel
# For longer strip lengths, lower this number to speed up the program as it slows with more pixels to manage.
# A value of 8 is good for a 240 - 300 led strip. I change it to 2 when I have 900 LEDs using 3 strips.
aa = 2

# Set speed of animations (1 - 1000) Normally set to 1000 for top speed, but set lower to debug
speed = 1000

# Tiny2040 board built in RGB LED and button
bootBtn = DigitalInOut(microcontroller.pin.GPIO23)
bootBtn.direction = Direction.INPUT

# Initialize I2C bus and sensor.
i2c = busio.I2C(scl=board.GP5, sda=board.GP4)
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# OPTIONAL: can set non-default values
vl53.distance_mode = 1
vl53.timing_budget = 100

#Initialise neopixels
pixels = neopixel.NeoPixel(board.GP2, num_pixels, auto_write=False)

#Configure UART
uart = busio.UART(board.GP0, board.GP1, baudrate=9600, timeout=0)
uartInterval = 10000000
lastUartTime = 0

#Global to track display of length for player 2
p2Length = 0

def setPixelBrightness(brightness):
    global pixLevel

    if brightness > 1:
        pixLevel = 0.2
    elif brightness < 0.2:
        pixLevel = 0.2
    else:
        pixLevel = brightness

    print(f"Set pixel brightness to {pixLevel}")
    pixels.brightness = pixLevel

setPixelBrightness(0.2)

class darter:
    def __init__(self, length=6, pos=0, speed=4, colour=(255,255,0), life=40, mode=0 ):
        self._length = length
        self.pos = pos
        self.speed = speed
        self.colour = colour
        self.halflen = int((length + 1) / 2)
        if self.halflen < 1:
            self.halflen = 1
        self.life = life
        self.mode = mode

        # Initialise class vars which are not set by arguments
        self.steps = 0

        # Mode specific behaviours
        if self.mode == 1:
            # Mode 1 slows down over time, and colour is based on speed
            self.colourFromSpeed()
            self.slowRate = 100 - self._length
            if self.slowRate < 10: self.slowRate = 10
        elif self.mode == 2:
            # Mode 2 is hot particles
            self.colourFromSpeed()
            self.life = 1000 #Life will be set to zero when speed is zero, so give a large value here
            self.slowRate = 10
        else:
            # No mode (or mode 0) holds colour and does not slow
            self.slowRate = 0
            #print(f"Created mode 0 at pos {pos} colour {colour}")

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self,newlength):
        self._length = newlength

    def update(self, pix):
        #Update position
        newpos = self.pos + self.speed
        # Reverse direction if off either end of strip
        if newpos > num_pixels * aa:
            self.speed = -self.speed
            newpos = num_pixels * aa
        elif newpos < 0:
            self.speed = -self.speed
            newpos = 0
        self.pos = newpos

        self.steps += 1

        if self.slowRate > 0:
            # Slow down after number of steps
            if self.steps > self.slowRate:
                self.steps = 0
                if self.speed >= 1:
                    self.speed += - 1
                elif self.speed <= -1:
                    self.speed += 1
                elif self.mode > 0:
                    # Kill stationary darters if not mode 0
                    self.life = 0
                if self.mode > 0:
                    self.colourFromSpeed()

        #Set pixels for new position
        for i in range(0, self.halflen ):
            brightness = (2 * (self.halflen - i) ) / (2 * self.halflen)
            bri2 = brightness * brightness

            r = self.colour[0] * bri2
            g = self.colour[1] * bri2
            b = self.colour[2] * bri2

            midIdx = int(self.pos / aa)

            newIdxFront = midIdx + i
            self.blendColours(newIdxFront,(r,g,b) )

            newIdxRear = midIdx - i
            self.blendColours(newIdxRear,(r,g,b) )

        #Update age
        self.life += -1
        #print(f"Position { self.pos } life={self.life}")

    def safeSetPixel(self,idx,colour):
        if 0 <= idx < num_pixels:
            pixels[idx] = colour

    def blendColours(self,idx,colour):
        if 0 <= idx < num_pixels:
            r = (pixels[idx][0] + colour[0])/2
            g = (pixels[idx][1] + colour[1])/2
            b = (pixels[idx][2] + colour[2])/2
            if r > 255: r = 255
            if g > 255: g = 255
            if b > 255: b = 255
            #if idx < 1:
            #    print(f"Old: {pixels[idx]}; New: {colour}; Blend: {r},{g},{b}")
            pixels[idx] = (r,g,b)

    def wipeDarterPixels(self):
        pos = int(self.pos / aa)
        i1 = pos - self.halflen + 1
        i2 = pos + self.halflen - 1
        #print(f"Darter {i}: position: {pos}; wiping range {i1} -> {i2}")
        for idx in range(i1,i2 + 1):
            self.safeSetPixel(idx,(0,0,0))

    def colourFromSpeed(self):
        #Set colour from speed
        if self.mode == 1:
            # Mode 1=rainbow
            absSpeed = abs(self.speed)
            r = 255
            g = 0
            b = 0
            if absSpeed > 30:
                b = 0 #Red
            elif absSpeed > 20:
                #Range 21 - 30 Red -> Yellow
                g = 255 - int( (absSpeed - 20) * 255/10 )
            elif absSpeed > 14:
                #Range 14 - 20 Yellow -> green
                r = int( (absSpeed - 14) * 255/7 )
                g = 255
                b = 0
            elif absSpeed > 5:
                #Range 6 - 13 green - blue
                r = 0
                g = int( (absSpeed - 6) * 255/8 )
                b = 255 - g
            else:
                #Range 1 - 5 blue -> magenta
                r = 255 - int( (absSpeed - 1) * 255/5 )
                g = 0
                b = 255
            #print(f"Abs Speed: {absSpeed} RGB ({r},{g},{b})")
        elif self.mode == 2:
            # Mode 2=Hot particles
            absSpeed = abs(self.speed)
            r = 255
            g = 255
            b = 255
            if absSpeed < 7:
                #Range 0 - 6
                #Translates to 1 - 7. Fade down green from 255 to 0 over range 0 - 7
                g = int( (absSpeed + 1) * 255/7 )
                b = 0
            elif absSpeed < 11:
                #Range 7 - 10
                #Translates to 1 - 4. Fade down blue from 255 to 0 over range 0 - 5
                b = int( (absSpeed-6) * 255/5 )

        #print(f"Abs Speed: {absSpeed} RGB ({r},{g},{b})")
        self.colour = (r,g,b)


def launchDarter(player, darterLength, darterCharge):
    global isIdle

    launchSpeed = int( (60 * darterLength + 3400) / (10 * darterCharge) )
    if launchSpeed < 1:
        launchSpeed = 1
    elif launchSpeed > 30:
        launchSpeed = 30
    life = int( (1000 - launchSpeed*20) * random.randrange(1,7) * num_pixels * aa / 9600)
    if player == 1:
        pos = 0
    else:
        pos = num_pixels * aa
        launchSpeed = -launchSpeed

    # Clear length indicator
    if player == 1:
        for i in range( int(darterLength / 3) ):
            pixels[i] = (0,0,0)
    else:
        for i in range( int(darterLength / 3) ):
            pixels[num_pixels - i - 1] = (0,0,0)

    # Create darter
    darters.append( darter(length=int(darterLength / 1.66),speed=launchSpeed,life=life, pos=pos, mode=1 ) )
    isIdle = False #Set flag to not idle to prevent random showers
    print(f"Launched player {player}: Length={darterLength} cm, Charge={darterCharge}, Speed={launchSpeed}, Life={life}" )


def showLength(player, length):
    setPixels = int(length / 3)
    for i in range(50):
        if player == 1:
            idx = i
        else:
            idx = num_pixels - i - 1
        if i < setPixels:
            pixels[idx] = (180,180,180)
        else:
            pixels[idx] = (0,0,0)


tracker = distanceHandler(launchDarter)

#Globals for tracking idle time and state
idleWait = 20000000000
isIdle = False

#Globals calculated from settings
particleSpeed = int(1 * aa)

#Game scoring
player1Score = 0
player2Score = 0

# Start in 1 player mode
gameMode = 1

# Track time since last input from each end (initialise as no activity for some time)
p1Idle = time.monotonic_ns() - idleWait
p2Idle = time.monotonic_ns() - idleWait
player1IsIdle = True
player2IsIdle = True

def resetPuck():
    # Reset puck position and speed
    puck.pos = int((num_pixels-puckLength)*aa/2)
    puck.speed = 0

    #Remove any remaining darters
    while len(darters) > 0:
        darters.pop(0)

    # Clear strip
    for i in range( num_pixels ):
        pixels[i] = (0,0,0)

def resetGame():
    global player1Score, player2Score

    p1Idle = time.monotonic_ns()
    p2Idle = time.monotonic_ns()

    #Flush messages until no more darters spawned
    print(f"Resetting game. Darters remaining: {len(darters)}")
    flushCounter = 0
    while len(darters) > 0 or flushCounter == 0:
        flushCounter += 1
        print(f"Flushing messages {flushCounter}")
        # Clear existing darters
        resetPuck()
        # Check for MQTT messages

    #Reset scores
    player1Score = 0
    player2Score = 0
    print("Reset scores")
    resetPuck()

darters = []
puckLength = int(6 + 6 * num_pixels / 300 )
puck = darter(length=puckLength,colour=(255,255,255) )
puck.slowRate = int(8 * num_pixels / 300)
resetPuck()
delay = 1 / speed

vl53.start_ranging()

#Initialise idle time counting and random shower variables
idleSince = time.monotonic_ns()
shower_speed = 1000000
num_showers = 1
last_shower_size = num_showers

while True:
    #Wipe each darter
    for i in range(len(darters)):
       darters[i].wipeDarterPixels()

    if gameMode == 2: #2 Player Puck Game
        #Wipe puck
        puck.wipeDarterPixels()

    # Get player 1 input from TOF sensor
    if vl53.data_ready:
        distance = vl53.distance
        vl53.clear_interrupt()

        if distance != None:
            tracker.processDistance(distance)

    # Get player 2 input from UART
    if time.monotonic_ns() - lastUartTime > uartInterval:
        lastUartTime = time.monotonic_ns()
        byte_read = uart.read(1)  # Read one byte over UART lines
        #print(f"Read: {type(byte_read)} = {byte_read}")
        databuffer = bytearray('')
        while byte_read and byte_read != b'\x00':
            databuffer.extend(byte_read)
            #print(f"Added: {type(byte_read)} = {byte_read}")
            #Read until zero sent
            byte_read = uart.read(1)  # Read next byte over UART lines
            #print(f"Read: {type(byte_read)} = {byte_read}")

        if len(databuffer):
            # Decode data
            i = len(databuffer)
            if 1 < i < 4:
                i += -1
                if databuffer[0] == 50:
                    length = databuffer[1]
                    charge = databuffer[2]
                    print(f"Launch: length={length}, charge={charge}")
                    launchDarter(2,length,charge)
                    p2Length = 0
                elif databuffer[0] == 49:
                    length = databuffer[1]
                    print(f"Loading: length={length}")
                    idleSince = time.monotonic_ns()
                    # Store player 2 length to be used to refresh loading indicator
                    p2Length = length
            elif databuffer[0] == 49:
                # Timeout on player 2 setting length, so reset
                showLength(2, 0)
                p2Length = 0

    # Check if length loading timeout exceeded
    if tracker.checkTimeout():
        showLength(1, tracker.length)

    #(Player 1): When building new darter, show length on start of strip
    if tracker.state == 1 and tracker.length > 0:
        print("Player 1 activity")
        p1Idle = time.monotonic_ns() # Reset player 1 idle time tracker
        showLength(1, tracker.length)

    #(Player 2): When building new darter, show length on start of strip
    if p2Length > 0:
        print("Player 2 activity")
        p2Idle = time.monotonic_ns() # Reset player 2 idle time tracker
        showLength(2, p2Length)

    # Update darters in the array. Using a while loop so we can manipulate the index as darters are added or destroyed
    i = 0
    while i < len(darters):
        d = darters[i]
        d.update(pixels)
        if d.life < 1:
            #Kill darter
            darters.pop(i)
            i += -1
            #Spawn particles if this darter is not a particle itself
            if d.mode == 1 or d.mode == 3:
                #Spawn particles
                lower = int(d.length / 5) + 4
                upper = int(d.length / 3) + 10
                # Limit max speed added to particles for direction of darter they spawned from
                if abs(d.speed) > particleSpeed:
                    if d.speed > 0:
                        maxSpd = particleSpeed
                    else:
                        maxSpd = -particleSpeed
                else:
                    maxSpd = d.speed
                spks = (random.randrange(lower,upper))
                #print(f"Died: len={d.length}; lower={lower}; upper={upper}; sparks={spks}; Total remaining: {len(darters)}")
                for count in range(spks):
                    if d.mode == 3:
                        # Mode 3 darter indicates idle trigger of hot particles (which are mode 2)
                        #print(f"Hot particle {count}")
                        spd = maxSpd + random.randrange(-12,12)
                        darters.append( darter(length=1,speed=spd,pos=int(d.pos + random.randrange(-d.halflen,d.halflen)*aa ),mode=2) )
                    else:
                        # Otherwise create particles the same colour as the darter which died
                        #print(f"Coloured particle {count}")
                        spd = maxSpd + random.randrange(-8,8)
                        darters.append( darter(length=1,speed=spd,pos=int(d.pos + random.randrange(-d.halflen,d.halflen)*aa ),mode=0,colour=d.colour) )
            #Turn off pixels from dead darter
            d.wipeDarterPixels()
            idleSince = time.monotonic_ns()
        elif gameMode == 2: #2 Player Puck Game
            #Check for collision with puck (except for particles)
            if d.mode == 1:
                coll = False
                if d.speed > 0:
                    #Player 1 darter
                    if d.pos + d.length/2 > puck.pos:
                        coll = True
                else:
                    #Player 2 darter
                    if d.pos - d.length/2 < puck.pos:
                        coll = True
                if coll:
                    #Kill darter and transfer energy into puck
                    d.life = 0
                    puck.speed = puck.speed + int((1 + d.length/20) * d.speed / 12)
                    print(f"Darter { i } collision of {d.length} darter at speed {d.speed}. Puck speed {puck.speed}, position {puck.pos}")
        elif d.mode == 1:
            # Handle interactions for darters which are mode 1 (i.e. not particles)
            if abs(d.speed) > 28 and d.length < 10:
                # If a short high speed darter, check if it has collided with any slower darters
                for chk in range( len(darters) ):
                    d2 = darters[chk]
                    if chk != i and d2.mode == 1 and abs(d.pos - d2.pos) < 5 * aa:
                        # Split darter
                        print(f"split a:{d.speed} t:{d2.speed}")
                        if d.speed > 0:
                            spdAdd = 6
                        else:
                            spdAdd = -6
                        d.speed -= (spdAdd * 2)
                        d.colourFromSpeed()
                        d3spd = d2.speed+spdAdd
                        darters.append( darter(length=d2.halflen,speed=d2.speed+spdAdd,pos=d2.pos,life=d2.life,mode=1) )
                        newHalflen = int(d2.halflen / 2)
                        if newHalflen < 1:
                            newHalflen = 1
                        d2.length = d2.halflen
                        d2.halflen = newHalflen
                        d2.speed -= spdAdd
                        d2.colourFromSpeed()
                        print(f"After a:{d.speed} t1:{d2.speed} t2:{d3spd}")
                        break;
            else:
                # Check if any other darter is in the same position moving with a similar speed
                for chk in range( len(darters) ):
                    d2 = darters[chk]
                    if chk != i and d2.mode == 1:
                        if abs(d.pos - d2.pos) < 5 * aa:
                            #print(f"Darters separation distance {abs(d.pos - d2.pos)} Speed1: {d.speed}; Speed2: {d2.speed}; Total remaining: {len(darters)}")
                            if abs(d.speed - d2.speed) < 8:
                                # Merge darters
                                len2 = d2.length
                                spd2 = d2.speed
                                life2 = d2.life
                                #print(f"Darters separation distance {abs(d.pos - d2.pos)} Speed1: {d.speed}; Speed2: {d2.speed}; Total remaining: {len(darters)}")
                                # Update d2 with combined properties of both
                                d2.speed = int((d.speed + d2.speed)/2)
                                d2.length = d.length + d2.length
                                d2.life = (d.life + d2.life)
                                #Kill darter
                                darters.pop(i)
                                i += -1
                                print(f"Merged darters; L1={d.length}, L2={len2}, S1={d.speed}, S2={spd2}, Lf1={d.life}, Lf2={life2}, New: len={d2.length}, spd={d2.speed}, lif={d2.life} Darters: {len(darters)}")
                                #Turn off pixels from dead darter
                                d.wipeDarterPixels()
                                break;

        #Increment loop index
        i += 1

    if gameMode == 2: #2 Player Puck Game
        # Update puck position
        puck.update(pixels)

        # Display scores
        # print(f"Showing scores {debugcount}")
        # debugcount += 1
        for i in range(player1Score):
            pixels[i] = (0,255,0)
        for i in range(num_pixels - player2Score, num_pixels):
            pixels[i] = (0,255,0)
            print(f"Set score pixel {i}")
            
        #Check if point scored by puck reaching either end of strip
        triggerWin = False
        if puck.pos < puckLength:
            #Player 2 scores
            player2Score += 1
            resetPuck()
            #Shower of red at pos 0
            for count in range(12):
                spd = random.randrange(1,20)
                darters.append( darter(length=1,speed=spd,pos=0,mode=0,colour=(255,0,0) ) )
            triggerWin = True
        elif puck.pos > (num_pixels - puckLength) * aa:
            #Player 2 scores
            player1Score += 1
            resetPuck()
            #Shower of red at end of strip
            for count in range(12):
                spd = random.randrange(1,20)
                darters.append( darter(length=1,speed=spd,pos=num_pixels*aa,mode=0,colour=(255,0,0) ) )
            triggerWin = True

        #Check for win
        if player1Score >= winning_score or player2Score >= winning_score:
            if triggerWin == True:
                triggerWin = False
                #Create flurry of sparkles in half of strip closest to winning player
                for i in range(4):
                    position = random.randrange(int(num_pixels * aa/2))
                    if player2Score >= winning_score:
                        position = position + int(num_pixels * aa/2)
                    spks = (random.randrange(4,6))
                    for count in range(spks):
                        spd = random.randrange(-12,12)
                        darters.append( darter(length=1,speed=spd,pos=position,mode=2) )

            # Wait until all sparkles are gone, then reset
            if len(darters) == 0:
                print("Triggering reset")
                resetGame()

    #Check for idle
    if time.monotonic_ns() - idleSince > idleWait and isIdle:
        #Spawn a random particle shower (mode 3 will trigger the hot particle colours)
        position = random.randrange(num_pixels * aa)
        darters.append( darter(length=1,speed=random.randrange(-particleSpeed,particleSpeed),pos=position,life=0,mode=3) )

        if num_showers > 0:
            #Short wait until next shower
            idleWait = random.randrange(1,20) * shower_speed
            num_showers += -1
            print(f"Random sparkle at {position}. Next burst in: {idleWait/1000000000:.2f} with {num_showers} remaining.")
        else:
            #Longer wait until next shower (random 10-20s + 1s per burst in last shower)
            idleWait = (random.randrange(100,200) + last_shower_size*10) * 100000000
            #Set a new random number of shortly spaced shower for next interval
            num_showers = random.randrange(1,10)
            last_shower_size = num_showers
            shower_speed = random.randrange(1000000,100000000)
            print(f"Random sparkle at {position}. Next shower in: {idleWait/1000000000:.2f} with {num_showers} bursts spaced by {20*shower_speed/1000000000:.2f}.")
            # Turn off idle state until all sparkles have died or we just keep making more showers
            isIdle = False
        idleSince = time.monotonic_ns()
    elif len(darters) == 0 and gameMode == 1:
        #Reset idle state when all darters (including sparkles) have died
        isIdle = True

    #print(f"Idle since: {time.monotonic_ns() - idleSince}")
    #pixels[839] = (255,0,0)
    #pixels[838] = (0,255,0)
    #pixels[837] = (0,0,255)
    pixels.show()

    # Check for button press
    if bootBtn.value == True:
        setPixelBrightness(pixLevel + 0.2)
        time.sleep(1)

    # Check idle times
    p1WasIdle = player1IsIdle
    player1IsIdle = (time.monotonic_ns() - p1Idle) > idleWait * gameMode
    if player1IsIdle and p1WasIdle == False:
        print("Player 1 went idle")
    p2WasIdle = player2IsIdle
    player2IsIdle = (time.monotonic_ns() - p2Idle) > idleWait * gameMode
    if player2IsIdle and p2WasIdle == False:
        print("Player 2 went idle")
    if gameMode == 1:
        if (player1IsIdle == False) and (player2IsIdle == False):
            # Interaction from both ends, so start 2 player game
            print("Switching to game mode 2")
            gameMode = 2
            resetGame()
    elif gameMode == 2:
        if (player1IsIdle) and (player2IsIdle):
            # Both players idle, so switch back to single player mode
            print("Switching to game mode 1")
            puck.wipeDarterPixels()
            gameMode = 1


    time.sleep(delay)

