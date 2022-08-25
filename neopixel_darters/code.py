"""
NeoPixel darters, an interactive RGB LED installation.
REQUIRED HARDWARE:
* RGB NeoPixel LEDs connected to pin GP0.
* TOF vl53l1 sensor on the I2C bus.
"""
import time
import random
import board
from digitalio import DigitalInOut, Direction, Pull
import microcontroller
import neopixel
import busio
import adafruit_vl53l1x

# Update this to match the number of NeoPixel LEDs connected to your board.
num_pixels = 840

# Anti-aliasing: ratio of position coordinate space for animations to actual pixels
# e.g. Set to 4 means the animation needs to move 4 spaces to move to the next neopixel
# For longer strip lengths, lower this number to speed up the program as it slows with more pixels to manage.
# A value of 8 is good for a 240 - 300 led strip. I change it to 4 when I have 900 LEDs using 3 strips.
aa = 2

# Set speed of animations (1 - 1000) Normally set to 1000 for top speed, but set lower to debug
speed = 1000

# Tiny2040 board built in RGB LED and button
ledR = DigitalInOut(board.LED_R)
ledR.direction = Direction.OUTPUT
ledG = DigitalInOut(board.LED_G)
ledG.direction = Direction.OUTPUT
ledB = DigitalInOut(board.LED_B)
ledB.direction = Direction.OUTPUT
bootBtn = DigitalInOut(microcontroller.pin.GPIO23)
bootBtn.direction = Direction.INPUT

def setBoardLED(r,g,b):
    ledR.value = not r
    ledG.value = not g
    ledB.value = not b

# Initialize I2C bus and sensor.
i2c = busio.I2C(scl=board.GP5, sda=board.GP4)
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# OPTIONAL: can set non-default values
vl53.distance_mode = 1
vl53.timing_budget = 100

#Initialise neopixels
pixels = neopixel.NeoPixel(board.GP0, num_pixels, auto_write=False)
pixLevel = 0.2

def setPixelBrightness(brightness):
    global pixLevel
    
    if brightness > 1:
        pixLevel = 0.2
    elif brightness < 0.2:
        pixLevel = 0.2
    else:
        pixLevel = brightness
    
    if pixLevel <= 0.3:
        setBoardLED(False,False,True)
    elif pixLevel < 0.5:
        setBoardLED(False,True,True)
    elif pixLevel < 0.7:
        setBoardLED(False,True,False)
    elif pixLevel < 0.9:
        setBoardLED(True,True,False)
    else:
        setBoardLED(True,False,False)
    
    print(f"Set pixel brightness to {pixLevel}")
    pixels.brightness = pixLevel

setPixelBrightness(0.2)

#Globals for touchless sensing
state = 0 # 0=Waiting for start condition, 1=setting length, 2=charging, 3=launch
length = 0
charge = 0
lastDist = 0
chargeStart = time.monotonic_ns()

#Globals for tracking idle time and state
idleWait = 10000000000
isIdle = False

#Globals calculated from settings
particleSpeed = int(1 * aa)


class darter:
    def __init__(self, length=6, pos=0, speed=4, colour=(255,255,0), life=40, mode=0 ):
        self.length = length
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
            self.slowRate = 100 - self.length
            if self.slowRate < 10: self.slowRate = 10
        elif self.mode == 2:
            # Mode 2 is hot particles
            self.colourFromSpeed()
            self.life = 1000 #Life will be set to zero when speed is zero, so give a large value here
            self.slowRate = 10

    def update(self, pix):
        #Update position
        newpos = self.pos + self.speed
        if newpos > num_pixels * aa:
            self.speed = -self.speed
            newpos = num_pixels * aa
        elif newpos < 0:
            self.speed = -self.speed
            newpos = 0
        self.pos = newpos
        
        self.steps += 1
        
        if self.mode > 0:
            # Slow down after number of steps
            if self.steps > self.slowRate:
                self.steps = 0
                if self.speed > 1:
                    self.speed += - 1
                elif self.speed < -1:
                    self.speed += 1
                else:
                    self.life = 0
                    
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
            #if 0 <= newIdxFront < num_pixels:
            #    pixels[newIdxFront] = (r,g,b)
            self.blendColours(newIdxFront,(r,g,b) )
            
            newIdxRear = midIdx - i
            #if 0 <= newIdxRear < num_pixels:
            #    pixels[newIdxRear] = (r,g,b)
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


darters = []
#darters.append( darter(length=12,life=1,pos=100*aa,speed=-15,mode=1) )
#darters.append( darter(length=60,life=1800,pos=160*aa,speed=15,mode=1) )
#darters.append( darter(length=30,life=840,speed=30,mode=1) )
#darters.append( darter(length=16,colour=(0,0,255),pos=20*aa,speed=6,life=200) )
"""
darters.append( darter(length=3,colour=(255,0,0),speed=30,life=200) )
darters.append( darter(length=15,colour=(0,255,0),speed=8,life=150) )
darters.append( darter(length=10,colour=(0,0,255),speed=7,life=100) )
darters.append( darter(length=20,colour=(0,200,200),speed=5,life=80) )
"""
delay = 1 / speed

#All off
for i in range( num_pixels ):
    pixels[i] = (0,0,0)

vl53.start_ranging()

#Initialise idle time counting and random shower variables
idleSince = time.monotonic_ns()
shower_speed = 1000000
num_showers = 1
last_shower_size = num_showers

while True:
    #wipe each darter
    for i in range(len(darters)):
       darters[i].wipeDarterPixels()
        
        
    if vl53.data_ready:
        distance = vl53.distance
        vl53.clear_interrupt()
        
        if distance != None:
            if state == 0:
                #Waiting for distance < 10
                #print("Waiting: Distance={} cm".format(distance))
                if distance < 10:
                    state = 1
                    lastDist = distance
                    length = 0
            elif state == 1:
                #print("Loading: Length= {}, Distance={} cm".format(length,distance))
                #Setting length, based on any distance > 15
                if distance > lastDist and distance > 15:
                    length = distance - 15
                elif length > 0 and distance < lastDist - 1:
                    state = 2
                    charge = 0
                    chargeStart = time.monotonic_ns()
                lastDist = distance
            elif state == 2:
                #print("Charging: Charge= {}, Distance={} cm".format(charge,distance))
                # Charge with continuous push from Length down to 10cm
                if distance > lastDist - 1:
                    #Reset charging
                    charge = 0
                    chargeStart = time.monotonic_ns()
                    lastDist = distance
                elif distance > length + 15:
                    #Revert to setting length again
                    state = 1
                    lastDist = distance
                    #Remove length indication
                    for i in range( 50 ):
                        pixels[i] = (0,0,0)
                elif distance < 10:
                    #Launch
                    charge = int( (time.monotonic_ns() - chargeStart)/1000000 )
                    state = 3
                    launchSpeed = int( (60 * length + 3400) / charge )
                    if launchSpeed < 1:
                        launchSpeed = 1
                    elif launchSpeed > 30:
                        launchSpeed = 30
                    life = int( (1000 - launchSpeed*20) * random.randrange(1,7) * num_pixels * aa / 9600)
                    darters.append( darter(length=int(length / 1.66),speed=launchSpeed,life=life, mode=1 ) )
                    isIdle = False #Set flag to not idle to prevent random showers
                    print(f"Launched: Length={length} cm, Charge={charge}, Speed={launchSpeed}, Life={life}" )
                #else:
                #    #Estimate charge from rate for change
                #    charge = int( (length + 15 - distance) * (time.monotonic_ns() - chargeStart)/(length * 1000000) )
            elif state == 3:
                if distance > 9:
                    state = 0
        
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
            if d.mode != 2:
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
                    spd = maxSpd + random.randrange(-8,8)
                    darters.append( darter(length=1,speed=spd,pos=int(d.pos + random.randrange(-d.halflen,d.halflen)*aa ),mode=2) )
            #Turn off pixels from dead darter
            d.wipeDarterPixels()
        elif d.mode == 1:
            # Handle interactions for darters which are mode 1 (i.e. not particles)
            if abs(d.speed) > 28 and d.length < 10:
                # If a short high speed darter, check if it has collided with any slower darters
                for chk in range( len(darters) ):
                    d2 = darters[chk]
                    if chk != i and d2.mode == 1 and abs(d.pos - d2.pos) < 5 * aa:
                        # Split darter
                        #print(f"split a:{d.speed} t:{d2.speed}")
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
                        #print(f"After a:{d.speed} t1:{d2.speed} t2:{d3spd}")
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
        
    
    #When building new darter, show length on start of strip
    if state == 1 and length > 0:
        for i in range( int(length / 3) ):
            pixels[i] = (180,180,180)
    
    #Check for idle
    if time.monotonic_ns() - idleSince > idleWait and isIdle:
        #Spawn a random particle shower
        position = random.randrange(num_pixels * aa)
        darters.append( darter(length=1,colour=(255,0,255),speed=random.randrange(-particleSpeed,particleSpeed),pos=position,life=0) )
        
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
    elif len(darters) == 0:
        #Reset idle state when all darters (including sparkles) have died
        isIdle = True
        
    #print(f"Idle since: {time.monotonic_ns() - idleSince}")
    
    pixels.show()
    
    #Check for button press
    if bootBtn.value == False:
        setPixelBrightness(pixLevel + 0.2)
        time.sleep(1)
        
    time.sleep(delay)
