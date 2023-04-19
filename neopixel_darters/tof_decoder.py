import time

class distanceHandler:
    """
    Distance Handler class: Processes distance readings from TOF sensor. Shared by programs running at
    both ends of the LED strip, so that both players inputs are handled exactly the same way. Tracks
    state of the player interaction with the sensor, and uses a call back function to trigger the
    launch of a darter.
    """
    def __init__(self,launch_handler):
        self._state = 0  # 0=Waiting for start condition, 1=setting length, 2=charging, 3=launch
        self._length = 0
        self.lastDist = 0
        self.lastChange = time.monotonic_ns()
        self.chargeStart = time.monotonic_ns()
        self.launch = launch_handler

    @property
    def length(self):
        return self._length

    @property
    def state(self):
        return self._state

    def checkTimeout(self):
        timeout = False
        if self.lastDist != 0 and time.monotonic_ns() - self.lastChange > 5000000000:
            self.reset()
            timeout = True
            
        return timeout

    def reset(self):
        """
            Reverts to waiting state and clears loaded length
        """
        self._state = 0
        self._length = 0
        self.lastDist = 0
        
    def processDistance(self, distance):
       
        self.lastChange = time.monotonic_ns()
        
        if self._state == 0:
            #Waiting for distance < 10
            #print("Waiting: Distance={} cm".format(distance))
            if distance < 10:
                self._state = 1
                self.lastDist = distance
                self._length = 0
        elif self._state == 1:
            #Setting length, based on any distance > 15
            if distance > self.lastDist and distance > 15:
                self._length = distance - 15
                self.lastDist = distance
                self.chargeStart = time.monotonic_ns() # If no push, then start timing from when length last increased
            elif self._length > 0 and distance < self.lastDist - 1:
                self._state = 2
                if distance > 12:
                    # Start timing from when push started
                    self.chargeStart = time.monotonic_ns()
            #print(f"Loading: Length={self._length}, Distance={distance} cm, chargeStart:{self.chargeStart}")
        elif self._state == 2:
            #print(f"Charging: Length={self._length}, Charge time: {int(time.monotonic_ns() - self.chargeStart)/1000000} ms, Distance={distance}, LastDist={self.lastDist}, chargeStart:{self.chargeStart}")
            # Charge with continuous push from Length down to 10cm
            if distance > self.lastDist + 15:
                #print("Reset state=1: Length= {}, Distance={} cm".format(self._length,distance))
                #Revert to setting length again
                self._state = 1
                #Remove length indication
                #for i in range( int(self._length / 3) ):
                #    pixels[i] = (0,0,0)
            elif distance > self.lastDist + 1:
                #Reset charging
                self.chargeStart = time.monotonic_ns()
                #print(f"Reset chargeStart:{self.chargeStart}")
            elif distance < 10:
                #Launch
                #print(f"Charge time: {int(time.monotonic_ns() - self.chargeStart)/1000000} ms, chargeStart:{self.chargeStart}")
                charge = int( (time.monotonic_ns() - self.chargeStart)/10000000 )
                if charge > 0:
                    self._state = 3
                    if charge > 255:
                        charge = 255
                    if self._length > 255:
                        self._length = 255
                    self.launch(1,self._length,charge)
            if distance < self.lastDist - 1:
                self.lastDist = distance
        elif self._state == 3:
            if distance > 9:
                self._state = 0
