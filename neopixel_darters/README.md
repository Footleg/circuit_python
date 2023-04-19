# LED Flinger #
This project is an interactive LED installation using a time of flight (TOF) sensor as a touchless input to launch strings of LEDs along an RGB LED strip. 
The code is written in Circuit Python, so should be easy to port to any microcontroller which has Circuit Python support and I2C. The only other hardware required 
is a neopixel LED strip (obviously), a TOF sensor and I used a button on one of the GPIO pins to toggle the brightness. You also need a 5V power supply capable of 
supplying the current for the number of LEDs you plan to use. Here is the list of parts I chose.

## Parts List ##
- Pimoroni Tiny 2040 https://shop.pimoroni.com/products/tiny-2040?variant=39560012234835
- VL53L1X Time of Flight (ToF) Sensor Breakout https://shop.pimoroni.com/products/vl53l1x-breakout?variant=12628497236051
- Neopixel Strip(s). I bought these Black PCB, 5m 60 IP67 strips: https://www.aliexpress.com/item/32682015405.html

## Alternative Parts Choices ##
### Microcontroller ###
Any board which supports Circuit Python and has an I2C interface should do. I chose the Tiny 2040 for the small size and USB-C power input which can potentially 
handle more current to supply power to the LEDs than a micro USB connector. I also took advantage of the built in button which I could repurpose as an input. 
You could easliy build this project with a Raspberry Pi Pico however, and add a button to one of the GPIO pins instead (or not use the button at all). While these 
boards are all 3.3V, they take a 5V input power supply, so you can route the 5V from the USB socket to the LED strip, and then for larger installations you can 
power the LED strips directly and have the Pico powered off this 5V supply without needing to power it off the USB.

### LED Strips ###
This project uses WS2812B individually addressable 5050 RGB LED strips, commonly referred to as Neopixels. I had no problems running the strips I chose directly off 
the 3.3V logic from the RP2040 chip. Some strips may not work reliably without a logic level shifter, so test this if you choose a different LED strip. The strips I 
choose were IP67 which means they come in a protective silicon sleeve which keeps water out (except at the ends which need sealing if you plan to install these 
outside in damp conditions). I went for 5m strips with 60 LEDs per metre. That's 300 LEDs per strip. I already had a 4m strip, so combined with 2 x 5m strips I had 
840 LEDs in my installation. My strips came with plugs and sockets on each end and additional power supply wires which can be soldered on to connectors of your 
choice. These connectors are common on these WS2812B LED strips. For more information about Neopixels, see this excellent guide: 
https://learn.adafruit.com/adafruit-neopixel-uberguide

### TOF Sensor ###
I chose the VL53L1X Time of Flight sensor because I already had some in my robot parts box, and there is direct support for them in the Circuit Python libraries. 
You should be able to use any VL53L1X board for this project. A budget alternative would be to use one of the cheap ultrasonic distance sensors common in robotics 
kits. You would need to find some example code online for reading a distance from these from Circuit Python and I don't know how fast you can take successive 
distance readings from them.The VL53L1X chip worked well for this project, but I have not yet found a way to protect it from the elements. Anything I tried as a 
window on a case caused the sensor to misbehave, so I ended up just mounting the bare board on the outside of the case.

## Hardware Assembly ##
The neopixel strips need connecting to 5V, GND and a GPIO pin for the data in line. You can use any available GPIO pin (just update the pin number in the code to 
match). My strips came with a loose socket with 3 wires which I soldered directly to my RP2040 board. Red to 5V, white to GND and the green wire to the GPIO pin.

The TOF breakout I used support 5V, but many do not. So best connect the power pin to the 3.3V power pin on the RP2040 board. The GND pin goes to GND, and the I2C 
SDA and SCL pins on the breakout need connecting to the GPIO pins you choose for the I2C bus on your board. See in the code where these pins are set. They have to 
be pins on your Pico or RP2040 board which can provide I2C communication (see the pinout diagram for your board).

## Software Setup ##
This project is coded in Circuit Python. See this guide for how to download the right version for your board and how to install it. 
https://learn.adafruit.com/welcome-to-circuitpython
Once you have Circuit Python installed, your board should appear as a USB drive when you plug it in to a computer. You will see a code.py file in there and a 
lib folder. You will need to download the Circuit Python libraries bundle for the version of Circuit Python you are using, and copy some required libraries into 
the lib folder. You need the following for this project:
- adafruit_bus_device (this is a folder of libraries, copy the whole folder over)
- adafruit_pixelbuf.mpy
- adafruit_vl53l1x.mpy
- neopixel.mpy

Now you can copy the code.py file from the github repository onto your device, replacing the existing file you see there. As soon as the new file is present, it 
should run. To debug the code or troubleshoot any problems, I like to use the Thonny editor. This works well for me with Circuit Python on both Windows PCs and Linux 
computers like the Raspberry Pi. Open the code file on your device in Thonny, and click Stop, followed by Run in the editor. You should see the output from the
program in the console now (make sure Thonny is set to Circuit Python mode).

## 2 Player Version ##
This project was extended to create a 2 player LED Flinger experience. A pair of Pico boards communicating over UART were used. The main board code is in the file code_p1.py and controls the LED strip and receives data over UART from the secondary board placed at the other end of the LED strip. The secondary board is not connected to the LED strip, but sends data gather from the TOF sensor attached to it over UART to the main board. The secondary board code is in the file codep2.py. Rename each of these to code.py on each Pico so they run on boot. I used RS485 boards to connect the UART pins between the Pico boards for reliable communication over 20m of twisted pair cable. Each Pico requires the tof_decoder.py file which ensures the same code is used for both players to interpret the inputs from the TOF sensors at each end.