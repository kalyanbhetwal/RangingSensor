import smbus
import RPi.GPIO as GPIO
import time
import datetime
import os

from openant.easy.node import Node
from openant.easy.channel import Channel

from rpi_ws281x import Adafruit_NeoPixel, Color


"""
   Lighting Variables
   NeoPixel strip light meaning
   Light 8 - SD card available
   Light 7 - LogFile can be written to
   Light 6 - Distance Calibration recorded
   Light 5 - GPS Fix
   Light 4 - Button Press
   Light 3 - Distance > 30cm successfully measured?
   Light 2 - Valid GPS coordinates received?
   Light 1 - Was the location recorded?
"""

LED0 = 7
LED1 = 6
LED2 = 5
LED3 = 4
LED4 = 3
LED5 = 2
LED6 = 1
LED7 = 0

# calibration global variable

calibration = 0

# LED strip configuration:
LED_COUNT = 8  # Number of NeoPixels
LED_PIN = 18    # GPIO pin the NeoPixels are connected to
LED_FREQ_HZ = 800000  # Frequency (usually 800kHz)
LED_DMA = 10  # DMA channel to use
LED_BRIGHTNESS = 255  # 0-255 (max brightness)
LED_INVERT = False  # True to invert the signal

strip = Adafruit_NeoPixel(
   LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)

# Initialize the library (must be called once before other functions).
strip.begin()


red = Color(255, 0, 0)
orange = Color(255, 128, 0)
yellow = Color(255, 255, 0)
green =  Color(0, 255, 0)
light_blue = Color(0, 255, 255)
blue =  Color(0, 0, 255)
off = Color(0, 0, 0)

# Function to set the color of a specific pixel
def set_pixel_color(pixel_index, color):
    strip.setPixelColor(pixel_index, color)

# Function to update the LEDs
def show_leds():
    strip.show()


# Define the base path where measurement folders will be created
base_path = "/home/kalyan/hydrodata"  # Replace with your desired base path

from RPLCD.i2c import CharLCD
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
lcd.clear()



# Definition of Variables for ANT
NETWORK_KEY = [0xE8, 0xE4, 0x33, 0xA9, 0xDD, 0x56, 0xC1, 0x43]

Device_Type = 16  
Device_Number = 12345  # Change if you need.
Channel_Period = 8192
Channel_Frequency = 66



# Define the GPIO pin for the pushbutton
button_pin = 17  # Change this to the actual GPIO pin you are using

# Configure GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up the button pin as an input with a pull-up resistor
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def lcdCalib():
    # Update the LCD:
    lcd.clear()
    lcd.cursor_pos = (0, 0) 
    lcd.write_string("Dist calibration") #Remind user to calibrate
    lcd.cursor_pos = (1, 1) 
    lcd.write_string("    Needed    ") #Remind user to calib  rate  

# Function to create a folder based on year and month
def create_folder(year, month):
    folder_name = os.path.join(base_path, f"{year}_{month:02d}")  # Format: base_path/YYYY-MM
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    return folder_name


# Function to write measurements to a file in a folder based on the year and month
def write_measurement_to_file(measurement):
    now = datetime.datetime.now()
    year, month = now.year, now.month
    folder_name = create_folder(year, month)
    datet = datetime.datetime.now().strftime("%Y-%m-%d")
    current_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    filename = f"{folder_name}/measurement_{datet}.txt"
    
    with open(filename, "a") as file:
        file.write(f"Timestamp: {current_date},")
        file.write(f"Measurement: {measurement} cm")
        file.write("\n")  # Add a separator or new line between entries

class LidarLiteV4:
    def __init__(self, bus_num=1, address=0x62):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus.SMBus(self.bus_num)

    def write_register(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)

    def read_register(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def start_measurement(self):
        self.write_register(0x00, 0x04)

    def wait_for_measurement(self):
        while True:
            status = self.read_register(0x01)
            if status & 0x01 == 0:
                break
            time.sleep(0.01)

    def read_distance(self):
        self.start_measurement()
        self.wait_for_measurement()
        low_byte = self.read_register(0x10)
        high_byte = self.read_register(0x11)
        return (high_byte << 8) | low_byte

class AntSendDemo:
    def __init__(self):

        self.ANTMessageCount = 0
        self.ANTMessagePayload = [0, 0, 0, 0, 0, 0, 0, 0]
        self.value_accumulator = 0
        self.values_received = 0
    def Create_Next_DataPage(self):
        # Define Variables
        self.ANTMessageCount += 1
        
        self.ANTMessagePayload = [0x31, self.ANTMessageCount, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
            # ANTMessageCount reset
        if self.ANTMessageCount > 254:
            self.ANTMessageCount = 0

        return self.ANTMessagePayload

    # TX Event
    def on_data(self, data):

        if self.values_received < 5:
            page = data[0]
            if page!=80 and page!=81 and (page & 0x0F) <= 7:
                self.values_received += 1
                self.value_accumulator += data[6]

            if self.values_received == 5:
                average = int(self.value_accumulator / 5)
                lcd.clear()
                lcd.write_string('Distance:' + str(average) +' cm')
                write_measurement_to_file(average)

        #print(f"on_data: {data}")
        
    def start_measurement(self, channel):
        print("Started to measure")
        self.Create_Next_DataPage()
        print("payload ", self.ANTMessagePayload)
        self.channel.send_acknowledged_data(self.ANTMessagePayload) 
        self.values_received = 0
        self.value_accumulator = 0
    # Open Channel
    def OpenChannel(self):

        self.node = Node()  # initialize the ANT+ device as node

        # CHANNEL CONFIGURATION
        self.node.set_network_key(0x00, NETWORK_KEY)  # set network key
        self.channel = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE) 
        # self.channel.set_id(
        #     Device_Number, Device_Type, 5
        # )  # set channel id as <Device Number, Device Type, Transmission Type>
        self.channel.set_period(Channel_Period)  # set Channel Period
        self.channel.set_rf_freq(Channel_Frequency)  # set Channel Frequency
        self.channel.set_search_timeout(12)
        self.channel.set_id(0, 16, 0)
        print("Starting a node")
        # Callback function for each TX event
        self.channel.on_broadcast_data = self.on_data
        GPIO.add_event_detect(17, GPIO.RISING, callback=self.start_measurement, bouncetime=500)
        #self.channel.on_burst_data = self.on_data

        try:
            self.channel.open()  # Open the ANT-Channel with given configuration
            self.node.start()
        except KeyboardInterrupt:
            print("Closing ANT+ Channel...")
            self.channel.close()
            self.node.stop()
        finally:
            print("Final checking...")
            # not sure if there is anything else we should check?! :)

def is_device_present(address=0x62, bus_num=1):
    try:
        bus = smbus.SMBus(bus_num)
        bus.read_byte_data(address, 0x00)
        return True
    except Exception as e:
        return False

def button_callback(channel):
    global is_measurement_started
    is_measurement_started = True

if __name__ == "__main__":
    lcd.write_string("Snow Depth Probe")
    lcd.cursor_pos = (1, 1) 
    lcd.write_string("Initializing...")
    print("Sensor Started")
    time.sleep(4)

    lcd.write_string("Searching for Devices")
    print("Searching for Devices")
    is_measurement_started = False
    is_i2c_present = is_device_present()

    if is_i2c_present:
        lcd.clear()
        lcd.write_string("I2C Device Found")
        print("I2C device found")
        time.sleep(2)

        lcdCalib()
        GPIO.wait_for_edge(button_pin, GPIO.FALLING)  # wait for the button press to calibrate

        newDistance = LidarLiteV4().read_distance()

        if newDistance > 5 and newDistance < 4500:  
            calibration = newDistance  
            lcd.write_string('Distance: ' + str(calibration) + ' cm')
            print("Calibration Distance: {} cm".format(calibration))

            strip.setPixelColor(LED2, blue) #Set third light to blue to indicate calibration measurement recorded
            strip.show()

        time.sleep(2)
        lcd.clear()
        GPIO.cleanup()
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        lcd.write_string("Start Measurement")
        # start measurement after calibration 

        try:
            GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=500)
            while True:
                if is_measurement_started:
                    distance = LidarLiteV4().read_distance()
                    # check if the measurement is within the bounds
                    if distance < 5 or distance > 1000:    
                         lcd.clear()
                         lcd.write_string('Invalid  Measurement')
                         ## add logic for neo pixel to indicate this
                    lcd.clear()
                    lcd.write_string('Distance: ' + str(distance) + ' cm')
                    write_measurement_to_file(distance)
                    print("Distance: {} cm".format(distance))
                    is_measurement_started = False
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass

        finally:
            GPIO.cleanup()
    else:
        ant_senddemo = AntSendDemo()
        try:
            ant_senddemo.OpenChannel()  # start
        except KeyboardInterrupt:
            print("Closing ANT+ Channel!")
        finally:
            print("Finally...")

        print("Close demo...")


