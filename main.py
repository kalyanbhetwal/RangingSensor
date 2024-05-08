import smbus
import RPi.GPIO as GPIO
import time
import datetime
import os
import logging
import serial
import pynmea2
from geopy.distance import geodesic

from openant.easy.node import Node
from openant.easy.channel import Channel

from rpi_ws281x import Adafruit_NeoPixel, Color

from RPLCD.i2c import CharLCD
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
lcd.clear()

# Define the base path where measurement folders will be created
base_path = "/home/kalyan/hydrodata/measurement"  # Replace with your desired base path
base_path_log = "/home/kalyan/hydrodata/logs"

# Define your serial port and baud rate
serial_port = '/dev/serial0'  # Replace with your actual serial port
baud_rate = 115200  # Replace with your actual baud rate

#Retry ANT connection
RETRY_COUNT = 5

# Create a serial object
ser = serial.Serial(serial_port, baud_rate, timeout=5)
current_fix = ''
current_fix_rmc = ''
next_fix = ''
next_fix_rmc = ''
start_location = ''

last_button_press_time = 0

TIMEOUT_THRESHOLD = 5

count = 0 #Keeps track of number of logs recorded

lightInterval = 1 #How often to update the lights
periodBlue = 5; #Max time between GPS updated readings (for light display)
periodRed = 2; #Max time between GPS updated readings (for light display)
updateLightTime = 0; #used to keep track of how often the lights have been updated
updateGPSTime = 0; #used to keep track of how often a valid GPS reading is received

startGPSTime = 0
startGPSTimeMS = 0


dist_flag= 0
first_dist_log = 0
# for LCD screen update
previousLCDsec = 0
lcdInterval = 3

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

LED0 = 7   #SD card available
LED1 = 6   #LogFile can be written to
LED2 = 5   #Distance Calibration recorded
LED3 = 4   #GPS Fix
LED4 = 3   #Button Press
LED5 = 2   #Distance > 30cm successfully measured?
LED6 = 1   #Valid GPS coordinates received?
LED7 = 0   #Was the location recorded?

# calibration global variable

calibration = 0

# NeoPixel configuration
LED_COUNT = 8      # Number of LED pixels
LED_PIN = 18       # GPIO pin connected to the pixels (use BCM numbering)
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800kHz)
LED_DMA = 10       # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255  # Set to 0 for darkest and 255 for brightest
LED_INVERT = False  # True to invert the signal (when using NPN transistor level shift)

# Create NeoPixel object
strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
strip.begin()

red = Color(255, 0, 0)
orange = Color(255, 128, 0)
yellow = Color(255, 255, 0)
green =  Color(0, 255, 0)
light_blue = Color(0, 255, 255)
blue =  Color(0, 0, 255)
off = Color(0, 0, 0)


#Function to Turn off all pixels
def reset_pixel_color():
    for i in range(LED_COUNT):
        strip.setPixelColor(i, Color(0, 0, 0))

# Function to set the color of a specific pixel
def set_pixel_color(pixel_index, color):
    strip.setPixelColor(pixel_index, color)


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
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

is_measurement_started = False

def lcdCalib():
    # Update the LCD:
    lcd.clear()
    lcd.cursor_pos = (0, 0) 
    lcd.write_string("Dist calibration") #Remind user to calibrate
    lcd.cursor_pos = (1, 1) 
    lcd.write_string("    Needed    ") #Remind user to calib  rate  

# Function to create a folder based on year and month
def create_folder(base_path, year, month):
    folder_name = os.path.join(base_path, f"{year}_{month:02d}")  # Format: base_path/YYYY_MM
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
    return folder_name

def proces_measurement(distance, calibration):
    if distance > 5 and  distance < 4500:
        strip.setPixelColor(LED5, blue) #Set sixth light to green to indicate valid distance
        strip.show()
        lcd.clear()
        lcdDepth(distance, calibration)
        gps_flag, gps_data, gps_data_dt = check_gps()
        if gps_flag:
            logging.info("GPS Ready")
            strip.setPixelColor(LED6, blue)
            strip.show()
            if write_measurement_to_file(distance, calibration, gps_data, gps_data_dt):
                strip.setPixelColor(LED7, blue)
                strip.setPixelColor(LED4, blue)
                strip.show()
            else:
                strip.setPixelColor(LED4, red) #Set seventh light to red to indicate failure to log coordinates and distance
                strip.setPixelColor(LED7, red)
                strip.show()
                logging.warning("Failed to log new GPS data.")
        else:
            #GPS coordinate isn't valid
            strip.setPixelColor(LED4, red)
            strip.setPixelColor(LED6, red)
            strip.setPixelColor(LED7, red)
            strip.show()
            #Print a debug message. Maybe we don't have enough satellites yet.
            time.sleep(2)
            lcd.clear()
            lcd.cursor_pos = (0, 0) #Set the cursor to column 1, line 1
            lcd.write_string("  No GPS data.  ") #Prints string "Display = " on the LCD
            lcd.cursor_pos = (1, 1)
            lcd.write_string(" Sats: ")
            if gps_data and gps_data.num_sats:
                lcd.write_string(gps_data.num_sats); #Prints the number of Sats
            else:
                lcd.write_string("0"); #Prints the number of Sats
        time.sleep(2)
        strip.setPixelColor(LED4, off)
        strip.setPixelColor(LED5, off)
        strip.setPixelColor(LED6, off)
        strip.setPixelColor(LED7, off)
        strip.show()
        lcdSats()
    elif distance==0:
        strip.setPixelColor(LED4, red)
        strip.setPixelColor(LED5, red)
        strip.setPixelColor(LED6, red)
        strip.setPixelColor(LED7, red)
        strip.show()
        lcd.clear()
        lcd.write_string("Check Battery")
        time.sleep(4)
        strip.setPixelColor(LED4, off)
        strip.setPixelColor(LED5, off)
        strip.setPixelColor(LED6, off)
        strip.setPixelColor(LED7, off)
        strip.show()
        #reset_pixel_color()
        strip.show()
        lcdSats()
    else:
        strip.setPixelColor(LED4, red)
        strip.setPixelColor(LED5, red)
        strip.setPixelColor(LED6, red)
        strip.setPixelColor(LED7, red)
        strip.show()
        lcdDepth(distance, calibration)
        time.sleep(4)
        strip.setPixelColor(LED4, off)
        strip.setPixelColor(LED5, off)
        strip.setPixelColor(LED6, off)
        strip.setPixelColor(LED7, off)
        strip.show()
        #reset_pixel_color()
        strip.show()
        lcdSats()

# Function to write measurements to a file in a folder based on the year and month
def write_measurement_to_file(measurement, calibration, gps_data, gps_data_dt):
    global count, start_location, first_dist_log, count
    logging.warning("Started writing to the file")
    now = datetime.datetime.now()
    year, month = now.year, now.month
    folder_name = create_folder(base_path, year, month)
    filename = f"{folder_name}/measurement_{gps_data_dt.datestamp}.txt"
    actual_Dist =   calibration - measurement
    #   "Count", "Distance","Calibration","raw measurement", "latitude", "longitude", "altitude", "sat_count", "HDOP", "dateTime", "GPSTime", "status"
    with open(filename, "a") as file:
        file.write(f"{count},")
        file.write(f"{actual_Dist:.2f},")
        file.write(f"{calibration:.2f},")
        file.write(f"{measurement:.2f},")
        file.write(f"{gps_data.latitude:.6f},")
        file.write(f"{gps_data.longitude:.6f},")
        file.write(f"{gps_data.altitude:.6f},")
        file.write(f"{gps_data.num_sats},")
        file.write(f"{gps_data.horizontal_dil},")

        time_stamp = gps_data_dt.timestamp
        date_stamp = gps_data_dt.datestamp

        datetime_str = f"{date_stamp} {time_stamp}"

        try:
            datetime_obj = datetime.datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S%z")
        except:
            lcd.clear()
            lcd.write_string("GPS Data None")
            lcd.write_string("Retake")
            strip.setPixelColor(LED4, red)
            strip.setPixelColor(LED5, red)
            strip.setPixelColor(LED6, red)
            strip.setPixelColor(LED7, red)
            strip.show()
            return

        # Convert to a string in the correct format
        human_readable_datetime = datetime_obj.strftime("%Y-%m-%d %H:%M:%S%z")
        file.write(f"{human_readable_datetime}")
        file.write("\n")  # Add a separator or new line between entries
        if first_dist_log == 0:
            start_location = gps_data
            first_dist_log = 1
        strip.setPixelColor(LED1, blue)
        strip.show()

        # if actual_Dist > 30:
        #     strip.setPixelColor(LED5, orange)
        #     strip.show()
        count = count + 1
    return True

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
        self.calibration_flag = True
        self.push_btn = False
        self.previous_packet = 0
        self.calibration = 0

    def Create_Next_DataPage(self):
        # Define Variables
        if self.start_frame >=255:
            self.ANTMessageCount = 0
        else:
            self.ANTMessageCount = self.start_frame + 1
    
        self.ANTMessagePayload = [0x31, self.ANTMessageCount, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        return self.ANTMessagePayload
    
    def printDataLCD(self, distance, cali):
        depth =  distance - cali
        # Display the distance on the LCD:
        lcd.clear()
        lcd.cursor_pos = (0, 0) #Set the cursor to column 1, line 1
        lcd.write_string(" Depth  at ")
        lcd.write_string(str(count))
        lcd.cursor_pos = (1, 5)
        lcd.write_string(str(depth)) #Prints the measured snow depth
        lcd.write_string(" cm") #Prints "cm" on the LCD

    def start_measurement(self, channel):
        self.push_btn = True
        strip.setPixelColor(LED4, green) #Set button pressed
        strip.show()
        logging.warning("Button Pressed")
        self.Create_Next_DataPage()
        logging.warning(self.ANTMessagePayload)
        logging.warning("payload ", self.ANTMessagePayload)
        self.channel.send_broadcast_data(self.ANTMessagePayload) 
        logging.warn("ACK received")
    
    def calibration_m(self):
        pass

    # Open Channel
    def OpenChannel(self):
        global RETRY_COUNT
        self.node = Node()  # initialize the ANT+ device as node
        # CHANNEL CONFIGURATION
        self.node.set_network_key(0x00, NETWORK_KEY)  # set network key
        self.channel = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE) 
        # self.channel.set_id(
        #     Device_Number, Device_Type, 5
        # )  # set channel id as <Device Number, Device Type, Transmission Type>
        self.channel.set_period(Channel_Period)  # set Channel Period
        self.channel.set_rf_freq(Channel_Frequency)  # set Channel Frequency
        self.channel.set_search_timeout(200)
        self.channel.set_id(0, 16, 0)

        GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=self.start_measurement, bouncetime=3000)
        logging.warning("Starting a node")
        lcd.clear()
        lcd.write_string("Ant Node Ready")
        time.sleep(2)
        lcdCalib() 

        self.channel.open()  # Open the ANT-Channel with given configuration
        self.start_frame = 0
        data = self.node.start(self.start_frame, True)
        self.start_frame = data[5]
        logging.warning(f"Staring frame id {self.start_frame}")
        while not self.push_btn:
            time.sleep(1)
            smart_delay(2)
        
        # self.push_btn = False
        # #self.channel.send_acknowledged_data(self.ANTMessagePayload) 
        # time.sleep(2)
        # if data[0]==0:
        #     lcd.clear()
        #     lcd.write_string("Recalibrate")

        while True:
            if self.push_btn:
                self.push_btn = False
                data = self.node.start(self.start_frame)
                if data[0]!=0:
                    break
                lcd.clear()
                lcd.write_string("Recalibrate")
                logging.warn("Recalibrate")

        self.start_frame = data[5]
        newDistance = ( data[7] <<8 | data[6])
        logging.warning(f"frame after calibration: {self.start_frame}")

        if newDistance > 5 and newDistance < 4500:  
            self.calibration = newDistance  
            lcd.clear()
            lcd.write_string('Calibration Distance: ' + str(self.calibration) + ' cm')
            logging.warning("Calibration Distance: {} cm".format(self.calibration))
            strip.setPixelColor(LED2, blue) #Set third light to blue to indicate calibration measurement recorded
            strip.show()
            time.sleep(2)
        lcd.clear()
        # lcd.write_string("Start...")
        # time.sleep(2)
        strip.setPixelColor(LED4, off) #Off the fifth light
        strip.show()
        try:
            while True:
                if self.push_btn:
                    self.push_btn = False
                    #self.Create_Next_DataPage(self.start_frame+1)
                    #self.channel.send_acknowledged_data(self.ANTMessagePayload) 
                    data = self.node.start(self.start_frame)
                    if data[0] !=0:
                        self.start_frame = data[5]
                    elif data[0] == 0:  #failed measurement and try one more time
                        while RETRY_COUNT > 0:
                            logging.warn("Retry")
                            self.start_measurement(self.channel) 
                            data = self.node.start(self.start_frame)
                            if data[0]!= 0:
                                self.start_frame = data[5]
                                RETRY_COUNT = 5
                                break
                            RETRY_COUNT = RETRY_COUNT - 1
                        self.start_frame = 0
                    logging.warning(data)
                    distance = ( data[7] <<8 | data[6])
                    calibration = self.calibration
                    proces_measurement(distance, calibration)
                smart_delay(1)
                updateLights()
                update_screen()
        except Exception as e:
            logging.warning(f"Error: {e}")
            self.channel.close()
            self.node.stop()
        finally:
            logging.warning("Final checking...")
            # not sure if there is anything else we should check?! :)

def is_device_present(address=0x62, bus_num=1):
    try:
        bus = smbus.SMBus(bus_num)
        bus.read_byte_data(address, 0x00)
        return True
    except Exception as e:
        return False
def button_callback(channel):
    global last_button_press_time, is_measurement_started
    # Get the current time
    current_time = time.time()
    # Check if it has been more than 1 seconds since the last button press
    if (current_time - last_button_press_time) > 1:
        is_measurement_started = True
        last_button_press_time = current_time
        strip.setPixelColor(LED4, green) #Set fifth light to orange to indicate button press
        strip.show()
        time.sleep(0.1)
    logging.warning("Button Pressed")

def setUP():
    #lcd.write_bytes(0x08)
    lcd.write_string("OTTO Depth Probe")
    lcd.cursor_pos = (1, 1) 
    lcd.write_string("Initializing...") 
    logging.info("Sensor Started")

def check_timeout(last_fix_time):
    # Check if a timeout has occurred
    elapsed_time = time.time() - last_fix_time
    if elapsed_time > TIMEOUT_THRESHOLD:
        logging.warning("Timeout: No valid GPS fix found within {} seconds.".format(TIMEOUT_THRESHOLD))
        return True
        #raise TimeoutError("GPS fix timeout")
    return False


def check_gps():
    global updateGPSTime, startGPSTime, current_fix, current_fix_rmc, next_fix, next_fix_rmc
    # Initialize the last_fix_time variable
    last_fix_time = time.time()
    flag_current_fix = False
    flag_current_fix_rmc = False
    try:
        while True:
            # Read a line of data from the GPS device
            sentence = ser.readline().decode("utf-8").strip()
            msg = pynmea2.parse(sentence)

            if check_timeout(last_fix_time):
                return False, None, None
        
            if isinstance(msg, pynmea2.GGA):
                next_fix = msg
                # Check if fix quality is valid (1 or 2 typically indicates a valid fix)
                if  msg.gps_qual in [1, 2]:
                    updateGPSTime = time.time()
                    current_fix = next_fix
                    updateLights()
                    if startGPSTime == 0:
                        startGPSTime = current_fix.timestamp
                    flag_current_fix = True
            if isinstance(msg, pynmea2.RMC):
                next_fix_rmc = msg
                flag_current_fix_rmc = True

            if flag_current_fix and flag_current_fix_rmc:
                return True, current_fix, next_fix_rmc
    except Exception as e:
        logging.warning(f"Error in GPS: {e}")
        return False, None

def lcdSats():
    if next_fix:
        if next_fix.horizontal_dil:
            hdopVal = float(next_fix.horizontal_dil)
            satellites = next_fix.num_sats
        else:
            hdopVal = 0.0
            satellites = 0
    else: 
        hdopVal = 0.0
        satellites = 0
    lcd.clear()
    lcd.cursor_pos = (0, 0)
    lcd.write_string(" Sats: ")
    lcd.write_string(str(satellites)); #Prints the # of Satellites
    # if (time.time() - updateGPSTime > periodRed):
    #     lcd.write_string(" (OLD)")
    lcd.cursor_pos = (1, 0)
    lcd.write_string(" HDOP: ")
    lcd.write_string(str(hdopVal)) # Prints the HDOP Value (not sure how accurate this is)
    # if (time.time() - updateGPSTime > periodRed):
    #     lcd.write_string(" (OLD)")

def lcdDepth(distance, calibration):
    global count
    depth =   calibration - distance
    # Display the distance on the LCD:
    lcd.cursor_pos = (0, 0) #Set the cursor to column 1, line 1
    lcd.write_string(" Depth  at ")
    lcd.write_string(str(count))
    lcd.cursor_pos = (1, 5)
    lcd.write_string(str(depth)) #Prints the measured snow depth
    lcd.write_string(" cm") #Prints "cm" on the LCD

def lcd_distance():
    global first_dist_log, start_location, next_fix, count, filename

    range_value = 0.0
    if first_dist_log == 0:
        range_value = 0.0
    else:
        start_loc = (float(start_location.latitude), float(start_location.longitude))
        curr_loc = (float(next_fix.latitude), float(next_fix.longitude))
        range_value = geodesic(start_loc, curr_loc).kilometers

    range_value = round(range_value * 1000,2)  # Convert to meters
    lcd.clear()
    lcd.cur_pos = (0, 0)
    lcd.write_string("Distance: ")
    lcd.write_string(str(range_value))  # Prints the distance from the calibration point
    lcd.write_string(" m")
    lcd.cur_pos = (1, 0)
    lcd.write_string(" Count: ")
    lcd.write_string(str(count))  # Prints the number of depths recorded
    # lcd.write_string("  (")
    # lcd.write_string(filename[3])  # Prints the logFileName
    # lcd.write_string(filename[4])  # Second digit
    #lcd.write_string(")")

def update_screen():
    global previousLCDsec, dist_flag

    if time.time() > previousLCDsec + lcdInterval:
        if dist_flag == 0:
            lcd_distance()
            dist_flag = 1
        else:
            lcdSats()
            dist_flag = 0
        previousLCDsec = time.time()


def updateLights():
    global updateLightTime, lightInterval
    try:
        if (time.time() > updateLightTime + lightInterval):
        # Update GPS light
            if (updateGPSTime == 0):
                strip.setPixelColor(LED3, red) #Set fourth light to red to indicate never received GPS signal

            elif (time.time() - updateGPSTime <= periodBlue):
                if next_fix:
                    if next_fix.horizontal_dil:
                        hdop = float(next_fix.horizontal_dil)
                        if hdop < 2:
                            strip.setPixelColor(LED3, blue) #Set fourth light to blue to indicate good GPS fix
                        else:
                            strip.setPixelColor(LED3, light_blue) #Set fourth light to light_blue to indicate valid GPS fix
                
            elif (time.time() - updateGPSTime > periodBlue): #test whether the periodBlue has elapsed
                if (time.time() - updateGPSTime > periodRed):
                    strip.setPixelColor(LED3, red) #Set fourth light to red to indicate no GPS signal
                else:
                    strip.setPixelColor(LED3, yellow) #Set fourth light to yellow to indicate poor GPS signal
            else:
                strip.setPixelColor(LED3, red)# Set fourth light to red to indicate no GPS connection
            updateLightTime = time.time()
            strip.show()
    except Exception as e:
        logging.warning(f"Exception: {e}")

def smart_delay(ms):
    global updateGPSTime, next_fix
    start = time.time()
    while True:
        if ser:
            sentence = ser.readline().decode("utf-8").strip()
            msg = pynmea2.parse(sentence)
            if isinstance(msg, pynmea2.GGA) and msg.gps_qual in [1, 2]:
                next_fix = msg
                updateGPSTime = time.time()
        if time.time() - start > ms:
            break


if __name__ == "__main__":
    now = datetime.datetime.now()
    year, month = now.year, now.month
    folder_name = create_folder(base_path_log, year, month )
    datet = datetime.datetime.now().strftime("%Y-%m-%d")
    current_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    logfileName = f"{folder_name}/logs_{datet}.txt"
    logging.basicConfig(
        filename=logfileName, level=logging.INFO,   format="%(asctime)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    ) 
    logger = logging.getLogger("Otto")

    # lcd strip
    for i in range(8):
        strip.setPixelColor(i, Color(0, 0, 255))  # Blue color
    strip.show()

    time.sleep(1)

    reset_pixel_color()
    strip.show()
    
    setUP()
    time.sleep(2)

    # gps
    flag, data, data_with_dt = check_gps()

    if flag:
        strip.setPixelColor(LED2, blue)
        strip.show()
        logging.warning("GPS Found")
    else:
        strip.setPixelColor(LED2, red)
        strip.show()
        logging.warning("No GPS Found")

    # check if sd card is good
    try:
        statvfs = os.statvfs('/')
        # Calculate available space in bytes
        available_space = statvfs.f_bsize * statvfs.f_bavail
        # Convert to megabytes (1 MB = 1024*1024*1024 bytes)
        available_space_mb = available_space / (1024.0 ** 3)
    except Exception as e:
        logging.warning(f"Exception: {e}")
        available_space_mb = 0

    if available_space_mb < 1:
        logging.warning("Insuffcient Disk Space")
        strip.setPixelColor(LED0, red)
        strip.show()
    else:
        strip.setPixelColor(LED0, blue)
        strip.setPixelColor(LED1, blue)
        strip.show()

    lcd.write_string("Searching for Devices")
    logging.warning("Searching for Devices")

    is_i2c_present = is_device_present()

    if is_i2c_present:
        lcd.clear()
        lcd.write_string("I2C Device Found")
        logging.warning("I2C device found")
        time.sleep(1)

        #Open the file and print "Calibration: " text at the top
        #printTest()
        lcdCalib()
        strip.setPixelColor(LED2, orange)
        GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=300) 
        while not is_measurement_started:
            time.sleep(1)
            updateLights()
        is_measurement_started = False

        newDistance = LidarLiteV4().read_distance()

        if newDistance > 5 and newDistance < 4500:  
            calibration = newDistance  
            lcd.clear()
            lcd.write_string('Calibration Distance: ' + str(calibration) + ' cm')
            logging.warning("Calibration Distance: {} cm".format(calibration))
            strip.setPixelColor(LED2, blue) #Set third light to blue to indicate calibration measurement recorded
            strip.show()
            time.sleep(2)
        lcd.clear()
        #lcd.write_string("Start...")
        strip.setPixelColor(LED4, off) #Off the fifth light
        strip.show()
     
        try:
            while True:
                if is_measurement_started:
                    distance = LidarLiteV4().read_distance()
                    proces_measurement(distance, calibration)
                    logging.warning("Setting is_measurement_started to False")
                    is_measurement_started = False
                smart_delay(2)
                updateLights()
                update_screen()
        except Exception as e:
            logging.warning(f"Exception: {e}")
        finally:
            GPIO.cleanup()
    else:
        lcd.clear()
        lcd.write_string("ANT Device..")
        ant_measure = AntSendDemo()
        try:
            ant_measure.OpenChannel()  # start
        except Exception as e:
            logging.warning(f"Exception: {e}")
            logging.warning("Closing ANT+ Channel!")
        finally:
            logging.warning("Finally...")


