import smbus
import RPi.GPIO as GPIO
import time
import datetime
import os
import logging
import psutil
import serial

import threading
from queue import Queue
from rpi_ws281x import PixelStrip, Color
import atexit
import signal
import sys

from openant.easy.node import Node
from openant.easy.channel import Channel

last_button_press_time = 0

TIMEOUT_THRESHOLD = 10

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
# Define your NeoPixel setup
LED_COUNT = 8
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 5
LED_BRIGHTNESS = 255
LED_INVERT = False
LED_CHANNEL = 0

# Create NeoPixel object
pixels = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
pixels.begin()

# Thread-safe queue for communication
pixel_queue = Queue()

# Flag to signal the thread to stop
stop_thread_flag = threading.Event()

def turn_off_all():
    for i in range(LED_COUNT):
        pixels.setPixelColor(i, Color(0, 0, 0))  # Turn off all pixels
    pixels.show()

# Function to run in a separate thread for NeoPixel control
def neo_pixel_thread():
    while not stop_thread_flag.is_set():
        # Check the queue for commands
        while not pixel_queue.empty():
            command = pixel_queue.get()
            if command['action'] == 'set_pixel':
                pixels.setPixelColor(command['pixel'], command['color'])
            elif command['action'] == 'clear_pixel':
                pixels.setPixelColor(command['pixel'], Color(0, 0, 0))  # Turn off the pixel
            pixel_queue.task_done()

        # Your NeoPixel control logic goes here
        # For example, you can run animations or other effects
        pixels.show()
        time.sleep(0.1)  # Adjust as needed

# Function to stop the NeoPixel thread
def stop_neo_pixel_thread():
    stop_thread_flag.set()
    neo_pixel_thread.join()  # Wait for the thread to finish
    for i in range(LED_COUNT):
        pixels.setPixelColor(i, Color(0, 0, 0))  # Turn off all pixels
    pixels.show()
    #GPIO.cleanup()
    sys.exit(0)  # Exit the program

# Function to handle KeyboardInterrupt
def handle_keyboard_interrupt(signum, frame):
    print("\nReceived KeyboardInterrupt. Cleaning up and exiting.")
    stop_neo_pixel_thread()

# Register the cleanup function with atexit
atexit.register(stop_neo_pixel_thread)

# Register the KeyboardInterrupt handler
signal.signal(signal.SIGINT, handle_keyboard_interrupt)

# Create a thread for NeoPixel control
neo_pixel_thread = threading.Thread(target=neo_pixel_thread)

# Start the thread
neo_pixel_thread.start()

# Function to turn on a specific pixel
def turn_on_pixel(pixel, color):
    pixel_queue.put({'action': 'set_pixel', 'pixel': pixel, 'color': color})

# Function to turn off a specific pixel
def turn_off_pixel(pixel):
    pixel_queue.put({'action': 'clear_pixel', 'pixel': pixel})


red = Color(255, 0, 0)
orange = Color(255, 128, 0)
yellow = Color(255, 255, 0)
green =  Color(0, 255, 0)
light_blue = Color(0, 255, 255)
blue =  Color(0, 0, 255)
off = Color(0, 0, 0)


# #Function to Turn off all pixels
# def reset_pixel_color():
#     for i in range(LED_COUNT):
#         turn_on_pixel(i, Color(0, 0, 0))

# # Function to set the color of a specific pixel
# def set_pixel_color(pixel_index, color):
#     turn_on_pixel(pixel_index, color)

# # Function to update the LEDs
# def show_leds():
#     


# Define the base path where measurement folders will be created
base_path = "/home/kalyan/hydrodata"  # Replace with your desired base path
base_path_log = "/home/kalyan/logs"

from RPLCD.i2c import CharLCD
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=16, rows=2, dotsize=8)
lcd.clear()

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


# Function to write measurements to a file in a folder based on the year and month
def write_measurement_to_file(measurement, gps_data, calibration):
    now = datetime.datetime.now()
    year, month = now.year, now.month
    folder_name = create_folder(base_path, year, month)
    datet = datetime.datetime.now().strftime("%Y-%m-%d")
    current_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    filename = f"{folder_name}/measurement_{datet}.txt"
    
    with open(filename, "a") as file:
        file.write(f"Timestamp: {current_date},")
        file.write(f"Measurement: {measurement - calibration} cm, ")
        file.write(f"latitude: {gps_data['latitude']} cm")
        file.write(f"longitude: {gps_data['longitude']} cm")
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
    print("PB pressed")

    # Check if it has been more than 0.5 seconds since the last button press
    if (current_time - last_button_press_time) > 1:
        is_measurement_started = True
        turn_on_pixel(LED4, orange)
        
        last_button_press_time = current_time
    logging.warning("Button Pushed")


def setUP():
    #lcd.write_bytes(0x08)
    lcd.write_string("Snow Depth Probe")
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
    # Define the serial port where the GPS device is connected
    serial_port = "/dev/serial0"  # Replace with the actual serial port device
    # Initialize the last_fix_time variable
    last_fix_time = time.time()
    try:
        # Open the serial port
        with serial.Serial(serial_port, baudrate=115200, timeout=1) as ser:
            while True:
                # Read a line of data from the GPS device
                sentence = ser.readline().decode("utf-8").strip()

                if check_timeout(last_fix_time):
                    #lcd.write_string("No GPS Found")
                    return False, None

                if sentence.startswith("$GPGGA"):
                    # Split the sentence into fields
                    fields = sentence.split(',')

                    # Extract fix quality (position fix indicator)
                    fix_quality = int(fields[6])

                    # Check if fix quality is valid (1 or 2 typically indicates a valid fix)
                    if fix_quality in [1, 2]:
                        # Extract other relevant information if needed
                        # For example: latitude, longitude, altitude
                        latitude = fields[2]
                        longitude = fields[4]
                        altitude = fields[9]

                        # Return True along with the GPS data
                        return True, {
                            "latitude": latitude,
                            "longitude": longitude,
                            "altitude": altitude
                        }
                    else:
                        # Return False if no valid GPS fix
                        logging.warning("No correct GPS String")
                        return False, None
    except serial.SerialException:
        logging.warning("Error in GPS")
        return False, None


def lcdDepth(distance):
    depth = calibration - distance
    # Display the distance on the LCD:
    lcd.cursor_pos = (0, 0) #Set the cursor to column 1, line 1
    lcd.write_string(" Depth  at ")
    #lcd.print(count)
    lcd.cursor_pos = (1, 5)
    lcd.write_string(str(depth)) #Prints the measured snow depth
    lcd.write_string(" cm") #Prints "m" on the LCD
    time.sleep(4)

if __name__ == "__main__":
    now = datetime.datetime.now()
    year, month = now.year, now.month
    folder_name = create_folder(base_path_log, year, month )
    datet = datetime.datetime.now().strftime("%Y-%m-%d")
    current_date = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    logfileName = f"{folder_name}/measurement_{datet}.txt"
    logging.basicConfig(
        filename=logfileName, level=logging.DEBUG,   format="%(asctime)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
    )  # just for Debugging purpose, outcomment this in live version

    logger = logging.getLogger("Otto")
    lcd.clear()
    setUP()
    time.sleep(4)

    # gps
    flag, data = check_gps()

    if flag:
        logging.warning("GPS Found")
    else:
        turn_on_pixel(LED7, red)
        
        logging.warning("No GPS Found")

    # lcd strip

    # check if sd card is good
    disk = psutil.disk_usage('/')
    required_free_space = 1 * 1024 * 1024 * 1024  # 1 GB in bytes

    if disk.free < required_free_space:
        logging.warning("Insuffcient Disk Space")
        turn_on_pixel(LED0, red)
        
    else:
        turn_on_pixel(LED0, blue)
        
        
    lcd.write_string("Searching for Devices")
    print("Searching for Devices")

    is_i2c_present = is_device_present()

    if is_i2c_present:
        lcd.clear()
        lcd.write_string("I2C Device Found")
        print("I2C device found")
        time.sleep(2)

        #Open the file and print "Calibration: " text at the top
        #printTest()
        lcdCalib()
        GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=300)
        #GPIO.wait_for_edge(button_pin, GPIO.FALLING)  # wait for the button press to calibrate
        while not is_measurement_started:
            pass
        is_measurement_started = False

        newDistance = LidarLiteV4().read_distance()

        if newDistance > 5 and newDistance < 4500:  
            calibration = newDistance  
            lcd.clear()
            lcd.write_string('Distance: ' + str(calibration) + ' cm')
            logging.warning("Calibration Distance: {} cm".format(calibration))
            turn_on_pixel(LED2, blue) #Set third light to blue to indicate calibration measurement recorded
            
            time.sleep(2)


        # # Setup GPIO
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        lcd.clear()
        lcd.write_string("Start...")
        # start measurement after calibration 
        try:
            # GPIO.add_event_detect(button_pin, GPIO.RISING, callback=button_callback, bouncetime=200)
            while True:
                if is_measurement_started:
                    logging.warning("I am in loop")
                    distance = LidarLiteV4().read_distance()
                    # check if the measurement is within the bounds
                    if distance > 5 and  distance < 4500:
                        turn_off_all()
                        turn_on_pixel(LED5, green) #Set sixth light to green to indicate valid distance
                        
                        time.sleep(0.1)
                        lcd.clear()
                        lcd.write_string("Distance: " + str(distance) + "cm")
                        time.sleep(0.1)
                        gps_flag, gps_data = check_gps()

                        if gps_flag:
                            turn_off_all()
                            turn_on_pixel(LED6, green) #Set seventh light to green to indicate GPS coordinates are valid
                            write_measurement_to_file(distance, gps_data, calibration)
                            #print("Distance: {} cm".format(distance))
                        else:
                            turn_on_pixel(LED4, red) #Set seventh light to red to indicate failure to log coordinates and distance
                            turn_on_pixel(LED7, red)
                            
                    else:
                        turn_on_pixel(LED5, orange)
                        turn_on_pixel(LED6, orange)
                        turn_on_pixel(LED7, orange)
                        
                    logging.warning("Reset is_measurement_started to False")
                    is_measurement_started = False
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nReceived KeyboardInterrupt. Cleaning up and exiting.")
            neo_pixel_thread.join()  # Wait for the thread to finish
        finally:
            GPIO.cleanup()