import RPi.GPIO as GPIO
import time
import board
import busio
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd
import adafruit_bus_device.i2c_device as i2c_device
import ranging

# Import the LIDAR-Lite v4 library
import adafruit_lidarlite

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
BUTTON_PIN = 17  # GPIO17
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Create LIDAR-Lite v4 object
I2C_ADDRESS = 0x62  # Replace with the actual address
lidar = adafruit_lidarlite.LIDARLite(i2c)

# Initialize LCD
lcd_columns = 16
lcd_rows = 2

# Pin assignments for the Raspberry Pi
lcd_rs = digitalio.DigitalInOut(board.D5)
lcd_en = digitalio.DigitalInOut(board.D6)
lcd_d4 = digitalio.DigitalInOut(board.D13)
lcd_d5 = digitalio.DigitalInOut(board.D19)
lcd_d6 = digitalio.DigitalInOut(board.D26)
lcd_d7 = digitalio.DigitalInOut(board.D12)

# Initialize the LCD object
lcd = characterlcd.Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
)

# Clear the display
lcd.clear()

# Display a message
lcd.message = "Hello, World!\nRaspberry Pi"

# Wait for a few seconds
time.sleep(3)

# Clear the display
lcd.clear()

### import for ant
from openant.easy.node import Node
from openant.easy.channel import Channel

# Definition of Variables
NETWORK_KEY = [0xE8, 0xE4, 0x33, 0xA9, 0xDD, 0x56, 0xC1, 0x43]
Device_Type = 16  # 124 = Stride & Distance Sensor
Device_Number = 12345  # Change if you need.
Channel_Period = 8192
Channel_Frequency = 66

##########################################################################


class AntSendDemo:
    def __init__(self):

        self.ANTMessageCount = 0
        self.ANTMessagePayload = [0, 0, 0, 0, 0, 0, 0, 0]

    def Create_Next_DataPage(self):
        # Define Variables
        self.ANTMessageCount += 1
        
        self.ANTMessagePayload = [0x31, self.ANTMessageCount, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
            # ANTMessageCount reset
        if self.ANTMessageCount > 131:
            self.ANTMessageCount = 0

        return self.ANTMessagePayload

    # RX Event
    def on_data(self, data):
        
        self.Create_Next_DataPage()
        print(self.ANTMessagePayload)
        self.channel.send_acknowledged_data(self.ANTMessagePayload) 
        page = data[0]
        if page == 80: # manufacturer id
            print("Manifacturer data ")
        elif page == 81:
            print("Acknowledgement ")
        elif (page & 0x0F) <= 7:
            distance = data[6]
            print(f"distance : {distance} cm")

        print(f"on_data: {data}")
        #self.ActualTime = time.time() - self.TimeProgramStart


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
        # Callback function for each RX event

        GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=self.on_data, bouncetime=200)

        # button_state = GPIO.input(BUTTON_PIN)
        # if button_state == GPIO.LOW:
        #     self.channel.on_broadcast_data = self.on_data
        #     self.channel.on_burst_data = self.on_data

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

def read_distance():
    distance = lidar.distance
    return distance


def i2c_measurement():
    try:
        while True:
            button_state = GPIO.input(BUTTON_PIN)
            if button_state == GPIO.LOW:
                distance = read_distance()
                if distance is not None:
                    distance_cm = distance / 100.0
                    print("Distance:", distance_cm, "cm")
                    lcd.clear()
                    lcd.message("Distance: {:.2f} cm".format(distance_cm))
                time.sleep(0.1)  # Debounce the button
    except KeyboardInterrupt:
        GPIO.cleanup()


if __name__=="__main__":
    flag = False  #if i2c device not found check for ANT
    try:
        with i2c_device.I2CDevice(i2c, I2C_ADDRESS):
            print("Device found at address: 0x{:02X}".format(I2C_ADDRESS))
            flag = True
    except:
        print("No bus device found")

    #Logic for getting data over ant
    if not flag:
        print("ANT+ Ranging Demo")
        ant_senddemo = AntSendDemo()

        try:
            ant_senddemo.OpenChannel()  # start
        except KeyboardInterrupt:
            print("Closing ANT+ Channel!")
        finally:
            print("Finally...")

        print("Close demo...")
    

    
