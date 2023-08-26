import RPi.GPIO as GPIO
import time
import board
import busio
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd

# Import the LIDAR-Lite v4 library
import adafruit_lidarlite

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
BUTTON_PIN = 17  # GPIO17
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Create LIDAR-Lite v4 object
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


def read_distance():
    distance = lidar.distance
    return distance

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
