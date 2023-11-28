#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import board
import busio

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
# Global variables to store the data
color_data = None
tirette_data = None

def color_callback(data):
    global color_data
    color_data = data.data

def tirette_callback(data):
    global tirette_data
    tirette_data = data.data

def update_display():
    # Create blank image for drawing
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    if color_data is not None and tirette_data is not None:
        display_text = f"Color: {color_data}\nTirette: {tirette_data}"
    else:
        display_text = "Waiting..."

    draw.text((0, 0), display_text, font=font, fill=255)

    # Display image
    oled.image(image)
    oled.show()

def listener():
    rospy.init_node('display_node', anonymous=True)
    rospy.Subscriber('/color', Bool, color_callback)
    rospy.Subscriber('/tirette', Bool, tirette_callback)

    while not rospy.is_shutdown():
        update_display()
        rospy.sleep(1)
    exit()

if __name__ == '__main__':
    listener()

