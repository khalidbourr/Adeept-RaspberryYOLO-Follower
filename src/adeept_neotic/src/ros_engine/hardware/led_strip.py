from rpi_ws281x import *
import argparse
import sys
import RPi.GPIO as GPIO


# LED strip configuration:
LED_COUNT = 16      # Number of LED pixels.
LED_PIN = 12      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
# True to invert the signal (when using NPN transistor level shift)
LED_INVERT = False
LED_CHANNEL = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# Get colors from LED node --> RGB message
parser = argparse.ArgumentParser()
parser.add_argument("RED", type=float, help="value of red color")
parser.add_argument("GREEN", type=float, help="value of green color")
parser.add_argument("BLUE", type=float, help="value of blue color")

args = parser.parse_args()


def set_color(red, green, blue):
    color = Color(red, green, blue)
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()


if __name__ == '__main__':

    if args.RED and args.GREEN and args.BLUE > 1.0:
        print('ERROR: maximum value is 1.0')
        sys.exit(1)
    else:
        RED = int(float(args.RED)*255)
        GREEN = int(float(args.GREEN)*255)
        BLUE = int(float(args.BLUE)*255)

    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ,
                              LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)

    # Intialize the library (must be called once before other functions).
    strip.begin()

    set_color(RED, GREEN, BLUE)