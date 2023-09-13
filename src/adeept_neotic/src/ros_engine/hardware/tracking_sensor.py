import RPi.GPIO as GPIO
import time


class TrackingSensor:
    def __init__(self):
        self.LINE_PIN_RIGHT = 19
        self.LINE_PIN_MIDDLE = 16
        self.LINE_PIN_LEFT = 20

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LINE_PIN_RIGHT, GPIO.IN)
        GPIO.setup(self.LINE_PIN_MIDDLE, GPIO.IN)
        GPIO.setup(self.LINE_PIN_LEFT, GPIO.IN)

    def run(self):
        self.direction = ''
        self.turn = ''
        self.status_right = GPIO.input(self.LINE_PIN_RIGHT)
        self.status_middle = GPIO.input(self.LINE_PIN_MIDDLE)
        self.status_left = GPIO.input(self.LINE_PIN_LEFT)

        # Detect if the line has been sensed
        if self.status_middle == 1:
            self.direction = 'forward'
            return True
        elif self.status_left == 1:
            self.turn = 'left'
            return True
        elif self.status_right == 1:
            self.turn = 'right'
            return True
        else:
            self.direction = 'no'
            return False
