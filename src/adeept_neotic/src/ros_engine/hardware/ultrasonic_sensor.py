import RPi.GPIO as GPIO
import time


class UltrasonicSensor:
    def __init__(self):
        # TRIG IO port
        self.TrigPin = 11  # Pin number of input terminal of ultrasonic module
        # ECHO IO port
        self.EchoPin = 8  # Pin number of output terminal of ultrasonic module

    def check_distance(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.TrigPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.EchoPin, GPIO.IN)
        # Set the input end of the module to high level and emit an initial sound wave
        GPIO.output(self.TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin, GPIO.LOW)
        # When the module no longer receives the initial sound wave
        while not GPIO.input(self.EchoPin):
            pass
        t1 = time.time()  # T1 -> time when the initial sound wave is emitted
        while GPIO.input(self.EchoPin):  # When the module receives the return sound wave
            pass
        t2 = time.time()  # T2 -> time when the return sound wave is captured
        distance = (t2-t1)*340/2
        return distance
