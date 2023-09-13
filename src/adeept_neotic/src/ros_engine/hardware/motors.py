import RPi.GPIO as GPIO


class Motors:
    def __init__(self):
        self.Motor_A_EN = 4
        self.Motor_B_EN = 17

        self.Motor_A_Pin1 = 26
        self.Motor_A_Pin2 = 21
        self.Motor_B_Pin1 = 27
        self.Motor_B_Pin2 = 18

        self.Dir_forward = 1
        self.Dir_backward = 0

        self.left_forward = 1
        self.left_backward = 0

        self.right_forward = 0
        self.right_backward = 1

        self.pwn_A = 0
        self.pwm_B = 0

    def motor_stop(self):  # Motor stops
        GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
        GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
        GPIO.output(self.Motor_A_EN, GPIO.LOW)
        GPIO.output(self.Motor_B_EN, GPIO.LOW)

    def motor_init(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Motor_A_EN, GPIO.OUT)
        GPIO.setup(self.Motor_B_EN, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_A_Pin2, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin1, GPIO.OUT)
        GPIO.setup(self.Motor_B_Pin2, GPIO.OUT)

        self.motor_stop()

        try:
            self.pwm_A = GPIO.PWM(self.Motor_A_EN, 1000)
            self.pwm_B = GPIO.PWM(self.Motor_B_EN, 1000)
        except:
            pass

    def motor_left(self, status, direction, speed):  # Motor 2 positive and negative rotation
        if speed > 100:  # to avoid problems with duty cycle
            speed = 100
        if status == 0:  # stop
            GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_B_EN, GPIO.LOW)
        else:
            if direction == self.Dir_backward:
                GPIO.output(self.Motor_B_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_B_Pin2, GPIO.LOW)
                self.pwm_B.start(100)
                self.pwm_B.ChangeDutyCycle(speed)
            elif direction == self.Dir_forward:
                GPIO.output(self.Motor_B_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_B_Pin2, GPIO.HIGH)
                self.pwm_B.start(0)
                self.pwm_B.ChangeDutyCycle(speed)

    # Motor 1 positive and negative rotation
    def motor_right(self, status, direction, speed):
        if speed > 100:  # to avoid problems with duty cycle
            speed = 100
        if status == 0:  # stop
            GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
            GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
            GPIO.output(self.Motor_A_EN, GPIO.LOW)
        else:
            if direction == self.Dir_forward:
                GPIO.output(self.Motor_A_Pin1, GPIO.HIGH)
                GPIO.output(self.Motor_A_Pin2, GPIO.LOW)
                self.pwm_A.start(100)
                self.pwm_A.ChangeDutyCycle(speed)
            elif direction == self.Dir_backward:
                GPIO.output(self.Motor_A_Pin1, GPIO.LOW)
                GPIO.output(self.Motor_A_Pin2, GPIO.HIGH)
                self.pwm_A.start(0)
                self.pwm_A.ChangeDutyCycle(speed)
        return direction

    def movement(self, direction, turn, speed):
        radius=0.6
        if direction == 'forward':
            if turn == 'right':
                self.motor_left(0, self.left_backward, int(speed*radius))
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(0, self.right_backward, int(speed*radius))
            else:
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_forward, speed)
        elif direction == 'backward':
            if turn == 'right':
                self.motor_left(0, self.left_forward, int(speed*radius))
                self.motor_right(1, self.right_backward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(0, self.right_forward, int(speed*radius))
            else:
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_backward, speed)
        elif direction == 'no':
            if turn == 'right':
                self.motor_left(1, self.left_backward, speed)
                self.motor_right(1, self.right_forward, speed)
            elif turn == 'left':
                self.motor_left(1, self.left_forward, speed)
                self.motor_right(1, self.right_backward, speed)
            else:
                self.motor_stop()
        else:
            pass
