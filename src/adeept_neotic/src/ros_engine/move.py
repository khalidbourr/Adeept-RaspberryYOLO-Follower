import rospy
from geometry_msgs.msg import Twist
import sys
sys.path.append('/home/bourr/Adeept_Neotic/adeept_ws/src/adeept_neotic/src')
from ros_engine.hardware.motors import Motors


motors = Motors()

def move(twistMsg):
    turn = ''
    lin_vel = 0
    direction = 'no'
    lin_vel = twistMsg.linear.x
    ang_vel = twistMsg.angular.z
    if lin_vel < 0.8:
        speed = abs(0.8 * 100)
    else:
        speed = abs(lin_vel * 100)
    if lin_vel > 0:
        direction = 'forward'
    elif lin_vel < 0:
        direction = 'backward'
    if ang_vel > 0:
        turn = 'right'
    elif ang_vel < 0:
        turn = 'left'
    elif lin_vel == 0 and ang_vel == 0:
        direction = 'no'
    motors.movement(direction, turn, speed)


def movement_actuator_node():
    print('--> MOTOR node')
    motors.motor_init()

    rospy.init_node('movement_actuator', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, move)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down: stopping motors due to KeyboardInterrupt")
        motors.motor_stop()
        pass

if __name__ == '__main__':
    movement_actuator_node()
