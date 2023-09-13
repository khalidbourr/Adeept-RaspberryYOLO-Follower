import rospy
from sensor_msgs.msg import Range
import sys
sys.path.append('/home/bourr/Adeept_Neotic/adeept_ws/src/adeept_neotic/src')
from ros_engine.hardware.ultrasonic_sensor import UltrasonicSensor

ultrasonic = UltrasonicSensor()
dist = 0

def timer_callback(event, distance_pub, distance_data):
    dist = ultrasonic.check_distance()
    #print(dist)
    distance_data.range = dist
    distance_pub.publish(distance_data)

def ultrasonic_node():
    print('--> ULTRASONIC node')
    rospy.init_node('ultrasonic_sensor', anonymous=True)
    distance_pub = rospy.Publisher('/range', Range, queue_size=10)

    distance_data = Range()

    timer_period = rospy.Duration(0.5)  # seconds
    timer = rospy.Timer(timer_period, lambda event: timer_callback(event, distance_pub, distance_data))

    rospy.spin()

    if KeyboardInterrupt:
        timer.shutdown()


if __name__ == '__main__':
    try:
        ultrasonic_node()
    except rospy.ROSInterruptException:
        pass
