import threading
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA

led = True

rospy.init_node('controller')

def velocity_thread():
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo('stampa C')

    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 1.0
    velocity_pub.publish(vel)

def led_thread():
    led_pub = rospy.Publisher('/led', ColorRGBA, queue_size=10)
    rospy.loginfo('stampa L')

    global led
    color = ColorRGBA()
    if led:
        color.r = 1.0
    else:
        color.r = 0.0
    led = not led

    led_pub.publish(color)

def range_thread():
    range_sub = rospy.Subscriber('/range', Range, getDistance)
    rospy.loginfo('stampa R')

def getDistance(r: Range):
    print(str(r.range))

def controller_node():
    thread1 = threading.Thread(target=led_thread)
    thread2 = threading.Thread(target=velocity_thread)
    thread3 = threading.Thread(target=range_thread)

    thread1.start()
    thread2.start()
    thread3.start()

    thread1.join()
    thread2.join()
    thread3.join()

    rospy.spin()

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass
