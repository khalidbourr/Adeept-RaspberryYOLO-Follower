import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import ColorRGBA, Bool

led = True
line = False

def getDistance(r: Range):
    print(str(r.range))

def getLine(b : Bool):
    global line
    line = b.data

def controller_node():
    rospy.init_node('controller', anonymous=True)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    led_pub = rospy.Publisher('/led', ColorRGBA, queue_size=10)
    range_sub = rospy.Subscriber('/range', Range, getDistance)
    line_sub = rospy.Subscriber('/line_found', Bool, getLine)

    def vel_callback(event):
        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = 1.0
        velocity_pub.publish(vel)

    def led_callback(event):
        global led
        color = ColorRGBA()
        if led:
            color.r = 1.0
        else:
            color.r = 0.0
        led = not led

        led_pub.publish(color)

    timer_period = 0.5  # seconds
    rospy.Timer(rospy.Duration(timer_period), vel_callback)
    rospy.Timer(rospy.Duration(timer_period), led_callback)

    rospy.spin()

    if KeyboardInterrupt:
        rospy.signal_shutdown('Shutting down: stopping controller')

if __name__ == '__main__':
    try:
        controller_node()
    except rospy.ROSInterruptException:
        pass

