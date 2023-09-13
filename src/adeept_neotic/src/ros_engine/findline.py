import rospy
from std_msgs.msg import Bool
import sys
sys.path.append('/home/bourr/Adeept_Neotic/adeept_ws/src/adeept_neotic/src')
from ros_engine.hardware.tracking_sensor import TrackingSensor

track_sensor = TrackingSensor()

def findline_sensor_node():
    print('--> LINE node')

    rospy.init_node('findline_sensor', anonymous=True)
    line_pub = rospy.Publisher('/line_found', Bool, queue_size=10)

    line = Bool()

    def timer_callback(event):
        line.data = track_sensor.run()
        line_pub.publish(line)

    timer_period = 0.5  # seconds
    rospy.Timer(rospy.Duration(timer_period), timer_callback)

    rospy.spin()

    if KeyboardInterrupt:
        rospy.signal_shutdown('Shutting down: stopping line sensor')

if __name__ == '__main__':
    try:
        findline_sensor_node()
    except rospy.ROSInterruptException:
        pass
