import rospy
import cv2
from sensor_msgs.msg import Image
from picamera import PiCamera
from picamera.array import PiRGBArray
from cv_bridge import CvBridge

camera = PiCamera()
bridge = CvBridge()

def setup():
    print('--> CAMERA node')
    rospy.init_node('camera_node')
    image_pub = rospy.Publisher('/img', Image, queue_size=10)

    def timer_callback(event):
        with PiRGBArray(camera) as stream:
            camera.capture(stream, format='bgr')
            
            # Convert BGR image to RGB
            rgb_image = cv2.cvtColor(stream.array, cv2.COLOR_BGR2RGB)

            image_data = bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
            rospy.loginfo('Publishing image')
            image_pub.publish(image_data)

    timer_period = 0.5  # seconds
    timer = rospy.Timer(rospy.Duration(timer_period), timer_callback)

    rospy.spin()

    if rospy.is_shutdown():
        timer.shutdown()
        rospy.loginfo('Shutting down: camera node')

if __name__ == '__main__':
    try:
        setup()
    except rospy.ROSInterruptException:
        pass
