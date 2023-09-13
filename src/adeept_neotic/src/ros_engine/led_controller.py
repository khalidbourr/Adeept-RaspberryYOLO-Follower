import pathlib
import os
import rospy
from std_msgs.msg import ColorRGBA

# Get current working path
path = str(pathlib.Path(__file__).parent.absolute())

# LEDs need to be managed as super users
def call_script(R, G, B):
    cmd = 'python3 ' + path + '/hardware/led_strip.py ' + \
        str(R) + ' ' + str(G) + ' ' + str(B)
    os.popen("sudo -S %s" % (cmd), 'w')

def manageLed(colorMsg):
    R = colorMsg.r
    G = colorMsg.g
    B = colorMsg.b

    call_script(R, G, B)

def led_actuator_node():
    print('--> LED node')
    rospy.init_node('led_actuator', anonymous=True)
    rospy.Subscriber('/led', ColorRGBA, manageLed)

    rospy.spin()

    if KeyboardInterrupt:
        call_script(0, 0, 0)
        rospy.signal_shutdown('Shutting down: stopping LEDs')

if __name__ == '__main__':
    try:
        led_actuator_node()
    except rospy.ROSInterruptException:
        pass
