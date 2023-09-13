#!/usr/bin/env python3

import rospy
# import cv2
# import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import ColorRGBA
import random

class ObjectFollower:

    def __init__(self):
        self.image_width = 1280
        self.image_height = 720
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bounding_box_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.callback)
        self.last_seen_angle = None  # Remember where we last saw the person
        self.turn_direction = 1  # Start by turning to the right, you can change to -1 for left
        self.recovery_attempts = 0  # Count recovery attempts
        self.base_recovery_speed = 0.001  # Base speed for recovery rotation
        self.in_recovery = False
        
        
        # Temporarily subscribe to /img to get image dimensions
        self.image_sub = rospy.Subscriber('/img', Image, self.image_callback)
        
        # Green light
        self.led_pub = rospy.Publisher('/led', ColorRGBA, queue_size=10)
         
    def turn_on_led(self):
        led = ColorRGBA()
        led.r = 0.0
        led.g = 1.0
        led.b = 0.0
        led.a = 1.0
        self.led_pub.publish(led)
        
    
    def image_callback(self, image):
        self.image_width = image.width
        self.image_height = image.height
        self.image_sub.unregister()  # Unregister from /img topic after getting dimensions
        
    def start_recovery(self):
        # If not already in recovery mode
        if not self.in_recovery:
            self.in_recovery = True
            self.recovery_start_time = rospy.Time.now()

            # Stop robot for stabilization
            cmd = Twist()
            cmd.angular.z = 0
            rospy.loginfo("No person detected. Stopping robot for stabilization.")
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(4)  # Wait to stabilize
            print(self.last_seen_angle)

            # Start recovery rotation
            if self.last_seen_angle:
                # Turn to the last seen position of the person
                recovery_speed = self.base_recovery_speed * (self.last_seen_angle / 180)  # This assumes last_seen_angle is in degrees
                print(recovery_speed)
            else:
                # Else rotate at base speed
                recovery_speed = self.base_recovery_speed

            # Occasionally switch rotation direction for dynamic search
            if self.recovery_attempts % 3 == 0:
                self.turn_direction *= -1

            cmd.angular.z = recovery_speed * self.turn_direction
            rospy.loginfo(f"Starting recovery rotation.")
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(2)  # Rotate for a moment to search
            
            # Increment recovery attempts
            self.recovery_attempts += 1
    def stop_recovery(self):
        self.in_recovery = False
        self.recovery_start_time = None
        self.recovery_attempts = 0  # Reset attempts when recovery ends

    def update_last_seen(self, angle):
        self.last_seen_angle = angle  
        
     
    def calculate_angle_to_person_from_box_position(self, box_center_x):
        # Find the offset from the center
        offset_x = box_center_x - self.image_width / 2

        # Calculate angle using a simple linear proportion
        # Assuming a camera FOV of about 60 degrees (this might differ based on your camera, so adjust accordingly)
        FOV = 60.0
        angle_offset = (offset_x / self.image_width) * FOV

        return angle_offset
     
                  
    
    def callback(self, bounding_boxes):
        self.turn_on_led()
        person_detected = False
        Kp = 0.001  # Proportional gain for turning
        Kp_size = 0.001  # Proportional gain for linear speed
        Kd = 0.0001
        min_angular_speed = -0.5  # Minimum angular speed
        max_angular_speed = 0.5  # Maximum angular speed
        min_linear_speed = 0  # Minimum linear speed
        max_linear_speed = 1.0  # Maximum linear speed
        desired_size = 100  # Replace with your desired size
        dead_zone = 100
        prev_offset_x = 0
        base_recovery_speed = 0.001
        recovery_angular_speed = random.choice([-1, 1]) * base_recovery_speed

    
        for box in bounding_boxes.bounding_boxes:
            if box.Class == "person" and box.probability >= 0.3:
                person_detected = True
            
            # Calculate how off-center the target is
                offset_x = box.center_x - (self.image_width // 2)
                print(f"offset_x:{offset_x}")
                actual_size = box.xmax - box.xmin  # Assuming you have this info in box
                print(f"actual_size:{actual_size}")
            # Proportional control for angular and linear speed
                alpha = 0.5
                smoothed_offset_x = alpha * offset_x + (1 - alpha) * prev_offset_x 

                angular_speed = Kp * smoothed_offset_x + Kd * (smoothed_offset_x - prev_offset_x) if abs(smoothed_offset_x) > dead_zone else 0
                prev_offset_x = offset_x
                linear_speed = Kp_size * abs(desired_size - actual_size)
            # Safety bounds
                angular_speed = max(-max_angular_speed, min(max_angular_speed, angular_speed))
                linear_speed = max(min_linear_speed, min(max_linear_speed, linear_speed))

            
            # Generate cmd_vel message based on offsets
                cmd = Twist()
                cmd.linear.x = linear_speed
                cmd.angular.z = angular_speed
                rospy.loginfo(f"Linear Speed: {cmd.linear.x}, Angular Speed: {cmd.angular.z}")
            
            # Publish cmd_vel message to move/rotate the robot
                self.cmd_vel_pub.publish(cmd)
                break  # Exit after the first person is found

        if not person_detected:
            #self.start_recovery()
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
             
        #else:
         #   self.stop_recovery()
            # Assuming you can determine the angle from the bounding box's position
          #  angle_to_person = self.calculate_angle_to_person_from_box_position(box.center_x)
           # self.update_last_seen(angle_to_person)
                                 







if __name__ == '__main__':
    rospy.init_node('object_follower')
    object_follower = ObjectFollower()
    rospy.spin()

