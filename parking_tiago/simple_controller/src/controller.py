#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import atan2
import time
from std_srvs.srv import Empty



def move(speed, distance, is_forward):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 


        #if (speed > 0.4):
            #print ('speed must be lower than 0.4')
            #return

        if (is_forward):
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        t0 = rospy.Time.now().to_sec()

        while True :
                rospy.loginfo("Tiago moves forwards")
                velocity_publisher.publish(velocity_message)

                loop_rate.sleep()
                t1 =  rospy.Time.now().to_sec()
                #rospy.Duration(1.0)
                
                distance_moved = (t1-t0) * speed
                print  ("distance_moved")               
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
    
def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    

    velocity_message = Twist()
    velocity_message.linear.x=0
    velocity_message.linear.y=0
    velocity_message.linear.z=0
    velocity_message.angular.x=0
    velocity_message.angular.y=0
    velocity_message.angular.z=0

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z = -abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Tiago rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()

        print ('current_angle_degree: ',current_angle_degree)
                       
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0 
    velocity_publisher.publish(velocity_message)



if __name__ == '__main__':
    try:
        
        rospy.init_node('Tiago_motion_pose')
        time.sleep(1.0)
        rotate (90, 90 , True)
        move(0.5,3.0,True)
        rotate (90, 90 , False)
        move(0.5,10.0,True)
        rotate (90, 90, True)
        move(0.5,2.5,True)


       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")   