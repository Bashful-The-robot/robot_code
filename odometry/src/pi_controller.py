#!/usr/bin/env python3

import rospy
import numpy
from robp_msgs.msg import DutyCycles
from robp_msgs.msg import Encoders
from geometry_msgs.msg import Twist 

#we define some constants
Kpa=0.02
Kpb=0.02
Kia=0.03
Kib=0.03
r=0.04921 #radious of the wheel
b=0.3 #distance between the wheels
f=20 #desired frequency in Hz
ticks=3072 #number of ticks per rev of the encoder

class CartesianController:
    
    def __init__(self):
        #we initialize some variables
        self.estimated_w_left=0
        self.estimated_w_right=0
        self.desired_w_left=0
        self.desired_w_right=0
        self.integral_error_left=0
        self.integral_error_right=0
        self.last_time=rospy.get_time()

        #we create the subscribers and the publishers
        rospy.Subscriber('/motor/encoders',Encoders,self.encoders_callback)
        rospy.Subscriber('/motor_controller/twist',Twist,self.twist_callback)
        self.duty_cycles_pub=rospy.Publisher('/motor/duty_cycles',DutyCycles)

    
    def twist_callback(self,msg):
        #we get the linear and angular velocity
        v=msg.linear.x
        w=msg.angular.z

        #we estimate the desired angular velocity of each wheel
        r=0.04921 #radious of the wheel
        b=0.3 #distance between the wheels
        self.desired_w_left=(2*v - w*b)/(2*r)
        self.desired_w_right=(2*v + w*b)/(2*r)
        
    
    def encoders_callback(self,msg):
        #we get the values of the encoder
        delta_encoder_left=msg.delta_encoder_left
        delta_encoder_right=msg.delta_encoder_right

        self.estimated_w_left=(2*numpy.pi*r*f*delta_encoder_left)/(ticks*r)
        self.estimated_w_right=(2*numpy.pi*r*f*delta_encoder_right)/(ticks*r)

        #now we calculate the errors
        left_error=self.desired_w_left-self.estimated_w_left
        right_error=self.desired_w_right-self.estimated_w_right
        current_time=rospy.get_time()
        dt=current_time-self.last_time
        self.last_time=current_time

      
        self.integral_error_left = self.integral_error_left + left_error*dt
        self.integral_error_right =self.integral_error_right + right_error*dt
        

        #we calculate now the duty-cycle for each wheel 
  
        duty_left = min(max(Kpa*left_error + Kia*self.integral_error_left, -1), 1)
        duty_right = min(max(Kpb*right_error + Kib*self.integral_error_right, -1), 1)

        #we publish the duty cycle
        duty_msg=DutyCycles()
        duty_msg.duty_cycle_left=duty_left
        duty_msg.duty_cycle_right=duty_right
        self.duty_cycles_pub.publish(duty_msg)



if __name__ == '__main__':

    rospy.init_node('cartesian_controller')

    controller=CartesianController()

    rate=rospy.Rate(10)
    
    while not rospy.is_shutdown():

        rate.sleep()




