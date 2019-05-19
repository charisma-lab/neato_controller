#!/usr/bin/env python

import rospy
import roslib
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 
from robo_serial.msg import robo_base_command

#############################################################
#############################################################
class TwistToMotors():
#############################################################
#############################################################

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.49)
        self.pub_motor = rospy.Publisher('roboBaseSub', roboCommand, queue_size=10)
        rospy.Subscriber('cmd_vel_mux/input/navi', Twist, self.twistCallback)
    
        self.cmdVel = robo_base_command()
        self.rate = rospy.get_param("~rate",20)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        self.dx = 0 
        self.dr = 0
        self.dy = 0
        self.velfactor=rospy.get_param("~mps_to_rpm", 152.18)
        self.ramping_enable = rospy.get_param("~ramping_enable", False)
        
        self.prev_left_vel=0
        self.prev_right_vel=0
        self.ramp_rate=rospy.get_param("~ramp_rate",0.3)      #unit in m/s^2        
        
    #############################################################
    def spin(self):
    #############################################################
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(20)
        then = rospy.Time.now()
        self.ticks_since_target = self.timeout_ticks
        ###### main loop  ######
        while not rospy.is_shutdown():
            while not rospy.is_shutdown(): # and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
    
    #############################################################
    ########################################################################### 
    def ramp_velocity(self,target_vel,previous_vel,ramp_rate):
        if (target_vel>=previous_vel):
	    sign=1
        else:
            sign=-1
        step_size=ramp_rate/self.rate
        delta=math.fabs(target_vel-previous_vel)
        if delta>=step_size:
	    command_vel=previous_vel+sign*step_size
        else:
            command_vel=target_vel
        return command_vel
             	
    #############################################################
    def spinOnce(self):
    #############################################################

        self.left = 1.0 * self.dx - self.dr * self.w / 2
        self.right = 1.0 * self.dx + self.dr * self.w / 2
        ###RAMPING SECTION#########
        if self.ramping_enable:
            self.prev_left_vel=self.ramp_velocity(self.left,self.prev_left_vel,self.ramp_rate)
            self.prev_right_vel=self.ramp_velocity(self.right,self.prev_right_vel,self.ramp_rate)
            self.cmdVel.LeftMotorTarget =int(self.prev_left_vel*self.velfactor)	
            self.cmdVel.RightMotorTarget =int(self.prev_right_vel*self.velfactor)
        ###########################
        
        self.cmdVel.LeftMotorTarget =int(self.left*self.velfactor)	
        self.cmdVel.RightMotorTarget =int(self.right*self.velfactor)
        #############################################################
       
        self.pub_motor.publish(self.cmdVel)
        self.ticks_since_target += 1

    def twistCallback(self,msg):
    #############################################################
        # rospy.loginfo("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    twistToMotors = TwistToMotors()
    twistToMotors.spin()
