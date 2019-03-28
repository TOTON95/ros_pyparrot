#!/usr/bin/env python3.6
import rospy
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from pyparrot.Minidrone import Mambo

import _thread

#Variable that will store the parameters of the mambo drone
mamboAdd = "e0:14:60:5c:3d:c7"
wifi = False
retries = 12 

#The mambo object
mambo = Mambo("",use_wifi=wifi)

#Checks if the drone is able to communicate with ROS
success = False

#Variables of the drone
tko = False
land = False
cannon = False
auto_tko = False
linX = 0
linY = 0 
Alt = 0 
Hdg = 0

#Sends the spin function to another thread 
def spin_th(name,envi):
    rospy.spin()

#Callback of the land command
def cb_land(data):
    global land
    rospy.loginfo("\n" + rospy.get_name() + " Land!!\n")
    land = True
    #mambo.safe_land(4)

#Callback of the take-off command
def cb_take_off(data):
    global tko
    rospy.loginfo("\n" + rospy.get_name() + " Take-Off!!\n")
    tko = True
    #mambo.safe_takeoff(4)

#CAllback of the velocities
def cb_cmd_vel(data):
    global linX
    global linY
    global Alt
    global Hdg
    rospy.loginfo(rospy.get_name() + "\n Linear_X: %s Linear_Y: %s Linear_Z: %s Angular_Z: %s",data.linear.x,data.linear.y,data.linear.z,data.angular.z)
    linX = data.linear.x
    linY = data.linear.y
    Alt = data.linear.z
    Hdg = data.angular.z
    #mambo.fly_direct(data.linear.y * 100, data.linear.x*100,data.angular.z*100,data.linear.z*100)

def cb_shoot_cannon(data):
    global cannon
    rospy.loginfo("\n Cannon activated \n")
    cannon = True

def cb_auto_take_off(data):
    global auto_tko
    rospy.loginfo("\n Auto TKO activated \n")
    auto_tko = True

#Initialization function
def init():
    global tko
    global land
    global cannon
    global auto_tko
    global linX
    global linY
    global Alt
    global Hdg
    rospy.init_node('mambo_node',anonymous=True)
    mamboAdd = rospy.get_param('~bt',str("e0:14:60:5c:3d:c7"))
    wifi = rospy.get_param('~mambo_wifi',False)
    retries = rospy.get_param('~mambo_retries',3)
    rospy.loginfo("\n" + rospy.get_name() + "\nParameters:\n" + mamboAdd + "\n" + str(wifi) + "\n" + str(retries) +"\n") 
    s_cmd_vel = rospy.Subscriber('cmd_vel',Twist,cb_cmd_vel)
    s_take_off = rospy.Subscriber('take_off',Empty,cb_take_off)
    s_land = rospy.Subscriber('land',Empty,cb_land)
    s_cannon = rospy.Subscriber('cannon',Empty,cb_shoot_cannon)
    s_auto_tko = rospy.Subscriber('auto_tko',Empty,cb_auto_take_off)
    mambo = Mambo(mamboAdd,use_wifi=wifi)
    success = mambo.connect(retries)
    if(success):
        mambo.smart_sleep(2)
        mambo.ask_for_state_update()
        mambo.smart_sleep(2)
        mambo.flat_trim()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if tko == True:
                mambo.safe_takeoff(3)
                tko = False

            if land == True:
                mambo.safe_land(2)
                land = False

            if cannon == True:
                mambo.fire_gun()
                cannon = False

            if auto_tko == True:
                mambo.turn_on_auto_takeoff()
                auto_tko = False

            mambo.fly_direct(roll = (-linY * 100), pitch = (linX*100),yaw = (-Hdg *100), vertical_movement = (Alt*100), duration=0.001)

            rate.sleep()

#Main function
if __name__=='__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        mambo.disconnect()
