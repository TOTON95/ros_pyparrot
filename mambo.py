#!/usr/bin/env python3.6
import rospy
import math
from std_msgs.msg import Empty, String, UInt8
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
need_to_change_mode = False
need_to_toggle_mode = False
p_mode = 0
linX = 0
linY = 0
Alt = 0
Hdg = 0

#Preffered piloting mode function
def pilotmode(Mambo,s_mode):
    if s_mode.data==0:
        mode = 'easy'
    elif s_mode.data==1:
        mode = 'medium'
    elif s_mode.data==2:
        mode = 'difficult'
    else:
        return

    #command_tuple = Mambo.command_parser.get_command_tuple("minidrone", "PilotingSettings","PreferredPilotingMode")
    (command_tuple, enum_tuple) = Mambo.command_parser.get_command_tuple_with_enum("minidrone", "PilotingSettings","PreferredPilotingMode",mode)

    #return Mambo.drone_connection.send_enum_command_packet_ack(command_tuple,s_mode.data);
    return Mambo.drone_connection.send_enum_command_packet_ack(command_tuple,enum_tuple);

def togglemode(Mambo):
    command_tuple = Mambo.command_parser.get_command_tuple("minidrone", "Piloting","TogglePilotingMode")
    return Mambo.drone_connection.send_noparam_command_packet_ack(command_tuple)

#Set preffered piloting mode
def cb_pilotmode(p):
    global p_mode
    global need_to_change_mode
    p_mode = p
    need_to_change_mode = True

#Toggle flight mode
def cb_toggle_mode(data):
    global need_to_toggle_mode
    rospy.loginfo("\n" + rospy.get_name() + " Toggling Mode...\n")
    need_to_toggle_mode = True

#Sends the spin function to another thread
def spin_th(name,envi):
    rospy.spin()

#Callback of the land command
def cb_land(data):
    global land
    rospy.loginfo("\n" + rospy.get_name() + " Land!!\n")
    land = True

#Callback of the take-off command
def cb_take_off(data):
    global tko
    rospy.loginfo("\n" + rospy.get_name() + " Take-Off!!\n")
    tko = True

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

    #Forced perturbation
#    linX += 0.3

#    if(linX < 0):
#	    linX -= 0.1
#    if(linY < 0):
#	    linX += 0.18

def cb_shoot_cannon(data):
    global cannon
    rospy.loginfo("\n Cannon activated \n")
    cannon = True

def cb_auto_take_off(data):
    global auto_tko
    rospy.loginfo("\n Auto TKO activated \n")
    auto_tko = True

def sat(value, max_value):
    if(value > max_value):
	    value = max_value
    if(value < -max_value):
	    value = -max_value
    return value


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
    global p_mode
    global need_to_change_mode
    global need_to_toggle_mode
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
    s_piloting_mode = rospy.Subscriber('piloting_mode',UInt8,cb_pilotmode)
    s_toggle_mode = rospy.Subscriber('toggle_mode',Empty,cb_toggle_mode)
    p_ready = rospy.Publisher('ready',String,queue_size=30)
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
                mambo.safe_takeoff(2)
                p_ready.publish("ok")
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

            if need_to_change_mode == True:
                if pilotmode(mambo,p_mode):
                    print("Changed mode successfully")
                else:
                    print("Failed changing mode")
                need_to_change_mode=False

            if need_to_toggle_mode == True:
                if togglemode(mambo):
                    print("Activating preffered mode...")
                else:
                    print("Failed activating preferred mode")
                need_to_toggle_mode = False

            r_y = round(linY,2)
            p_x = round(linX,2)
            v_z = round(Alt,2)
            y_z = round(Hdg,2)

            r_y = sat(r_y, 0.98)
            p_x = sat(p_x, 0.98)
            v_z = sat(v_z, 0.98)
            y_z = sat(y_z, 0.98)
            mambo.fly_direct(roll = (-r_y * 100), pitch = (p_x*100),yaw = (-y_z *100), vertical_movement = (v_z*100), duration=0.01)
           # linY = 0
           # linX = 0
           # Hdg = 0
           # Alt = 0

            rate.sleep()

        #mambo.disconnect()


#Main function
if __name__=='__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        mambo.disconnect()
