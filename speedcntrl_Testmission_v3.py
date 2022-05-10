#!/usr/bin/env python
#----

import matplotlib as mpl
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from nav_msgs.msg import *
from tf.transformations import euler_from_quaternion
import numpy as np
import math
#----


# VAR
x =     0.0 
y =     0.0
z =     0.0
yaw =   0.0
roll =  0.0
pitch = 0.0
x_pos = 0.0
y_pos = 0.0
out_x = 0.0
out_y = 0.0
min_pitch = 0.0
PI = math.pi
 # TESTA 1


# SETPOINT HEIGHT, XY AND YAW
wp = np.array([[150,150 , 20], [0,150, 20], [-150,-150, 40], [0, 0, 20]])
max_error = 5 # ?
max_error_landing = 1
wp_goal = 0 # ?


goal_yaw =math.radians(45) #[Rad]

# CONTROLLER GAINS HEIGHT,XY AND YAW
# HEIGHT
KP_z = 1
KI_z = 0.001
KD_z= 0.1
# XY
k_vx = 0.1 #0.1 be4
k_px = 3
k_vy = 0.1
k_py = 3
# YAW
Kp_angular = 3 
Kd_angular = 0.1
Ki_angular = 0.0

# SATURATION FOR VEL COMMANDS
K = 3 # XY
SAT = 3 # HEIGHT

def POS(msg): # Get position data from ros
    global x
    global y
    global z
    global yaw

    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    rot_q = msg.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def  SPEED(data): # Get linear speed-commands from ros
    global vx,vy,vz
    vx=data.twist.twist.linear.x
    vy=data.twist.twist.linear.y
    vz=data.twist.twist.linear.z


if __name__ == '__main__':
    try:
        rospy.init_node('vel_controller', anonymous=False)
        pubvel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) # Velocity pub
        sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, POS) # Position sub
        sub_odom = rospy.Subscriber('/mavros/global_position/local', Odometry, SPEED,queue_size=1) # Velocity sub
        pubset = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1) # Position pub start up seq 
        rate = rospy.Rate(10) #10 hz


        try:
            rospy.wait_for_service('mavros/cmd/arming', 20)
            rospy.wait_for_service('mavros/set_mode', 20)
            rospy.wait_for_service('mavros/cmd/vtol_transition', 20)
            rospy.wait_for_service('mavros/cmd/land', 20)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        arming_srv = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        land_srv = rospy.ServiceProxy('mavros/cmd/land', CommandTOL )
        mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)  
        vtol_transition_service = rospy.ServiceProxy('mavros/cmd/vtol_transition', CommandVtolTransition)

        hover_point = [0,0,10] # hover at 10 m while starting up...

        cmd_pos = PoseStamped()
        cmd_pos.pose.position.x = hover_point[0]
        cmd_pos.pose.position.y = hover_point[1]
        cmd_pos.pose.position.z = hover_point[2]

        arming_srv(True)
        rospy.loginfo("UAV is armed")
        for i in range(0,35): #publish some messages to be able to activate offboard mode
            pubset.publish(cmd_pos)
            rate.sleep()
        
        mode_res = mode_srv(0, "OFFBOARD")
        rospy.loginfo("OFFBOARD INITIATED, TAKE OFF NEXT...")

        for i in range(0,50): #publish some messages to be able to activate offboard mode
           pubset.publish(cmd_pos)
           rate.sleep()



        for i in range(0,50): #publish some messages to be able to activate offboard mode
            pubset.publish(cmd_pos)
            rate.sleep()


        while not rospy.is_shutdown():
            # USED FOR BOTH MODES
            distance = np.linalg.norm(wp[wp_goal] - np.array([x, y, z])) #distance to goal
            last_waypoint = len(wp)
            if distance < max_error and wp_goal + 1 < np.size(wp,0):
                wp_goal = wp_goal +1
                rospy.loginfo ("Moving to:" + str(wp[wp_goal]))
            goal_x = wp[wp_goal,0] #[m]
            goal_y = wp[wp_goal,1] #[m]
            goal_z = wp[wp_goal,2] #[m]
            cmd_vel = TwistStamped()
            
            xy_distance = ((((goal_x - x )**2) + ((goal_y-y)**2) )**0.5)
            
            if wp_goal +1 < last_waypoint:
                case =  2
                rospy.loginfo_once(case)
                
            if wp_goal +1 == last_waypoint and xy_distance < 40 :# 
                case = 1

                
                rospy.loginfo_once(case)    
            
            if case == 1:
                #RM
                
                h = Header()
                state = 3
                vtol_transition_service(h, state);
                
                cmd_pos.pose.position.x = 0
                cmd_pos.pose.position.y = 0
                cmd_pos.pose.position.z = 20
                pubset.publish(cmd_pos)
                if xy_distance < 1:
                    land_srv()#land
                    rospy.spin()
                    rospy.loginfo_once("landing")

            
            if case == 2:
                #control in FW MODE.
                h = Header()
                state = 4
                vtol_transition_service(h, state);
                angular_errorP = 0.0
                total_angular_error = 0.0

                
                FW_angle_goal = math.atan2(goal_y - y, goal_x - x)
                theta = FW_angle_goal - yaw #error

                while(abs(theta)-PI > 0.001): #Turn right way
                    if(theta >PI):
                        theta = theta -2*PI
                
                    if(theta < -PI):
                        theta = theta +2*PI
                print("yaw error", math.degrees(theta)) #HERE
                angular_speed = theta*Kp_angular + ((theta-angular_errorP)*Kd_angular) + (total_angular_error*Ki_angular) # more
                angular_errorP = theta
                total_angular_error = total_angular_error + theta

                x_out = k_vx*(k_px*(goal_x-x) - vx)
                y_out = k_vy*(k_py*(goal_y - y) - vy)
                
                if x_out > K:
                    x_out = K
                if x_out < -K:
                    x_out = -K

                if y_out > K:
                    y_out = K

                if y_out < -K:
                    y_out = -K
                

                error_z = 0
                integral_error_z = 0
                error_last_z = 0
                derivate_error_z = 0
                output_z = 0
                time = rospy.get_time() 
                error_z= goal_z - z 
                integral_error_z += error_z * time
                derivate_error_z = (error_z - error_last_z) / time
                error_last_z = error_z

                output_z = KP_z * error_z + KI_z * integral_error_z * KD_z * derivate_error_z
                # END LINEAR PID

            
                if output_z >= SAT:
                    output_z = SAT   
                #print("X ERROR", goal_x - x, "Y ERROR", goal_y -y)

                #cmd_vel.twist.linear.x = x_out# out_x
                #cmd_vel.twist.linear.y = y_out#out_y # output_y
                # YAW CONTROLLER HEADING TOWARDS NEXT POINT
                cmd_vel.twist.linear.x = x_out# out_x
                cmd_vel.twist.linear.y = y_out#out_y
                cmd_vel.twist.linear.z = -output_z #  - in FW
                cmd_vel.twist.angular.z = angular_speed
                pubvel.publish(cmd_vel)
 

    except rospy.ROSInterruptException:
        pass




