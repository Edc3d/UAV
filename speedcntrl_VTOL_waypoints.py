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
mode = 0


# SETPOINT HEIGHT, XY AND YAW
wp = np.array([[100, 100, 15], [10,5,20], [100,100,35], [-100,-150,20]])
max_error = 15 # ?
wp_goal = 0 # ?


goal_yaw =math.radians(45) #[Rad]

# CONTROLLER GAINS HEIGHT,XY AND YAW
# HEIGHT
KP_z = 1
KI_z = 0.001
KD_z= 0.1
# XY
k_vx = 0.1
k_px = 3
k_vy = 0.1
k_py = 3
# YAW
Kp_angular = 3 
Kd_angular = 0.1
Ki_angular = 0.0

# SATURATION FOR VEL COMMANDS
K = 2 # XY
SAT = 2 # HEIGHT

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
            #---- VTOL SWITCH

            
            #---- VTOL SWITCH END

            #---- YAW CONTROL
            PI = math.pi
            theta = (goal_yaw - yaw) #theta is anglediff
            #rospy.loginfo ("Moving to:" + str(wp[wp_goal]))

            distance = np.linalg.norm(wp[wp_goal] - np.array([x, y, z])) #distance to goal
            last_waypoint = len(wp)
            TEST = 1

            while TEST == 1:
                Mode = 1# 1 = FW MODE
                rospy.loginfo("VTOL")
                h = Header()
                state = 4
                vtol_transition_service(h, state);
                rate.sleep()

            #print(wp_goal)
            #print(distance)
            #1 works good in Rotormode
            if distance < max_error and wp_goal + 1 < np.size(wp,0):
                wp_goal = wp_goal +1
                rospy.loginfo ("Moving to:" + str(wp[wp_goal]))
            goal_x = wp[wp_goal,0] #[m]
            goal_y = wp[wp_goal,1] #[m]
            goal_z = wp[wp_goal,2] #[m]
              
            angular_errorP = 0.0
            total_angular_error = 0.0

            

            while(abs(theta)-PI > 0.001): #Turn right way
                if(theta >PI):
                    theta = theta -2*PI
                
                if(theta < -PI):
                    theta = theta +2*PI


            # FIX WINGED CONTROL
            if mode == 1:
                print("blabla")
                #CONTROL
            # ROTOR MODE CONTROL
            if mode == 0:
                #CONTROL
                print("blabla")

                # YAW CONTROL
                angular_speed = theta*Kp_angular + ((theta-angular_errorP)*Kd_angular) + (total_angular_error*Ki_angular) # more
            
 
                # --- ROTATE FRAMES
                x_Rz = math.cos(yaw)*x + math.sin(yaw)*y
                y_Rz = -math.sin(yaw)*x + math.cos(yaw)*y
 
                goal_x_Rz = math.cos(yaw)*goal_x + math.sin(yaw)*goal_y
                goal_y_Rz = -math.sin(yaw)*goal_x + math.cos(yaw)*goal_y


                #---- OUTPUTS XY
                x_out = k_vx*(k_px*(goal_x_Rz-x_Rz) - vx)
                y_out = k_vy*(k_py*(goal_y_Rz - y_Rz) - vy)


            
                if x_out > K:
                    x_out = K
                if x_out < -K:
                    x_out = -K

                if y_out > K:
                    y_out = K

                if y_out < -K:
                    y_out = -K

            #LINEAR PID Z PART
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


                cmd_vel = TwistStamped()
                cmd_vel.twist.linear.x = x_out# out_x
                cmd_vel.twist.linear.y = y_out#out_y # output_y
                cmd_vel.twist.linear.z = -1*output_z #  - in FW
                flag = 0

                if distance < max_error and wp_goal +1 == last_waypoint:
                    rospy.loginfo_once("adjusting yaw")
                
                    cmd_vel.twist.angular.z = angular_speed
                

                if wp_goal +1 == last_waypoint and abs(theta)  < 0.01:

                    flag = 1
                    rospy.loginfo_once("angle reached")



                if flag == 1 : #and abs(goal_x - x) < 0.05 and abs(goal_y -y) <0.05
                    land_srv()#land
                    rospy.spin()
                #rospy.loginfo_once("Landing")

                



                angular_errorP = theta
                total_angular_error = total_angular_error + theta

           
            

            
                pubvel.publish(cmd_vel)
           

    except rospy.ROSInterruptException:
        pass




