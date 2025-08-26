#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np
from include import initialize
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Header
from vehicle_data.msg import Reconstruction, ObserverError

# Constants for formation control
NEIGHBOR_DISTANCE_THRESHOLD = 5.0  # Threshold distance for neighbor detection
uav_num = 6 # Number of UAVs in the formation
initial_formation = np.array([[0,0,3],[10,0,3],[10,10,3],[0,10,3],[5,-8.66,3],[5,18.66,3]])

A = np.array([[0,0,0,1,1,0],[0,0,1,0,1,0],[0,1,0,0,0,1],[1,0,0,0,0,1],[1,1,0,0,0,0],[0,0,1,1,0,0]])

hover = 0 # Hovering flag

uavn = [initialize(i) for i in range(uav_num)] # Initialize the topics 

uav_id =  int(sys.argv[1]) # UAV id
uav = initialize(uav_id) # Current UAV initializer
set_mode = uav.set_mode_client() # Setting mode service
arm_cmd = uav.arming_client() # Arming service

# Defining the target position message
uav.desired_traj.header = Header()
uav.desired_traj.header.frame_id = "base_footprint"
uav.desired_traj.coordinate_frame = 1
uav.desired_traj.type_mask = 0

# States initialization
x1hatd = np.zeros(3)
x1hat = np.zeros(3)
x2hatd = np.zeros(3)
x2hat = np.zeros(3) 
x3hat = np.zeros(3)
x3hatd = np.zeros(3)
xtild = np.zeros(3) 
e1 = np.zeros(3)
e2 = np.zeros(3)
intsc = np.zeros(3)
sc = np.zeros(3) 
s = np.zeros(3)
ints = np.zeros(3)
intsaux = np.zeros(3)
saux = np.zeros(3)

error = 0.0
counter = 0

threshhold = 0.1

# Gain of Controller
k1x = 0.5
k1y = 0.5
k1z = 4.5
k2x = 0.25
k2y = 0.25
k2z = 4.5
c1x = 0.2
c1y = 0.2
c1z = 2.5

#Gain of Observer
lamda1x = 1.9
lamda2x = 1.5
lamda3x = 1.1
lamda1y = 1.9
lamda2y = 1.5
lamda3y = 1.1

kp = 0.2
kv = 1

# CSTC function
def CSTC(dt,t):
	
    global x1hat, x1hatd, x2hat, x2hatd, x3hat, x3hatd, xtild, e1, e2, intsc, sc, s, ints, intsaux, saux, error, counter, check, A
    
    check = 0

    # Compute the position errors
    x1 = np.array([uav.pos[0], uav.pos[1], uav.pos[2]])
    x2 = np.array([uav.vel[0], uav.vel[1], uav.vel[2]])
    x1des = np.array([uav.pos_sp[0], uav.pos_sp[1], uav.pos_sp[2]])
    x2des = np.array([uav.vel_sp[0], uav.vel_sp[1], uav.vel_sp[2]])

    # previous inputs
    uxx = 0.0
    uyy = 0.0

    e1 = x1 - x1des
    e2 = x2 - x2des

    uav.current_error = [e1[0], e1[1], e2[0], e2[1]]

    # Publish the relative measurement message
    uav.error_fault_pub.publish(uav.current_error) 

    if t > 70:
        A = np.array([[0,1,0,1,0,0],[1,0,1,0,0,0],[0,1,0,0,0,1],[1,0,0,0,0,1],[0,0,0,0,0,0],[0,0,1,1,0,0]])
    if t > 130:
        A = np.array([[0,1,0,1,0,0],[1,0,1,0,0,0],[0,1,0,1,0,0],[1,0,1,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])

    # Check the neighbors of current UAV and compute consensus sliding mode term
    for i in range(6):
         if A[uav_id,i]:
		saux[0] = kp*(uav.current_error[0]-uavn[i].error[0]) + kv*(uav.current_error[2]-uavn[i].error[2]) 			
		saux[1] = kp*(uav.current_error[1]-uavn[i].error[1]) + kv*(uav.current_error[3]-uavn[i].error[3]) 	
		error += abs(np.linalg.norm((uav.pos + initial_formation[uav_id]) - (uavn[i].pos + initial_formation[i])))
		counter = counter + 1
                error = (error - 10.0)
    # Sliding surfaces
    s[0] = e2[0] + c1x*e1[0]
    s[1] = e2[1] + c1y*e1[1]
    s[2] = e2[2] + c1z*e1[2]

    intsc += sc*dt
    intsc[0] = np.clip(intsc[0],-0.1,0.1)
    intsc[1] = np.clip(intsc[1],-0.1,0.1)
    intsaux += saux*dt
    ints += s*dt
    intsaux[0] = np.clip(intsaux[0],-0.1,0.1)
    intsaux[1] = np.clip(intsaux[1],-0.1,0.1)
    saux[0] = np.clip(intsaux[0],-0.2,0.2)
    saux[1] = np.clip(intsaux[1],-0.2,0.2)

    sc = s + ints + intsaux

    # Position controllers
    # to do, add desired xdd
    ux = -c1x*x2hat[0] + c1x*x2des[0] - k1x*pow(abs(sc[0]),1/2)*uav.sat(sc[0],threshhold) - k2x*intsc[0]
    uy = -c1y*x2hat[1] + c1y*x2des[1] - k1y*pow(abs(sc[1]),1/2)*uav.sat(sc[1],threshhold) - k2y*intsc[1]
    uz = -c1z*x2[2] + c1z*x2des[2] - k1z*pow(abs(sc[2]),1/2)*uav.sat(sc[2],threshhold) -k2z*intsc[2]
    ux = np.clip(ux,-0.6,0.6)
    uy = np.clip(uy,-0.6,0.6)
    #uxx = ux
    #uyy = uy

    for i in range(6):
        check = check + A[uav_id,i]

    if check == 0:
        ux = 0
        uy = 0

    # Publish the reconstruction message
    uav.reconstruct_fault_pub.publish(uav.reconstructed)
    U = np.array([ux, uy, 0])
    rospy.loginfo("ux =%f uy =%f scx=%f scy=%f", ux,uy,sc[0],sc[1])
    rospy.loginfo("error =%f counter =%f", error, counter)
    return U

if __name__ == '__main__':

    # Initialization
    rospy.init_node('quadrotor_controller'+str(uav_id)) 
    rospy.loginfo("1. UAVs are initialized")

    rospy.sleep(5)

    rate = rospy.Rate(50)  # Loop speed 100Hz	

    # Building an initial desired set points
    uav.desired_traj.header.stamp = rospy.Time.now()
    uav.desired_traj.position.x = 0
    uav.desired_traj.position.y = 0 
    uav.desired_traj.position.z = 0
    uav.desired_traj.velocity.x = 0
    uav.desired_traj.velocity.y = 0
    uav.desired_traj.velocity.z = 0
    uav.desired_traj.acceleration_or_force.x = 0 
    uav.desired_traj.acceleration_or_force.y = 0
    uav.desired_traj.acceleration_or_force.z = 0
    U = np.zeros(3)

    # Publishing zero setpoints before OFFBOARD flight mode to avoid failsafe
    for i in range(50):
        uav.pos_setpoint_pub.publish(uav.desired_traj)
        rate.sleep()

    # Setting OFFBOARD flight mode
    set_mode = uav.set_mode_client(0, "OFFBOARD")  # 0 is custom mode
    while not set_mode:
        rospy.loginfo("Failed to send OFFBOARD mode command for vehicle%d", uav_id)
        set_mode = uav.set_mode_client(0, "OFFBOARD")  # 0 is custom mode
    
    # Arm the motors
    arm_cmd = uav.arming_client(True)
    if not arm_cmd.success:
        rospy.loginfo("vehicle%d failed to send arm command", uav_id)

    # Starting the mission
    rospy.loginfo("2. Vehicle%d is ready to fly", uav_id)
    
    # initialize time
    initial_time1 = rospy.get_time() 
    initial_time = rospy.get_time() 
    prev_time = rospy.get_time() 

    while not rospy.is_shutdown():

	# Hovering loop
	while uav.pos[2]<3 and hover==0 or (initial_time - initial_time1) < 20:
            pos_sp = np.array([0, 0, 3])
            vel_sp = np.array([0, 0, 0])
            acc_sp = np.array([0, 0, 0])
            uav.desired_traj.header.stamp = rospy.Time.now()            
            [uav.desired_traj.position.x, uav.desired_traj.position.y, uav.desired_traj.position.z] = pos_sp
            [uav.desired_traj.velocity.x, uav.desired_traj.velocity.y, uav.desired_traj.velocity.z]  = vel_sp
            [uav.desired_traj.acceleration_or_force.x, uav.desired_traj.acceleration_or_force.y, uav.desired_traj.acceleration_or_force.z] = acc_sp
            uav.desired_traj.header.stamp = rospy.Time.now() 
            uav.pos_setpoint_pub.publish(uav.desired_traj)
            initial_time = rospy.get_time()
            prev_time = rospy.get_time() 
        
        hover = 1    

        current_time = rospy.get_time()
        dt = current_time - prev_time
        prev_time = current_time   
        current_time = rospy.get_time()
        t = current_time - initial_time

        if (t>10) and (t<50):
	    uav.desired_traj.type_mask = 27
            pos_sp = np.array([0.5*t-5, 0, 3])
            vel_sp = np.array([0.5, 0, 0])
            U = CSTC(dt,t)

        if (t>50) and (t<90):
	    uav.desired_traj.type_mask = 27
            pos_sp = np.array([0.5*t-5, 0.5*t-25, 3])
            vel_sp = np.array([0.5, 0.5, 0])
            U = CSTC(dt,t)
        if (t>90) and (t<170):
	    uav.desired_traj.type_mask = 27
            pos_sp = np.array([-0.5*t+85, 20, 3])
            vel_sp = np.array([-0.5, 0, 0])
            U = CSTC(dt,t)
        if (t>170):
	    uav.desired_traj.type_mask = 0
            pos_sp = np.array([0, 20, 0])
            vel_sp = np.array([0, 0, 0])
            #U = CSTC(dt)


        current_time = rospy.get_time()

        [uav.desired_traj.position.x, uav.desired_traj.position.y, uav.desired_traj.position.z] = pos_sp
        [uav.desired_traj.velocity.x, uav.desired_traj.velocity.y, uav.desired_traj.velocity.z]  = vel_sp
        [uav.desired_traj.acceleration_or_force.x, uav.desired_traj.acceleration_or_force.y, uav.desired_traj.acceleration_or_force.z] = acc_sp + U
        uav.desired_traj.header.stamp = rospy.Time.now()      
        uav.pos_setpoint_pub.publish(uav.desired_traj)

        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

    # Spin the node
    rospy.spin()
