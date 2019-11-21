#!/usr/bin/env python

# chmod +x nodes/"   ".py

import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

error_integral=np.array([0.,0.,0.])
error=1000

global_pos= Pose()#np.array([0.,0.,0.])
global_command= Twist()
global_vel=Twist()
global_waypoint=Point()

global_marker_center=np.array([0.,0.])
global_marker_center_avg=np.array([0.,0.])

islanded=True

pub_commands= rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
pub_takeoff= rospy.Publisher('bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1) #trigger with pub_land.publish()

initial_cmd = Point()



def quat_mult(a,b):
    
    c = np.array([0.,0.,0.,0.])
    c[0] = (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3] )
    c[1] = (a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2] )
    c[2] = (a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1] )
    c[3] = (a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0] )
    return c

def odomlistener():
    #okay so we start this listener node
    global global_pos
    global islanded

    update_setpoint(initial_cmd)

    rospy.init_node('moveto_controller', anonymous=True, log_level=rospy.WARN)
    rospy.Subscriber('/bebop/odom', Odometry, updateOdom)
    rospy.Subscriber('/bebop/land',Empty, setlanded)
    ############################# This should be an e,y,x
    rospy.Subscriber('/moveto_cmd_body',Point, update_setpoint)


    print('node started')
    time.sleep(2.)
    islanded=False

    telemrate = 10
    rate = rospy.Rate(telemrate)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        # 4.3 and 2.1 from blue
        moveto_body() # Runs the controller at 10 hz
        rate.sleep()    

def updateOdom(msg):
    global global_pos
    global global_vel
    rospy.loginfo(msg.pose.pose)
    rospy.loginfo(msg.twist.twist)

    #global_pos=np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
    #print(global_pos)
    global_pos=msg.pose.pose
    global_vel=msg.twist.twist

def setlanded(msg):
    global islanded
    islanded=True
    pub_land.publish()

def update_setpoint(data):
    global global_pos
    global global_vel
    global global_command
    global global_waypoint
    global expected_pos_inertial
    global x
    global y
    global z

    x = data.x # Positive fwd, body frame
    y = data.y # Positive left, body frame
    z = data.z # positive up, INERTIAL FRAME

    #okay so have a vector in the actual body frame, want to convert to inertial/odom frame
    quat_B_to_I= np.array([global_pos.orientation.w, -global_pos.orientation.x, -global_pos.orientation.y, -global_pos.orientation.z])
    quat_B_to_I_inv= np.array([global_pos.orientation.w, global_pos.orientation.x, global_pos.orientation.y, global_pos.orientation.z])
    command_quat_body=np.array([0, x,y,z])
    temp= quat_mult(quat_B_to_I_inv,command_quat_body)
    command_quat_inertial= quat_mult(temp,quat_B_to_I)
    command_vect_inertial= command_quat_inertial[1:]

    #okay now have command in the odom frame, relative to body, so make it relative to origin

    expected_pos_inertial= np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z]) + command_vect_inertial
    global_waypoint.x=expected_pos_inertial[0]
    global_waypoint.y=expected_pos_inertial[1]
    global_waypoint.z= z #expected_pos_inertial[2] # ACCEPT THE COMMANDED Z AS INERTIAL

def moveto_body():

    global global_pos
    global global_vel
    global global_command
    global expected_pos_inertial
    global islanded
    global global_waypoint
    global error_integral
    global error
    move_array=np.array([0.,0.,0.])

    if islanded==False:
        #print(np.array([x,y,z]))

        print('error-------------------------------------------------------------------------')
        print(error)
        current_pos_inertial=np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])
        move_vect_inertial= expected_pos_inertial-current_pos_inertial

        #okay need to convert to vector in body frame to figure out where to move
        #using current orientation global_pos in case something is flukey
        quat_I_to_B= np.array([global_pos.orientation.w, global_pos.orientation.x, global_pos.orientation.y, global_pos.orientation.z])
        quat_I_to_B_inv= np.array([quat_I_to_B[0], -quat_I_to_B[1], -quat_I_to_B[2],-quat_I_to_B[3]])
        
        move_quat_inertial= np.array([0, move_vect_inertial[0],  move_vect_inertial[1], move_vect_inertial[2]])
        temp= quat_mult(quat_I_to_B_inv,move_quat_inertial)
        move_quat_body= quat_mult(temp,quat_I_to_B)
        move_vect_body= move_quat_body[1:]# this is basically your error vector

        velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.z])
        error_integral=error_integral+move_vect_body

        #move_vect_body[2]=1.29*move_vect_body[2]
        move_array[0]=.08*move_vect_body[0] - .16*velocity_vect_body[0] + .001*error_integral[0] #TUNE THIS
        move_array[1]=.08*move_vect_body[1] - .16*velocity_vect_body[1] + .001*error_integral[1]
        move_array[2]=.53*move_vect_body[2] - .10*velocity_vect_body[2] + .001*error_integral[2]

        #timedelay= .1#TUNE THIS
        print('move vect')
        print(move_array)
        print('move_vect_body')
        print(move_vect_body)
        print('velocity_vect_body')
        print(velocity_vect_body)
        print('error_integral')
        print(error_integral)
        print(' ')
        print('command is')
        print(np.array([x,y,z]))
        print('expected_pos_inertial')
        print(expected_pos_inertial)
        print('current_pos_inertial')
        print(current_pos_inertial)



        global_command.linear.x=move_array[0]
        global_command.linear.y=move_array[1]
        global_command.linear.z=move_array[2]
        global_command.angular.x=0
        global_command.angular.y=0
        global_command.angular.z=0



        pub_commands.publish(global_command)
        #time.sleep(timedelay)
        # pub_commands.publish(global_command)
        # time.sleep(timedelay)
        # pub_commands.publish(global_command)
        # time.sleep(timedelay)
        error= np.linalg.norm(np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])-expected_pos_inertial)
        #error=.4
        #pub_waypoints.publish(global_waypoint)

        #print('TRYING TO LAND! !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        # pub_land.publish()
        # islanded=True
    else:
        print('Landed')
        pub_land.publish()
        islanded=True


if __name__ == '__main__':

    odomlistener()
    # rospy.spin()