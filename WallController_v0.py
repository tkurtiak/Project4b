#!/usr/bin/env python


## Helical Trajectory Commander that listens to Bebop odometry messages
## And commands bebop pitch/roll/yaw to achieve a Helix

# Make this program exacutable
# chmod +x nodes/HelixTrajectoryController.py

import rospy
import time
import numpy as np
#import tf  # use tf to easily calculate orientations


from nav_msgs.msg import Odometry # We need this message type to read position and attitude from Bebop nav_msgs/Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

global_pos= Twist()#np.array([0.,0.,0.])
global_command= Twist()
global_vel=Twist()
global_waypoint=Point()

global_marker_center=np.array([0.,0.])
global_marker_center_avg=np.array([0.,0.])

islanded=True

pub_commands= rospy.Publisher('bebop/cmd_vel',Twist,queue_size=1)
pub_takeoff= rospy.Publisher('bebop/takeoff',Empty,queue_size=1)
pub_land= rospy.Publisher('bebop/land',Empty,queue_size=1) #trigger with pub_land.publish()


framex = 640
framey = 480 

isfirststep=False

def quat_mult(a,b):
    
    c = np.array([0.,0.,0.,0.])
    c[0] = (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3] )
    c[1] = (a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2] )
    c[2] = (a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1] )
    c[3] = (a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0] )
    return c

def callback(msg):
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

def updatecenter(msg):
    global global_marker_center # In pixel coordinates where 0,0 is top left
    global_marker_center[0]=msg.x - framex # in camera center coords. pos right
    global_marker_center[1]=-msg.y + framey # in cemera center coords.  pos up
    rospy.loginfo(msg)


def odomlistener():
    #okay so we start this listener node
    global global_pos
    global islanded

    rospy.init_node('odomlistener_move', anonymous=True, log_level=rospy.WARN)
    rospy.Subscriber('/bebop/odom', Odometry, callback)
    rospy.Subscriber('/bebop/land',Empty, setlanded)
    rospy.Subscriber('/wall_center_point',Point, updatecenter)  
    print('node started')
    time.sleep(2.)
    print('taking off')
    pub_takeoff.publish()
    time.sleep(6.)
    islanded=False

    telemrate = 10
    rate = rospy.Rate(telemrate)
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        # 4.3 and 2.1 from blue
        trackwall()

        rate.sleep()	



def trackwall():
    
    global isfirststep
    global global_pos
    global global_marker_center
    global global_marker_center_avg
    global Z_cmd
    #x, y are initial coordinates of the landing (gaussian mean or whatever the fuck)

    FOVx=50 #deg
    FOVy=34 #deg


    while isfirststep!=True:
        print('Im doing the first steparoony now')
        # moveto_body(x,0,0) #move over ish the bitch
        # # -ve here is to the right
        # moveto_body(0,-y,0) #move over ish the bitch
        Z_cmd = 2
        moveto_body(0,0,Z_cmd) #move up to see it

        print('Im done with the first steparoony')
        isfirststep=True #first step is done, don't ever do this shit again

    while isfirststep==True:
        wall_x = global_marker_center[0]
        wall_y = global_marker_center[1]

        if wall_x != 0 and wall_y != 0:

            # Determine if the wall is Tall or short
            wallheight_thresh = .1 # percent of the image height above or below the centrer to consider the wall short or tall
            # If the wall is below us when we are high, the wall must be short
            if 2*wall_y/framey < wallheight_thresh and global_pos.position.z>1.8:
                # Wall is short
                Z_cmd = 2 # set z height to 2 meters
                print('It looks like the wall is SHORT')
            # If the wall is above us when we are low, the wall must be tall
            elif 2*wall_y/framey  > wallheight_thresh and global_pos.position.z<1:
                # wall is tallk
                Z_cmd = 0.5 # set z height to 0.5 meters
                print('It looks like the wall is TALL')

            # now take a 0.3 meter step towards the wall
            step = 0.3
            phi_angle_to_wall = (wall_x/(framex/2)*FOVx) *3.14159/180 # angle in radians, defined positive right
            print('Angle to Wall:',phi_angle_to_wall*180/3.14)
            x_cmd = step*np.cos(phi_angle_to_wall)
            y_cmd = -step*np.sin(phi_angle_to_wall) # positive movement is left, so we want to move right if angle is positive
            
            #deg_offsets= (vector2center/(np.array([160,120])))*np.array([FOVx,FOVy])
            #marker_loc=np.tan(deg_offsets*np.pi/180)*global_pos.position.z

            # If we are getting erronious angles, we're probably past the wall and should land now.  We need a more robust signal for passing the wall
            if phi_angle_to_wall>30*3.14159/180:
                print('Crazy angle... landing')
                pub_land.publish()
                # moveto_body(0,0,-.5)
            else:
                print('Executing X,Y body, Z inertial',x_cmd,y_cmd,Z_cmd)
                moveto_body(x_cmd,y_cmd,Z_cmd)

        # pub_land.publish()




################# Can we make Z not be in body frame?
def moveto_body(x,y,z):
    global global_pos
    global global_vel
    global global_command
    global islanded
    global global_waypoint


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
    global_waypoint.z= z #expected_pos_inertial[2]

    #alright so this is the desired waypoint in the odometry frame

    move_array=np.array([0.,0.,0.])
    #print('STARTING MOVE')
    if islanded==False:
        print('STARTING MOVE')
        print(np.array([x,y,z]))

        error=1000.
        error_integral=np.array([0.,0.,0.])
        while error>.15 and islanded==False:# and np.linalg.norm(np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.y]))>.1:
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


            #HOLD UP THE VELOCITY IS IN "CHILD FRAME" WHICH IS SOME OTHER BULLSHIT from HEADER FRAME
            # #we also have 
            # velocity_quat_inertial= np.array([0., global_vel.linear.x, global_vel.linear.y,global_vel.linear.z])
            # #so lets put it in body frame
            # temp2= quat_mult(quat_I_to_B_inv,velocity_quat_inertial)
            # velocity_quat_body= quat_mult(temp2,quat_I_to_B)
            # velocity_vect_body= velocity_quat_body[1:]# this is basically your error vector derivitive
            #velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y,global_vel.linear.z])
            velocity_vect_body= np.array([global_vel.linear.x, global_vel.linear.y, global_vel.linear.z])
            error_integral=error_integral+move_vect_body


            #move_vect_body[2]=1.29*move_vect_body[2]

            move_array[0]=.08*move_vect_body[0] - .16*velocity_vect_body[0] + .001*error_integral[0] #TUNE THIS
            move_array[1]=.08*move_vect_body[1] - .16*velocity_vect_body[1] + .001*error_integral[1]
            move_array[2]=.53*move_vect_body[2] - .10*velocity_vect_body[2] + .001*error_integral[2]


            timedelay= .1#TUNE THIS
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

            if rospy.is_shutdown():
                break

            pub_commands.publish(global_command)
            time.sleep(timedelay)
            # pub_commands.publish(global_command)
            # time.sleep(timedelay)
            # pub_commands.publish(global_command)
            # time.sleep(timedelay)
            error= np.linalg.norm(np.array([global_pos.position.x, global_pos.position.y, global_pos.position.z])-expected_pos_inertial)
            #error=.4
            pub_waypoints.publish(global_waypoint)

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