#!/usr/bin/env python
# license removed for brevity
from __future__ import division

import numpy as np
import math

import rospy
import tf as trans

import utm

from random import *

from std_msgs.msg import Float32MultiArray, UInt8, Int32MultiArray, Float64, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix

import socket

hostname = socket.gethostname()
if hostname == 'usrg-desktop':
    GAZEBO_SIMULATION = False
elif hostname == 'usrg-ZenBook' or hostname == 'usrgasus-ZenBook-Pro-Duo-UX581GV-UX581GV':
    GAZEBO_SIMULATION = True
else:
    GAZEBO_SIMULATION = False

GAZEBO_SIMULATION = True #?
using_GPS_test = False

eps = 0.0000001
Real_North_offset_rad = 193 * (math.pi / 180.0) #0 # 193
Real_North_offset_deg = 193

def get_transform_point(x, y, z):
    # gazebo coordinate + 193'(positive x direction) = North coordinate (Real)
    # and gazebo initial x position same building left bottom point
    #     gazebo initial y position is minus 20 from left bottom point
    #                                     x  <-------
    #               -------------                   |
    #               |           |                   |
    #               | BUILDING  |                   |
    #               |           |                    y
    #               |           |
    #               -------------
    #               ^
    #               |   20m         
    #               |            
    #    initial position in gazebo
    #
    # if we change initial position (p, q) in gazebo coordinate, we need to adjust p, q point in gazebo
    # p is initial_x_offset, q is initial_y_offset

    initial_x_offset = 0
    initial_y_offset = 0

    temp_x = x - initial_x_offset
    temp_y = y - initial_y_offset

    yaw_offset = Real_North_offset_rad

    real_x = temp_x * math.cos(yaw_offset) + temp_y *math.sin(yaw_offset)
    real_y = temp_x * -1 * math.sin(yaw_offset) + temp_y * math.cos(yaw_offset)
    real_z = z

    return [real_x, real_y, real_z]

class ros_class:
    def __init__(self):
        self.t_cur = 0.0                                    ## # ros time
        self.count = 0                                      ## # The amount of bursted ballon
        self.done = False                                   ## # Not used in any function

        self.pub_GoalAction_uav3 = 0                        ##
        self.pub_VelCmd = 0                                 ##
        self.Cur_Pos_m = [0.0, 0.0, 0.0]                    ##
        self.Cur_Vel_mps = [0.0, 0.0, 0.0]

        self.Cur_GPS = [0, 0, 0]
        self.Starting_GPS = [0, 0, 0]
        self.Cur_GPS_heading = 0.0

        self.init_x = [0.0, 0.0, 0.0]
        self.q = []
        self.euler = [0.0, 0.0, 0.0]
        self.init = [0.0, 0.0, 0.0]

        self.cmd = [0.0, 0.0, 0.0, 0.0]

        self.image_pos = [0.0, 0.0]
        self.u = 0
        self.v = 0
        self.Cur_Tar_m = [0.0, 0.0]
        self.GoalAction_uav3 = Float32MultiArray()          ##

        self.flag_uav3 = -1                                 ## TODO This is used but have to find what it is
        self.PubMissionCallback = 0

        ## Detection
        self.flag_detection = 0
        self.size = [10, 10]
        self.impos = [480, 360]

        ## Mission Planning
        self.flag_mission = 0
        self.mission = 0

        self.WP_index = 0
        self.WP_dist = 10.0                                     ## Temporary variable. Waypoint distance. Initial WP_distance must be higher than WP_eta. It should be same with offboard_node_target.cpp (goal_dist = 10.0;)
        self.WP_dir = 0.0                                       ## Temporary variable. Waypoint direction.
        self.WP_eta = 0.5                                       ## if waypoint distance smaller than this distance, waypoint will be updated.
        self.WP_angle_threshold = 5.0                           ## if waypoint angle smaller than this threshold, waypoint will be updated.
        
        self.WayPoint_X = []
        self.WayPoint_Y = []
        self.WayPoint_Z = []
        self.WayPoint_heading = []
        self.pipe_GPS_list = []
        self.pipe_GPS_heading_list = []
        self.WP_num = 0
        self.index_pipe = []

        self.delPos = [0.0, 0.0, 0.0]                           ## Temporary variable to calculate waypoint distance and direction
        self.PreWayPoint = [0.0, 0.0, 0.0]                      ## Temporary variable to calculate waypoint distance and direction

        self.count_balloon = 0
        self.count_balloon_not = 0
        self.count_disappear = 0

        ## Added by TY
        self.flag_extinguish_end = 0
        self.count_facade_fire = 0
        self.total_facade_fire = 15                              ## It should be updated in challenge course.

        self.prev_facade_x = 0.0
        self.prev_facade_y = 0.0
        self.prev_facade_dist = 0.0
        self.pre_flag_extinguish_end = 0

        self.building_back_fire_flag = False

    def set_WayPoint(self):
        if GAZEBO_SIMULATION == True:
            #                                                                          5                                        10      
            self.WayPoint_X =           [    2.5,   2.5,    2.5,    2.5,    2.5,     0.0,     0.0,  -10.0,   -10.0,  -12.5,  -12.5,  -12.5,  -12.5,  -12.5,  -12.5,  -12.5,   -10.0]
            self.WayPoint_Y =           [  -24.0, -20.0,  -29.0,  -29.0,  -32.5,   -32.5,   -32.5,  -32.5,   -32.5,  -32.5,  -32.5,  -28.0,  -28.0,  -22.0,  -22.0,  -17.5,   -17.5]
            self.WayPoint_Z =           [    2.0,  11.0,   11.0,    2.0,    2.0,     2.0,    11.0,   11.0,     2.0,    2.0,    8.0,    8.0,   11.0,   11.0,    6.0,    6.0,     8.0]
            self.WayPoint_heading =     [  180.0, 180.0,  180.0,  180.0,  180.0,    90.0,    90.0,   90.0,    90.0,   90.0,   90.0,    0.0,    0.0,    0.0,    0.0,    0.0,   270.0]
            #                              <<<<<<<<Left_pipe>>>>>>>>>>>             <<<<<<<<     back_pipe    >>>>>>>>>>>>           <<<<<<<   right_pipe >>>>>>>>>           window                                                                                                   
        else:
            if using_GPS_test == True:
                self.pipe_GPS_list = [
                    # start - left
                    [24.4174441, 54.4356676, -20.5446274055], #0

                    # left
                    [24.4174657, 54.4366741, -18.5446274055], #1
                    [24.4174657, 54.4366741, -12.5446274055], #2
                    [24.4175150, 54.4366661, -14.5446274055], #3
                    [24.4175498, 54.4366810, -14.5446274055], #4
                    [24.4175498, 54.4366810, -20.5446274055], #5

                    # left-back corner point
                    [24.4176190, 54.4367031, -20.5446274055]  #6

                    # back
                    # [24.4176115, 54.4367975, -19.5446274055], #6
                    # [24.4176115, 54.4367975, -12.5446274055], #7
                    # [24.4175981, 54.4368452, -14.5446274055], #8
                    # [24.4175815, 54.4369047, -12.5446274055], #9
                    # [24.4175815, 54.4369047, -23.5446274055], #10


                    
                    # back-rignht corner point
                    # [24.4175450, 54.4370213, -19.5446274055], #11

                    # landiing point
                    # [24.4176190, 54.4367031, -19.5446274055], #11

                    # # right
                    # [24.4174778, 54.4370041, -19.5446274055], #12
                    # [24.4174778, 54.4370041, -12.5446274055], #13
                    # [24.4174167, 54.4369876, -14.5446274055], #14
                    # [24.4173809, 54.4369622, -12.5446274055], #15
                    # [24.4173809, 54.4369622, -17.5446274055], #16

                    # # right-front corner point
                    # [24.4173235, 54.4369018, -17.5446274055], #17
                    
                    # # window
                    # [24.4173361, 54.4368366, -17.5446274055]  #18
                ]   ## lat/log/alt

                self.pipe_GPS_heading_list = [  #compass haeding
                    103,
                    103, 103, 103, 103, 103,    # left
                    103                          # left-back corner point
                    # 193, 193, 193, 193, 193,    # back
                    # 13,                        # back-right corner point
                #     283, 283, 283, 283, 283,    # right
                #     103, 13                     # right-front corner point / window
                ]

                # self.pipe_GPS_list = [
                #     [24.4171412, 54.4375711, -21],
                #     [24.4173034, 54.4375863, -21],
                #     [24.4173459, 54.4374938, -21],
                #     [24.4172316, 54.4374381, -21],

                    
                #     [24.4171412, 54.4375711, -21],
                #     [24.4173034, 54.4375863, -21],
                #     [24.4173459, 54.4374938, -21],
                #     [24.4172316, 54.4374381, -21]
                # ]

                # self.pipe_GPS_heading_list = [ #compass heading
                #     103,
                #     193,
                #     283,
                #     13,

                #     103,
                #     193,
                #     283,
                #     13
                # ]
                # self.pipe_GPS_heading_list = [
                #     13,
                #     13,
                #     13,
                #     13,

                #     13,
                #     13,
                #     13,
                #     13
                # ]

                # self.index_pipe = [0, 5, 7, 11, 13, 17, 18] # add index!!
                self.index_pipe = [0, 2, 0] #######################

                self.pipe_GPS_list = map(self.pipe_GPS_list.__getitem__, self.index_pipe)
                self.pipe_GPS_heading_list = map(self.pipe_GPS_heading_list.__getitem__, self.index_pipe)

                # Starting_GPS initialization?
                starting_utm = utm.from_latlon(self.Starting_GPS[0], self.Starting_GPS[1])

                for gps_x, gps_y, gps_z in self.pipe_GPS_list:
                    pipe_utm = utm.from_latlon(gps_x, gps_y)
                    utm_offset = [(pipe_utm[0] - starting_utm[0]), (pipe_utm[1] - starting_utm[1]), (gps_z - self.Starting_GPS[2])]
                    gazebo_offset = get_transform_point(utm_offset[0], utm_offset[1], utm_offset[2])
                    
                    self.WayPoint_X.append(gazebo_offset[0])
                    self.WayPoint_Y.append(gazebo_offset[1])
                    self.WayPoint_Z.append(gazebo_offset[2])
                
                for gps_heading in self.pipe_GPS_heading_list:
                    Wrap_heading = 450.0 - gps_heading     
                    if(Wrap_heading > 360.0):
                        self.WayPoint_heading.append(Wrap_heading - 360.0)
                    else:
                        self.WayPoint_heading.append(Wrap_heading)

            else:
                # Temp_WayPoint_X = [
                #     11.8,   11.8,   11.8,   11.8,   11.8,               # left
                #     11.8,                                               # left-back corner            
                #     2.8,    2.8,    -2.2,   -7.2,   -8.2,               # back
                #     -14.2,                                              # back-right corner
                #     -14.2,  -14.2,  -14.2,  -14.2,  -14.2,              # right
                #     -17.2,  -15.2                                       # right-front corner and window
                # ]
                
                # Temp_WayPoint_Y = [
                #     -13.7,  -13.7,  -19.2,  -23.7,  -23.7,              # left
                #     -29.7,                                              # left-back corner
                #     -29.7,  -29.7,  -29.7,  -29.7,  -29.7,              # back
                #     -29.7,                                              # back-right corner
                #     -23.7,  -23.7,  -17.7,  -13.2,  -13.2,              # right
                #     -6.7,   -6.7                                        # right-front corner and window
                # ]
                
                # Temp_WayPoint_Z = [
                #     5.8,    13.6,   11.0,   11.0,   3.3,
                #     8.0,
                #     5.8,    13.1,   10.8,   13.0,   1.0,
                #     8.0,
                #     5.0,    13.0,   10.8,   13.0,   7.0,
                #     8.0,    8.0
                # ]

                # self.WayPoint_heading = [
                #     257.0,  257.0,  257.0,  257.0,  257.0, .0
                # ]

                Temp_WayPoint_X = [
                    -6.0,                                               # front-left corer
                    -6.0,   -6.0,   -6.0,   -6.0,   -6.0,               # left           
                    -6.0,                                               # left-back corner
                    -12.0,  -12.0,  -17.2,  -24.7,  -24.7,              # back
                    -37.0,                                              # back-right corner
                    -37.0,  -37.0,  -37.0,  -37.0,  -37.0,              # right
                    -37.0                                               # end
                ]
                        
                Temp_WayPoint_Y = [
                    -2.7,                                               # front-left corner
                    -8.5,   -8.5,   -15.6,  -19.5,  -19.5,              # left
                    -26.0,                                              # left-back corner
                    -26.0,  -26.0,  -26.0,  -26.0,  -26.0,              # back
                    -26.0,                                              # back-right corner
                    -15.7,  -15.7,  -9.7,   -5.2,   -5.2,               # right
                    0.0                                                 # end 
                ]
                        
                Temp_WayPoint_Z = [
                    5.8,                                                # front-left corner
                    5.8,    13.6,   11.0,   11.0,   3.3,                # left
                    8.0,                                                # left-back corner
                    5.8,    13.1,   10.8,   13.0,   1.0,                # back
                    8.0,                                                # back-right corner
                    5.0,    13.0,   10.8,   13.0,   7.0,                # right
                    8.0                                                 # end
                ]

                self.WayPoint_heading = [
                    77.0,                                              # front-left corner
                    347.0,  347.0,  347.0,  347.0,  347.0,              # left
                    347.0,                                              # left-back corner
                    257.0,  257.0,  257.0,  257.0,  257.0,              # back
                    257.0,                                              # back-right-corner
                    167.0,  167.0,  167.0,  167.0,  167.0,              # right
                    77.0                                                # end
                ]


                self.index_pipe = [0, 2, 0] #######################

                Temp_WayPoint_X = map(Temp_WayPoint_X.__getitem__, self.index_pipe)
                Temp_WayPoint_Y = map(Temp_WayPoint_Y.__getitem__, self.index_pipe)
                Temp_WayPoint_Z = map(Temp_WayPoint_Z.__getitem__, self.index_pipe)
                self.WayPoint_heading = map(self.WayPoint_heading.__getitem__, self.index_pipe)

                for i in range(len(Temp_WayPoint_X)):
                    gazebo_offset = get_transform_point(Temp_WayPoint_X[i], Temp_WayPoint_Y[i], Temp_WayPoint_Z[i])

                    self.WayPoint_X.append(gazebo_offset[0])
                    self.WayPoint_Y.append(gazebo_offset[1])
                    self.WayPoint_Z.append(gazebo_offset[2])

        self.WP_num = len(self.WayPoint_X)                      ##

    def reset(self):
        #angle = np.arctan2(random()-0.5, random()-0.5)
        #self.init[0] = 1.0*np.cos(angle)
        #self.init[1] = 1.0*np.sin(angle)
        #self.init[2] = (random() + 1.0)

        # Case 1.
        self.init[0] = 0.0
        self.init[1] = 0.0
        self.init[2] = 2.0

        self.N_ballon = self.total_facade_fire
        self.mission = 1

        self.t_cur = 0.0
        self.count = 0

    def uav3_odom_callback(self, data):

        self.Cur_Pos_m[0] = data.pose.pose.position.x
        self.Cur_Pos_m[1] = data.pose.pose.position.y
        self.Cur_Pos_m[2] = data.pose.pose.position.z

        self.q = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,)

        self.euler = trans.transformations.euler_from_quaternion(self.q)
        self.flag_mission = 1

    def uav3_gps_callback(self, data):
        self.Cur_GPS[0] = data.latitude
        self.Cur_GPS[1] = data.longitude
        self.Cur_GPS[2] = data.altitude

    def uav3_compass_heading(self, data):
        self.Cur_GPS_heading = data.data

    def detection_callback(self, data):
        self.flag_detection = data.data[0]
        self.size[0] = data.data[1]
        self.size[1] = data.data[2]
        self.impos[0] = data.data[3]
        self.impos[1] = data.data[4]

    def uav1_mission_callback(self, data):
        # flag_extinguish_end
        # [0] : not end
        # [1] : enough pump
        # [2] : no water
        self.pre_flag_extinguish_end = self.flag_extinguish_end

        self.flag_extinguish_end = data.data[0]
        if self.flag_extinguish_end == 1 and self.pre_flag_extinguish_end != self.flag_extinguish_end:
            # ONLY update when drone go specific WP (line : 298, self.mission == 2: 's final part)
            # self.WP_index = self.WP_index + 1
            if self.WP_index >= self.WP_num:
                self.WP_index = 0
        # else if self.flag_extinguish_end == 2: # NOT NEEDED?
            
    def uav1_building_back_callback(self, data):
        if data.data == 0:
            self.building_back_fire_flag = False
        else:
            self.building_back_fire_flag = True
    
    def mission_flow(self):
        if self.mission == 1:
            ## Mission Start & Take-off
            self.GoalAction_uav3.data.append(1)     # Mission   ## 1 is auto takeoff
            self.GoalAction_uav3.data.append(0)     # init x
            self.GoalAction_uav3.data.append(0)     # init y
            self.GoalAction_uav3.data.append(2.0)   # init z
            self.GoalAction_uav3.data.append(0)     # init r
            self.GoalAction_uav3.data.append(0.5)   # init vx
            self.GoalAction_uav3.data.append(1.5)   # init vz
            self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
            self.GoalAction_uav3.data = []
            dist = np.fabs(2.0 - self.Cur_Pos_m[2])
            if dist < 0.2:
                self.mission = 2

        if self.mission == 2:
            ## Mission to extinughing facade fire
            ##
            ##      14                  13
            ##      6                   5
            ## 15 7 -------------------- 4  12
            ##      |                  |
            ##      |                  |
            ##      |     BUILDING     |
            ##      |                  |
            ##      |                  |
            ##      |                  |
            ## 16 8 -------------------- 3  11
            ##      1                  2  (height = 6.5m)     
            ##      9                 10  (height = 10.5m)
            ##
            ##
            ##              X (Starting point)
            ##
            ##
            ##  - The drone will sweep the building using waypoing
            #     Starting point -> 1 -> 2 -> ... -> 8 ->
            #     Change altitude -> 9 -> ... -> 16 -> Starting point
            ##  - Drone orientation should be changed by waypoint.
            ##  - Sould be considered at waypoint 5,6,13,14.
            ##    On this region, Drone will fly above the ground floor.
            ##    Therefore, drone altitude should be considered ground floor.(Approx. 2.5m)
            ##
            ##      Waypoint |  Heading  | Altitude
            ##      ===============================
            ##        1,  2  |    0 deg  |   6.5m
            ##        3,  4  |   90 deg  |   6.5m
            ##        5,  6  |  180 deg  |   6.5m
            ##        7,  8  |  270 deg  |   6.5m
            ##        9, 10  |    0 deg  |  10.5m
            ##       11, 12  |   90 deg  |  10.5m
            ##       13, 14  |  180 deg  |  10.5m
            ##       15, 16  |  270 deg  |  10.5m
            ##      ===============================
            ##
            ##
            ##  For Testing in Kaist
            ##
            ##      Waypoint |  Heading  | Altitude
            ##      ==============================
            ##        1,  2  |    0 deg  |   1.5m
            ##        3,  4  |   90 deg  |   1.5m
            ##

            # self.delPos[0] = self.prev_facade_x - self.Cur_Pos_m[0]
            # self.delPos[1] = self.prev_facade_y - self.Cur_Pos_m[1]
            # self.self.prev_facade_dist = np.sqrt(self.delPos[0]*self.delPos[0] + self.delPos[1]*self.delPos[1])

            if self.count_facade_fire == self.total_facade_fire:
                
                self.GoalAction_uav3.data.append(3)                                 # Mission   ## 3 is waypoint flight
                self.GoalAction_uav3.data.append(self.WayPoint_X[self.WP_index])    # init x
                self.GoalAction_uav3.data.append(self.WayPoint_Y[self.WP_index])    # init y
                self.GoalAction_uav3.data.append(self.WayPoint_Z[self.WP_index])    # init z
                self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r
                self.GoalAction_uav3.data.append(1.0)                               # init vx
                self.GoalAction_uav3.data.append(1.0)                               # init vz
                self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                self.GoalAction_uav3.data = []

                if(self.WP_index == self.WP_num - 1):
                    self.mission = 3

            if self.building_back_fire_flag == 1:                                   # GO BACK SIDE BUILDING!
                self.GoalAction_uav3.data.append(3)
                self.GoalAction_uav3.data.append(self.WayPoint_X[self.WP_index])    # init x
                self.GoalAction_uav3.data.append(self.WayPoint_Y[self.WP_index])    # init y
                self.GoalAction_uav3.data.append(self.WayPoint_Z[self.WP_index])    # init z
                self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r
                self.GoalAction_uav3.data.append(1.0)                               # init vx
                self.GoalAction_uav3.data.append(0.5)                               # init vz
                self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                self.GoalAction_uav3.data = []

                if self.WP_index > 0:
                    self.GoalAction_uav3.data.append(21)
                    self.GoalAction_uav3.data.append(self.WayPoint_X[self.WP_index])    # init x
                    self.GoalAction_uav3.data.append(self.WayPoint_Y[self.WP_index])    # init y
                    self.GoalAction_uav3.data.append(self.WayPoint_Z[self.WP_index])    # init z
                    self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r
                    self.GoalAction_uav3.data.append(1.0)                               # init vx
                    self.GoalAction_uav3.data.append(0.5)                               # init vz
                    self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                    self.GoalAction_uav3.data = []
            else:
                if self.flag_extinguish_end == 0:
                    self.GoalAction_uav3.data.append(21)                                # Mission   ## 0 is facade mission #########21
                    self.GoalAction_uav3.data.append(self.WayPoint_X[self.WP_index])    # init x
                    self.GoalAction_uav3.data.append(self.WayPoint_Y[self.WP_index])    # init y
                    self.GoalAction_uav3.data.append(self.WayPoint_Z[self.WP_index])    # init z
                    self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r
                    self.GoalAction_uav3.data.append(1.0)                               # init vx
                    self.GoalAction_uav3.data.append(0.5)                               # init vz
                    self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                    self.GoalAction_uav3.data = []
                elif self.flag_extinguish_end == 1 :                                    # enough pump on
                    self.GoalAction_uav3.data.append(21)                                 # Mission   ## 3 is waypoint flight #########21
                    self.GoalAction_uav3.data.append(self.WayPoint_X[self.WP_index])    # init x
                    self.GoalAction_uav3.data.append(self.WayPoint_Y[self.WP_index])    # init y
                    self.GoalAction_uav3.data.append(self.WayPoint_Z[self.WP_index])    # init z
                    self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r
                    self.GoalAction_uav3.data.append(1.0)                               # init vx
                    self.GoalAction_uav3.data.append(0.5)                               # init vz
                    self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                    self.GoalAction_uav3.data = []

                    if(self.pre_flag_extinguish_end != self.flag_extinguish_end):
                        self.count_facade_fire += 1
                # self.WP_index += 1

                        # # Add jump WP_index
                        # if(self.WP_index < 5):
                        #     self.WP_index = 5
                        # elif(self.WP_index > 5 and self.WP_index < 11):
                        #     self.WP_index = 11

                elif self.flag_extinguish_end == 2:                                    # no water
                    # reset_position = [0.0, -3.0, 2.0]  # need check
                    self.GoalAction_uav3.data.append(3)                                # Mission   ## 3 is waypoint flight
                    self.GoalAction_uav3.data.append(self.WayPoint_X[-1])                # init x
                    self.GoalAction_uav3.data.append(self.WayPoint_Y[-1])                # init y
                    self.GoalAction_uav3.data.append(self.WayPoint_Z[-1])                # init z
                    self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index]))                       # init r ??? how to know yaw!?
                    self.GoalAction_uav3.data.append(0.3)                               # init vx
                    self.GoalAction_uav3.data.append(1.0)                               # init vz
                    self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
                    self.GoalAction_uav3.data = []

            # print(self.flag_extinguish_end)


            ## Calculate WP_distance, WP_direction
            if self.WP_index == 0:
                self.PreWayPoint[0] = self.Cur_Pos_m[0]
                self.PreWayPoint[1] = self.Cur_Pos_m[1]
                self.PreWayPoint[2] = self.Cur_Pos_m[2]
            else:
                self.PreWayPoint[0] = self.WayPoint_X[self.WP_index-1]
                self.PreWayPoint[1] = self.WayPoint_Y[self.WP_index-1]
                self.PreWayPoint[2] = self.WayPoint_Z[self.WP_index-1]

            self.delPos[0] = self.WayPoint_X[self.WP_index] - self.Cur_Pos_m[0]
            self.delPos[1] = self.WayPoint_Y[self.WP_index] - self.Cur_Pos_m[1]
            self.delPos[2] = self.WayPoint_Z[self.WP_index] - self.Cur_Pos_m[2]

            self.WP_dist = np.sqrt(self.delPos[0]*self.delPos[0] + self.delPos[1]*self.delPos[1] + self.delPos[2]*self.delPos[2])
            self.WP_dir = np.arctan2(self.WayPoint_Y[self.WP_index]-self.PreWayPoint[1]+eps, self.WayPoint_X[self.WP_index]-self.PreWayPoint[0])

            ## Waypoint update
            if self.WP_dist < self.WP_eta and abs(self.WP_dir) < self.WP_angle_threshold:
                # if self.WP_index != 1:
                #   self.WP_index = self.WP_index + 1			                
                self.WP_index = self.WP_index + 1
             
            if self.WP_index >= self.WP_num:
                self.WP_index = 0

        if self.mission == 3:
            ## Searching Path
            self.GoalAction_uav3.data.append(2)  # Mission
            self.GoalAction_uav3.data.append(self.WayPoint_X[-1])  # init x
            self.GoalAction_uav3.data.append(self.WayPoint_Y[-1])  # init y
            self.GoalAction_uav3.data.append(self.WayPoint_Z[-1])  # init z
            self.GoalAction_uav3.data.append(self.WayPoint_heading[-1])  # init r
            self.GoalAction_uav3.data.append(1.0)  # init vx
            self.GoalAction_uav3.data.append(-0.5)  # init vz
            self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
            self.GoalAction_uav3.data = []


            # self.GoalAction_uav3.data.append(21)    # Mission
            # self.GoalAction_uav3.data.append(self.WayPoint_X[0]) # init x
            # self.GoalAction_uav3.data.append(self.WayPoint_Y[0]) # init y
            # self.GoalAction_uav3.data.append(self.WayPoint_Z[0])  # init z
            # self.GoalAction_uav3.data.append(math.radians(self.WayPoint_heading[self.WP_index])) # init r
            # self.GoalAction_uav3.data.append(0.5)  # init vx
            # self.GoalAction_uav3.data.append(1.0)  # init vz
            # self.pub_GoalAction_uav3.publish(self.GoalAction_uav3)
            # self.GoalAction_uav3.data = []

            # ## Check the facade1 is completed
            # if self.flag_detection == 1:
            #     self.count_facade_fire = self.count_facade_fire + 1

            # ## Check the facade mission completed
            # if self.count_facade_fire >= self.total_facade_fire:
            #     self.mission = 3
            #     self.count_facade_fire = 0


        # if self.mission == 3:
        #     print("here is for mission 3. Which is indoor fire")            ## TODO




"""
        if self.mission == 3:
            ## Tracking Mode
            self.GoalAction_uav1.data.append(6)    # Mission
            self.GoalAction_uav1.data.append(30.0) # init x
            self.GoalAction_uav1.data.append(50.0) # init y
            self.GoalAction_uav1.data.append(2.0)  # init z
            self.GoalAction_uav1.data.append(3.14) # init r
            self.GoalAction_uav1.data.append(2.0)  # init vx
            self.GoalAction_uav1.data.append(1.0)  # init vz
            self.pub_GoalAction_uav1.publish(self.GoalAction_uav1)
            self.GoalAction_uav1.data = []

            if self.flag_detection == 1:
                self.count_balloon_not = 0
            else:
                self.count_balloon_not = self.count_balloon_not + 1

            if self.count_balloon_not > 20.0*2.0:
                self.mission = 2

            if self.size[0]*self.size[1] > 10000.0:
                if self.flag_detection == 0:
                    self.count_disappear = self.count_disappear + 1

                if self.count_disappear > 20.0 * 2.0:
                    self.N_ballon = self.N_ballon - 1
                    self.mission = 2
                    self.count_disappear = 0
"""

def main():
    ros = ros_class()
    rospy.init_node('mission_fire', anonymous=True)

    rospy.Subscriber("/uav1/mavros/global_position/local", Odometry, ros.uav3_odom_callback, queue_size=2)
    rospy.Subscriber("/uav1/mavros/global_position/global", NavSatFix, ros.uav3_gps_callback, queue_size=2)
    rospy.Subscriber("/uav1/mavros/global_position/compass_hdg", Float64, ros.uav3_compass_heading, queue_size=2)

    rospy.Subscriber("/uav1/detection", Float32MultiArray, ros.detection_callback, queue_size=2)

    rospy.Subscriber("/uav1/mission_flag", Int32MultiArray, ros.uav1_mission_callback, queue_size=1)
    rospy.Subscriber("/uav1/building_back_flag", Int32, ros.uav1_building_back_callback, queue_size=1)
    
    
    
    ros.pub_GoalAction_uav3 = rospy.Publisher("/uav1/GoalAction", Float32MultiArray, queue_size=10)

    rate = rospy.Rate(20)  # 20hz

    ros.reset()
    ros.Starting_GPS = ros.Cur_GPS

    while(ros.Starting_GPS[0] == 0 and ros.Starting_GPS[1] == 0 and ros.Starting_GPS[2] == 0):
       ros.Starting_GPS = ros.Cur_GPS

    ros.set_WayPoint()

    while not rospy.is_shutdown():

        if (ros.t_cur % 1.0) == 0:
            print("[time] %.3f [mission] %d [WP index] %d" % (ros.t_cur, ros.mission, ros.WP_index))
            print("[flag_extinguish_end] %d" % (ros.flag_extinguish_end))
            print("[count_facade_fire] %d [total_facade_fire] %d" % (ros.count_facade_fire, ros.total_facade_fire))
            print("[flag] %d [cur]  %.3f  %.3f  %.3f" % (ros.flag_mission, ros.WayPoint_X[ros.WP_index], ros.WayPoint_Y[ros.WP_index], ros.WayPoint_Z[ros.WP_index]))
            print("[WP_dist] %.3f [WP_dir] %.3f" % (ros.WP_dist, ros.WP_dir))
            print("[Cur GPS] %.8f, %.8f, %.8f [Cur GPS heading] %.3f" % (ros.Cur_GPS[0], ros.Cur_GPS[1], ros.Cur_GPS[2], ros.Cur_GPS_heading))
            print("\n\n\n")

        ros.mission_flow()

        #
        # Init --> Disarm and Ready to Arming
        #
        if ros.flag_uav3 == 0:
            ros.GoalAction_uav3.data.append(0)  # Mission
            ros.pub_GoalAction_uav3.publish(ros.GoalAction_uav3)
            ros.GoalAction_uav3.data = []
            ros.PubMissionCallback = 1                                  ##TODO Where is this variable used?
            #if ros.t_cur > 2.0:
            #    ros.flag = 1
        #
        # takeoff
        #
        if ros.flag_uav3 == 1:
            ros.GoalAction_uav3.data.append(1)  # Mission
            ros.GoalAction_uav3.data.append(0.0) # init x
            ros.GoalAction_uav3.data.append(0.0) # init y
            ros.GoalAction_uav3.data.append(1.5)  # init z
            ros.GoalAction_uav3.data.append(0.0)  # init r
            ros.GoalAction_uav3.data.append(1.0)  # init vx
            ros.GoalAction_uav3.data.append(1.5)  # init vz
            ros.pub_GoalAction_uav3.publish(ros.GoalAction_uav3)
            ros.GoalAction_uav3.data = []

        #
        # waypoint
        #
        if ros.flag_uav3 == 2:
            ros.GoalAction_uav3.data.append(3)  # Mission
            ros.GoalAction_uav3.data.append(0.0)  # init x
            ros.GoalAction_uav3.data.append(0.0)  # init y
            ros.GoalAction_uav3.data.append(2.0)  # init z
            ros.GoalAction_uav3.data.append(0.0)  # init r
            ros.GoalAction_uav3.data.append(2.0)  # init vx
            ros.GoalAction_uav3.data.append(1.0)  # init vz
            ros.pub_GoalAction_uav3.publish(ros.GoalAction_uav3)
            ros.GoalAction_uav3.data = []
        #
        # landing
        #
        if ros.flag_uav3 == 3:
            ros.GoalAction_uav3.data.append(2)  # Mission
            ros.GoalAction_uav3.data.append(30.0)  # init x
            ros.GoalAction_uav3.data.append(50.0)  # init y
            ros.GoalAction_uav3.data.append(10.0)  # init z
            ros.GoalAction_uav3.data.append(3.14)  # init r
            ros.GoalAction_uav3.data.append(1.0)  # init vx
            ros.GoalAction_uav3.data.append(-0.5)  # init vz
            ros.pub_GoalAction_uav3.publish(ros.GoalAction_uav3)
            ros.GoalAction_uav3.data = []

        if ros.flag_mission == 1:
            ros.count = ros.count + 1
            ros.t_cur = ros.count / 20.0

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
