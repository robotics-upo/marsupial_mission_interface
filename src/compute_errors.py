#!/usr/bin/env python3
# license removed for brevity
import rospy
from tf import TransformListener
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Vector3Stamped
from upo_actions.msg import Navigate3DActionGoal
from upo_actions.msg import NavigateActionGoal
from upo_actions.msg import Navigate3DActionResult
from upo_actions.msg import NavigateActionResult
from math import sqrt
from datetime import datetime
from datetime import timedelta
import numpy as np

# Global data
uav_waypoint = None
ugv_waypoint = None

class MissionReport:
    def __init__(self):
        self.tt = 0
        self.t = 0
        # Subscribers
        rospy.Subscriber("/Navigation/result", NavigateActionResult, self.ugvReachedGoalCB)
        rospy.Subscriber("/UAVNavigation3D/result", Navigate3DActionResult, self.uavReachedGoalCB)
        rospy.Subscriber("/Navigation/goal", NavigateActionGoal, self.ugvGoalCB)
        rospy.Subscriber("/UAVNavigation3D/goal", Navigate3DActionGoal, self.uavGoalCB)
        rospy.Subscriber("/tie_controller/length_status", Float32, self.lengthCB)
        rospy.Subscriber("/tie_controller/set_length", Float32, self.setLengthCB)
        self.tf_list = None
        self.tf_text_file = None
        self.ugv_time_vector = []
        self.uav_time_vector = []
        self.ugv_velocities = []
        self.uav_velocities = []
        self.ugv_distances = []
        self.uav_distances = []
        self.ugv_ate = []
        self.uav_ate = []
        self.ugv_goal = None
        self.uav_goal = None
        self.uav_pos = None
        self.ugv_pos = None
        self.init_ugv_pos = None
        self.init_uav_pos = None
        self.uav_xy_errors = []
        self.uav_z_errors = []
        self.ugv_xy_errors = []
        self.tether_errors = []
        self.curr_length = None
        self.uav_reached_goal = True
        self.ugv_reached_goal = True
        # Parameters

        # Frames: the first is the UAV, second UGV
        tfs = rospy.get_param("~frames", default="base_link arco/base_link") 
        self.tf_list = tfs.split(" ")
        print("Frames:", self.tf_list)
        self.tf_filename = rospy.get_param('~tf_filename', default='length_stats.txt')

        self.stats_filename = rospy.get_param('~stats_filename', default='stats.txt')
        self.global_frame_id = rospy.get_param('~global_frame_id', default="world")
        print("Global Frame:", self.global_frame_id)

        # Related to vehicles
        self.ugv_speed = float(rospy.get_param('~ugv_speed', default='0.25'))
        self.uav_speed = float(rospy.get_param('~uav_speed', default='0.25'))

        print("UAV speed: ", self.uav_speed, "\tUGV Speed: ", self.ugv_speed)
    
    def ugvGoalCB(self, data):
        self.ugv_reached_goal = False
        p = data.goal.global_goal.position
        self.ugv_prev_goal = self.ugv_pos
        self.ugv_goal = np.array([p.x, p.y, p.z])
        self.ugv_distance = np.linalg.norm(self.ugv_goal - self.ugv_prev_goal)
        #self.ugv_distances.append(self.ugv_distance)
        print ("UGV New goal: ", self.ugv_goal, "Distance: ", self.ugv_distance)
        self.ugv_s = float(data.header.stamp.secs) + data.header.stamp.nsecs * 1e-9
        self.init_ugv_pos = self.ugv_pos

    def uavGoalCB(self, data):
        nsec = data.header.stamp.nsecs
        sec = data.header.stamp.secs
        self.uav_s = float(sec) + 1e-9*nsec
        self.init_uav_pos = self.uav_pos
        self.uav_reached_goal = False
        p = data.goal.global_goal.pose.position
        self.uav_prev_goal = self.uav_pos
        self.uav_goal = np.array([p.x, p.y, p.z])
        self.uav_distance = np.linalg.norm(self.uav_goal - self.init_uav_pos)
        
        print ("UAV New goal: ", self.uav_goal, "Distance: ", self.uav_distance)
        

    def ugvReachedGoalCB(self, data):
        self.ugv_reached_goal = data.result.arrived
        nsec = data.header.stamp.nsecs
        sec = data.header.stamp.secs
        self.ugv_distance = np.linalg.norm(self.ugv_pos - self.init_ugv_pos)
        self.ugv_distances.append(self.ugv_distance)
        
        ugv_f = float(sec) + nsec*1e-09
        delta_t = ugv_f - self.ugv_s
        self.ugv_time_vector.append(delta_t)
        self.ugv_speed = self.ugv_distance / delta_t
        self.ugv_velocities.append(self.ugv_speed)

        self.ugv_ate.append(self.get2Ddist(self.ugv_pos, self.ugv_goal)-0.25)
        
        print("Received UGV Reached Goal CB. Delta_T: ", delta_t ,
              "Speed: ", self.ugv_speed )

    def uavReachedGoalCB(self,data):
        nsec = data.header.stamp.nsecs
        sec = data.header.stamp.secs
        uav_f = float(sec) + 1e-9*nsec
        self.uav_reached_goal = data.result.arrived
        self.uav_z_errors.append(self.uav_pos[2] - self.uav_goal[2])
        self.uav_distance = self.get2Ddist(self.uav_pos, self.init_uav_pos)
        self.uav_distances.append(self.uav_distance)
        
        delta_t = uav_f - self.uav_s
        self.uav_time_vector.append(delta_t)
        self.uav_speed = self.uav_distance / delta_t
        self.uav_velocities.append(self.uav_speed)
        self.uav_ate.append(self.get2Ddist(self.uav_pos, self.uav_goal)-0.25)
        
        

        print("Received UAV Reached Goal CB. Delta_T: ", delta_t ,
              "Speed: ", self.uav_speed )

    def lengthCB(self, data):
        self.curr_length = data.data

    def setLengthCB(self, data):
        self.length_ref = data.data

    def update_errors(self):
        if (self.curr_length is None or self.uav_goal is None
            or self.ugv_goal is None or self.uav_pos is None
            or self.ugv_pos is None):
            return
        # Get the tether related errors
        if self.uav_reached_goal and self.ugv_reached_goal:
            return

        self.tether_errors.append(self.curr_length - self.length_ref + 0.5)

        exy = self.calculate_xy_error(self.uav_goal, self.init_uav_pos, self.uav_pos)
        self.uav_xy_errors.append(exy)
        

        exy = self.calculate_xy_error(self.ugv_goal, self.init_ugv_pos, self.ugv_pos)
        self.ugv_xy_errors.append(exy)
        
    def calculate_xy_error(self, next_wp, last_wp, pos):
        delta_wp = next_wp - last_wp
        delta_wp[2] = 0.0
        u_wp = delta_wp / np.linalg.norm(delta_wp)
        r = pos - last_wp
        exy = sqrt(np.dot(r,r) - np.dot(u_wp, r) ** 2)
        return exy
    
    def get2Ddist(self, pos1, pos2):
        p1 = pos1[:-1]
        p2 = pos2[:-1]
        return np.linalg.norm(p1 - p2)             

    def update_pos(self):
        t = rospy.Time.now()
        cont = 0
        for i in self.tf_list:
            try:
                pos, ori = tf_listener.lookupTransform(self.global_frame_id, i, rospy.Time(0))

                if cont == 0:
                    self.uav_pos=np.array(pos)
                else:
                    self.ugv_pos=np.array(pos)

                cont += 1

                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Malo")

    def export_stats(self):
        stats_file = open(self.stats_filename, 'w')
        l = min(len(self.uav_time_vector), len(self.ugv_time_vector))
        stats_file.write("\% uav_time\tuav_dist\tuav_vel \tuav_z\tugv_time\tugv_dist\tugv_vel\n")
        for i in range(l):
            stats_file.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\n"%( self.uav_time_vector[i],
                                                     self.uav_distances[i],
                                                     self.uav_velocities[i],
                                                     self.uav_z_errors[i],
                                                     self.ugv_time_vector[i],
                                                     self.ugv_distances[i],
                                                     self.ugv_velocities[i]))
        stats_file.close()

    def export_length_stats(self):
        tf_file = open(self.tf_filename, "w")
        tf_file.write("# uav_xy_e\tugv_xy_e\ttether_e\n")
        for i in range(len(self.tether_errors)):
            tf_file.write("%f\t%f\t%f\t\n"%( self.uav_xy_errors[i],
                                                self.ugv_xy_errors[i],
                                                self.tether_errors[i]))
        tf_file.close()
                                                
    def load_stats(self):
        with  open(self.stats_filename, 'r') as stats_file:
            data = np.loadtxt(stats_file)
            self.uav_time_vector = data[:,0]
            self.uav_distances = data[:,1]
            self.uav_velocities = data[:,2]
            self.ugv_time_vector = data[:,3]
            self.ugv_distances = data[:,4]
            self.ugv_velocities = data[:,5]
        with open(self.tf_filename, 'r') as stats_file:
            data = np.loadtxt(stats_file)
            self.uav_xy_errors = data[:,0]
            self.uav_z_errors = data[:,1]
            self.ugv_xy_errors = data[:,2]
            self.tether_errors = data[:,3]


        
        
    def calculate_errors(self):
        self.ugv_time_errors = []
        self.ugv_vel_errors = []
        self.uav_time_errors = []
        self.uav_vel_errors = []
        for i in range(len(self.uav_time_vector)): 
            # Get errors
            vel = self.uav_velocities[i]
            dis = self.uav_distances[i]
            t = self.uav_time_vector[i]
            print (vel,dis, t)
            error_vel, error_time = self.calculate_error(vel,dis, t, self.uav_speed)
            if (error_vel is not None):
                self.uav_time_errors.append(error_time)
                self.uav_vel_errors.append(error_vel)
        for i in range(len(self.ugv_time_vector)):
            # Get errors UGV
            v = self.ugv_velocities[i]
            d = self.ugv_distances[i]
            t = self.ugv_time_vector[i]
            error_vel, error_time = self.calculate_error(v, d, t, self.ugv_speed)
            if (error_vel is not None):
                self.ugv_time_errors.append(error_time)
                self.ugv_vel_errors.append(error_vel)
        # Error report
        print ("UAV Error times: ", self.uav_time_errors)
        print ("UAV Error velocities: ", self.uav_vel_errors)
        print ("\n\nUGV Error times: ", self.ugv_time_errors)
        print ("UGV Error velocities: ", self.ugv_vel_errors)

        print ("\n\n\n")
        stats_time_uav = self.get_vector_stats(self.uav_time_errors[:-1])
        print ("UAV time error (min/meanRMSE/max)",  stats_time_uav)
        stats_vel_uav = self.get_vector_stats(self.uav_vel_errors[:-1])
        print ("UAV velocity error (min/mean/RMSE/max)",  stats_vel_uav)

        print ("\n")
        stats_time_ugv = self.get_vector_stats(self.ugv_time_errors[:-1])
        print ("UGV time error (min/mean/RMSE/max)",  stats_time_ugv)
        stats_vel_ugv = self.get_vector_stats(self.ugv_vel_errors[:-1])
        print ("UGV velocity error (min/mean/RMSE/max)",  stats_vel_ugv)

        print ("\n")
        stats_length = self.get_vector_stats(self.tether_errors[:-1])
        print ("Tether length error (min/mean/RMSE/max)",  stats_length)

        print ("\n")
        stats_ugv_xy = self.get_vector_stats(self.ugv_xy_errors[:-1])
        print ("UGV xy error (min/mean/RMSE/max)",  stats_ugv_xy)

        print ("\n")
        stats_uav_xy = self.get_vector_stats(self.uav_xy_errors[:-1])
        print ("UAV xy error (min/mean/RMSE/max)",  stats_uav_xy)
        stats_uav_z = self.get_vector_stats(self.uav_z_errors[:-1])
        print ("UAV z error (min/mean/RMSE/max)",  stats_uav_z)

        stats_uav_ate = self.get_vector_stats(self.uav_ate[:-1])
        print("UAV ATE:", stats_uav_ate )
        stats_ugv_ate = self.get_vector_stats(self.ugv_ate[:-1])
        print("UGV ATE:", stats_ugv_ate )
        

    def get_vector_stats(self, vec):
        min = np.amin(np.abs(vec))
        max = np.amax(np.abs(vec))
        mean = np.abs(vec).mean()
        rmse = sqrt(np.square(vec).mean())
        std = np.std(vec)

        return min, mean, rmse,  max, std

    def calculate_error(self, v, d, t, v_0):
        e_v = None
        e_t = -1.0
        if (d > 0.15 and v > 0.05 and v < 0.5):
            e_v = v - v_0
            e_t = t - d/v_0
        return e_v, e_t
                
    def print_stats(self):
        print ("\n\nUGV\n\n")
        print ("UGV time vector\n", self.ugv_time_vector)
        print ("\nUGV velocities:\n", self.ugv_velocities)
        print ("Distances:\n", self.ugv_distances)
    
        print ("\n\nUAV\n\n")
        print ("\nUAV time vector:\n", self.uav_time_vector)
        print ("\nUAV velocities:\n", self.uav_velocities)
        print ("Distances:\n", self.uav_distances)

if __name__ == '__main__':
    rospy.init_node('mission_report')
    tf_listener = TransformListener()
    mission_report = MissionReport()
  
    # Create a rate
    hz = rospy.get_param("~rate", default = 10.0)
    rate = rospy.Rate(hz)

    get_data_from_file = bool(rospy.get_param("~get_data_from_file", default = "false"))
    if get_data_from_file:
        print("Reading data from file")
        mission_report.load_stats()
        mission_report.print_stats()
        mission_report.calculate_errors()
    else:
        # Getting the data from ROS
        try:
            
            while not rospy.is_shutdown():
                mission_report.update_pos()
                mission_report.update_errors()
                rate.sleep()
        except (rospy.exceptions.ROSInterruptException):
                print("Closing mission report\n\n")
                
        mission_report.print_stats()
        mission_report.export_stats()
        mission_report.export_length_stats()
        mission_report.calculate_errors()

