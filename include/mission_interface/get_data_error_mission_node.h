#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <cmath>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "mission_interface/load_file_mission.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/Navigate3DAction.h>
#include <upo_actions/TakeOffAction.h>

using namespace std;

struct errorDistance
{
	float xy, z;
};

class exportDataMission{

typedef actionlib::SimpleActionClient<upo_actions::Navigate3DAction> Navigate3DClient;
typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;

public:
	exportDataMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
    void updateStates();
    void ugvReachedGoalCB(const upo_actions::NavigateActionResultConstPtr &msg);
    void uavReachedGoalCB(const upo_actions::Navigate3DActionResultConstPtr &msg);
    void lengthStatusCB(const std_msgs::Float32ConstPtr &msg);
    void computeError();
    void exportDataError();
    void initializeVariables();
    void getDistanceTfToTrajectory(geometry_msgs::TransformStamped p_ , geometry_msgs::Vector3 r1_, geometry_msgs::Vector3 r2_, errorDistance &d_);

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;
	
    ros::Subscriber ugv_reached_goal_sub_, uav_reached_goal_sub_, lenth_cat_sub_;

    std::unique_ptr<Navigate3DClient> uavNavigation3DClient;
    std::unique_ptr<NavigateClient> NavigationClient; // For UGV

    trajectory_msgs::MultiDOFJointTrajectory Tj;
    std::vector<float> tether_length_vector, v_length_traj_cat, v_time_ugv, v_time_uav, v_time_traj_ugv, v_time_traj_uav;
    std::vector<float> v_error_t_ugv, v_error_t_uav, v_stand_dev_t_ugv, v_stand_dev_t_uav;
    std::vector<geometry_msgs::Point> v_pose_traj_ugv, v_pose_traj_uav;

    geometry_msgs::TransformStamped uav_tf, ugv_tf;
    geometry_msgs::Point initial_pos_uav, initial_pos_ugv; 
    geometry_msgs::Point current_pos_uav, current_pos_ugv; 
    bool ugv_reached_goal, uav_reached_goal, latch_topic, get_raw_data; 
    float length_status;
    int count;
    int total_pts = -1;
    std::string mission_path, map_name, statistical_results_path;

    int ugv_f_sec, ugv_f_nsec,ugv_s_sec ,ugv_s_nsec, uav_f_sec, uav_f_nsec, uav_s_sec , uav_s_nsec;

    errorDistance error_ugv ,error_uav;
    std::vector<errorDistance> v_error_ugv , v_error_uav;

    float sum_error_ugv_xy, sum_error_ugv_z, sum_error_uav_xy, sum_error_uav_z, sum_error_length;
    float stand_dev_ugv_xy, stand_dev_ugv_z, stand_dev_uav_xy, stand_dev_uav_z, stand_dev_length;
    float average_ugv_xy, average_ugv_z, average_uav_xy, average_uav_z, average_length;
    float max_ugv_xy, max_ugv_z, max_uav_xy, max_uav_z, min_ugv_xy, min_ugv_z, min_uav_xy, min_uav_z, max_length, min_length;
    float sum_error_t_ugv, sum_error_t_uav, stand_dev_t_ugv, mean_t_ugv, mean_t_uav, stand_dev_t_uav, max_t_ugv, max_t_uav, min_t_ugv, min_t_uav;

private:
    std::string uav_base_frame, ugv_base_frame, world_frame;
    double scale_sphere_marker;

protected:

};