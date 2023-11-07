#ifndef MISSION_INTERFACE_H_
#define MISSION_INTERFACE_H_

#include <iostream>
#include <cstdlib>
#include <string>
#include <math.h>
#include <time.h>
#include <sys/timeb.h>
#include <fstream>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <upo_actions/ExecutePathAction.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/Navigate3DAction.h>
#include <upo_actions/TakeOffAction.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>

#include <yaml-cpp/yaml.h>
#include <std_srvs/Trigger.h>

#include "mission_interface/compute_catenary_3D.h"
#include "marsupial_mission_interface/vector_float.h"

#define STOP_ARCO_MISSION_BUTTON   	      1


class MissionInterface
{
  typedef actionlib::SimpleActionClient<upo_actions::Navigate3DAction> Navigate3DClient;
  typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;
  typedef actionlib::SimpleActionClient<upo_actions::TakeOffAction> TakeOffClient;

public:
  MissionInterface(std::string node_name_);
  // ~MissionInterface();
  ros::NodeHandlePtr nh;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;

  std::unique_ptr<actionlib::SimpleClientGoalState> state;
  std::unique_ptr<Navigate3DClient> uavNavigation3DClient;
  std::unique_ptr<NavigateClient> NavigationClient; // For UGV
  std::unique_ptr<TakeOffClient> takeOffClient;
  std::unique_ptr<tf::TransformListener> tf_list_ptr;

  void executeMission();

  void resetFlags();
  void configTopics();
  void configServices();
  void readWaypoints(const std::string &path_file);

  void uavReadyForMissionCB(const std_msgs::BoolConstPtr &msg);
  void ugvReadyForMissionCB(const std_msgs::BoolConstPtr &msg);
  void loadMissionCB(const std_msgs::String &msg);
  void gpsCB(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void startMissionCB(const std_msgs::BoolConstPtr &msg);
  void lengthReachedCB(const std_msgs::BoolConstPtr &msg);
  bool isInitialPose();
  bool UAVisOnTheGround();
  void markerPoints();
  void joyReceivedCB(const sensor_msgs::Joy::ConstPtr& joy);
  void interpolate(float dist);
  void cancelMission();


  bisectionCat BisCat;
  
  ros::Subscriber ugv_state_mission_sub_, uav_state_mission_sub_, start_mission_sub_, gps_sub_;
  ros::Subscriber load_trajectory_sub_, length_reached_sub_, joy_sub_;
  geometry_msgs::Pose init_uav_pose, init_ugv_pose;
  geometry_msgs::Vector3 initial_pose;
  float takeoff_height;
  bool take_off, length_reached;
  double height , flying_height;
  double received_initial_pose;

  trajectory_msgs::MultiDOFJointTrajectory globalTrajectory;
  std::vector<float> tether_length_vector;
  upo_actions::Navigate3DGoal uav_goal3D;
  upo_actions::NavigateGoal ugv_goal3D;
  upo_actions::ExecutePathResult action_result;
    
  trajectory_msgs::MultiDOFJointTrajectory trajectory;
  ros::Publisher traj_uav_pub_, traj_ugv_pub_, catenary_length_pub_;
  ros::Publisher traj_lines_ugv_pub_;
  ros::Publisher traj_lines_uav_pub_, catenary_marker_pub_, reset_length_pub_;

  std::string path_file, ros_node_name;
  std::string ugv_base_frame, uav_base_frame, ugv_odom_frame, uav_odom_frame, world_frame; 
  std::string map_name;
  bool debug;
  int num_wp;
  bool ugv_ready, uav_ready, is_ugv_in_waypoint, is_uav_in_waypoint, start_mission, do_takeoff;
  bool able_tracker_uav, able_tracker_ugv; 
  std::unique_ptr<tf2_ros::TransformListener> tf2_list;
  ros::Time time_count_ugv, time_count_uav;
  double time_max;
  double sent_new_uav_wp , sent_new_ugv_wp ;
  bool used_length_reached; //variable created to test mission node with or without length_reached status

  //Joystick Stuff
  int stopArcoMissionButton;

private:
  double offset_map_dll_x ,offset_map_dll_y ,offset_map_dll_z, offset_ugv_map_dll_x ,offset_ugv_map_dll_y ,offset_ugv_map_dll_z;


};

#endif

