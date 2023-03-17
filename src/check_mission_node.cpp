#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mission_interface/compute_catenary_3D.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <upo_actions/NavigateAction.h>
#include <upo_actions/Navigate3DAction.h>
#include <upo_actions/TakeOffAction.h>

#define PRINTF_BLUE "\x1B[34m"

class checkMission{

typedef actionlib::SimpleActionClient<upo_actions::Navigate3DAction> Navigate3DClient;
typedef actionlib::SimpleActionClient<upo_actions::NavigateAction> NavigateClient;
typedef actionlib::SimpleActionClient<upo_actions::TakeOffAction> TakeOffClient;

public:
	  checkMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
	  void readWaypoints(const std::string &path_file);
    void interpolate(float dist);
    void markerPoints();
    void updateMarkers();
    void ugvReachedGoalCB(const upo_actions::NavigateActionResultConstPtr &msg);
    void uavReachedGoalCB(const upo_actions::Navigate3DActionResultConstPtr &msg);
    void uavTakeOffCB(const upo_actions::TakeOffActionResultConstPtr &msg);
    void goalPointMarker();
    void computeError();

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;
	
    ros::Publisher traj_uav_pub_, traj_ugv_pub_, catenary_length_pub_, goal_point_pub_;
    ros::Publisher traj_lines_ugv_pub_, traj_lines_uav_pub_, catenary_marker_pub_, reset_length_pub_, lines_ugv_pub_, lines_uav_pub_;
    ros::Subscriber ugv_reached_goal_sub_, uav_reached_goal_sub_, uav_take_off_sub_;

    visualization_msgs::Marker lines_ugv, lines_uav; 

    std::unique_ptr<Navigate3DClient> uavNavigation3DClient;
    std::unique_ptr<NavigateClient> NavigationClient; // For UGV
    std::unique_ptr<TakeOffClient> takeOffClient;

    bisectionCat BisCat;

    trajectory_msgs::MultiDOFJointTrajectory trajectory;
    std::vector<float> tether_length_vector;
    std::vector<geometry_msgs::Point> v_pose_traj_ugv, v_pose_traj_uav;

    geometry_msgs::TransformStamped uav_tf, ugv_tf;
    geometry_msgs::Point initial_pos_uav, initial_pos_ugv; 
    geometry_msgs::Point current_pos_uav, current_pos_ugv; 
    bool ugv_reached_goal, uav_reached_goal,uav_take_off; 
    int count;
    int total_pts = -1;
    double offset_map_dll_x ,offset_map_dll_y ,offset_map_dll_z;
    std::string mission_path, map_name, statistical_results_path;
    double final_position_x, final_position_y, final_position_z;
    bool finish_process = false;

private:
    std::string uav_base_frame, ugv_base_frame, world_frame;
    double scale_sphere_marker;

protected:

};

checkMission::checkMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	  printf("\n\tInitialazing Check_Mission_NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));
    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

    pnh.param("mission_path", mission_path, (std::string) "~/");
    pnh.param("statistical_results_path", statistical_results_path, (std::string) "~/");
    pnh.param("map_name", map_name, (std::string) "stage");
    pnh.param("offset_map_"+map_name+"/offset_map_dll_x", offset_map_dll_x, (double)0.0);
    pnh.param("offset_map_"+map_name+"/offset_map_dll_y", offset_map_dll_y, (double)0.0);
    pnh.param("offset_map_"+map_name+"/offset_map_dll_z", offset_map_dll_z, (double)0.0);
    pnh.param("scale_sphere_marker", scale_sphere_marker, (double)0.4);
    pnh.param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
    pnh.param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
    pnh.param("world_frame", world_frame, (std::string) "world");
    pnh.param(map_name+"_pos_final_1/pose/x", final_position_x,(double) 0.0);
    pnh.param(map_name+"_pos_final_1/pose/y", final_position_y,(double) 0.0);
    pnh.param(map_name+"_pos_final_1/pose/z", final_position_z,(double) 0.0);

    traj_ugv_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_ugv", 100);
    traj_uav_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_uav", 100);
    traj_lines_ugv_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_lines_ugv", 100);
    traj_lines_uav_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("trajectory_lines_uav", 100);
    catenary_marker_pub_= pnh.advertise<visualization_msgs::MarkerArray>("trajectory_catenary", 100);
    lines_ugv_pub_ = pnh.advertise<visualization_msgs::Marker>("lines_ugv", 100);
    lines_uav_pub_ = pnh.advertise<visualization_msgs::Marker>("lines_uav", 100);
    bool latch_topic = true;
    goal_point_pub_= pnh.advertise<visualization_msgs::Marker>("/random_planner_node/goal_point", 100,  latch_topic);
	  printf("\n\t\tInitialazed Publishes !!\n");
    ugv_reached_goal_sub_ = pnh.subscribe<upo_actions::NavigateActionResult>("/Navigation/result", 100, &checkMission::ugvReachedGoalCB, this);
    uav_reached_goal_sub_ = pnh.subscribe<upo_actions::Navigate3DActionResult>("/UAVNavigation3D/result", 100, &checkMission::uavReachedGoalCB, this);
    uav_take_off_sub_ = pnh.subscribe<upo_actions::TakeOffActionResult>("/TakeOff/result", 100, &checkMission::uavTakeOffCB, this);


    // DONT FORGET DELETED
    std::string x_ = "/"+map_name+"_pos_final_1/pose/x";
    std::string y_ = "/"+map_name+"_pos_final_1/pose/y";
    std::string z_ = "/"+map_name+"_pos_final_1/pose/z";
    printf("\n\tGoal position:\n");
    printf("\t\t %s = %f\n",x_.c_str(),final_position_x+offset_map_dll_x);
    printf("\t\t %s = %f\n",y_.c_str(),final_position_y+offset_map_dll_y);
    printf("\t\t %s = %f\n",z_.c_str(),final_position_z+offset_map_dll_z);

    count = 0;
    ugv_reached_goal = uav_reached_goal = uav_take_off = false;
    v_pose_traj_ugv.clear();
    v_pose_traj_uav.clear();
      
    readWaypoints(mission_path);

    float interpolation_dist;
    pnh.param<float>("interpolation_distance", interpolation_dist, 0.2);
    interpolate(interpolation_dist);
    total_pts = trajectory.points.size()-1; //always after interpolation

    for (int i = 0 ; i < 5 ; i ++){
      ros::spinOnce();
    }

  for (size_t i =0; i < trajectory.points.size(); i++ ){
    printf("[%lu] UGV[%f %f %f][%f %f %f %f] UAV[%f %f %f][%f %f %f %f] TETHER[%f]\n",
	   i, trajectory.points.at(i).transforms[0].translation.x,
	   trajectory.points.at(i).transforms[0].translation.y,
	   trajectory.points.at(i).transforms[0].translation.z,
	   trajectory.points.at(i).transforms[0].rotation.x,
	   trajectory.points.at(i).transforms[0].rotation.y,
	   trajectory.points.at(i).transforms[0].rotation.z,
	   trajectory.points.at(i).transforms[0].rotation.w,
	   trajectory.points.at(i).transforms[1].translation.x,
	   trajectory.points.at(i).transforms[1].translation.y,
	   trajectory.points.at(i).transforms[1].translation.z,
	   trajectory.points.at(i).transforms[1].rotation.x,
	   trajectory.points.at(i).transforms[1].rotation.y,
	   trajectory.points.at(i).transforms[1].rotation.z,
	   trajectory.points.at(i).transforms[1].rotation.w,
	   tether_length_vector[i]);
  }
	printf("\n\tFinished read file YAML !!\n");

}


void checkMission::ugvReachedGoalCB(const upo_actions::NavigateActionResultConstPtr &msg)
{
  ugv_reached_goal = msg->result.arrived;
}

void checkMission::uavReachedGoalCB(const upo_actions::Navigate3DActionResultConstPtr &msg)
{
  uav_reached_goal = msg->result.arrived;
}

void checkMission::uavTakeOffCB(const upo_actions::TakeOffActionResultConstPtr &msg)
{
  uav_take_off = msg->result.success;
}

void checkMission::readWaypoints(const std::string &path_file)
{
  YAML::Node file = YAML::LoadFile(path_file);

  trajectory.points.clear(); 

  trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;

  traj_marsupial_.transforms.resize(2);
  traj_marsupial_.velocities.resize(2);
  traj_marsupial_.accelerations.resize(2);

  int size_ = (file["marsupial_ugv"]["size"].as<int>()) ; 
  std::string ugv_pos_data, uav_pos_data, tether_data;
  double ugv_pos_x, ugv_pos_y, ugv_pos_z, ugv_rot_x, ugv_rot_y, ugv_rot_z, ugv_rot_w;
  double uav_pos_x, uav_pos_y, uav_pos_z, uav_rot_x, uav_rot_y, uav_rot_z, uav_rot_w;
  printf("offset_map_dll=[%f %f %f]\n",offset_map_dll_x,offset_map_dll_y,offset_map_dll_z);
  for (int i = 0; i < size_; i++) {
    // It begin in 1 because first point is given as initial point.
    ugv_pos_data = "poses" + std::to_string(i);
    uav_pos_data = "poses" + std::to_string(i);
    tether_data = "length" + std::to_string(i);
    try {    
        ugv_pos_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;;
        ugv_pos_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;;
        ugv_pos_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;;
        ugv_rot_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
        ugv_rot_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
        ugv_rot_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
        ugv_rot_w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
        uav_pos_x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;
        uav_pos_y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;
        uav_pos_z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;
        uav_rot_x = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
        uav_rot_y = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
        uav_rot_z = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
        uav_rot_w = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	
        traj_marsupial_.transforms[0].translation.x = ugv_pos_x;
        traj_marsupial_.transforms[0].translation.y = ugv_pos_y;
        traj_marsupial_.transforms[0].translation.z = ugv_pos_z;
        traj_marsupial_.transforms[0].rotation.x = ugv_rot_x;
        traj_marsupial_.transforms[0].rotation.y = ugv_rot_y;
        traj_marsupial_.transforms[0].rotation.z = ugv_rot_z;
        traj_marsupial_.transforms[0].rotation.w = ugv_rot_w;
        traj_marsupial_.velocities[0].linear.x = 0.0;
        traj_marsupial_.velocities[0].linear.y = 0.0;
        traj_marsupial_.velocities[0].linear.z = 0.0;
        traj_marsupial_.accelerations[0].linear.x = 0.0;
        traj_marsupial_.accelerations[0].linear.y = 0.0;
        traj_marsupial_.accelerations[0].linear.z = 0.0;
        traj_marsupial_.transforms[1].translation.x = uav_pos_x;
        traj_marsupial_.transforms[1].translation.y = uav_pos_y;
        traj_marsupial_.transforms[1].translation.z = uav_pos_z;
        traj_marsupial_.transforms[1].rotation.x = uav_rot_x;
        traj_marsupial_.transforms[1].rotation.y = uav_rot_y;
        traj_marsupial_.transforms[1].rotation.z = uav_rot_z;
        traj_marsupial_.transforms[1].rotation.w = uav_rot_w;
        traj_marsupial_.velocities[1].linear.x = 0.0;
        traj_marsupial_.velocities[1].linear.y = 0.0;
        traj_marsupial_.velocities[1].linear.z = 0.0;
        traj_marsupial_.accelerations[1].linear.x = 0.0;
        traj_marsupial_.accelerations[1].linear.y = 0.0;
        traj_marsupial_.accelerations[1].linear.z = 0.0;
        traj_marsupial_.time_from_start = ros::Duration(0.5);
        trajectory.points.push_back(traj_marsupial_);

	    float length = 2.0;
        try {
        length = file["tether"][tether_data]["length"].as<double>();
        } 
        catch (std::exception &e) {}

        tether_length_vector.push_back(length); // TODO calculate the distance bw UAV and UGV

    }catch(std::exception &e) {
      ROS_INFO("Skipping waypoint %d", i);
    }
  }
  std::cout << "YAML FILE readed. YAML FILE NAME: " << path_file << std::endl;
  std::cout << "Number of points: " << trajectory.points.size() << std::endl;
}

void checkMission::interpolate(float dist) 
{
  trajectory_msgs::MultiDOFJointTrajectory new_trajectory;
  std::vector<float> new_length_vector;
  
  if (trajectory.points.size() < 2)
    return;
  new_trajectory.points.push_back(trajectory.points.at(0));
  new_length_vector.push_back(tether_length_vector.at(0));

  tf::Vector3 uav_p0, uav_p1, ugv_p0, ugv_p1, uav_curr_p, ugv_curr_p;
  tf::Quaternion uav_q0, uav_q1, ugv_q0, ugv_q1, uav_curr_q, ugv_curr_q;
  float length0, length1, curr_length;
  length0 = tether_length_vector.at(0);
  vector3MsgToTF(trajectory.points.at(0).transforms[1].translation, uav_p0);
  vector3MsgToTF(trajectory.points.at(0).transforms[0].translation, ugv_p0);
  quaternionMsgToTF(trajectory.points.at(0).transforms[1].rotation, uav_q0);
  quaternionMsgToTF(trajectory.points.at(0).transforms[0].rotation, ugv_q0);

  trajectory_msgs::MultiDOFJointTrajectoryPoint marsupial_point_;
  marsupial_point_.transforms.resize(2);
  marsupial_point_.velocities.resize(2);
  marsupial_point_.accelerations.resize(2);

  for (size_t i = 1; i < trajectory.points.size(); i++ ) {
    length1 = tether_length_vector.at(i);
    vector3MsgToTF(trajectory.points.at(i).transforms[1].translation, uav_p1);
    vector3MsgToTF(trajectory.points.at(i).transforms[0].translation, ugv_p1);
    quaternionMsgToTF(trajectory.points.at(i).transforms[1].rotation, uav_q1);
    quaternionMsgToTF(trajectory.points.at(i).transforms[0].rotation, ugv_q1);
    
    float dist_uav = uav_p1.distance(uav_p0);
    float dist_ugv = ugv_p1.distance(ugv_p0);
    float n = ceil(dist_uav/dist);
    float n_ugv = ceil(dist_ugv/dist);
    n = std::max(n_ugv, n);
    float delta_length = (length1 - length0)/n;
    float delta = 1.0 / (float) n;
    for (int j = 1; j <= n; j++) {
      
      curr_length = length0 + delta_length * j;
      ugv_curr_p.setInterpolate3( ugv_p0, ugv_p1, delta * j);
      uav_curr_p.setInterpolate3( uav_p0, uav_p1, delta * j);
      uav_curr_q = uav_q0;
      uav_curr_q.slerp(uav_q1, delta * j);
      ugv_curr_q = ugv_q0;
      ugv_curr_q.slerp(ugv_q1, delta * j);

      vector3TFToMsg(ugv_curr_p, marsupial_point_.transforms[0].translation);
      quaternionTFToMsg(ugv_curr_q, marsupial_point_.transforms[0].rotation);
      vector3TFToMsg(uav_curr_p, marsupial_point_.transforms[1].translation);
      quaternionTFToMsg(uav_curr_q, marsupial_point_.transforms[1].rotation);

      new_trajectory.points.push_back(marsupial_point_);
      new_length_vector.push_back(curr_length);
    }
    uav_p0 = uav_p1;
    uav_q0 = uav_q1;
    ugv_p0 = ugv_p1;
    ugv_q0 = ugv_q1;
    length0 = length1;
  }
  new_trajectory.header = trajectory.header;
  tether_length_vector = new_length_vector;
  trajectory = new_trajectory;

  ROS_INFO("Interpolated trajectory. New points: %lu", new_length_vector.size());
}

void checkMission::updateMarkers()
{
  try{
      uav_tf = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
      // ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).", uav_base_frame.c_str(),uav_odom_frame.c_str());
    }    
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get UAV Pose (base_frame: %s - world_frame: %s), so not possible to set UAV start point; tf exception: %s",
                uav_base_frame.c_str(),world_frame.c_str(),ex.what());
    }

  try{
      ugv_tf = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
      // ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).", ugv_base_frame.c_str(),uav_odom_frame.c_str());
    }    
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get UGV Pose (base_frame: %s - world_frame: %s), so not possible to set UGV start point; tf exception: %s",
                ugv_base_frame.c_str(),world_frame.c_str(),ex.what());
    }

  if (uav_take_off){
    printf("\tUAV take Off succeessful\n");
    initial_pos_ugv.x = ugv_tf.transform.translation.x;
    initial_pos_ugv.y = ugv_tf.transform.translation.y;
    initial_pos_ugv.z = ugv_tf.transform.translation.z;
    initial_pos_uav.x = uav_tf.transform.translation.x;
    initial_pos_uav.y = uav_tf.transform.translation.y;
    initial_pos_uav.z = uav_tf.transform.translation.z;
    v_pose_traj_ugv.push_back(initial_pos_ugv);
    v_pose_traj_uav.push_back(initial_pos_uav);
    uav_take_off = false;
    count++;
  }

  if (ugv_reached_goal && uav_reached_goal)
  {
    printf("\tReached succeessfully WayPoint %i\n",count);
    current_pos_ugv.x = ugv_tf.transform.translation.x;
    current_pos_ugv.y = ugv_tf.transform.translation.y;
    current_pos_ugv.z = ugv_tf.transform.translation.z;
    current_pos_uav.x = uav_tf.transform.translation.x;
    current_pos_uav.y = uav_tf.transform.translation.y;
    current_pos_uav.z = uav_tf.transform.translation.z;
    v_pose_traj_ugv.push_back(current_pos_ugv);
    v_pose_traj_uav.push_back(current_pos_uav);
    lines_ugv.header.frame_id = "world";
    lines_ugv.header.stamp = ros::Time::now();
    lines_ugv.ns = "lines_ugv";
    lines_ugv.id = count-1;
    lines_ugv.action = visualization_msgs::Marker::ADD;
    lines_ugv.type = visualization_msgs::Marker::LINE_STRIP;
    lines_ugv.lifetime = ros::Duration(0);
    lines_ugv.points.push_back(initial_pos_ugv);
    lines_ugv.points.push_back(current_pos_ugv);
    lines_ugv.pose.orientation.x = 0.0;
    lines_ugv.pose.orientation.y = 0.0;
    lines_ugv.pose.orientation.z = 0.0;
    lines_ugv.pose.orientation.w = 1.0;
    lines_ugv.scale.x = 0.1;
    lines_ugv.color.a = 1.0;
    lines_ugv.color.r = 1.0;
    lines_ugv.color.g = 1.0;
    lines_ugv.color.b = 1.0;
    lines_uav.header.frame_id = "world";
    lines_uav.header.stamp = ros::Time::now();
    lines_uav.ns = "lines_uav";
    lines_uav.id = count-1;
    lines_uav.action = visualization_msgs::Marker::ADD;
    lines_uav.type = visualization_msgs::Marker::LINE_STRIP;
    lines_uav.lifetime = ros::Duration(0);
    lines_uav.points.push_back(initial_pos_uav);
    lines_uav.points.push_back(current_pos_uav);
    lines_uav.pose.orientation.x = 0.0;
    lines_uav.pose.orientation.y = 0.0;
    lines_uav.pose.orientation.z = 0.0;
    lines_uav.pose.orientation.w = 1.0;
    lines_uav.scale.x = 0.1;
    lines_uav.color.a = 1.0;
    lines_uav.color.r = 0.0;
    lines_uav.color.g = 0.0;
    lines_uav.color.b = 0.0;
    lines_ugv_pub_.publish(lines_ugv);
    lines_uav_pub_.publish(lines_uav);
    initial_pos_ugv.x = current_pos_ugv.x;
    initial_pos_ugv.y = current_pos_ugv.y;
    initial_pos_ugv.z = current_pos_ugv.z;
    initial_pos_uav.x = current_pos_uav.x;
    initial_pos_uav.y = current_pos_uav.y;
    initial_pos_uav.z = current_pos_uav.z;
    ugv_reached_goal = false;
    uav_reached_goal = false;
    count++;
    ros::Duration(0.1).sleep();
  }
}

void checkMission::markerPoints()
{
  visualization_msgs::MarkerArray _marker_ugv, _marker_uav ,_lines_ugv, _lines_uav, _cat_marker; 
  trajectory_msgs::MultiDOFJointTrajectory _traj = trajectory;
	geometry_msgs::Point _p1, _p2; 

  _marker_ugv.markers.resize(_traj.points.size());
  _marker_uav.markers.resize(_traj.points.size());
  _lines_ugv.markers.resize(_traj.points.size());
  _lines_uav.markers.resize(_traj.points.size());

  for (size_t i = 0; i < _traj.points.size(); ++i){
    //For UGV
    _marker_ugv.markers[i].header.frame_id = "world";
    _marker_ugv.markers[i].header.stamp = ros::Time::now();
    _marker_ugv.markers[i].ns = "traj_ugv";
    _marker_ugv.markers[i].id = i+1;
    _marker_ugv.markers[i].action = visualization_msgs::Marker::ADD;
    if (i % 5 == 0)
      _marker_ugv.markers[i].type = visualization_msgs::Marker::CUBE;
    else
      _marker_ugv.markers[i].type = visualization_msgs::Marker::SPHERE;
    _marker_ugv.markers[i].lifetime = ros::Duration(0);
    _marker_ugv.markers[i].pose.position.x = _traj.points.at(i).transforms[0].translation.x;
    _marker_ugv.markers[i].pose.position.y = _traj.points.at(i).transforms[0].translation.y; 
    _marker_ugv.markers[i].pose.position.z = _traj.points.at(i).transforms[0].translation.z;
    _marker_ugv.markers[i].pose.orientation.x = 0.0;
    _marker_ugv.markers[i].pose.orientation.y = 0.0;
    _marker_ugv.markers[i].pose.orientation.z = 0.0;
    _marker_ugv.markers[i].pose.orientation.w = 1.0;
    _marker_ugv.markers[i].scale.x = scale_sphere_marker;
    _marker_ugv.markers[i].scale.y = scale_sphere_marker;
    _marker_ugv.markers[i].scale.z = scale_sphere_marker;
    _marker_ugv.markers[i].color.a = 1.0;
    _marker_ugv.markers[i].color.r = 0.1;
    _marker_ugv.markers[i].color.g = 1.0;
    _marker_ugv.markers[i].color.b = 0.1;
    //FOR UAV
    _marker_uav.markers[i].header.frame_id = "world";
    _marker_uav.markers[i].header.stamp = ros::Time::now();
    _marker_uav.markers[i].ns = "traj_uav";
    _marker_uav.markers[i].id = i+1;
    _marker_uav.markers[i].action = visualization_msgs::Marker::ADD;
    if (i % 5 == 0)
      _marker_uav.markers[i].type = visualization_msgs::Marker::CUBE;
    else
      _marker_uav.markers[i].type = visualization_msgs::Marker::SPHERE;
    _marker_uav.markers[i].lifetime = ros::Duration(0);
    _marker_uav.markers[i].pose.position.x = _traj.points.at(i).transforms[1].translation.x;
    _marker_uav.markers[i].pose.position.y = _traj.points.at(i).transforms[1].translation.y; 
    _marker_uav.markers[i].pose.position.z = _traj.points.at(i).transforms[1].translation.z;
    _marker_uav.markers[i].pose.orientation.x = 0.0;
    _marker_uav.markers[i].pose.orientation.y = 0.0;
    _marker_uav.markers[i].pose.orientation.z = 0.0;
    _marker_uav.markers[i].pose.orientation.w = 1.0;
    _marker_uav.markers[i].scale.x = scale_sphere_marker;
    _marker_uav.markers[i].scale.y = scale_sphere_marker;
    _marker_uav.markers[i].scale.z = scale_sphere_marker;
    _marker_uav.markers[i].color.a = 1.0;
    _marker_uav.markers[i].color.r = 0.1;
    _marker_uav.markers[i].color.g = 0.1;
    _marker_uav.markers[i].color.b = 1.0;

    if (i > 0) {
      _p1.x = _traj.points.at(i-1).transforms[0].translation.x;
      _p1.y = _traj.points.at(i-1).transforms[0].translation.y;
      _p1.z = _traj.points.at(i-1).transforms[0].translation.z;
      _p2.x = _traj.points.at(i).transforms[0].translation.x;
      _p2.y = _traj.points.at(i).transforms[0].translation.y;
      _p2.z = _traj.points.at(i).transforms[0].translation.z;
      _lines_ugv.markers[i].header.frame_id = "world";
      _lines_ugv.markers[i].header.stamp = ros::Time::now();
      _lines_ugv.markers[i].ns = "lines_uav";
      _lines_ugv.markers[i].id = i-1;
      _lines_ugv.markers[i].action = visualization_msgs::Marker::ADD;
      _lines_ugv.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      _lines_ugv.markers[i].lifetime = ros::Duration(0);
      _lines_ugv.markers[i].points.push_back(_p1);
      _lines_ugv.markers[i].points.push_back(_p2);
      _lines_ugv.markers[i].pose.orientation.x = 0.0;
      _lines_ugv.markers[i].pose.orientation.y = 0.0;
      _lines_ugv.markers[i].pose.orientation.z = 0.0;
      _lines_ugv.markers[i].pose.orientation.w = 1.0;
      _lines_ugv.markers[i].scale.x = 0.1;
      _lines_ugv.markers[i].color.a = 1.0;
      _lines_ugv.markers[i].color.r = 0.1;
      _lines_ugv.markers[i].color.g = 1.0;
      _lines_ugv.markers[i].color.b = 0.1;
      _p1.x = _traj.points.at(i-1).transforms[1].translation.x;
      _p1.y = _traj.points.at(i-1).transforms[1].translation.y;
      _p1.z = _traj.points.at(i-1).transforms[1].translation.z;
      _p2.x = _traj.points.at(i).transforms[1].translation.x;
      _p2.y = _traj.points.at(i).transforms[1].translation.y;
      _p2.z = _traj.points.at(i).transforms[1].translation.z;
      _lines_uav.markers[i].header.frame_id = "world";
      _lines_uav.markers[i].header.stamp = ros::Time::now();
      _lines_uav.markers[i].ns = "lines_uav";
      _lines_uav.markers[i].id = i-1;
      _lines_uav.markers[i].action = visualization_msgs::Marker::ADD;
      _lines_uav.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
      _lines_uav.markers[i].lifetime = ros::Duration(0);
      _lines_uav.markers[i].points.push_back(_p1);
      _lines_uav.markers[i].points.push_back(_p2);
      _lines_uav.markers[i].pose.orientation.x = 0.0;
      _lines_uav.markers[i].pose.orientation.y = 0.0;
      _lines_uav.markers[i].pose.orientation.z = 0.0;
      _lines_uav.markers[i].pose.orientation.w = 1.0;
      _lines_uav.markers[i].scale.x = 0.1;
      _lines_uav.markers[i].color.a = 1.0;
      _lines_uav.markers[i].color.r = 0.1;
      _lines_uav.markers[i].color.g = 0.1;
      _lines_uav.markers[i].color.b = 1.0;
    }
  }	

  traj_ugv_pub_.publish(_marker_ugv);
  traj_uav_pub_.publish(_marker_uav);
  traj_lines_ugv_pub_.publish(_lines_ugv);
  traj_lines_uav_pub_.publish(_lines_uav);

  std::vector<geometry_msgs::Point> points_catenary_final;
  for (size_t i = 0; i < _traj.points.size(); ++i){
    points_catenary_final.clear();
    BisCat.configBisection(tether_length_vector[i], 
			   _marker_ugv.markers[i].pose.position.x,
			   _marker_ugv.markers[i].pose.position.y,
			   _marker_ugv.markers[i].pose.position.z + 0.4, 
			   _marker_uav.markers[i].pose.position.x,
			   _marker_uav.markers[i].pose.position.y,
			   _marker_uav.markers[i].pose.position.z);
    BisCat.getPointCatenary3D(points_catenary_final);

    _cat_marker.markers.resize(points_catenary_final.size());
            
    for (size_t j = 0; j < points_catenary_final.size(); ++j){
      // double c_color1 = ((double)i / (double)points_catenary_final.size());
			// double c_color2 = ((double)i / (double)points_catenary_final.size());
      _cat_marker.markers[j].header.frame_id = "world";
      _cat_marker.markers[j].header.stamp = ros::Time::now();
      _cat_marker.markers[j].ns = "cat_marker";
      _cat_marker.markers[j].id = i*1000+j;
      _cat_marker.markers[j].action = visualization_msgs::Marker::ADD;
      if (j % 5 == 0 )
	_cat_marker.markers[j].type = visualization_msgs::Marker::CUBE;
      else
	_cat_marker.markers[j].type = visualization_msgs::Marker::SPHERE;
      _cat_marker.markers[j].lifetime = ros::Duration(0);
      _cat_marker.markers[j].pose.position.x = points_catenary_final[j].x; 
      _cat_marker.markers[j].pose.position.y = points_catenary_final[j].y; 
      _cat_marker.markers[j].pose.position.z = points_catenary_final[j].z;

      _cat_marker.markers[j].pose.orientation.x = 0.0;
      _cat_marker.markers[j].pose.orientation.y = 0.0;
      _cat_marker.markers[j].pose.orientation.z = 0.0;
      _cat_marker.markers[j].pose.orientation.w = 1.0;
      _cat_marker.markers[j].scale.x = 0.1;
      _cat_marker.markers[j].scale.y = 0.1;
      _cat_marker.markers[j].scale.z = 0.1;
      _cat_marker.markers[j].color.a = 1.0;
      _cat_marker.markers[j].color.r = 1.0 - i/_traj.points.size();
      _cat_marker.markers[j].color.g = i/_traj.points.size();
      _cat_marker.markers[j].color.b = i/_traj.points.size();
      // _cat_marker.markers[i].color.r = 1.0 - c_color1;
			// _cat_marker.markers[i].color.g = c_color2;
			// _cat_marker.markers[i].color.b = 0.5;
			// _cat_marker.markers[i].color.a = 1.0; 
    }	
    catenary_marker_pub_.publish(_cat_marker);
  }
}

void checkMission::goalPointMarker()
{
	visualization_msgs::Marker marker_;
	marker_.header.frame_id = "world";
	marker_.header.stamp = ros::Time();
	marker_.ns = "goal_point";
	marker_.id = 0;
	marker_.type = visualization_msgs::Marker::SPHERE;
	marker_.action = visualization_msgs::Marker::ADD;
	marker_.lifetime = ros::Duration(0);
	marker_.pose.position.x = final_position_x + offset_map_dll_x;
	marker_.pose.position.y = final_position_y + offset_map_dll_y;
	marker_.pose.position.z = final_position_z + offset_map_dll_z;
	marker_.pose.orientation.x = 0.0;
	marker_.pose.orientation.y = 0.0;
	marker_.pose.orientation.z = 0.0;
	marker_.pose.orientation.w = 1.0;
	marker_.scale.x = 0.4;
	marker_.scale.y = 0.4;
	marker_.scale.z = 0.4;
	marker_.color.r = 1.0;
	marker_.color.g = 0.0;
	marker_.color.b = 0.0;
	marker_.color.a = 1.0; 
	
	goal_point_pub_.publish(marker_);
}

void checkMission::computeError()
{
  trajectory_msgs::MultiDOFJointTrajectory _t = trajectory;
  std::vector<geometry_msgs::Point> v_error_ugv , v_error_uav;
  v_error_ugv.clear(); v_error_uav.clear();

  for(int i = 0 ; i< total_pts; i++)
  {
    geometry_msgs::Point error_ugv_ ,error_uav_;
    error_ugv_.x = (v_pose_traj_ugv[i].x - _t.points.at(i).transforms[0].translation.x);
    error_ugv_.y = (v_pose_traj_ugv[i].y - _t.points.at(i).transforms[0].translation.y);
    error_ugv_.z = (v_pose_traj_ugv[i].z - _t.points.at(i).transforms[0].translation.z);
    v_error_ugv.push_back(error_ugv_);
    error_uav_.x = (v_pose_traj_uav[i].x - _t.points.at(i).transforms[1].translation.x);
    error_uav_.y = (v_pose_traj_uav[i].y - _t.points.at(i).transforms[1].translation.y);
    error_uav_.z = (v_pose_traj_uav[i].z - _t.points.at(i).transforms[1].translation.z);
    v_error_uav.push_back(error_uav_);
    printf("[1]Errors ->  ugv[%f %f %f]  uav[%f %f %f]\n",error_ugv_.x, error_ugv_.y, error_ugv_.z,error_uav_.x, error_uav_.y, error_uav_.z);
  }

  float sum_error_ugv_x, sum_error_ugv_y, sum_error_ugv_z, sum_error_uav_x, sum_error_uav_y, sum_error_uav_z;
  sum_error_ugv_x = sum_error_ugv_y = sum_error_ugv_z = sum_error_uav_x = sum_error_uav_y = sum_error_uav_z = 0.0;
  float average_ugv_x, average_ugv_y, average_ugv_z, average_uav_x, average_uav_y, average_uav_z;
  average_ugv_x = average_ugv_y = average_ugv_z = average_uav_x = average_uav_y = average_uav_z = 0.0;
  float max_ugv_x, max_ugv_y, max_ugv_z, max_uav_x, max_uav_y, max_uav_z, min_ugv_x, min_ugv_y, min_ugv_z, min_uav_x, min_uav_y, min_uav_z;
  max_ugv_x = max_ugv_y = max_ugv_z = max_uav_x = max_uav_y = max_uav_z = 0.0;
  min_ugv_x = min_ugv_y = min_ugv_z = min_uav_x = min_uav_y = min_uav_z = 100000.0;

  for(int i = 0 ; i< total_pts; i++){
    sum_error_ugv_x = v_error_ugv[i].x + sum_error_ugv_x; 
    sum_error_ugv_y = v_error_ugv[i].y + sum_error_ugv_y; 
    sum_error_ugv_z = v_error_ugv[i].z + sum_error_ugv_z; 
    sum_error_uav_x = v_error_uav[i].x + sum_error_uav_x; 
    sum_error_uav_y = v_error_uav[i].y + sum_error_uav_y; 
    sum_error_uav_z = v_error_uav[i].z + sum_error_uav_z; 

    if(min_ugv_x > v_error_ugv[i].x)
      min_ugv_x = v_error_ugv[i].x;
    if(min_ugv_y > v_error_ugv[i].y)
      min_ugv_y = v_error_ugv[i].y;
    if(min_ugv_z > v_error_ugv[i].z)
      min_ugv_z = v_error_ugv[i].z;
    if(max_ugv_x < v_error_ugv[i].x)
      max_ugv_x = v_error_ugv[i].x;
    if(max_ugv_y < v_error_ugv[i].y)
      max_ugv_y = v_error_ugv[i].y;
    if(max_ugv_z < v_error_ugv[i].z)
      max_ugv_z = v_error_ugv[i].z;

    if(min_uav_x > v_error_uav[i].x)
      min_uav_x = v_error_uav[i].x;
    if(min_uav_y > v_error_uav[i].y)
      min_uav_y = v_error_uav[i].y;
    if(min_uav_z > v_error_uav[i].z)
      min_uav_z = v_error_uav[i].z;
    if(max_uav_x < v_error_uav[i].x)
      max_uav_x = v_error_uav[i].x;
    if(max_uav_y < v_error_uav[i].y)
      max_uav_y = v_error_uav[i].y;
    if(max_uav_z < v_error_uav[i].z)
      max_uav_z = v_error_uav[i].z;
    printf("[2]Errors ->  ugv[%f %f %f]  uav[%f %f %f]\n",v_error_ugv[i].x, v_error_ugv[i].y,v_error_ugv[i].z, v_error_uav[i].x, v_error_uav[i].y, v_error_uav[i].z);
    printf("[1]Average ->  ugv[%f %f %f]  uav[%f %f %f]\n",sum_error_ugv_x, sum_error_ugv_y, sum_error_ugv_z,
                                                           sum_error_uav_x, sum_error_uav_y, sum_error_uav_z);
  }
  average_ugv_x = sum_error_ugv_x/total_pts;
  average_ugv_y = sum_error_uav_x/total_pts;
  average_ugv_z = sum_error_ugv_y/total_pts;
  average_uav_x = sum_error_uav_y/total_pts;
  average_uav_y = sum_error_ugv_z/total_pts;
  average_uav_z = sum_error_uav_z/total_pts;

  float stand_dev_ugv_x, stand_dev_ugv_y, stand_dev_ugv_z, stand_dev_uav_x, stand_dev_uav_y, stand_dev_uav_z;
  stand_dev_ugv_x = stand_dev_ugv_y = stand_dev_ugv_z = stand_dev_uav_x = stand_dev_uav_y = stand_dev_uav_z = 0.0;
  for(int i = 0 ; i< total_pts; i++){
    stand_dev_ugv_x = sqrt(pow(v_error_ugv[i].x-average_ugv_x,2)/(total_pts-1)); 
    stand_dev_ugv_y = sqrt(pow(v_error_ugv[i].y-average_ugv_y,2)/(total_pts-1));
    stand_dev_ugv_z = sqrt(pow(v_error_ugv[i].z-average_ugv_z,2)/(total_pts-1));
    stand_dev_uav_x = sqrt(pow(v_error_uav[i].x-average_uav_x,2)/(total_pts-1));
    stand_dev_uav_y = sqrt(pow(v_error_uav[i].y-average_uav_y,2)/(total_pts-1));
    stand_dev_uav_z = sqrt(pow(v_error_uav[i].z-average_uav_z,2)/(total_pts-1));
  }


  // Save Data for analysis
  std::ofstream ofs_data_analysis;
  std::string output_file = statistical_results_path+"statistical_results_from_validation_experiments.txt";
  std::ifstream ifile1;
  ifile1.open(output_file);
  if(ifile1) {
      std::cout << output_file <<" : File exists !!!!!!!!!! " << std::endl;
  } else {
  ofs_data_analysis.open(output_file.c_str(), std::ofstream::app);
  ofs_data_analysis <<"total_pts"<<std::endl;
  ofs_data_analysis <<total_pts<<std::endl;
  ofs_data_analysis <<"average_ugv_x;average_ugv_y;average_ugv_z;average_uav_x;average_uav_y;average_uav_z"<<std::endl;
  ofs_data_analysis <<average_ugv_x<<";"
                    <<average_ugv_y<<";"
                    <<average_ugv_z<<";"
                    <<average_uav_x<<";"
                    <<average_uav_y<<";"
                    <<average_uav_z<<std::endl;
  ofs_data_analysis <<"stand_dev_ugv_x;stand_dev_ugv_y;stand_dev_ugv_z;stand_dev_uav_x;stand_dev_uav_y;stand_dev_uav_z"<<std::endl;
  ofs_data_analysis <<stand_dev_ugv_x<<";"
                    <<stand_dev_ugv_y<<";"
                    <<stand_dev_ugv_z<<";"
                    <<stand_dev_uav_x<<";"
                    <<stand_dev_uav_y<<";"
                    <<stand_dev_uav_z<<std::endl;
  ofs_data_analysis <<"max_ugv_x,max_ugv_y;max_ugv_z;max_uav_x;max_uav_y;max_uav_z"<<std::endl;
  ofs_data_analysis <<max_ugv_x<<";"
                    <<max_ugv_y<<";"
                    <<max_ugv_z<<";"
                    <<max_uav_x<<";"
                    <<max_uav_y<<";"
                    <<max_uav_z<<std::endl;
  ofs_data_analysis <<"min_ugv_x;min_ugv_y;min_ugv_z;min_uav_x;min_uav_y;min_uav_z"<<std::endl;
  ofs_data_analysis <<min_ugv_x<<";"
                    <<min_ugv_y<<";"
                    <<min_ugv_z<<";"
                    <<min_uav_x<<";"
                    <<min_uav_y<<";"
                    <<min_uav_z<<std::endl;

  ofs_data_analysis.close();
  std::cout << output_file <<" : Saved data for analysis !!!!!!!!!! " << std::endl;
  }
  finish_process = true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Check Mission Node");

  ros::NodeHandlePtr nh;
  ros::NodeHandle pnh("~");

  checkMission cm(nh, pnh);
  bool continue_process = true;

      
	while (ros::ok() && continue_process) {
    ros::spinOnce();
    if (cm.count < cm.total_pts)
      cm.updateMarkers();
    else
      cm.computeError();
    cm.markerPoints();
    cm.goalPointMarker();

    if(cm.finish_process){
      continue_process = cm.finish_process;
      printf("cm.finish_process %s\n",cm.finish_process?"true":"false");
    }
  }	
	
	return 0;
}