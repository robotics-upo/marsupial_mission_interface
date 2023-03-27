#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h> //for interpolation method

using namespace std;

class loadFileMission{
  public:
      loadFileMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
      void readWaypoints();
      void interpolate(float dist);
      void exportDataMissionAndTime(trajectory_msgs::MultiDOFJointTrajectory &t_,vector<float> &v_l_,vector<float> &vg,vector<float> &va);

      trajectory_msgs::MultiDOFJointTrajectory Tj;
      std::vector<float> tether_length_vector, v_time_traj_ugv, v_time_traj_uav;

      float interpolation_dist, offset_map_dll_x ,offset_map_dll_y ,offset_map_dll_z, vel_ugv, vel_uav;
      std::string mission_path, map_name;

  private:
      std::string uav_base_frame, ugv_base_frame, world_frame;
};

loadFileMission::loadFileMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	  printf("\n\tInitialazing Check_Mission_NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));
    pnh.param("mission_path", mission_path, (std::string) "~/");
    pnh.param("map_name", map_name, (std::string) "stage");
    pnh.param("offset_map_"+map_name+"/offset_map_dll_x", offset_map_dll_x, (float)0.0);
    pnh.param("offset_map_"+map_name+"/offset_map_dll_y", offset_map_dll_y, (float)0.0);
    pnh.param("offset_map_"+map_name+"/offset_map_dll_z", offset_map_dll_z, (float)0.0);
    pnh.param("world_frame", world_frame, (std::string) "world");
    pnh.param("interpolation_distance", interpolation_dist, (float)0.2);
    pnh.param("vel_ugv", vel_ugv, (float)1.0);
    pnh.param("vel_uav", vel_uav, (float)1.0);

    printf("loadFileMission : vel_ugv = %f , vel_uav = %f , interpolation_d = %f\n",vel_ugv, vel_uav, interpolation_dist);

    bool latch_topic = true;
    v_time_traj_ugv.clear(); v_time_traj_uav.clear();

    readWaypoints();
    interpolate(interpolation_dist);

    for (size_t i =0; i < Tj.points.size(); i++ ){
      printf("[%lu] UGV[%f %f %f][%f %f %f %f] UAV[%f %f %f][%f %f %f %f] TETHER[%f]\n",i, 
      Tj.points.at(i).transforms[0].translation.x, 
      Tj.points.at(i).transforms[0].translation.y, 
      Tj.points.at(i).transforms[0].translation.z,
      Tj.points.at(i).transforms[0].rotation.x, 
      Tj.points.at(i).transforms[0].rotation.y, 
      Tj.points.at(i).transforms[0].rotation.z, 
      Tj.points.at(i).transforms[0].rotation.w,
      Tj.points.at(i).transforms[1].translation.x, 
      Tj.points.at(i).transforms[1].translation.y, 
      Tj.points.at(i).transforms[1].translation.z,
      Tj.points.at(i).transforms[1].rotation.x, 
      Tj.points.at(i).transforms[1].rotation.y, 
      Tj.points.at(i).transforms[1].rotation.z, 
      Tj.points.at(i).transforms[1].rotation.w,
      tether_length_vector[i]);
      //Compute time for compute trajectory
      if (i < Tj.points.size()-1){
        double d_ = sqrt(pow(Tj.points.at(i+1).transforms[0].translation.x - Tj.points.at(i).transforms[0].translation.x,2)+
                        pow(Tj.points.at(i+1).transforms[0].translation.y - Tj.points.at(i).transforms[0].translation.y,2)+
                        pow(Tj.points.at(i+1).transforms[0].translation.z - Tj.points.at(i).transforms[0].translation.z,2));
        v_time_traj_ugv.push_back(d_/vel_ugv); 
        d_ = sqrt(pow(Tj.points.at(i+1).transforms[1].translation.x - Tj.points.at(i).transforms[1].translation.x,2)+
                  pow(Tj.points.at(i+1).transforms[1].translation.y - Tj.points.at(i).transforms[1].translation.y,2)+
                  pow(Tj.points.at(i+1).transforms[1].translation.z - Tj.points.at(i).transforms[1].translation.z,2));
        v_time_traj_uav.push_back(d_/vel_uav); 
      }
    }
    printf("\n\tFinished read file YAML !!\n");
}

void loadFileMission::readWaypoints()
{
  YAML::Node file = YAML::LoadFile(mission_path);
  trajectory_msgs::MultiDOFJointTrajectoryPoint tj_m_;
  std::string ugv_pos_data, uav_pos_data, tether_data;
  geometry_msgs::Transform ugv_pose, uav_pose;
  float length;
  
  Tj.points.clear(); 
  tj_m_.transforms.resize(2); tj_m_.velocities.resize(2); tj_m_.accelerations.resize(2);
  int size_ = (file["marsupial_ugv"]["size"].as<int>()) ; 
  
  for (int i = 0; i < size_; i++) {
    // It begin in 1 because first point is given as initial point.
    ugv_pos_data = "poses" + std::to_string(i); uav_pos_data = "poses" + std::to_string(i); tether_data = "length" + std::to_string(i);
    try {    
        ugv_pose.translation.x  = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;;
        ugv_pose.translation.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;;
        ugv_pose.translation.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;;
        ugv_pose.rotation.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
        ugv_pose.rotation.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
        ugv_pose.rotation.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
        ugv_pose.rotation.w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
        uav_pose.translation.x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;
        uav_pose.translation.y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;
        uav_pose.translation.z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;
        uav_pose.rotation.x = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
        uav_pose.rotation.y = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
        uav_pose.rotation.z = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
        uav_pose.rotation.w = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
        length = file["tether"][tether_data]["length"].as<double>();
        tj_m_.transforms[0].translation = ugv_pose.translation;
        tj_m_.transforms[0].rotation = ugv_pose.rotation;
        tj_m_.velocities[0].linear.x = 0.0; tj_m_.velocities[0].linear.y = 0.0; tj_m_.velocities[0].linear.z = 0.0;
        tj_m_.accelerations[0].linear.x = 0.0; tj_m_.accelerations[0].linear.y = 0.0; tj_m_.accelerations[0].linear.z = 0.0; 
        tj_m_.transforms[1].translation = uav_pose.translation;
        tj_m_.transforms[1].rotation = uav_pose.rotation;
        tj_m_.velocities[1].linear.x = 0.0; tj_m_.velocities[1].linear.y = 0.0; tj_m_.velocities[1].linear.z = 0.0;
        tj_m_.accelerations[1].linear.x = 0.0; tj_m_.accelerations[1].linear.y = 0.0; tj_m_.accelerations[1].linear.z = 0.0;
        tj_m_.time_from_start = ros::Duration(0.5);
        Tj.points.push_back(tj_m_);
        tether_length_vector.push_back(length); // TODO calculate the distance bw UAV and UGV

    }catch(std::exception &e) {
      ROS_INFO("Skipping waypoint %d", i);
    }
  }
  std::cout << "YAML FILE readed. YAML FILE NAME: " << mission_path << std::endl << "Number of points: " << Tj.points.size() << std::endl;
}

void loadFileMission::interpolate(float dist) 
{
  trajectory_msgs::MultiDOFJointTrajectory new_trajectory;
  std::vector<float> new_length_vector;
  
  if (Tj.points.size() < 2)
    return;
  new_trajectory.points.push_back(Tj.points.at(0));
  new_length_vector.push_back(tether_length_vector.at(0));

  tf::Vector3 uav_p0, uav_p1, ugv_p0, ugv_p1, uav_curr_p, ugv_curr_p;
  tf::Quaternion uav_q0, uav_q1, ugv_q0, ugv_q1, uav_curr_q, ugv_curr_q;
  float length0, length1, curr_length;
  length0 = tether_length_vector.at(0);
  vector3MsgToTF(Tj.points.at(0).transforms[1].translation, uav_p0);
  vector3MsgToTF(Tj.points.at(0).transforms[0].translation, ugv_p0);
  quaternionMsgToTF(Tj.points.at(0).transforms[1].rotation, uav_q0);
  quaternionMsgToTF(Tj.points.at(0).transforms[0].rotation, ugv_q0);

  trajectory_msgs::MultiDOFJointTrajectoryPoint marsupial_point_;
  marsupial_point_.transforms.resize(2);
  marsupial_point_.velocities.resize(2);
  marsupial_point_.accelerations.resize(2);

  for (size_t i = 1; i < Tj.points.size(); i++ ) {
    length1 = tether_length_vector.at(i);
    vector3MsgToTF(Tj.points.at(i).transforms[1].translation, uav_p1);
    vector3MsgToTF(Tj.points.at(i).transforms[0].translation, ugv_p1);
    quaternionMsgToTF(Tj.points.at(i).transforms[1].rotation, uav_q1);
    quaternionMsgToTF(Tj.points.at(i).transforms[0].rotation, ugv_q1);
    
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
  new_trajectory.header = Tj.header;
  tether_length_vector = new_length_vector;
  Tj = new_trajectory;

  ROS_INFO("Interpolated trajectory. New points: %lu", new_length_vector.size());
}

void loadFileMission::exportDataMissionAndTime(trajectory_msgs::MultiDOFJointTrajectory &t_,vector<float> &v_l_,vector<float> &vg,vector<float> &va){
  t_ = Tj;
  v_l_ = tether_length_vector;
  vg = v_time_traj_ugv;
  va = v_time_traj_uav;
}