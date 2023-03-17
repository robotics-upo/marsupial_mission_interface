
#include <mission_interface/mission_interface.h>
#include <tf/transform_datatypes.h>
#include<stdlib.h>

MissionInterface::MissionInterface(std::string node_name_)
{
  nh.reset(new ros::NodeHandle("~"));
    
  tfBuffer.reset(new tf2_ros::Buffer);
  tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

  nh->param("path_file", path_file, (std::string) "~/");
  nh->param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
  nh->param("ugv_odom_frame", ugv_odom_frame, (std::string) "ugv/odom");
  nh->param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
  nh->param("uav_odom_frame", uav_odom_frame, (std::string) "ugv/odom");
  nh->param("world_frame", world_frame, (std::string) "world");
  nh->param("map_name", map_name, (std::string) "stage");
  nh->param("offset_map_"+map_name+"/offset_map_dll_x", offset_map_dll_x, (double)0.0);
  nh->param("offset_map_"+map_name+"/offset_map_dll_y", offset_map_dll_y, (double)0.0);
  nh->param("offset_map_"+map_name+"/offset_map_dll_z", offset_map_dll_z, (double)0.0);
  nh->param("flying_height", flying_height, (double)0.3);
  nh->param("time_max", time_max, (double)1.5);

  nh->param<bool>("able_tracker_ugv",able_tracker_ugv, true);
  nh->param<bool>("able_tracker_uav",able_tracker_uav, true);
  nh->param<bool>("used_length_reached",used_length_reached, true);
  nh->param("takeoff_height",takeoff_height, (float)1.0);
  nh->param<bool>("do_takeoff",do_takeoff, true);


  nh->param<int>("stop_arco_mission_button", stopArcoMissionButton, STOP_ARCO_MISSION_BUTTON);

  ros_node_name = node_name_;
  ROS_INFO("Initialized Node : %s", ros_node_name.c_str());

  take_off = false;
  received_initial_pose = false;
  is_ugv_in_waypoint = is_uav_in_waypoint = false;
  sent_new_ugv_wp = sent_new_uav_wp = false;

  double sleep_time = 0.0;
  nh->param<double>("sleep_time", sleep_time, 2.0);
  ROS_INFO("Sleep %f seconds", sleep_time);
  ros::Duration(sleep_time).sleep();


  geometry_msgs::TransformStamped uav_tf_;
  readWaypoints(path_file);

  float interpolation_dist;
  nh->param<float>("interpolation_distance", interpolation_dist, 0.2);
  interpolate(interpolation_dist);
  configTopics();

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
  resetFlags();
  markerPoints();
  ros::spinOnce();
  configServices();
    
    if(used_length_reached){
      ROS_INFO("Reset Tether Length to: %f", tether_length_vector[0]);
      std_msgs::Float32 reset_msg_;
      reset_msg_.data = tether_length_vector[0];
      for(int i = 0; i < 3 ; i++){
        reset_length_pub_.publish(reset_msg_);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
      }
    }

  markerPoints();

  if (able_tracker_uav){
    try{
      uav_tf_ = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
      initial_pose  = uav_tf_.transform.translation;
      ROS_INFO("\tGot initial UAV position uav_tf_[%f %f %f]",uav_tf_.transform.translation.x,uav_tf_.transform.translation.y,uav_tf_.transform.translation.z);
      received_initial_pose = true;
    }    
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get position initial UAV (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s",
        uav_base_frame.c_str(),uav_odom_frame.c_str(),ex.what());
    }
  }
}

void MissionInterface::interpolate(float dist) {
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

//Config standard services and action lib servers and clients
void MissionInterface::configServices()
{
    if(able_tracker_uav){
        ROS_INFO("%s Node: Initialazing UAV Navigation Client", ros_node_name.c_str());
        uavNavigation3DClient.reset(new Navigate3DClient("/UAVNavigation3D", true));
        uavNavigation3DClient->waitForServer();
        ROS_INFO("%s Node: Initialazing TakeOff Client", ros_node_name.c_str());
        takeOffClient.reset(new TakeOffClient("/TakeOff", true));
        takeOffClient->waitForServer();
    }
    if(able_tracker_ugv){
        ROS_INFO("%s Node: Initialazing UGV Navigation Client", ros_node_name.c_str());
        NavigationClient.reset(new NavigateClient("/Navigation", true));
        NavigationClient->waitForServer();
    }
}

void MissionInterface::configTopics()
{
  ROS_INFO("%s Node: Initialazing Topics", ros_node_name.c_str());
  if(able_tracker_ugv){
    ugv_state_mission_sub_ = nh->subscribe<std_msgs::Bool>
      ("ugv_ready_for_mission", 1, &MissionInterface::ugvReadyForMissionCB, this);
  }
  if(able_tracker_uav){
    uav_state_mission_sub_ = nh->subscribe<std_msgs::Bool>
      ("uav_ready_for_mission", 1, &MissionInterface::uavReadyForMissionCB, this);
  }
  start_mission_sub_ = nh->subscribe<std_msgs::Bool>("start_mission", 1, &MissionInterface::startMissionCB, this);
  gps_sub_ = nh->subscribe<sensor_msgs::NavSatFix>("gps", 1, &MissionInterface::gpsCB, this);
  length_reached_sub_ = nh->subscribe("/tie_controller/length_reached", 1, &MissionInterface::lengthReachedCB, this);
  load_trajectory_sub_ = nh->subscribe("load_mission", 1, &MissionInterface::loadMissionCB, this);
  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/arco/joy", 5, &MissionInterface::joyReceivedCB,this);
  
  traj_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_ugv", 100);
  traj_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_uav", 100);
  catenary_length_pub_ = nh->advertise<std_msgs::Float32>("/tie_controller/set_length", 1);
  traj_lines_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_lines_ugv", 100);
  traj_lines_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_lines_uav", 100);
  catenary_marker_pub_= nh->advertise<visualization_msgs::MarkerArray>("trajectory_catenary", 100);
  reset_length_pub_= nh->advertise<std_msgs::Float32>("/tie_controller/reset_length_estimation", 100); 
}

void MissionInterface::ugvReadyForMissionCB(const std_msgs::BoolConstPtr &msg)
{
  ugv_ready = msg->data;
}

void MissionInterface::uavReadyForMissionCB(const std_msgs::BoolConstPtr &msg)
{
  uav_ready = msg->data;
}

void MissionInterface::startMissionCB(const std_msgs::BoolConstPtr &msg)
{
  start_mission = msg->data;
}

void MissionInterface::lengthReachedCB(const std_msgs::BoolConstPtr &msg) {
  length_reached = msg->data;
}

void MissionInterface::loadMissionCB(const std_msgs::String &msg)
{
  if (start_mission) {
    ROS_ERROR("Ignored load mission petition: a mission is currently being executed");
  } else {
    ROS_INFO("Loading mission file: %s", msg.data.c_str());
    readWaypoints(msg.data);
    path_file = msg.data;
  }
}

// GPS altitude callback
void MissionInterface::gpsCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  std_msgs::Float64 height_msg; 
  height = msg->altitude;
}

void MissionInterface::joyReceivedCB(const sensor_msgs::Joy::ConstPtr& joy)
{ 
 if (joy->buttons[stopArcoMissionButton]) {
      ROS_ERROR("MissionInterface : JOYSTICK is forcing to STOP Arco Mission Path Tracker");
      NavigationClient->cancelAllGoals();
  }
}

void MissionInterface::executeMission()
{
  int size_ = trajectory.points.size();
    
  if(!start_mission){
    printf("\tMarsupial Mission NOT Initialized. Check star mission topic\n");
    ros::Duration(2.0).sleep();
    resetFlags();
  }

  if(start_mission){
    // To force variable in TRUE and continue with execution tracker even if UGV is not able
    if(!able_tracker_ugv)   
      ugv_ready = true;
    // To force variable in TRUE and continue with execution tracker even if UAV is not able
    if(!able_tracker_uav)   
      uav_ready = true; 

    if(ugv_ready && uav_ready)
    {
      bool uav_in_on_ground_ = UAVisOnTheGround();
      if (do_takeoff){
        std::cout << "UAV is on the ground? : " << uav_in_on_ground_ << std::endl;
                  
        if(uav_in_on_ground_){
          std::cout << "\tSending takeoff height" << std::endl;
          upo_actions::TakeOffGoal takeOffGoal;
          std_msgs::Float32 takeoff_height_;
          takeoff_height_.data = takeoff_height;
          takeOffGoal.takeoff_height = takeoff_height_;
          takeOffClient->sendGoal(takeOffGoal);
          while (!takeOffClient->waitForResult(ros::Duration(time_max))){
            std::cout << "\tSent takeoff height ... waiting action server" << std::endl;
          }
              
          if (takeOffClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            printf("\tUAV take Off succeessful\n");
            take_off = true;
          }

          if(!take_off){
            ROS_ERROR("DRONE COULDN'T TAKE OFF, IS STILL IN GROUND: FIRST TAKE-OFF");
            uav_ready = ugv_ready = false;
            ROS_ERROR("Check platform NOT ready to TAKE-OFF: Use teleop control to land UAV");
          }
          printf("\t\tUAV Take off - ready to execute trajectory: Sending WP %i\n",num_wp + 1);
        }
        else{
          printf("\tUAV already took off - ready to perform maneuver to go first WP\n");
        }
      }else{
          uav_in_on_ground_ =false;
      }

      if(num_wp < size_) {

        if(able_tracker_ugv){
          ugv_goal3D.global_goal.position.x = trajectory.points.at(num_wp).transforms[0].translation.x;
          ugv_goal3D.global_goal.position.y = trajectory.points.at(num_wp).transforms[0].translation.y;
          ugv_goal3D.global_goal.position.z = trajectory.points.at(num_wp).transforms[0].translation.z;
          ugv_goal3D.global_goal.orientation.x = trajectory.points.at(num_wp).transforms[0].rotation.x;
          ugv_goal3D.global_goal.orientation.y = trajectory.points.at(num_wp).transforms[0].rotation.y;
          ugv_goal3D.global_goal.orientation.z = trajectory.points.at(num_wp).transforms[0].rotation.z;
          ugv_goal3D.global_goal.orientation.w = trajectory.points.at(num_wp).transforms[0].rotation.w;
        }
        if(able_tracker_uav){
          uav_goal3D.global_goal.pose.position.x = trajectory.points.at(num_wp).transforms[1].translation.x;
          uav_goal3D.global_goal.pose.position.y = trajectory.points.at(num_wp).transforms[1].translation.y;
          uav_goal3D.global_goal.pose.position.z = trajectory.points.at(num_wp).transforms[1].translation.z;
          uav_goal3D.global_goal.pose.orientation.x = trajectory.points.at(num_wp).transforms[1].rotation.x;
          uav_goal3D.global_goal.pose.orientation.y = trajectory.points.at(num_wp).transforms[1].rotation.y;
          uav_goal3D.global_goal.pose.orientation.z = trajectory.points.at(num_wp).transforms[1].rotation.z;
          uav_goal3D.global_goal.pose.orientation.w = trajectory.points.at(num_wp).transforms[1].rotation.w;
        }
        std_msgs::Float32 catenary_msg;
        catenary_msg.data = tether_length_vector.at(num_wp);
        ros::Duration(0.2).sleep();  //DONT forget delete

        catenary_length_pub_.publish(catenary_msg);

        if(able_tracker_ugv && !sent_new_ugv_wp){
          ROS_INFO_COND(debug, "Sending UGV WayPoint [%i/%i] to tracker: goal[%f %f %f]", num_wp, size_,
            ugv_goal3D.global_goal.position.x,
            ugv_goal3D.global_goal.position.y,
            ugv_goal3D.global_goal.position.z);
          NavigationClient->sendGoal(ugv_goal3D);
          ROS_INFO_COND(debug, "Sent WayPoint for UGV ... Waiting for finishing maneuver");
          sent_new_ugv_wp = true;
          time_count_ugv = ros::Time::now();
        }
        if(able_tracker_uav && !sent_new_uav_wp ){
          ROS_INFO_COND(debug, "Sending UAV WayPoint [%i/%i] to tracker: goal[%f %f %f]", num_wp, size_,
            uav_goal3D.global_goal.pose.position.x,
            uav_goal3D.global_goal.pose.position.y,
            uav_goal3D.global_goal.pose.position.z);
          uavNavigation3DClient->sendGoal(uav_goal3D);
          ROS_INFO_COND(debug, "Sent WayPoint for UAV ... Waiting for finishing maneuver");
          sent_new_uav_wp = true;
          time_count_uav = ros::Time::now();
        }

         // To reset variable after waypoint WayPoint. If not able defaults to true
        is_ugv_in_waypoint = !able_tracker_ugv;  // Reset variable after WayPoint reached
        is_uav_in_waypoint = !able_tracker_uav;
        length_reached = false;

        // TODO: wait also for the UGV client
        if ( (!able_tracker_ugv || NavigationClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && 
               (!able_tracker_uav || uavNavigation3DClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) )
          {
            if(able_tracker_ugv){
              is_ugv_in_waypoint = true;
              ROS_INFO("UGV Path Tracker: Goal [%i/%i] Achieved",num_wp + 1,size_);
            }
            if(able_tracker_uav){
              is_uav_in_waypoint = true;
              ROS_INFO("UAV Path Tracker: Goal [%i/%i] Achieved",num_wp + 1,size_);
            }
          }
          else if ((!able_tracker_ugv || NavigationClient->getState() == actionlib::SimpleClientGoalState::ABORTED) &&
                   (!able_tracker_uav || uavNavigation3DClient->getState() == actionlib::SimpleClientGoalState::ABORTED))
          {
            if(able_tracker_ugv){
              ROS_INFO_COND(debug, "UGV Goal aborted by path tracker");
              resetFlags();
              return;
            }
            if(able_tracker_uav){
              ROS_INFO_COND(debug, "UAV Goal aborted by path tracker");
              resetFlags();
              return;
            }
          }
          else if ((!able_tracker_ugv || NavigationClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED) &&
                   (!able_tracker_uav || uavNavigation3DClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED))
          { 
            if(able_tracker_ugv){
              ROS_INFO_COND(debug, "UGV Goal preempted by path tracker");
              resetFlags();
              return;
            }
            if(able_tracker_uav){
              ROS_INFO_COND(debug, "UAV Goal preempted by path tracker");
              resetFlags();
              return;
            }
          }

        // Wait for proper length
        if (used_length_reached){
          ROS_INFO("Waiting for the cable system to get the commanded length");
          while (!length_reached && ros::ok()) {
            ros::Duration(0.2).sleep();
            ros::spinOnce();
          }
        } else{
          length_reached = true;
        }

        // printf("is_ugv_in_waypoint: %s",is_ugv_in_waypoint?"true":"false");
        // printf(" , is_uav_in_waypoint: %s",is_uav_in_waypoint?"true":"false");
        // printf(" , length_reached: %s\n",length_reached?"true":"false");

        if(is_ugv_in_waypoint && is_uav_in_waypoint && length_reached) {
          std::cout <<" " << std::endl;
          ROS_INFO("\t\tSuccessfully achieved WayPoint [%i/%i]\n",num_wp + 1,size_);
          sent_new_ugv_wp = sent_new_uav_wp = false;
          num_wp++;
        } else {
          //ROS_WARN("\t\tExecuting maneauvere to reach WayPoint [%i/%i]", num_wp + 1,size_);
        }

        if(able_tracker_ugv && ros::Time::now() - time_count_ugv > ros::Duration(time_max)) {
          std::cout <<" " << std::endl;
          ROS_ERROR("\t\tWasn't posible to reach UGV WayPoint [%i/%i] ", num_wp + 1,size_);
          NavigationClient->cancelAllGoals();
          if(able_tracker_uav)
            uavNavigation3DClient->cancelAllGoals();
          resetFlags();
        }

        if(able_tracker_uav && ros::Time::now() - time_count_uav > ros::Duration(time_max)) {
          std::cout <<" " << std::endl;
          ROS_ERROR("\t\tWasn't posible to reach UAV WayPoint [%i/%i] ", num_wp + 1,size_);
          if(able_tracker_ugv)
            NavigationClient->cancelAllGoals();
          uavNavigation3DClient->cancelAllGoals();
          resetFlags();
        }

        if (num_wp == size_){
          printf("\n\tWell Done !!!\n");
          printf("\n\tMission Finished: WPs achived\n");
          printf("\n\tInitializing flags !!\n");
          resetFlags();
        }
      }
    } 
    else {
      if(!ugv_ready && !uav_ready && able_tracker_ugv && able_tracker_uav)    
        printf("\t\tUGV and UAV Platform NOT ready for execute mission\n");
      else if(!ugv_ready && able_tracker_ugv)    
        printf("\t\tUGV Platform NOT ready for execute mission\n");
      else if(!uav_ready && able_tracker_uav)    
        printf("\t\tUAV Platform NOT ready for execute mission\n");
    
      ros::Duration(2.0).sleep();
    }
    ros::spinOnce();
  }
}

void MissionInterface::resetFlags()
{
  num_wp = 0;
  ugv_ready = false;
  uav_ready = false;
  is_ugv_in_waypoint = false;
  is_uav_in_waypoint = false;
  start_mission = false;
  length_reached = false;
  sent_new_uav_wp = sent_new_ugv_wp = false;
}



// Todo: let the user enter them with external files
void MissionInterface::readWaypoints(const std::string &path_file)
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
	init_uav_pose.orientation.z =
	  file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
	init_uav_pose.orientation.w =
	  file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	if (i==0) {
	    init_ugv_pose.position.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>() + offset_map_dll_x;;
	    init_ugv_pose.position.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>() + offset_map_dll_y;;
	    init_ugv_pose.position.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>() + offset_map_dll_z;;
	    init_ugv_pose.orientation.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
	    init_ugv_pose.orientation.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
	    init_ugv_pose.orientation.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
	    init_ugv_pose.orientation.w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
	    init_uav_pose.position.x = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>()) + offset_map_dll_x;
	    init_uav_pose.position.y = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>()) + offset_map_dll_y;	    
      init_uav_pose.position.z = (file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>()) + offset_map_dll_z;
	    init_uav_pose.orientation.x =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
	    init_uav_pose.orientation.y =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
	    init_uav_pose.orientation.z =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
	    init_uav_pose.orientation.w =
	      file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
	}
           
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
	} catch (std::exception &e) {}
	tether_length_vector.push_back(length); // TODO calculate the distance bw UAV and UGV
    } catch(std::exception &e) {
      ROS_INFO("Skipping waypoint %d", i);
    }
  }
  std::cout << "YAML FILE readed. YAML FILE NAME: " << path_file << std::endl;
  std::cout << "Number of points: " << trajectory.points.size() << std::endl;
}

bool MissionInterface::isInitialPose()
{
  geometry_msgs::TransformStamped ugv_tf_, uav_tf_;
  double dist_goal_ugv = 0.2;
  double dist_goal_uav = 0.3;
  bool is_ugv_init_pos_ = false;
  bool is_uav_init_pos_ = false;
  if(able_tracker_ugv){
    try{
      ugv_tf_ = tfBuffer->lookupTransform(ugv_odom_frame, ugv_base_frame, ros::Time(0));
      ROS_INFO("Mission Interface: Got UGV Pose (base_frame: %s - odom_frame: %s).",
	       ugv_base_frame.c_str(),ugv_odom_frame.c_str());
    }
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get UGV Pose (base_frame: %s - odom_frame: %s), so not possible to set UGV start point; tf exception: %s",
	       ugv_base_frame.c_str(),ugv_odom_frame.c_str(),ex.what());
    }
    ROS_INFO("Mission interface: ugv_ini_pos=[%f %f %f] ugv_base_link=[%f %f %f]",
	     init_ugv_pose.position.x ,init_ugv_pose.position.y ,init_ugv_pose.position.z,
	     ugv_tf_.transform.translation.x,
	     ugv_tf_.transform.translation.y,
	     ugv_tf_.transform.translation.z);
    if ( fabs(init_ugv_pose.position.x - ugv_tf_.transform.translation.x) < dist_goal_ugv && 
	 fabs(init_ugv_pose.position.y - ugv_tf_.transform.translation.y) < dist_goal_ugv &&
	 fabs(init_ugv_pose.position.z - ugv_tf_.transform.translation.z) < dist_goal_ugv ){
      is_ugv_init_pos_ =  true;
    }
  }
  if(able_tracker_uav){
    try{
      uav_tf_ = tfBuffer->lookupTransform(uav_odom_frame, uav_base_frame, ros::Time(0));
      ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).",
	       uav_base_frame.c_str(),uav_odom_frame.c_str());
    }    
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get UAV Pose (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s",
	       uav_base_frame.c_str(),uav_odom_frame.c_str(),ex.what());
    }
    if ( fabs(init_uav_pose.position.x - uav_tf_.transform.translation.x) < dist_goal_uav && 
	 fabs(init_uav_pose.position.y - uav_tf_.transform.translation.y) < dist_goal_uav &&
	 fabs(init_uav_pose.position.z - uav_tf_.transform.translation.z) < dist_goal_uav ){
      is_uav_init_pos_ = true;
    }
    ROS_INFO("Mission interface: uav_ini_pos=[%f %f %f] uav_base_link=[%f %f %f]",
	     init_uav_pose.position.x ,init_uav_pose.position.y ,init_uav_pose.position.z,
	     uav_tf_.transform.translation.x,uav_tf_.transform.translation.y,
	     uav_tf_.transform.translation.z);
  }

  if ((is_ugv_init_pos_ || !able_tracker_ugv) && (is_uav_init_pos_ || !able_tracker_uav))
    return true;
  else
    return false;
}

bool MissionInterface::UAVisOnTheGround()
{
    geometry_msgs::TransformStamped uav_tf_;

    if (!able_tracker_uav)
      return false;

    try{
        uav_tf_ = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
        // printf("\t\tMission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).", uav_base_frame.c_str(),world_frame.c_str());
    }    
    catch (tf2::TransformException &ex){
        ROS_WARN("Mission Interface: Couldn't get UAV Pose (world_frame: %s - base_frame: %s), so not possible to set UAV start point; tf exception: %s",
		 world_frame.c_str(),uav_base_frame.c_str(),ex.what());
    }

    if (uav_tf_.transform.translation.z - initial_pose.z > flying_height)
        return false;
    else    
        return true;
}

void MissionInterface::markerPoints()
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
    _marker_ugv.markers[i].scale.x = 0.4;
    _marker_ugv.markers[i].scale.y = 0.4;
    _marker_ugv.markers[i].scale.z = 0.4;
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
    _marker_uav.markers[i].scale.x = 0.4;
    _marker_uav.markers[i].scale.y = 0.4;
    _marker_uav.markers[i].scale.z = 0.4;
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
      _lines_ugv.markers[i].color.g = 0.1;
      _lines_ugv.markers[i].color.b = 1.0;
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
    }	
    catenary_marker_pub_.publish(_cat_marker);
  }
}
