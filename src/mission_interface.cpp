
#include <mission_interface/mission_interface.h>

MissionInterface::MissionInterface(std::string node_name_)
{
  nh.reset(new ros::NodeHandle("~"));
    
  tfBuffer.reset(new tf2_ros::Buffer);
  tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

  nh->param("path_file", path_file, (std::string) "~/");
  nh->param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
  nh->param("ugv_odom_frame", ugv_odom_frame, (std::string) "ugv/odom");
  nh->param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
  nh->param("uav_odom_frame", uav_odom_frame, (std::string) "uav/odom");
  nh->param("world_frame", world_frame, (std::string) "world");
  nh->param("map_name", map_name, (std::string) "stage");
  nh->param("offset_map_"+map_name+"/offset_map_dll_x", offset_map_dll_x, (double)0.0);
  nh->param("offset_map_"+map_name+"/offset_map_dll_y", offset_map_dll_y, (double)0.0);
  nh->param("offset_map_"+map_name+"/offset_map_dll_z", offset_map_dll_z, (double)0.0);
  nh->param("flying_height", flying_height, (double)0.3);

  nh->param<bool>("able_tracker_ugv",able_tracker_ugv, true);
  nh->param<bool>("able_tracker_uav",able_tracker_uav, true);
  nh->param("takeoff_height",takeoff_height, (float)1.0);
    
    
  ros_node_name = node_name_;
  ROS_INFO("Initialized Node : %s", ros_node_name.c_str());

  take_off = false;
  received_initial_pose = false;

  geometry_msgs::TransformStamped uav_tf_;
   

  resetFlags();
  configTopics();
  configServices();
  readWayPoints();
  markerPoints(trajectory);

  for (size_t i =0; i < trajectory.points.size(); i++ ){
    printf("[%lu] UGV[%f %f %f][%f %f %f %f] UAV[%f %f %f][%f %f %f %f]\n",
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
	   trajectory.points.at(i).transforms[1].rotation.w);
  }

  try{
    uav_tf_ = tfBuffer->lookupTransform(world_frame, uav_odom_frame, ros::Time(0));
    ROS_INFO("\tPosition initial UAV gotten");
    initial_pose  = uav_tf_.transform.translation;
    received_initial_pose = true;
  }    
  catch (tf2::TransformException &ex){
    ROS_WARN("Mission Interface: Couldn't get position initial UAV (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s",
	     world_frame.c_str(),uav_odom_frame.c_str(),ex.what());
  }
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
        ugvNavigation3DClient.reset(new Navigate3DClient("/UGVNavigation3D", true));
        ugvNavigation3DClient->waitForServer();
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
  start_mission_sub_ = nh->subscribe<std_msgs::Bool>("start_mission", 1,
						     &MissionInterface::startMissionCB, this);
  gps_sub_ = nh->subscribe<sensor_msgs::NavSatFix>("gps", 1, &MissionInterface::gpsCB, this);
  traj_ugv_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_ugv", 100);
  traj_uav_pub_ = nh->advertise<visualization_msgs::MarkerArray>("trajectory_uav", 100);
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

// GPS altitude callback
void MissionInterface::gpsCB(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	std_msgs::Float64 height_msg; 
	height = msg->altitude;
}

void MissionInterface::executeMission()
{
    int size_ = trajectory.points.size();
    
    if(!start_mission){
        printf("\tMarsupial Mission NOT Initialized. Check star mission topic\n");
        ros::Duration(2.0).sleep();
        resetFlags();
    }

    while(start_mission){
      // To force variable in TRUE and continue with execution tracker even if UGV is not able
      if(!able_tracker_ugv)   
	ugv_ready = true;
      // To force variable in TRUE and continue with execution tracker even if UAV is not able
      if(!able_tracker_uav)   
	uav_ready = true; 

      if(ugv_ready && uav_ready)
        {
	  bool uav_in_on_ground_ = UAVisOnTheGround();
	  std::cout << "UAV is on the ground? : " << uav_in_on_ground_ << std::endl;
            
	  if(uav_in_on_ground_){
	    std::cout << "\tSending takeoff height" << std::endl;
	    upo_actions::TakeOffGoal takeOffGoal;
	    std_msgs::Float32 takeoff_height_;
	    takeoff_height_.data = takeoff_height;
	    takeOffGoal.takeoff_height = takeoff_height_;
	    takeOffClient->sendGoal(takeOffGoal);
	    while (!takeOffClient->waitForResult(ros::Duration(30.0))){
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
	  }
	  else{
	    printf("\tUAV is already take off - ready to perform maneuver to go first WP\n");
	  }

	  while(take_off || !uav_in_on_ground_){
	    printf("\t\tUAV Take off - ready to execute trajectory: Sending WP %i\n",num_wp);
	    ros::Duration(1.0).sleep();

	    is_ugv_in_waypoint = false;   // Reset variable after WayPoint reached
	    is_uav_in_waypoint = false;  // To reset variable after waypoint WayPoint

	    if(able_tracker_ugv){
	      ugv_goal3D.global_goal.pose.position.x =
		trajectory.points.at(num_wp).transforms[0].translation.x;
	      ugv_goal3D.global_goal.pose.position.y =
		trajectory.points.at(num_wp).transforms[0].translation.y;
	      ugv_goal3D.global_goal.pose.position.z =
		trajectory.points.at(num_wp).transforms[0].translation.z;
	      ugv_goal3D.global_goal.pose.orientation.x =
		trajectory.points.at(num_wp).transforms[0].rotation.x;
	      ugv_goal3D.global_goal.pose.orientation.y =
		trajectory.points.at(num_wp).transforms[0].rotation.y;
	      ugv_goal3D.global_goal.pose.orientation.z =
		trajectory.points.at(num_wp).transforms[0].rotation.z;
	      ugv_goal3D.global_goal.pose.orientation.w =
		trajectory.points.at(num_wp).transforms[0].rotation.w;
	    }
	    if(able_tracker_uav){
	      uav_goal3D.global_goal.pose.position.x =
		trajectory.points.at(num_wp).transforms[1].translation.x;
	      uav_goal3D.global_goal.pose.position.y =
		trajectory.points.at(num_wp).transforms[1].translation.y;
	      uav_goal3D.global_goal.pose.position.z =
		trajectory.points.at(num_wp).transforms[1].translation.z;
	      uav_goal3D.global_goal.pose.orientation.x =
		trajectory.points.at(num_wp).transforms[1].rotation.x;
	      uav_goal3D.global_goal.pose.orientation.y =
		trajectory.points.at(num_wp).transforms[1].rotation.y;
	      uav_goal3D.global_goal.pose.orientation.z =
		trajectory.points.at(num_wp).transforms[1].rotation.z;
	      uav_goal3D.global_goal.pose.orientation.w =
		trajectory.points.at(num_wp).transforms[1].rotation.w;
	    }

	    if(able_tracker_ugv){
	      ROS_INFO_COND(debug, "Sendig new WayPoint UGV to tracker: goal[%f %f %f]",
			    ugv_goal3D.global_goal.pose.position.x,
			    ugv_goal3D.global_goal.pose.position.y,
			    ugv_goal3D.global_goal.pose.position.z);
	      ugvNavigation3DClient->sendGoal(ugv_goal3D);
	      ROS_INFO_COND(debug, "Sent WayPoint for UGV ... Waiting for finishing maneuver");
	    }
	    if(able_tracker_uav){
	      ROS_INFO_COND(debug, "Sendig new WayPoint UAV to tracker: goal[%f %f %f]",
			    uav_goal3D.global_goal.pose.position.x,
			    uav_goal3D.global_goal.pose.position.y,
			    uav_goal3D.global_goal.pose.position.z);
	      uavNavigation3DClient->sendGoal(uav_goal3D);
	      ROS_INFO_COND(debug, "Sent WayPoint for UAV ... Waiting for finishing maneuver");
	    }

	    if(!able_tracker_ugv)   // To force continue with execution 
	      is_ugv_in_waypoint=true; 
	    if(!able_tracker_uav)   // To force continue with execution 
	      is_uav_in_waypoint=true; 

	    ros::Time start_time = ros::Time::now();
	    uavNavigation3DClient->waitForResult(ros::Duration(20.0));
	    if(able_tracker_ugv){
	      if(ugvNavigation3DClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
		  is_ugv_in_waypoint = true;
		  ROS_INFO("UGV Path Tracker: Goal [%i/%i] Achieved",num_wp,size_);
		}
	      else if (ugvNavigation3DClient->getState() ==
		       actionlib::SimpleClientGoalState::ABORTED)
		{
		  ROS_INFO_COND(debug, "Goal aborted by path tracker");
		  resetFlags();
		  return;
		}
	      else if (ugvNavigation3DClient->getState() ==
		       actionlib::SimpleClientGoalState::PREEMPTED)
		{
		  ROS_INFO_COND(debug, "Goal preempted by path tracker");
		  resetFlags();
		  return;
		}
	    }
	    if(able_tracker_uav){
	      if(uavNavigation3DClient->getState() ==
		 actionlib::SimpleClientGoalState::SUCCEEDED)
		{
		  is_uav_in_waypoint = true;
		  ROS_INFO("UAV Path Tracker: Goal [%i/%i] Achieved",num_wp,size_);
		}
	      else if (uavNavigation3DClient->getState() ==
		    actionlib::SimpleClientGoalState::ABORTED)
		{
		  ROS_INFO_COND(debug, "Goal aborted by path tracker");
		  resetFlags();
		  return;
		}
	      else if (uavNavigation3DClient->getState() ==
		       actionlib::SimpleClientGoalState::PREEMPTED)
		{
		  ROS_INFO_COND(debug, "Goal preempted by path tracker");
		  resetFlags();
		  return;
		}
	    }
	    if(is_ugv_in_waypoint && is_uav_in_waypoint) {
	      std::cout <<" " << std::endl;
	      printf("\t\tSuccessfully achived WayPoint [%i/%i]\n",num_wp,size_);
	      num_wp++;
	    } else {
	      std::cout <<" " << std::endl;
	      ROS_ERROR("\t\tNot Possible to achived WayPoint [%i/%i] - Check robot system",
			num_wp,size_);
	      resetFlags();
	      break;
	    }
	    if (num_wp == size_){
	      printf("\n\tWell Done !!!\n");
	      printf("\n\tMission Finished: WPs achived\n");
	      printf("\n\tInitializing flags !!\n");
	      resetFlags();
	      break;
	    }
	  }
	  uav_ready = ugv_ready = false;
	  start_mission = false;
      } else {
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
}

void MissionInterface::readWayPoints()
{
  YAML::Node file = YAML::LoadFile(path_file);

  trajectory_msgs::MultiDOFJointTrajectoryPoint traj_marsupial_;
  traj_marsupial_.transforms.resize(2);
  traj_marsupial_.velocities.resize(2);
  traj_marsupial_.accelerations.resize(2);

  int size_ = (file["marsupial_ugv"]["size"].as<int>()) ; 
  std::string ugv_pos_data, uav_pos_data;
  double ugv_pos_x, ugv_pos_y, ugv_pos_z, ugv_rot_x, ugv_rot_y, ugv_rot_z, ugv_rot_w;
  double uav_pos_x, uav_pos_y, uav_pos_z, uav_rot_x, uav_rot_y, uav_rot_z, uav_rot_w;
  printf("offset_map_dll=[%f %f %f]\n",offset_map_dll_x,offset_map_dll_y,offset_map_dll_z);
  for (int i=0 ; i < size_; i++ ){ // It begin in 1 because first point is given as initial point.
    ugv_pos_data = "poses"+ std::to_string(i);
    uav_pos_data = "poses"+ std::to_string(i);
    if (i==0){
      init_ugv_pose.position.x =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
      init_ugv_pose.position.y =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
      init_ugv_pose.position.z =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
      init_ugv_pose.orientation.x =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
      init_ugv_pose.orientation.y =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
      init_ugv_pose.orientation.z =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
      init_ugv_pose.orientation.w =
	file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
      init_uav_pose.position.x =
	(file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>()) +
	offset_map_dll_x;
      init_uav_pose.position.y =
	(file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>()) +
	offset_map_dll_y;
      init_uav_pose.position.z =
	(file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>()) +
	offset_map_dll_z;
      init_uav_pose.orientation.x =
	file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
      init_uav_pose.orientation.y =
	file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
      init_uav_pose.orientation.z =
	file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
      init_uav_pose.orientation.w =
	file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
    }
    // else{
    ugv_pos_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
    ugv_pos_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
    ugv_pos_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
    ugv_rot_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
    ugv_rot_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
    ugv_rot_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
    ugv_rot_w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
    uav_pos_x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>()
      + offset_map_dll_x;
    uav_pos_y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>()+
      offset_map_dll_y;
    uav_pos_z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>()+
      offset_map_dll_z;
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
  }

  // It is substract -1 because first position it is the initial point
  if ((size_- 1) == trajectory.points.size()){
    std::cout << "YAML FILE successfully readed" << std::endl;
    std::cout << "YAML FILE NAME: " << path_file << std::endl;
  }
  else
    ROS_ERROR("DATA PROBLEM IN YAML_FILE !!!");
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
      uav_tf_ = tfBuffer->lookupTransform(world_frame, uav_odom_frame, ros::Time(0));
      ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).",
	       uav_odom_frame.c_str(),world_frame.c_str());
    }    
    catch (tf2::TransformException &ex){
      ROS_WARN("Mission Interface: Couldn't get UAV Pose (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s",
	       world_frame.c_str(),uav_odom_frame.c_str(),ex.what());
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
    try{
        uav_tf_ = tfBuffer->lookupTransform(world_frame, uav_odom_frame, ros::Time(0));
        ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).",
		 uav_odom_frame.c_str(),world_frame.c_str());
        initial_pose  = uav_tf_.transform.translation;
    }    
    catch (tf2::TransformException &ex){
        ROS_WARN("Mission Interface: Couldn't get UAV Pose (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s",
		 world_frame.c_str(),uav_odom_frame.c_str(),ex.what());
    }

    if (uav_tf_.transform.translation.z - initial_pose.z > flying_height)
        return false;
    else    
        return true;
}

void MissionInterface::markerPoints(trajectory_msgs::MultiDOFJointTrajectory _traj)
{
  visualization_msgs::MarkerArray _marker_ugv, _marker_uav; 

  _marker_ugv.markers.resize(_traj.points.size());
  _marker_uav.markers.resize(_traj.points.size());

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
    _marker_ugv.markers[i].scale.x = 0.1;
    _marker_ugv.markers[i].scale.y = 0.1;
    _marker_ugv.markers[i].scale.z = 0.1;
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
    _marker_uav.markers[i].scale.x = 0.1;
    _marker_uav.markers[i].scale.y = 0.1;
    _marker_uav.markers[i].scale.z = 0.1;
    _marker_uav.markers[i].color.a = 1.0;
    _marker_uav.markers[i].color.r = 0.1;
    _marker_uav.markers[i].color.g = 0.1;
    _marker_uav.markers[i].color.b = 1.0;
  }	
  traj_ugv_pub_.publish(_marker_ugv);
  traj_uav_pub_.publish(_marker_uav);
}
