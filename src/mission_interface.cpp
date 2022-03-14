
#include <mission_interface/mission_interface.h>

MissionInterface::MissionInterface(std::string node_name_)
{
    nh.reset(new ros::NodeHandle("~"));
    
    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    
    nh->param("path_file", path_file, (std::string) "~/");
    nh->param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
    nh->param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
    nh->param("ugv_odom_frame", ugv_odom_frame, (std::string) "ugv/odom");
    nh->param("uav_odom_frame", uav_odom_frame, (std::string) "world");
	nh->param<bool>("able_tracker_ugv",able_tracker_ugv, true);
	nh->param<bool>("able_tracker_uav",able_tracker_uav, true);
    
    ros_node_name = node_name_;
    ROS_INFO("Initialized Node : %s", ros_node_name.c_str());

    resetFlags();
    configTopics();
    configServices();
    readWayPoints();

    for (size_t i =0; i < trajectory.points.size(); i++ ){
        printf("[%lu] UGV[%f %f %f][%f %f %f %f] UAV[%f %f %f][%f %f %f %f]\n",i,trajectory.points.at(i).transforms[0].translation.x,trajectory.points.at(i).transforms[0].translation.y,trajectory.points.at(i).transforms[0].translation.z,
        trajectory.points.at(i).transforms[0].rotation.x,trajectory.points.at(i).transforms[0].rotation.y,trajectory.points.at(i).transforms[0].rotation.z,trajectory.points.at(i).transforms[0].rotation.w,
        trajectory.points.at(i).transforms[1].translation.x,trajectory.points.at(i).transforms[1].translation.y,trajectory.points.at(i).transforms[1].translation.z,
        trajectory.points.at(i).transforms[1].rotation.x,trajectory.points.at(i).transforms[1].rotation.y,trajectory.points.at(i).transforms[1].rotation.z,trajectory.points.at(i).transforms[1].rotation.w);
    }
}

//Config standard services and action lib servers and clients
void MissionInterface::configServices()
{
    if(able_tracker_uav){
        ROS_INFO("%s Node: Initialazing UAV Navigation Client", ros_node_name.c_str());
        uavNavigation3DClient.reset(new Navigate3DClient("/UAVNavigation3D", true));
        uavNavigation3DClient->waitForServer();
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
        ugv_state_mission_sub_ = nh->subscribe<std_msgs::Bool>("ugv_ready_for_mission", 1, &MissionInterface::ugvReadyForMissionCB, this);
    }
    if(able_tracker_uav){
        uav_state_mission_sub_ = nh->subscribe<std_msgs::Bool>("uav_ready_for_mission", 1, &MissionInterface::uavReadyForMissionCB, this);
    }
    start_mission_sub_ = nh->subscribe<std_msgs::Bool>("start_mission", 1, &MissionInterface::startMissionCB, this);
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

void MissionInterface::executeMission()
{
    int size_ = trajectory.points.size();

    if(num_wp >= size_){
        ROS_INFO("%s Node: The Path has been successfully navigated !!!", ros_node_name.c_str());
        resetFlags();
        return;
    }
    
    if(start_mission){
        if(!able_tracker_ugv)   // To force variable in TRUE and continue with execution tracker even if UGV is not able
            ugv_ready=true; 
        if(!able_tracker_uav)   // To force variable in TRUE and continue with execution tracker even if UAV is not able
            uav_ready=true; 

        if(ugv_ready && uav_ready){

            if (num_wp== 0){
                if ( isInitialPose()){  //Initial Pos is set manually
                ROS_INFO("Marsupial platforms in its initial position !!!");
                num_wp++;
                }
            }
            else{
                if(able_tracker_ugv){
                    ugv_goal3D.global_goal.pose.position.x = trajectory.points.at(num_wp).transforms[0].translation.x;
                    ugv_goal3D.global_goal.pose.position.y = trajectory.points.at(num_wp).transforms[0].translation.y;
                    ugv_goal3D.global_goal.pose.position.z = trajectory.points.at(num_wp).transforms[0].translation.z;
                    ugv_goal3D.global_goal.pose.orientation.x = trajectory.points.at(num_wp).transforms[0].rotation.x;
                    ugv_goal3D.global_goal.pose.orientation.y = trajectory.points.at(num_wp).transforms[0].rotation.y;
                    ugv_goal3D.global_goal.pose.orientation.z = trajectory.points.at(num_wp).transforms[0].rotation.z;
                    ugv_goal3D.global_goal.pose.orientation.w = trajectory.points.at(num_wp).transforms[0].rotation.w;
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

                if(!able_tracker_ugv)   // To force continue with execution if tracker is not able for UGV
                    is_ugv_in_waypoint=true; 
                if(!able_tracker_uav)   // To force continue with execution if tracker is not able for UAV
                    is_uav_in_waypoint=true; 

                if( is_ugv_in_waypoint && is_uav_in_waypoint ){
                    ROS_INFO_COND(debug, "Sendig new WayPoint to tracker");
                    if(able_tracker_ugv)
                        ugvNavigation3DClient->sendGoal(ugv_goal3D);
                    if(able_tracker_uav)
                        uavNavigation3DClient->sendGoal(uav_goal3D);
                }
                is_ugv_in_waypoint = false;   // To reset variable after achieve WayPoint and sended new goal 
                is_uav_in_waypoint = false;  // To reset variable after achieve WayPoint and sended new goal 

                if(able_tracker_ugv){
                    state.reset(new actionlib::SimpleClientGoalState(ugvNavigation3DClient->getState()));
                    
                    if (*state == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        is_ugv_in_waypoint = true;
                        ROS_INFO("UGV Path Tracker: Goal [%i/%i] achieved",num_wp,size_);
                    }
                    else if (*state == actionlib::SimpleClientGoalState::ABORTED)
                    {
                        ROS_INFO_COND(debug, "Goal aborted by path tracker");
                        resetFlags();
                        return;
                        
                    }
                    else if (*state == actionlib::SimpleClientGoalState::PREEMPTED)
                    {
                        ROS_INFO_COND(debug, "Goal preempted by path tracker");
                        resetFlags();
                        return;
                    }
                }

                if(able_tracker_uav){
                    state.reset(new actionlib::SimpleClientGoalState(uavNavigation3DClient->getState()));
                    
                    if (*state == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        is_uav_in_waypoint = true;
                        ROS_INFO("UAV Path Tracker: Goal [%i/%i] achieved",num_wp,size_);
                    }
                    else if (*state == actionlib::SimpleClientGoalState::ABORTED)
                    {
                        ROS_INFO_COND(debug, "Goal aborted by path tracker");
                        resetFlags();
                        return;
                    }
                    else if (*state == actionlib::SimpleClientGoalState::PREEMPTED)
                    {
                        ROS_INFO_COND(debug, "Goal preempted by path tracker");
                        resetFlags();
                        return;
                    }

                    if(is_ugv_in_waypoint && is_uav_in_waypoint) {
                        is_ugv_in_waypoint = is_uav_in_waypoint= true;
                        ROS_INFO("Successfully achived WayPoint [%i/%i]",num_wp,size_);
                        num_wp++;
                    }
                }
            }
        }
        else{
            if(!ugv_ready && !uav_ready && able_tracker_ugv && able_tracker_uav)    
                ROS_WARN("UGV and UAV Platform are still NOT ready for execute mission");
            else if(!ugv_ready && able_tracker_ugv)    
                ROS_WARN("UGV Platform is still NOT ready for execute mission");
            else if(!uav_ready && able_tracker_uav)    
                ROS_WARN("UAV Platform is still NOT ready for execute mission");
            ros::Duration(2.0).sleep();
        }
    }
    else{
        ROS_ERROR("Marsupial Mission NOT Initialized. Check TOPICS");
        ros::Duration(2.0).sleep();
        resetFlags();
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

    int size_ = (file["marsupial_ugv"]["size"].as<int>()) - 1; // It is substract -1 because first position it is the initial point
    std::string ugv_pos_data, uav_pos_data;
    double ugv_pos_x, ugv_pos_y, ugv_pos_z, ugv_rot_x, ugv_rot_y, ugv_rot_z, ugv_rot_w;
    double uav_pos_x, uav_pos_y, uav_pos_z, uav_rot_x, uav_rot_y, uav_rot_z, uav_rot_w;
    for (int i=0 ; i < size_; i++ ){ // It begin in 1 because first point is given as initial point.
        ugv_pos_data = "poses"+ std::to_string(i);
        uav_pos_data = "poses"+ std::to_string(i);
        if (i==0){
            init_ugv_pose.position.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
            init_ugv_pose.position.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
            init_ugv_pose.position.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
            init_ugv_pose.orientation.x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
            init_ugv_pose.orientation.y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
            init_ugv_pose.orientation.z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
            init_ugv_pose.orientation.w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
            init_uav_pose.position.x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>();
            init_uav_pose.position.y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>();
            init_uav_pose.position.z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>();
            init_uav_pose.orientation.x = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["x"].as<double>();
            init_uav_pose.orientation.y = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["y"].as<double>();
            init_uav_pose.orientation.z = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["z"].as<double>();
            init_uav_pose.orientation.w = file["marsupial_uav"][uav_pos_data]["pose"]["orientation"]["w"].as<double>();
        }
        else{
            ugv_pos_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["x"].as<double>();
            ugv_pos_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["y"].as<double>();
            ugv_pos_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["position"]["z"].as<double>();
            ugv_rot_x = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["x"].as<double>();
            ugv_rot_y = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["y"].as<double>();
            ugv_rot_z = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["z"].as<double>();
            ugv_rot_w = file["marsupial_ugv"][ugv_pos_data]["pose"]["orientation"]["w"].as<double>();
            uav_pos_x = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["x"].as<double>();
            uav_pos_y = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["y"].as<double>();
            uav_pos_z = file["marsupial_uav"][uav_pos_data]["pose"]["position"]["z"].as<double>();
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
    }

    if (size_ == trajectory.points.size()){
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
            ROS_INFO("Mission Interface: Got UGV Pose (base_frame: %s - odom_frame: %s).", ugv_base_frame.c_str(),ugv_odom_frame.c_str());
        }
        catch (tf2::TransformException &ex){
            ROS_WARN("Mission Interface: Couldn't get UGV Pose (base_frame: %s - odom_frame: %s), so not possible to set UGV start point; tf exception: %s", ugv_base_frame.c_str(),ugv_odom_frame.c_str(),ex.what());
        }
        ROS_INFO("Mission interface: ugv_ini_pos=[%f %f %f] ugv_base_link=[%f %f %f]", init_ugv_pose.position.x ,init_ugv_pose.position.y ,init_ugv_pose.position.z,
                                                                ugv_tf_.transform.translation.x,ugv_tf_.transform.translation.y,ugv_tf_.transform.translation.z);
        if ( fabs(init_ugv_pose.position.x - ugv_tf_.transform.translation.x) < dist_goal_ugv && 
             fabs(init_ugv_pose.position.y - ugv_tf_.transform.translation.y) < dist_goal_ugv &&
             fabs(init_ugv_pose.position.z - ugv_tf_.transform.translation.z) < dist_goal_ugv ){
             is_ugv_init_pos_ =  true;
         }
    }
    if(able_tracker_uav){
        try{
            uav_tf_ = tfBuffer->lookupTransform(uav_odom_frame, uav_base_frame, ros::Time(0));
            ROS_INFO("Mission Interface: Got UAV Pose (base_frame: %s - odom_frame: %s).", uav_base_frame.c_str(),uav_odom_frame.c_str());
        }    
        catch (tf2::TransformException &ex){
            ROS_WARN("Mission Interface: Couldn't get UAV Pose (base_frame: %s - odom_frame: %s), so not possible to set UAV start point; tf exception: %s", uav_base_frame.c_str(),uav_odom_frame.c_str(),ex.what());
        }
        if ( fabs(init_uav_pose.position.x - uav_tf_.transform.translation.x) < dist_goal_uav && 
             fabs(init_uav_pose.position.y - uav_tf_.transform.translation.y) < dist_goal_uav &&
             fabs(init_uav_pose.position.z - uav_tf_.transform.translation.z) < dist_goal_uav ){
             is_uav_init_pos_ = true;
         }
         ROS_INFO("Mission interface: uav_ini_pos=[%f %f %f] uav_base_link=[%f %f %f]", init_uav_pose.position.x ,init_uav_pose.position.y ,init_uav_pose.position.z,
                                                                uav_tf_.transform.translation.x,uav_tf_.transform.translation.y,uav_tf_.transform.translation.z);
    }

    if ((is_ugv_init_pos_ || !able_tracker_ugv) && (is_uav_init_pos_ || !able_tracker_uav))
        return true;
    else
        return false;

}

