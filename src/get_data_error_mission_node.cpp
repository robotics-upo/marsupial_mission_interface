#include "mission_interface/get_data_error_mission_node.h"

exportDataMission::exportDataMission(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	  printf("\n\tInitialazing Check_Mission_NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));
    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

    pnh.param("mission_path", mission_path, (std::string) "~/");
    pnh.param("statistical_results_path", statistical_results_path, (std::string) "~/");
    pnh.param("map_name", map_name, (std::string) "stage");
    pnh.param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
    pnh.param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
    pnh.param("world_frame", world_frame, (std::string) "world");
    pnh.param("get_raw_data", get_raw_data, (bool) false);

	  printf("\n\t\tInitialazed Publishes !!\n");
    ugv_reached_goal_sub_ = pnh.subscribe<upo_actions::NavigateActionResult>("/Navigation/result", 100, &exportDataMission::ugvReachedGoalCB, this);
    uav_reached_goal_sub_ = pnh.subscribe<upo_actions::Navigate3DActionResult>("/UAVNavigation3D/result", 100, &exportDataMission::uavReachedGoalCB, this);
    lenth_cat_sub_ = pnh.subscribe<std_msgs::Float32>("/tie_controller/length_status", 100, &exportDataMission::lengthStatusCB, this);

    initializeVariables();
    // Load and export mission
    loadFileMission lm(nh, pnh);
    lm.exportDataMissionAndTime(Tj, tether_length_vector, v_time_traj_ugv, v_time_traj_uav);
    total_pts = Tj.points.size()-1; //fill after interpolation with real number of WPs

}

void exportDataMission::initializeVariables()
{
    latch_topic = true;
    count = 0;
    ugv_reached_goal = uav_reached_goal = false;
    v_pose_traj_ugv.clear(); v_pose_traj_uav.clear(); v_length_traj_cat.clear(); 
    v_time_traj_ugv.clear(); v_time_traj_uav.clear(); Tj.points.clear();
    v_error_ugv.clear(); v_error_uav.clear();
    v_stand_dev_t_ugv.clear(); v_stand_dev_t_uav.clear();

    sum_error_ugv_xy = sum_error_ugv_z = sum_error_uav_xy  = sum_error_uav_z = sum_error_length = max_t_ugv = max_t_uav = 0.0;
    sum_error_t_ugv = sum_error_t_uav = 0.0;
    max_ugv_xy = max_ugv_z = max_uav_xy = max_uav_z = max_length =0.0;
    min_ugv_xy = min_ugv_z = min_uav_xy = min_uav_z = min_length = min_t_ugv = min_t_uav  = 100000.0;
}

void exportDataMission::ugvReachedGoalCB(const upo_actions::NavigateActionResultConstPtr &msg)
{
  ugv_reached_goal = msg->result.arrived;
  ugv_f_sec = msg->status.goal_id.stamp.sec;
  ugv_f_nsec = msg->status.goal_id.stamp.nsec;
  ugv_s_sec = msg->header.stamp.sec;
  ugv_s_nsec = msg->header.stamp.nsec;
}

void exportDataMission::uavReachedGoalCB(const upo_actions::Navigate3DActionResultConstPtr &msg)
{
  uav_reached_goal = msg->result.arrived;
  uav_f_sec= msg->status.goal_id.stamp.sec;
  uav_f_nsec= msg->status.goal_id.stamp.nsec;
  uav_s_sec = msg->header.stamp.sec;
  uav_s_nsec = msg->header.stamp.nsec;
}

void exportDataMission::lengthStatusCB(const std_msgs::Float32ConstPtr &msg)
{
  length_status = msg->data;
}

void exportDataMission::updateStates()
{
  try{
      uav_tf = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
      // ROS_WARN("Check Mission: Couldn't get UAV Pose (base_frame: %s - world_frame: %s); tf exception: %s", uav_base_frame.c_str(),world_frame.c_str(),ex.what());
    }
  try{
      ugv_tf = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
      // ROS_WARN("Check Mission: Couldn't get UGV Pose(base_frame: %s - world_frame: %s); tf exception: %s", ugv_base_frame.c_str(),world_frame.c_str(),ex.what());
    }

    if (count > 0 && count <total_pts){
      getDistanceTfToTrajectory(ugv_tf, Tj.points.at(count-1).transforms[0].translation, Tj.points.at(count).transforms[0].translation, error_ugv);
      getDistanceTfToTrajectory(uav_tf, Tj.points.at(count-1).transforms[1].translation, Tj.points.at(count).transforms[1].translation, error_uav);
      v_error_ugv.push_back(error_ugv);
      v_error_uav.push_back(error_uav);
    }

  if (ugv_reached_goal && uav_reached_goal)
  {
    current_pos_ugv.x = ugv_tf.transform.translation.x;
    current_pos_ugv.y = ugv_tf.transform.translation.y;
    current_pos_ugv.z = ugv_tf.transform.translation.z;
    current_pos_uav.x = uav_tf.transform.translation.x;
    current_pos_uav.y = uav_tf.transform.translation.y;
    current_pos_uav.z = uav_tf.transform.translation.z;
    float time_ugv = ugv_s_sec+ugv_s_nsec*1e-9-(ugv_f_sec+ugv_f_nsec*1e-9);
    float time_uav = (uav_s_sec+uav_s_nsec*1e-9-(uav_f_sec+uav_f_nsec*1e-9))-178.35;
    v_time_ugv.push_back(time_ugv); v_time_uav.push_back(time_uav);
    printf("  Reached succeessfully WayPoint: %i --> ",count);
    printf("length[%f/%f] p_ugv[%f %f %f / %f %f %f] p_uav[%f %f %f / %f %f %f] t_ugv[%f] t_uav[%f]\n", length_status, tether_length_vector[count],
        current_pos_ugv.x, current_pos_ugv.y, current_pos_ugv.z,
        Tj.points.at(count).transforms[0].translation.x, Tj.points.at(count).transforms[0].translation.y, Tj.points.at(count).transforms[0].translation.z,
        current_pos_uav.x, current_pos_uav.y, current_pos_uav.z,
        Tj.points.at(count).transforms[1].translation.x, Tj.points.at(count).transforms[1].translation.y, Tj.points.at(count).transforms[1].translation.z,
        time_ugv, time_uav);

    v_pose_traj_ugv.push_back(current_pos_ugv); v_pose_traj_uav.push_back(current_pos_uav); v_length_traj_cat.push_back(length_status);
    initial_pos_ugv.x = current_pos_ugv.x; initial_pos_ugv.y = current_pos_ugv.y; initial_pos_ugv.z = current_pos_ugv.z;
    initial_pos_uav.x = current_pos_uav.x; initial_pos_uav.y = current_pos_uav.y; initial_pos_uav.z = current_pos_uav.z;
    ugv_reached_goal = uav_reached_goal = false;
    count++;
  }
}

void exportDataMission::getDistanceTfToTrajectory(geometry_msgs::TransformStamped p_ , geometry_msgs::Vector3 r1_, geometry_msgs::Vector3 r2_, errorDistance &d_)
{
  /* 
    XY-ditance base on distance from point to a line in R^2 : d = |A*x + B*y+C| / sqrt(A^2 + B^2)
    z-distance base on Vector and parametric equation of a line in R^3 : <x, y, z> = <P_x, P_y, P_z> + lambda * <Q_x-P_x, Q_y-P_y, Q_z-P_z>                                                 
  */
  
  //First: get distance inXY
  float A , B , C;
  float x_, y_, z_; // are the value of the point on the line
  float threshold_ = 0.001;
  A = -1.0*(r2_.y - r1_.y);
  B = (r2_.x - r1_.x);
  C = -1.0*(B * r1_.y + A* r1_.x);

  if (fabs(A) < threshold_ && fabs(B) < threshold_){
    d_.xy = 0.0;
  }else if (fabs(A) < threshold_){
    d_.xy = fabs(p_.transform.translation.y-r2_.y);
  }else if (fabs(B)< threshold_){
    d_.xy = fabs(p_.transform.translation.x-r2_.x);
  }else{
    d_.xy = fabs(A*p_.transform.translation.x + B* p_.transform.translation.y + C) /sqrt(A*A + B*B);
  }
  //Second: get distance in Z
  if (fabs(A) < threshold_ || fabs(B) < threshold_){
    d_.z = fabs(p_.transform.translation.z - r2_.z);
  }
  else{  // otherwise both point are in the same position
    x_ = (B*B * p_.transform.translation.x - A*B* p_.transform.translation.y - A*C)/(A*A + B*B);
    y_ = (-1.0*A*B * p_.transform.translation.x + A*A * p_.transform.translation.y - B*C)/(A*A + B*B);
    float lambda_ = (x_ -  r2_.x)/ (r2_.x-r1_.x);
    // float lambda_ = (y_ -  r2_.y)/ (r2_.y-r1_.y);
    z_ = (r2_.z-r1_.z) *lambda_ + r2_.z;
    d_.z = fabs(p_.transform.translation.z - z_);
  }
}

void exportDataMission::computeError()
{
  std::vector<float> v_error_length;  v_error_length.clear();
  float sum_stand_dev_length = 0.0;

  // Compute Mean error for catenary
  for(int i = 0 ; i< count; i++)  {
    float error_length_ = fabs(v_length_traj_cat[i] - tether_length_vector[i]);
    v_error_length.push_back(error_length_);  //Catanary error analisys

    sum_error_length = v_error_length[i] + sum_error_length;
    if(max_length < v_error_length[i])
      max_length = v_error_length[i];
    if(min_length > v_error_length[i])
      min_length = v_error_length[i];
  }
  average_length = sum_error_length/v_error_ugv.size();
  // Compute Standard Deviation for catenary
  for(int i = 0 ; i< count; i++) {
    sum_stand_dev_length = pow(v_error_length[i] - average_length,2) + sum_stand_dev_length;
  }
  stand_dev_length = sqrt(sum_stand_dev_length)/v_error_ugv.size();

  // Compute Mean error for UGV and UAV distance 
  for(int i = 0 ; i< v_error_ugv.size(); i++){
    sum_error_ugv_xy = v_error_ugv[i].xy + sum_error_ugv_xy; 
    sum_error_ugv_z = v_error_ugv[i].z + sum_error_ugv_z; 
    sum_error_uav_xy = v_error_uav[i].xy + sum_error_uav_xy; 
    sum_error_uav_z = v_error_uav[i].z + sum_error_uav_z; 

    if(min_ugv_xy > v_error_ugv[i].xy)
      min_ugv_xy = v_error_ugv[i].xy;
    if(min_ugv_z > v_error_ugv[i].z)
      min_ugv_z = v_error_ugv[i].z;
    if(max_ugv_xy < v_error_ugv[i].xy)
      max_ugv_xy = v_error_ugv[i].xy;
    if(max_ugv_z < v_error_ugv[i].z)
      max_ugv_z = v_error_ugv[i].z;

    if(min_uav_xy > v_error_uav[i].xy)
      min_uav_xy = v_error_uav[i].xy;
    if(min_uav_z > v_error_uav[i].z)
      min_uav_z = v_error_uav[i].z;
    if(max_uav_xy < v_error_uav[i].xy)
      max_uav_xy = v_error_uav[i].xy;
    if(max_uav_z < v_error_uav[i].z)
      max_uav_z = v_error_uav[i].z;
  }
  average_ugv_xy = sum_error_ugv_xy/v_error_ugv.size();
  average_ugv_z = sum_error_ugv_z/v_error_ugv.size();
  average_uav_xy = sum_error_uav_xy/v_error_ugv.size();
  average_uav_z = sum_error_uav_z/v_error_ugv.size();

  // Compute Standard Deviation for UGV and UAV distance
  float sum_stand_dev_ugv_xy, sum_stand_dev_ugv_z, sum_stand_dev_uav_xy, sum_stand_dev_uav_z;
  sum_stand_dev_ugv_xy = sum_stand_dev_ugv_z = sum_stand_dev_uav_xy = sum_stand_dev_uav_z = 0.0;
  for(int i = 0 ; i< v_error_ugv.size(); i++){
    sum_stand_dev_ugv_xy = pow(v_error_ugv[i].xy-average_ugv_xy,2) + sum_stand_dev_ugv_xy; 
    sum_stand_dev_ugv_z  = pow(v_error_ugv[i].z-average_ugv_z,2) + sum_stand_dev_ugv_z;
    sum_stand_dev_uav_xy = pow(v_error_uav[i].xy-average_uav_xy,2) + sum_stand_dev_uav_xy;
    sum_stand_dev_uav_z  = pow(v_error_uav[i].z-average_uav_z,2)+ sum_stand_dev_uav_z;
  }
  stand_dev_ugv_xy = sqrt(sum_stand_dev_ugv_xy)/v_error_ugv.size();
  stand_dev_ugv_z  = sqrt(sum_stand_dev_ugv_z)/v_error_ugv.size();
  stand_dev_uav_xy = sqrt(sum_stand_dev_ugv_xy)/v_error_ugv.size();
  stand_dev_uav_z  = sqrt(sum_stand_dev_ugv_z)/v_error_ugv.size();

  // Compute Mean error for time 
  for (size_t i=0 ; i<v_time_ugv.size();i++)
  {
    float error_t_ugv = fabs(v_time_ugv[i] - v_time_traj_ugv[i]);
    float error_t_uav = fabs(v_time_uav[i] - v_time_traj_uav[i]);
    v_error_t_ugv.push_back(error_t_ugv); v_error_t_uav.push_back(error_t_uav);
    sum_error_t_ugv = error_t_ugv + sum_error_t_ugv;
    sum_error_t_uav = error_t_uav + sum_error_t_uav;

    if(max_t_ugv < error_t_ugv)
       max_t_ugv = error_t_ugv;
    if(min_t_ugv > error_t_ugv)
       min_t_ugv = error_t_ugv;
    if(max_t_uav < error_t_uav)
       max_t_uav = error_t_uav;
    if(min_t_uav > error_t_uav)
       min_t_uav = error_t_uav;
  }
  mean_t_ugv = sum_error_t_ugv / v_time_ugv.size();
  mean_t_uav = sum_error_t_uav / v_time_uav.size();

  float sum_stand_dev_t_ugv, sum_stand_dev_t_uav;
  // Compute Standard Deviation for time 
  for (size_t i=0 ; i<v_time_ugv.size();i++)
  {
    float error_stand_dev_t_ugv = (v_error_t_ugv[i] - mean_t_ugv)*(v_error_t_ugv[i] - mean_t_ugv);
    float error_stand_dev_t_uav = (v_error_t_uav[i] - mean_t_ugv)*(v_error_t_uav[i] - mean_t_ugv);
    v_stand_dev_t_ugv.push_back(error_stand_dev_t_ugv); v_stand_dev_t_uav.push_back(error_stand_dev_t_uav);
    sum_stand_dev_t_ugv = error_stand_dev_t_ugv + sum_stand_dev_t_ugv;
    sum_stand_dev_t_uav = error_stand_dev_t_uav + sum_stand_dev_t_uav;
  }
  stand_dev_t_ugv = sqrt(sum_stand_dev_t_ugv/v_time_ugv.size());
  stand_dev_t_uav = sqrt(sum_stand_dev_t_uav/v_time_ugv.size());
}

void exportDataMission::exportDataError(){
  // Get date for name
  time_t ttime = time(0);
  tm *lt_ = localtime(&ttime);
  string date = to_string(1900 + lt_->tm_year)+"-"+to_string(1 + lt_->tm_mon)+"-"+to_string(lt_->tm_mday)+"-"+to_string(lt_->tm_hour)+to_string(lt_->tm_min)+to_string(1 + lt_->tm_sec);
  
  std::ofstream ofs_data_analysis, ofs_time_analisys, ofs_raw_distance;
  std::ifstream ifile1;

  // Save Data for trajectory tracking analysis
  std::string output_file = statistical_results_path+"statistical_results_from_validation_experiments_"+date+".txt";
  ifile1.open(output_file);
  if(ifile1) {
      std::cout << output_file <<" : File exists !!!!!!!!!! " << std::endl;
  } else {
    ofs_data_analysis.open(output_file.c_str(), std::ofstream::app);
    ofs_data_analysis <<"total_pts"<<std::endl;
    ofs_data_analysis <<total_pts<<std::endl;
    ofs_data_analysis <<"average_ugv_xy;average_uav_xy;average_uav_z;average_length"<<std::endl;
    ofs_data_analysis <<average_ugv_xy<<";"<<average_uav_xy<<";"<<average_uav_z<<";"<<average_length<<std::endl;
    ofs_data_analysis <<"stand_dev_ugv_xy;stand_dev_uav_xy;stand_dev_uav_z;stand_dev_length"<<std::endl;
    ofs_data_analysis <<stand_dev_ugv_xy<<";"<<stand_dev_uav_xy<<";"<<stand_dev_uav_z<<";"<<stand_dev_length<<std::endl;
    ofs_data_analysis <<"max_ugv_xy;max_uav_xy;max_uav_z;max_length"<<std::endl;
    ofs_data_analysis <<max_ugv_xy<<";"<<max_uav_xy<<";"<<max_uav_z<<";"<<max_length<<std::endl;
    ofs_data_analysis <<"min_ugv_xy;min_uav_xy;min_uav_z;min_length"<<std::endl;
    ofs_data_analysis <<min_ugv_xy<<";"<<min_uav_xy<<";"<<min_uav_z<<";"<<min_length<<std::endl;
    ofs_data_analysis.close();
    std::cout << output_file <<" : Saved data for analysis !!!!!!!!!! " << std::endl;
  }

  // Save Data for time analysis
  output_file = statistical_results_path+"time_results_from_validation_experiments_"+date+".txt";
  ifile1.open(output_file);
  
  if(ifile1) {
      std::cout << output_file <<" : File exists !!!!!!!!!! " << std::endl;
  } else {
    ofs_time_analisys.open(output_file.c_str(), std::ofstream::app);
    ofs_time_analisys <<"error_time_ugv;error_time_uav;stand_dev_t_ugv;stand_dev_t_ugv;"<<std::endl;
    for (size_t i=0 ; i<v_time_ugv.size();i++)
    {
      ofs_time_analisys <<v_error_t_ugv[i]<<";"<<v_error_t_uav[i]<<";"<<v_stand_dev_t_ugv[i]<<";"<<v_stand_dev_t_uav[i]<<";"<<std::endl;
    }
    ofs_time_analisys <<"mean_t_ugv;mean_t_uav;stand_dev_t_ugv;stand_dev_t_uav;max_t_ugv;min_t_ugv;max_t_uav;min_t_uav;"<<std::endl;
    ofs_time_analisys <<mean_t_ugv<<";"<<mean_t_uav<<";"<<stand_dev_t_ugv<<";"<<stand_dev_t_uav<<";"
                      <<max_t_ugv<<";"<<min_t_ugv<<";"<<max_t_uav<<";"<<min_t_uav<<";"<<std::endl;
    ofs_time_analisys.close();
    std::cout << output_file <<" : Saved data for analysis !!!!!!!!!! " << std::endl;
  }

   // Save Data for distance analysis
  if(get_raw_data){
    output_file = statistical_results_path+"raw_distance_"+date+".txt";
    ifile1.open(output_file);
    if(ifile1) {
        std::cout << output_file <<" : File exists !!!!!!!!!! " << std::endl;
    } else {
      ofs_raw_distance.open(output_file.c_str(), std::ofstream::app);
      ofs_raw_distance <<"ugv_xy;uav_xy;uav_z;"<<std::endl;
      for (size_t i=0 ; i<v_error_ugv.size();i++)
      {
        ofs_raw_distance <<v_error_ugv[i].xy<<";"<<v_error_uav[i].xy<<";"<<v_error_uav[i].z<<";"<<std::endl;
      }
      ofs_raw_distance.close();
      std::cout << output_file <<" : Saved data for analysis !!!!!!!!!! " << std::endl;
    }
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Get Data Error Mission Node");

  ros::NodeHandlePtr nh;
  ros::NodeHandle pnh("~");

  exportDataMission cm(nh, pnh);
  bool continue_process = true;

	while (ros::ok() && continue_process) {
    ros::spinOnce();
    if (cm.count <= cm.total_pts) // This value is bacause -2 
      cm.updateStates();
    else{
      cm.computeError();
      cm.exportDataError();
      continue_process = false;
    }
  }	
	
	return 0;
}