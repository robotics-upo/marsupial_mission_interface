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
    pnh.param("vel_ugv", vel_ugv, (float)1.0);
    pnh.param("vel_uav", vel_uav, (float)1.0);
    pnh.param("d_min_wp", d_min_wp, (float)0.1);

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
    v_error_ugv.clear(); v_error_uav.clear(); v_gt_d_wp_ugv.clear(); v_gt_d_wp_uav.clear();
    v_vel_ugv.clear(); v_vel_uav.clear(); v_acc_ugv.clear(); v_acc_uav.clear();

    sum_error_ugv_xy = sum_error_ugv_z = sum_error_uav_xy  = sum_error_uav_z = sum_error_length = max_t_ugv = max_t_uav = 0.0;
    sum_error_t_ugv = sum_error_t_uav = 0.0;
    sum_error_v_ugv = sum_error_v_uav = sum_error_a_ugv = sum_error_a_uav = sum_v_ugv_ = 0.0;
    max_ugv_xy = max_ugv_z = max_uav_xy = max_uav_z = max_length = max_v_ugv = max_v_uav = max_a_uav = max_a_uav = 0.0;
    min_ugv_xy = min_ugv_z = min_uav_xy = min_uav_z = min_length = min_t_ugv = min_t_uav = min_v_uav = min_v_ugv = min_a_uav = min_a_ugv = 100000.0;
    prev_sum_v_ugv = prev_time = 0.0;
    prev_size = -1;
}

void exportDataMission::ugvReachedGoalCB(const upo_actions::NavigateActionResultConstPtr &msg)
{
  ugv_reached_goal = msg->result.arrived;
  ugv_f_sec = msg->status.goal_id.stamp.sec;
  ugv_f_nsec = msg->status.goal_id.stamp.nsec;
  ugv_s_sec = msg->header.stamp.sec;
  ugv_s_nsec = msg->header.stamp.nsec;
  ugv_tf = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
}

void exportDataMission::uavReachedGoalCB(const upo_actions::Navigate3DActionResultConstPtr &msg)
{
  uav_reached_goal = msg->result.arrived;
  uav_f_sec= msg->status.goal_id.stamp.sec;
  uav_f_nsec= msg->status.goal_id.stamp.nsec;
  uav_s_sec = msg->header.stamp.sec;
  uav_s_nsec = msg->header.stamp.nsec;
  uav_tf = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
}

void exportDataMission::lengthStatusCB(const std_msgs::Float32ConstPtr &msg)
{
  length_status = msg->data;
}

void exportDataMission::updateStates()
{
  float time_ugv, time_uav, d_wp_ugv, d_wp_uav;
  if (ugv_reached_goal && uav_reached_goal)
  {
    curr_p_ugv = ugv_tf.transform.translation;
    curr_p_uav = uav_tf.transform.translation;
    if (count > 0){
      d_wp_ugv = sqrt(pow(curr_p_ugv.x-init_p_ugv.x,2)+pow(curr_p_ugv.y-init_p_ugv.y,2));
      d_wp_uav = sqrt(pow(curr_p_uav.x-init_p_uav.x,2)+pow(curr_p_uav.y-init_p_uav.y,2)+pow(curr_p_uav.z-init_p_uav.z,2));
      time_ugv = fabs((ugv_s_sec+ugv_s_nsec*1e-9-(ugv_f_sec+ugv_f_nsec*1e-9))-0.27); // To substract constant because of time difference between machines
      time_uav = fabs((uav_s_sec+uav_s_nsec*1e-9-(uav_f_sec+uav_f_nsec*1e-9))-177.82); // To substract constant because of time difference between machines
      v_d_wp_ugv.push_back(d_wp_ugv); v_d_wp_uav.push_back(d_wp_uav);
      v_time_ugv.push_back(time_ugv); v_time_uav.push_back(time_uav);
    printf("  Reached succeessfully WayPoint: %i --> ",count);
    printf("length[%f/%f] p_ugv[%f %f %f / %f %f %f] p_uav[%f %f %f / %f %f %f] t_ugv[%f] t_uav[%f]\n", length_status, tether_length_vector[count],
        curr_p_ugv.x, curr_p_ugv.y, curr_p_ugv.z,
        Tj.points.at(count).transforms[0].translation.x, Tj.points.at(count).transforms[0].translation.y, Tj.points.at(count).transforms[0].translation.z,
        curr_p_uav.x, curr_p_uav.y, curr_p_uav.z,
        Tj.points.at(count).transforms[1].translation.x, Tj.points.at(count).transforms[1].translation.y, Tj.points.at(count).transforms[1].translation.z,
        time_ugv, time_uav);
    
      float d_min_wp_ugv = sqrt(pow(Tj.points.at(count).transforms[0].translation.x - Tj.points.at(count-1).transforms[0].translation.x,2)+
                          pow(Tj.points.at(count).transforms[0].translation.y - Tj.points.at(count-1).transforms[0].translation.y,2));
      float d_min_wp_uav = sqrt(pow(Tj.points.at(count).transforms[1].translation.x - Tj.points.at(count-1).transforms[1].translation.x,2)+
                          pow(Tj.points.at(count).transforms[1].translation.y - Tj.points.at(count-1).transforms[1].translation.y,2)+
                          pow(Tj.points.at(count).transforms[1].translation.z - Tj.points.at(count-1).transforms[1].translation.z,2));
      v_gt_d_wp_ugv.push_back(d_min_wp_ugv); v_gt_d_wp_uav.push_back(d_min_wp_uav);
      float _v_ugv = d_wp_ugv/time_ugv;
      float _v_uav = d_wp_uav/time_uav;
      if (d_min_wp_ugv < d_min_wp){
        _v_ugv = 0.0;
      }
      if (d_min_wp_uav < d_min_wp){
        _v_uav = 0.0;
      }

      // printf("%i | d_wp_ugv=%f/%f , time_ugv=%f/%f , v_ugv=%f  | d_wp_uav=%f/%f , time_uav=%f/%f , v_uav=%f\n",count, 
      //               d_wp_ugv, d_min_wp_ugv, time_ugv, v_time_traj_ugv[count], _v_ugv, 
      //               d_wp_uav, d_min_wp_uav, time_uav, v_time_traj_uav[count], _v_uav);
    }
    else{
      v_d_wp_ugv.push_back(0.0); v_d_wp_uav.push_back(0.0);
      v_time_ugv.push_back(0.0); v_time_uav.push_back(0.0);
      v_gt_d_wp_ugv.push_back(0.0); v_gt_d_wp_uav.push_back(0.0);
    }
    v_pose_traj_ugv.push_back(curr_p_ugv); v_pose_traj_uav.push_back(curr_p_uav); v_length_traj_cat.push_back(length_status);

    init_p_ugv.x = curr_p_ugv.x; init_p_ugv.y = curr_p_ugv.y; init_p_ugv.z = curr_p_ugv.z;
    init_p_uav.x = curr_p_uav.x; init_p_uav.y = curr_p_uav.y; init_p_uav.z = curr_p_uav.z;
    ugv_reached_goal = uav_reached_goal = false;
    count++;
  }
  else{
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

    sum_error_length = error_length_ + sum_error_length;
    if(max_length < v_error_length[i])
       max_length = v_error_length[i];
    if(min_length > v_error_length[i])
       min_length = v_error_length[i];
  }
  average_length = sum_error_length/v_error_ugv.size();

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

  // Compute Mean error for time , velocity and accelerations
  for (size_t i=0 ; i<v_time_ugv.size();i++)
  {
    // time
    // float error_t_ugv = fabs(v_time_ugv[i] - v_time_traj_ugv[i]);
    float error_t_ugv = fabs(v_time_ugv[i]);
    // float error_t_uav = fabs(v_time_uav[i] - v_time_traj_uav[i]);
    float error_t_uav = fabs(v_time_uav[i]);
    v_error_t_ugv.push_back(error_t_ugv); v_error_t_uav.push_back(error_t_uav);
    sum_error_t_ugv = error_t_ugv + sum_error_t_ugv;
    sum_error_t_uav = error_t_uav + sum_error_t_uav;
    if (i> 0){
      if(max_t_ugv < error_t_ugv)
        max_t_ugv = error_t_ugv;
      if(min_t_ugv > error_t_ugv)
        min_t_ugv = error_t_ugv;
      if(max_t_uav < error_t_uav)
        max_t_uav = error_t_uav;
      if(min_t_uav > error_t_uav)
        min_t_uav = error_t_uav;
    }
    // velocities
    float error_v_ugv, error_v_uav;
    if (v_gt_d_wp_ugv[i] > d_min_wp){
      // error_v_ugv = fabs(v_d_wp_ugv[i]/v_time_ugv[i] - vel_ugv);
      error_v_ugv = fabs(v_d_wp_ugv[i]/v_time_ugv[i]);
    }else
      error_v_ugv = 0.0;
    if (v_gt_d_wp_uav[i] > d_min_wp){
      // error_v_uav = fabs(v_d_wp_uav[i]/v_time_uav[i] - vel_uav);
      error_v_uav = fabs(v_d_wp_uav[i]/v_time_uav[i]);
    }else
      error_v_uav = 0.0;
    v_error_v_ugv.push_back(error_v_ugv); v_error_v_uav.push_back(error_v_uav);
    sum_error_v_ugv = error_v_ugv + sum_error_v_ugv;
    sum_error_v_uav = error_v_uav + sum_error_v_uav;
    if (i> 0){
      if(max_v_ugv < error_v_ugv)
        max_v_ugv = error_v_ugv;
      if(min_v_ugv > error_v_ugv)
        min_v_ugv = error_v_ugv;
      if(max_v_uav < error_v_uav)
        max_v_uav = error_v_uav;
      if(min_v_uav > error_v_uav)
        min_v_uav = error_v_uav;
    }
  // acceleration
    float error_a_ugv, error_a_uav;
    if (i > 0){
      if (v_gt_d_wp_ugv[i] > d_min_wp)
        error_a_ugv = fabs(v_d_wp_ugv[i]/v_time_ugv[i]-v_d_wp_ugv[i-1]/v_time_ugv[i-1])/(v_time_ugv[i]+v_time_ugv[i-1]);
      else
        error_a_ugv = 0.0;
      if (v_gt_d_wp_uav[i] > d_min_wp && (v_time_uav[i-1] > 0.0001)){
        error_a_uav = fabs(v_d_wp_uav[i]/v_time_uav[i]-v_d_wp_uav[i-1]/v_time_uav[i-1])/(v_time_uav[i]+v_time_uav[i-1]);
      }
      else
        error_a_uav = 0.0;
      v_error_a_ugv.push_back(error_a_ugv); v_error_a_uav.push_back(error_a_uav);
      sum_error_a_ugv = error_a_ugv + sum_error_a_ugv;
      sum_error_a_uav = error_a_uav + sum_error_a_uav;
      if (i> 0){
        if(max_a_ugv < error_a_ugv)
          max_a_ugv = error_a_ugv;
        if(min_a_ugv > error_a_ugv)
          min_a_ugv = error_a_ugv;
        if(max_a_uav < error_a_uav)
          max_a_uav = error_a_uav;
        if(min_a_uav > error_a_uav)
          min_a_uav = error_a_uav;
      }
    }
  }
  mean_t_ugv = sum_error_t_ugv / v_time_ugv.size();
  mean_v_ugv = sum_error_v_ugv / v_time_ugv.size();
  mean_a_ugv = sum_error_a_ugv / v_time_ugv.size(); // Lengh two units less
  mean_t_uav = sum_error_t_uav / v_time_uav.size();
  mean_v_uav = sum_error_v_uav / v_time_uav.size();
  mean_a_uav = sum_error_a_uav / v_time_uav.size(); // Lengh two units less
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
    ofs_time_analisys <<"error_time_ugv;error_time_uav;error_vel_ugv;error_vel_uav;error_acc_ugv;error_acc_uav;"<<std::endl;
    for (size_t i=0 ; i<v_time_ugv.size();i++)
    {
      ofs_time_analisys <<v_error_t_ugv[i]<<";"<<v_error_t_uav[i]<<";"<< v_error_v_ugv[i]<<";"<<v_error_v_uav[i]<<";";
      if (i< v_time_ugv.size()-1)                  
        ofs_time_analisys<<v_error_a_ugv[i]<<";"<<v_error_a_uav[i]<<";"<<std::endl;
      else
        ofs_time_analisys<<std::endl;
    }
    ofs_time_analisys <<"mean_t_ugv;max_t_ugv;min_t_ugv;mean_t_uav;max_t_uav;min_t_uav;"<<std::endl;
    ofs_time_analisys <<mean_t_ugv<<";"<<max_t_ugv<<";"<<min_t_ugv<<";"<<mean_t_uav<<";"<<max_t_uav<<";"<<min_t_uav<<";"<<std::endl;
    ofs_time_analisys <<"mean_v_ugv;max_v_ugv;min_v_ugv;mean_v_uav;max_v_uav;min_v_uav;"<<std::endl;
    ofs_time_analisys <<mean_v_ugv<<";"<<max_v_ugv<<";"<<min_v_ugv<<";"<<mean_v_uav<<";"<<max_v_uav<<";"<<min_v_uav<<";"<<std::endl;
    ofs_time_analisys <<"mean_a_ugv;max_a_ugv;min_a_ugv;mean_a_uav;max_a_uav;min_a_uav;"<<std::endl;
    ofs_time_analisys <<mean_a_ugv<<";"<<max_a_ugv<<";"<<min_a_ugv<<";"<<mean_a_uav<<";"<<max_a_uav<<";"<<min_a_uav<<";"<<std::endl;
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