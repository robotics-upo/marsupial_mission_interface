#include <iostream>
// #include <fstream>
#include <string>
#include <vector>
// #include <yaml-cpp/yaml.h>

// #include <geometry_msgs/PoseStamped.h>
// #include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>


#include "mission_interface/compute_catenary_3D.h"

#define PRINTF_BLUE "\x1B[34m"

class CatenaryMarker{

public:
	CatenaryMarker(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
  void markerPointsCB(const std_msgs::Float32ConstPtr &msg);

  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::unique_ptr<tf2_ros::TransformListener> tf2_list;

	
  // ros::NodeHandlePtr nh;
  ros::Subscriber catenary_sub_;
  ros::Publisher catenary_marker_pub_;

  bisectionCat BisCat;

  std::string ugv_base_frame, uav_base_frame, world_frame; 
  std::vector<geometry_msgs::Point> points_catenary_final;
  visualization_msgs::MarkerArray _cat_marker; 

  geometry_msgs::TransformStamped uav_tf, ugv_tf;

private:

protected:

};

CatenaryMarker::CatenaryMarker(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	  printf("\n\tInitialazing Check Marker NODE !!\n");
    
    nh.reset(new ros::NodeHandle("~"));

    tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));

    pnh.param("ugv_base_frame", ugv_base_frame, (std::string) "ugv_base_link");
    pnh.param("uav_base_frame", uav_base_frame, (std::string) "uav_base_link");
    pnh.param("world_frame", world_frame, (std::string) "world");

    catenary_sub_ = nh->subscribe("/tie_controller/length_status", 5, &CatenaryMarker::markerPointsCB,this);
    catenary_marker_pub_= pnh.advertise<visualization_msgs::MarkerArray>("catenary_marker", 100);

	  printf("CatenaryMarker: Frames UGV (base_frame: %s - world_frame: %s)\n", ugv_base_frame.c_str(),world_frame.c_str());
	  printf("CatenaryMarker: Frames UAV (base_frame: %s - world_frame: %s)\n", uav_base_frame.c_str(),world_frame.c_str());
}

void CatenaryMarker::markerPointsCB(const std_msgs::Float32ConstPtr &msg)
{

  try{
    uav_tf = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
    ROS_INFO("\tCatenaryMarker: Got UAV transform world-base_link[%f %f %f]",uav_tf.transform.translation.x,uav_tf.transform.translation.y,uav_tf.transform.translation.z);
  }    
  catch (tf2::TransformException &ex){
    ROS_WARN("CatenaryMarker: Couldn't get position initial UAV (base_frame: %s - world_frame: %s), so not possible to set UAV start point; tf exception: %s",
      uav_base_frame.c_str(),world_frame.c_str(),ex.what());
  }

  try{
    ugv_tf = tfBuffer->lookupTransform(world_frame, ugv_base_frame, ros::Time(0));
    ROS_INFO("\tCatenaryMarker: Got UGV transform world-base_link[%f %f %f]",ugv_tf.transform.translation.x, ugv_tf.transform.translation.y, ugv_tf.transform.translation.z);

    }
    catch (tf2::TransformException &ex){
      ROS_WARN("CatenaryMarker: Couldn't get UGV Pose (base_frame: %s - world_frame: %s), so not possible to set UGV start point; tf exception: %s",
	       ugv_base_frame.c_str(),world_frame.c_str(),ex.what());
    }

  points_catenary_final.clear();
  
    BisCat.configBisection(msg->data, 
			   ugv_tf.transform.translation.x,
			   ugv_tf.transform.translation.y,
			   ugv_tf.transform.translation.z + 0.4, 
			   uav_tf.transform.translation.x,
			   uav_tf.transform.translation.y,
			   uav_tf.transform.translation.z);
    BisCat.getPointCatenary3D(points_catenary_final);

    _cat_marker.markers.resize(points_catenary_final.size());
            
    for (size_t i = 0; i < points_catenary_final.size(); ++i){
      // double c_color1 = ((double)i / (double)points_catenary_final.size());
			// double c_color2 = ((double)i / (double)points_catenary_final.size());
      _cat_marker.markers[i].header.frame_id = "world";
      _cat_marker.markers[i].header.stamp = ros::Time::now();
      _cat_marker.markers[i].ns = "cat_marker";
      _cat_marker.markers[i].id = i*1000;
      _cat_marker.markers[i].action = visualization_msgs::Marker::ADD;
      if (i % 5 == 0 )
	      _cat_marker.markers[i].type = visualization_msgs::Marker::CUBE;
      else
	      _cat_marker.markers[i].type = visualization_msgs::Marker::SPHERE;
      _cat_marker.markers[i].lifetime = ros::Duration(0);
      _cat_marker.markers[i].pose.position.x = points_catenary_final[i].x; 
      _cat_marker.markers[i].pose.position.y = points_catenary_final[i].y; 
      _cat_marker.markers[i].pose.position.z = points_catenary_final[i].z;

      _cat_marker.markers[i].pose.orientation.x = 0.0;
      _cat_marker.markers[i].pose.orientation.y = 0.0;
      _cat_marker.markers[i].pose.orientation.z = 0.0;
      _cat_marker.markers[i].pose.orientation.w = 1.0;
      _cat_marker.markers[i].scale.x = 0.1;
      _cat_marker.markers[i].scale.y = 0.1;
      _cat_marker.markers[i].scale.z = 0.1;
      _cat_marker.markers[i].color.a = 1.0;
      _cat_marker.markers[i].color.r = 1.0 - i/points_catenary_final.size();
      _cat_marker.markers[i].color.g = i/points_catenary_final.size();
      _cat_marker.markers[i].color.b = i/points_catenary_final.size();
      // _cat_marker.markers[i].color.r = 1.0 - c_color1;
			// _cat_marker.markers[i].color.g = c_color2;
			// _cat_marker.markers[i].color.b = 0.5;
			// _cat_marker.markers[i].color.a = 1.0; 
    }	
    catenary_marker_pub_.publish(_cat_marker);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Catenary Marker Node");

  ros::NodeHandlePtr nh;
  ros::NodeHandle pnh("~");

  CatenaryMarker cm(nh, pnh);

	while (ros::ok()) {
    ros::spinOnce();
  }	
	
	return 0;
}