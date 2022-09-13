#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>

#define PRINTF_BLUE "\x1B[34m"

class getMissionData{

public:
	getMissionData(ros::NodeHandlePtr nh, ros::NodeHandle pnh);
	void statusMissionCB(const std_msgs::Bool::ConstPtr &msg_);
	void lengthStatusCB(const std_msgs::Float32::ConstPtr &msg_);
	void setLengthCB(const std_msgs::Float32::ConstPtr &msg_);
	void getPoseUAV();
    void writeData(geometry_msgs::Point p_ , float l1_, float l2_);

	// ros::NodeHandlePtr nh;
    // ros::NodeHandle pnh("~");

	tf::TransformBroadcaster br;
	tf::Transform trans_ugv, trans_uav;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tf2_list;
    std::unique_ptr<tf::TransformListener> tf_list_ptr;
	
	ros::Subscriber status_mission_sub_, length_status_sub_, set_length_sub_;
	std::string uav_base_frame, ugv_base_frame, world_frame, path;
	
	geometry_msgs::TransformStamped pose_uav;

    std::vector<geometry_msgs::Point> v_pos_uav;
    std::vector<double> v_length_status , v_set_length;
	bool status_flag;
    float set_length, length_status; 

protected:

};

getMissionData::getMissionData(ros::NodeHandlePtr nh, ros::NodeHandle pnh)
{
	printf("\n\tInitialazing get_Mission_Data_NODE !!\n");
	
	nh.reset(new ros::NodeHandle("~"));
  	
	tfBuffer.reset(new tf2_ros::Buffer);
    tf2_list.reset(new tf2_ros::TransformListener(*tfBuffer));
    tf_list_ptr.reset(new tf::TransformListener(ros::Duration(5)));

    status_flag = false;

	pnh.param("path", path, (std::string)"/home/");
	pnh.param("uav_base_frame", uav_base_frame, (std::string)"uav_base_link");
  	pnh.param("ugv_base_frame", ugv_base_frame, (std::string)"ugv_base_link");
  	pnh.param("world_frame", world_frame, (std::string)"world");

    status_mission_sub_ = nh->subscribe("/tie_controller/length_reached", 2000, &getMissionData::statusMissionCB,this);
    length_status_sub_ = nh->subscribe("/tie_controller/length_status", 10, &getMissionData::lengthStatusCB, this);
    set_length_sub_ = nh->subscribe("/tie_controller/set_length", 10, &getMissionData::setLengthCB, this);
}

void getMissionData::statusMissionCB(const std_msgs::Bool::ConstPtr &msg_)
{
	float status_data_= msg_->data;
    geometry_msgs::Point p_uav_;

    if (status_data_ && !status_flag){
    	ROS_INFO(PRINTF_BLUE "Receiving data from topic: /tie_controller/length_reached");
        ROS_INFO(PRINTF_BLUE "Receiving data from topic: /tie_controller/length_status");
        ROS_INFO(PRINTF_BLUE "Receiving data from topic: /tie_controller/set_length");
        status_flag = true;
        getPoseUAV();
        p_uav_.x = pose_uav.transform.translation.x;
        p_uav_.y = pose_uav.transform.translation.y;
        p_uav_.z = pose_uav.transform.translation.z;
        v_pos_uav.push_back(p_uav_);
        v_set_length.push_back(set_length);
        v_length_status.push_back(length_status);
        int s_ = v_pos_uav.size();
        printf("sizes: v_pos_uav=[%lu], v_set_length=[%lu], v_length_status=[%lu]\n", v_pos_uav.size(), v_set_length.size(), v_length_status.size());
        printf("v_pos_uav [%f %f %f] , v_set_length=[%f], v_length_status=[%f]\n",v_pos_uav[s_-1].x, v_pos_uav[s_-1].y,v_pos_uav[s_-1].z, 
            v_set_length[s_-1], v_length_status[s_-1]);
        writeData(p_uav_, set_length, length_status);
    }
    else{
        status_flag = false;
    }
}

void getMissionData::lengthStatusCB(const std_msgs::Float32::ConstPtr &msg_)
{
	length_status= msg_->data;
}

void getMissionData::setLengthCB(const std_msgs::Float32::ConstPtr &msg_)
{
	set_length = msg_->data;
}

void getMissionData::getPoseUAV()
{
	try{
        pose_uav = tfBuffer->lookupTransform(world_frame, uav_base_frame, ros::Time(0));
    }catch (tf2::TransformException &ex){
        ROS_WARN("get_mission_data_node: Couldn't get UAV Pose (frame: %s), so not possible to set Tether start point; tf exception: %s", world_frame.c_str(),ex.what());
    }
}

void getMissionData::writeData(geometry_msgs::Point p_ , float l1_, float l2_)
{
    std::ofstream ofs;
	std::string name_output_file = path+"mission_data_tethered_marsupial.txt";
	ofs.open(name_output_file.c_str(), std::ofstream::app);
	if (ofs.is_open()) 
		ofs << p_.x << ";" << p_.y << ";" << p_.z << ";" << l1_ << ";" << l2_ << ";" << std::endl;
	ofs.close();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_mission_data_node");

	ros::NodeHandlePtr nh;
    ros::NodeHandle pnh("~");
    getMissionData gmd(nh,pnh);

	while (ros::ok()) {
        ros::spinOnce();
        // r.sleep();
    }	
	
	return 0;
}
