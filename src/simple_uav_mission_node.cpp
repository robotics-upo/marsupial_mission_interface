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
#include <upo_actions/Navigate3DAction.h>
#include <upo_actions/TakeOffAction.h>
#include <upo_actions/LandingAction.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include <yaml-cpp/yaml.h>
#include <std_srvs/Trigger.h>


class MissionInterface
{
    typedef actionlib::SimpleActionClient<upo_actions::Navigate3DAction> Navigate3DAction;
    typedef actionlib::SimpleActionClient<upo_actions::TakeOffAction> TakeOffAction;
    typedef actionlib::SimpleActionClient<upo_actions::LandingAction> LandingAction;

    enum taskType {unknown = 0, take_off, land, wp, home, set_home, wait, wait_trigger};

    struct missionTask
    {
        taskType type_;
        double altitude_;
        double seconds_;
        int trigger_;
        double x_, y_, z_;
        double qx_, qy_, qz_, qw_;
    };

public:
    MissionInterface(std::string node_name)
    {
        // Local node handler
        nh_.reset(new ros::NodeHandle("~"));  
        
        // Read node parameters
        nh_->param("mission_file_path", mission_file_, (std::string) "~/");
        nh_->param("base_frame", base_frame_, (std::string) "base_link");
        nh_->param("world_frame", world_frame_, (std::string) "world");
        nh_->param("start_immediately", startImmediately_, true);
        
        // Load yaml file with the mission tasks
        loadMission(mission_file_);
        printf("Mission:\n");
        for (size_t i =0; i < mission_.size(); i++ ){
            missionTask &t = mission_[i];
            switch (t.type_)
            {
            case take_off:
                printf("[%lu] take-off: %lf m\n", i, t.altitude_);           
                break;
            case land:
                printf("[%lu] land\n", i);           
                break;
            case wp:
                printf("[%lu] wp: [%lf %lf %lf][%lf %lf %lf %lf]\n", i, t.x_, t.y_, t.z_, t.qx_, t.qy_, t.qz_, t.qw_);           
                break;
            case home:
                printf("[%lu] home\n", i);           
                break;
            case set_home:
                printf("[%lu] set-home\n", i);           
                break;
            case wait:
                printf("[%lu] wait: %lf s\n", i, t.seconds_);           
                break;
            case wait_trigger:
                if(t.seconds_ >= 0)
                    printf("[%lu] wait-trigger: %lf s trigger %d\n", i, t.seconds_, t.trigger_);    
                else
                    printf("[%lu] wait-trigger: INF s trigger %d\n", i, t.trigger_);     
                break;
            }
        }
        
        // setup subscribers 
        startMission_ = false;
        if(startImmediately_)
            startMission_ = true;
        startMissionSub_ = nh_->subscribe<std_msgs::Bool>("startMission", 1, &MissionInterface::startMissionCallback, this);
        for(int i=0; i<10; i++)
        {
            inputTriggers_[i] = false;
            std::string topicName = "inputTrigger" + std::to_string(i);
            const boost::function<void (const std_msgs::BoolConstPtr &)> triggerFunc = boost::bind(&MissionInterface::inputTriggerCallback, this, _1, i);
            triggerSubs_.push_back(nh_->subscribe<std_msgs::Bool>(topicName, 100, triggerFunc));
        }

        // setup services
        printf("Initialazing UAV Navigation Client\n");
        navigate3DClient_.reset(new Navigate3DAction("/UAVNavigation3D", true));
        navigate3DClient_->waitForServer();
        printf("Initialazing TakeOff Client\n");
        takeOffClient_.reset(new TakeOffAction("/TakeOff", true));
        takeOffClient_->waitForServer();
        printf("Initialazing Landing Client\n");
        landingClient_.reset(new LandingAction("/Landing", true));
        landingClient_->waitForServer();

        // Mission does not start until a UAV pose is received
        printf("Waiting for UAV pose... ");
        tf::StampedTransform homeTf;
        try {
            tfListener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(5.0));
            tfListener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), homeTf);
            printf(" done!\n");
        }
        catch (tf::TransformException ex) {
            printf("Mission Interface: Couldn't get position initial UAV (base_frame: %s - world_frame: %s), so not possible to set UAV start point; tf exception: %s\n",
                world_frame_.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    void executeMission(void)
    {
        int taskIndex = 0;
        missionTask homeWp = {unknown};

        if(!startMission_) {
            printf("Waiting for mission start trigger ...\n");
            while(ros::ok() && !startMission_) {
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
        }
        printf("Mission started:\n");

        while(ros::ok()){
            
            // Get current task
            missionTask t = mission_[taskIndex];
            switch (t.type_) {
            case taskType::take_off: {
                upo_actions::TakeOffGoal takeOffGoal;
                printf("[%d] taking-off ... \n", taskIndex); 
                takeOffGoal.takeoff_height.data = t.altitude_;
                takeOffClient_->sendGoal(takeOffGoal);
                while(!takeOffClient_->waitForResult(ros::Duration(30.0))){}
                if (takeOffClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    printf("[%d] \tsucceessful\n", taskIndex);
                else{
                    printf("[%d] \tfail\n", taskIndex);
                    printf("Mission cancelled\n");
                    return;
                }
                break;
            }

            case taskType::land: {
                upo_actions::LandingGoal landingGoal;
                printf("[%d] Landing ...\n", taskIndex); 
                landingClient_->sendGoal(landingGoal);
                while(!landingClient_->waitForResult(ros::Duration(30.0))){}
                if (landingClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    printf("[%d] \tsucceessful\n", taskIndex);
                else{
                    printf("[%d] \tfail\n", taskIndex);
                    printf("Mission cancelled\n");
                    return;
                }
                break;
            }

            case taskType::wp: {
                upo_actions::Navigate3DGoal wpGoal;
                wpGoal.global_goal.pose.position.x = t.x_;
                wpGoal.global_goal.pose.position.y = t.y_;
                wpGoal.global_goal.pose.position.z = t.z_;
                wpGoal.global_goal.pose.orientation.x = t.qx_;
                wpGoal.global_goal.pose.orientation.y = t.qy_;
                wpGoal.global_goal.pose.orientation.z = t.qz_;
                wpGoal.global_goal.pose.orientation.w = t.qw_;
                printf("[%d] wp [%lf, %lf, %lf] ...\n", taskIndex, t.x_, t.y_, t.z_); 
                navigate3DClient_->sendGoal(wpGoal);
                while(!navigate3DClient_->waitForResult(ros::Duration(200.0))){}
                if (navigate3DClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    printf("[%d] \tsucceessful\n", taskIndex);
                else{
                    printf("[%d] \tfail\n", taskIndex);
                    printf("Mission cancelled\n");
                    return;
                }
                break;
            }

            case taskType::home: {
                printf("[%d] Homing ...\n", taskIndex); 
                if(homeWp.type_ != taskType::wp) {
                    printf("[%d] \tfail. Home position not set.\n", taskIndex); 
                    printf("Mission cancelled\n");
                    return;
                }  
                upo_actions::Navigate3DGoal wpGoal;
                wpGoal.global_goal.pose.position.x = homeWp.x_;
                wpGoal.global_goal.pose.position.y = homeWp.y_;
                wpGoal.global_goal.pose.position.z = homeWp.z_;
                wpGoal.global_goal.pose.orientation.x = homeWp.qx_;
                wpGoal.global_goal.pose.orientation.y = homeWp.qy_;
                wpGoal.global_goal.pose.orientation.z = homeWp.qz_;
                wpGoal.global_goal.pose.orientation.w = homeWp.qw_;
                navigate3DClient_->sendGoal(wpGoal);
                while(!navigate3DClient_->waitForResult(ros::Duration(200.0))){}
                if (navigate3DClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    printf("[%d] \tsucceessful\n", taskIndex);
                else{
                    printf("[%d] \tfail\n", taskIndex);
                    printf("Mission cancelled\n");
                    return;
                }
                break;
            }

            case taskType::set_home: {
                printf("[%d] Setting home position ...\n", taskIndex);    
                tf::StampedTransform homeTf;
                try {
                    tfListener_.waitForTransform(world_frame_, base_frame_, ros::Time(0), ros::Duration(5.0));
                    tfListener_.lookupTransform(world_frame_, base_frame_, ros::Time(0), homeTf);
                }
                catch (tf::TransformException ex) {
                    printf("[%d] \tfail. Could not get robot position.\n", taskIndex);  
                    printf("Mission cancelled\n"); 
                    return;
                }
                homeWp.type_ = taskType::wp;
                homeWp.x_ = homeTf.getOrigin().getX();
                homeWp.y_ = homeTf.getOrigin().getY();
                homeWp.z_ = homeTf.getOrigin().getZ();
                homeWp.qx_ = homeTf.getRotation().getX();
                homeWp.qy_ = homeTf.getRotation().getY();
                homeWp.qz_ = homeTf.getRotation().getZ();
                homeWp.qw_ = homeTf.getRotation().getW();
                printf("[%d] \tsucceessful\n", taskIndex);
                break;
            }

            case taskType::wait: {
                printf("[%d] Wating %lf s...\n", taskIndex, t.seconds_);    
                ros::Time t0 = ros::Time::now(); 
                while((ros::Time::now()-t0).toSec() < t.seconds_ && ros::ok()) {
                    ros::Duration(0.1).sleep();
                    ros::spinOnce();
                }  
                printf("[%d] \tsucceessful\n", taskIndex);
                break;
            }

            case taskType::wait_trigger: {
                printf("[%d] Wating %lf s for input trigger %d\n", taskIndex, t.seconds_, t.trigger_); 
                ros::Time t0 = ros::Time::now(); 
                while( ((ros::Time::now()-t0).toSec() < t.seconds_ || t.seconds_ < 0) && ros::ok() && inputTriggers_[t.trigger_] == false) {
                    ros::Duration(0.1).sleep();
                    ros::spinOnce();
                } 
                printf("[%d] \tsucceessful\n", taskIndex); 
                break;
            }
            }

            taskIndex++;
            ros::spinOnce();
        }
    }

    void loadMission(const std::string &path_file)
    {
        YAML::Node file = YAML::LoadFile(path_file);

        // read number of taks  
        int size = (file["length"].as<int>());

        // read tasks one by one 
        mission_.clear();
        for (int i = 0; i < size; i++) {
            missionTask t;

            // Create the task text to search
            std::string task = "task" + std::to_string(i);

            // Read task type and parse it
            std::string typeString = file[task]["type"].as<std::string>();
            if(typeString.compare("take-off") == 0)
                t.type_ = taskType::take_off;
            else if(typeString.compare("land") == 0)
                t.type_ = taskType::land;
            else if(typeString.compare("wp") == 0)
                t.type_ = taskType::wp;
            else if(typeString.compare("home") == 0)
                t.type_ = taskType::home;
            else if(typeString.compare("set-home") == 0)
                t.type_ = taskType::set_home;
            else if(typeString.compare("wait") == 0)
                t.type_ = taskType::wait;
            else if(typeString.compare("wait-trigger") == 0)
                t.type_ = taskType::wait_trigger;
            else 
            {
                ROS_WARN("%s: Unknown type %s. skipping.", typeString.c_str(), task.c_str()); 
                continue;
            }
            
            // Read task parameters depending on the task type
            switch(t.type_)
            {
                case taskType::take_off:
                    t.altitude_ = file[task]["altitude"].as<double>();
                    if(t.altitude_ <= 0)
                    {
                        ROS_WARN("%s: altitude must be >0. skipping.", typeString.c_str()); 
                        continue;
                    } 
                    break;
                case taskType::wait:
                    t.seconds_ = file[task]["seconds"].as<double>();
                    if(t.seconds_ <= 0)
                    {
                        ROS_WARN("%s: seconds must be >0. skipping.", typeString.c_str()); 
                        continue;
                    } 
                    break;
                case taskType::wait_trigger:
                    t.seconds_ = file[task]["seconds"].as<double>();
                    t.trigger_ = file[task]["trigger"].as<int>();
                    if(t.trigger_ < 0 || t.trigger_ > 9)
                    {
                        ROS_WARN("%s: Input trigger id must be in interval [0,9]. skipping.", typeString.c_str()); 
                        continue;
                    }   
                    break;
                case taskType::wp:
                    t.x_ = file[task]["position"]["x"].as<double>();
                    t.y_ = file[task]["position"]["y"].as<double>();
                    t.z_ = file[task]["position"]["z"].as<double>();
                    t.qx_ = file[task]["orientation"]["x"].as<double>();
                    t.qy_ = file[task]["orientation"]["y"].as<double>();
                    t.qz_ = file[task]["orientation"]["z"].as<double>();
                    t.qw_ = file[task]["orientation"]["w"].as<double>();
                    break;
            }

            // Store the task into the mission
            mission_.push_back(t);
        }
        std::cout << "YAML FILE readed. YAML FILE NAME: " << path_file << std::endl;
        std::cout << "Number of tasks: " << mission_.size() << std::endl;
    }

    void startMissionCallback(const std_msgs::BoolConstPtr &msg)
    {
        startMission_ = msg->data;
    }  

    void inputTriggerCallback(const std_msgs::BoolConstPtr &msg, int trigger)
    {
        inputTriggers_[trigger] = msg->data;
    } 

    // Node params
    bool startImmediately_;
    std::string base_frame_, world_frame_, mission_file_; 

    // Subscribers
    ros::Subscriber startMissionSub_;
    std::vector<ros::Subscriber> triggerSubs_;

    // Services
    std::unique_ptr<Navigate3DAction> navigate3DClient_;
    std::unique_ptr<TakeOffAction> takeOffClient_;
    std::unique_ptr<LandingAction> landingClient_;

    // Node variables
    std::vector<missionTask> mission_;
    bool startMission_;
    ros::NodeHandlePtr nh_;
    bool inputTriggers_[10];

    // TF stuff
    tf::TransformListener tfListener_;
};


int main(int argc, char **argv)
{
    std::string node_name = "simple_uav_mission_node";

	ros::init(argc, argv, node_name);
    MissionInterface mission(node_name);
    mission.executeMission();


    return 0;
}
