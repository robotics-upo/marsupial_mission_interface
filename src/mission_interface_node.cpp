#include <mission_interface/mission_interface.h>

int main(int argc, char **argv)
{
    std::string node_name = "mission_interface_node";

	ros::init(argc, argv, node_name);

    std::cout << "Initializing node: " << node_name << std::endl;
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    MissionInterface mission(node_name);

	ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        mission.executeMission();

        loop_rate.sleep();
    }
    return 0;
}
