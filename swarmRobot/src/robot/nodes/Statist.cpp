#include <cstdlib>
#include <ctime>

#include "../flock/FlockControl.h"
#include "../robot_system/RobotPilot.h"

int main(int argc, char **argv) {
    const int index = std::stoi(argv[1]);

    ros::init(argc, argv, "statist");
    Position spawn{5.5f, 5.5f};

    RobotPilot pilot(index, spawn);
    subscribeToFlock();
    ros::spinOnce();

    ros::Rate sleeper(1);
    while (ros::ok()) {
        pilot.hold();
        ros::spinOnce();
        sleeper.sleep();
    }
}