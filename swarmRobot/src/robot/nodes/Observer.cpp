//
// Created by smanier on 30.09.18.
//

#include <turtlesim/Spawn.h>
#include "ros/ros.h"

#include <string>
#include <functional>
#include <chrono>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>

#include "../flock/FlockControl.h"
#include "../flock/Statistics.h"
#include "../../logger/Logger.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Observer");
    ros::start();

    subscribeToFlock();

    ofstream file;
    string filePostfix = "_localRange" + to_string(FLOCK_LOCALITY_RANGE)
                         + "_speed" + to_string(ROBOT_SPEED_PERCENT)
                         + "_angle" + to_string(FLOCK_RANDOM_TURN_DEGREE);
    const string fileName = "/home/smanier/catkin_ws/src/beginner_tutorials/Statistics" + filePostfix + ".txt";
    file.open(fileName);

    if (!file.good()) {
        throw "Cant open file!";
    }

    unsigned int intervall = 10;
    int count = intervall;
    while (ros::ok() and (count <= (60 * 60))) {
        sleep(intervall);
        ros::spinOnce();

        statistics(file);

        count += intervall;
    }
    file.close();

    Logger(INFO, "FINISHED!").newLine("File written:").newLine("-> "  + fileName);
    return 0;
}