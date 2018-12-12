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

std::string fileName;
std::ofstream file;

void initStatistic() {
    const std::string filePostfix = "_localRange" + std::to_string(FLOCK_LOCALITY_RANGE)
                                    + "_speed" + std::to_string(ROBOT_SPEED_PERCENT)
                                    + "_angle" + std::to_string(FLOCK_RANDOM_TURN_DEGREE);
    fileName = "/home/smanier/catkin_ws/src/beginner_tutorials/Statistics" + filePostfix + ".txt";
    file.open(fileName);

    if (!file.good()) {
        throw "Cant open file!";
    }
}

void writeOutStatistic() {
    statistics(file);
}

int main(int argc, char **argv) {
    initFlockObservation();
    // ===== INITS =====
    ros::init(argc, argv, "Observer");
    ros::start();
    LOGGER_CAPTION = "Observer";
    subscribeToFlock();

    ros::NodeHandle node;
    ros::Publisher missionPublisher = node.advertise<swarmRobot::MissionNew>(TOPIC_MISSION_NEW, 1000);
    swarmRobot::MissionNew mission;
    mission.robot_index_from = 0;
    mission.robot_index_to = 3;
    mission.object_position_x = 5;
    mission.object_position_y = 5;
    mission.object_size_x = 1;
    mission.object_size_y = 1;
    mission.target_x = 3;
    mission.target_y = 3;


    /*int in;
    Logger(ALWAYS, "Waiting...");
    std::cin >> in;*/
    ros::Rate sleeper(0.9);
    sleeper.sleep();
    missionPublisher.publish(mission);
    Logger(CRITICAL, "Mission published");

    Object object(new Rectangle(
            Position{mission.object_position_x, mission.object_position_y},
            Size{mission.object_size_x, mission.object_size_y}
    ));
    Position destination{mission.target_x, mission.target_y};


    // ===== START =====
    bool robotsInPosition = false;
    while (ros::ok() and (destination.distance(object.getCentre()) > 0.01f)) {
        std::vector<Robot> missionRobots;
        for (unsigned int i = mission.robot_index_from; i < (mission.robot_index_to + 1); i++) {
            missionRobots.push_back(globalSwarm[i]);
        }

        if (!robotsInPosition) { // Trigger for starting the transport
            bool inPosition = true;
            for (const Robot &missionRobot : missionRobots) {
                if (!object.inBoundary(missionRobot.position)) {
                    inPosition = false;
                }
            }

            robotsInPosition = inPosition;

            if (robotsInPosition) {
                Logger(INFO, "Robots in position. Starting transport");
            }
        }

        if (robotsInPosition) { // Moves only if robots are in position
            for (const Robot &missionRobot : missionRobots) {
                if (!object.inBoundary(missionRobot.position)) {
                    Logger(CRITICAL, "Robot leaving object: %d!", missionRobot.index);
                }
            }

            const Position &flockPos = flockCentre(missionRobots);
            object.move_absolute(flockPos);
            const Position &centre = object.getCentre();
            Logger(INFO, "Object: %s, Destination: %s, Distance: %f",
                   centre.toString().c_str(), destination.toString().c_str(), centre.distance(destination));
        }

        ros::spinOnce();
        sleeper.sleep();
    }

    file.close();

    Logger(INFO, "FINISHED!").newLine("File written:").newLine("-> " + fileName);
    return 0;
}