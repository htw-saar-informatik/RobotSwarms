//
// Created by smanier on 30.09.18.
//

#ifndef PROJECT_FLOCKCONTROLL_H
#define PROJECT_FLOCKCONTROLL_H

#include "ros/ros.h"
#include <ros/subscriber.h>
#include <turtlesim/Pose.h>
#include "../../msgs/Mission.h"

#include "../robot_system/Robot.h"
#include "../../various/Constants.h"
#include "../../logger/Logger.h"

#include <cmath>

std::array<Robot, FLOCK_SIZE> initFlockObservation() {
    std::array<Robot, FLOCK_SIZE> out;
    for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
        out[i] = Robot(i);
    }
    return out;
}

std::array<Robot, FLOCK_SIZE> swarm = initFlockObservation();
void flockCallback(const turtlesim::PoseConstPtr &msg, const unsigned int topic) {
    Robot &robot = swarm[topic];
    robot.position.x = msg->x;
    robot.position.y = msg->y;
    robot.angle = radToDegree(msg->theta);

    if (std::isnan(msg->x) || std::isnan(msg->y) || std::isnan(msg->theta)) {
        Logger(CRITICAL, "%d is NAN!", topic);
        exit(-1);
    }
}

std::vector<ros::Subscriber> flockSubscriber;
void subscribeToFlock() {
    ros::NodeHandle node;

    for (int i = 0; i < FLOCK_SIZE; i++) {
        boost::function<void(const turtlesim::PoseConstPtr &)> callback = bind(flockCallback, _1, i);
        flockSubscriber.push_back(node.subscribe("turtle" + std::to_string(i) + "/pose", 1, callback));
    }
}

void missionCallback(const beginner_tutorials::Mission &mission) {
    // TODO DO SOMETHING HERE!
    Logger(CRITICAL, "Got a new mission: Robots %d - %d", mission.index_from, mission.index_to);
}

Angle averageDegree(const std::vector<Robot> &robots) {
    std::vector<Angle> angles;

    for (const auto &i : robots) {
        angles.push_back(i.angle);
    }

    return meanAngle(angles);
}

bool flockIsReady() {
    for (int i = 0; i < FLOCK_SIZE; i++) {
        const Position &current = swarm[i].position;

        if ((current.x == 0) and (current.y == 0)) {
            Logger(INFO, "Robot %d not ready yet", i);
            return false;
        }
    }

    return true;
}

#endif //PROJECT_FLOCKCONTROLL_H
