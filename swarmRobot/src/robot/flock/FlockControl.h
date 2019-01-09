//
// Created by smanier on 30.09.18.
//

#pragma once

#include "ros/ros.h"
#include <ros/subscriber.h>
#include <turtlesim/Pose.h>
#include "../../msgs/MissionFinished.h"
#include "../../msgs/MissionNew.h"

#include "../robot_system/Robot.h"
#include "../../various/Constants.h"
#include "../../logger/Logger.h"

#include <cmath>

std::array<Robot, FLOCK_SIZE> globalSwarm;
std::vector<unsigned int> mySwarmIndices;
std::vector<unsigned int> robotsNotInPosition;

void flockCallback(const turtlesim::PoseConstPtr &msg, const unsigned int index) {
    Robot &robot = globalSwarm[index];
    robot.position.x = msg->x;
    robot.position.y = msg->y;
    robot.angle = radToDegree(msg->theta);

    if (std::isnan(msg->x) or std::isnan(msg->y) or std::isnan(msg->theta)) {
        Logger(CRITICAL, "%d is NAN!", index);
        exit(-1);
    }
}

std::vector<ros::Subscriber> flockSubscriber;
void subscribeToFlock() {
    ros::NodeHandle node;

    for (int i = 0; i < FLOCK_SIZE; i++) {
        boost::function<void(const turtlesim::PoseConstPtr &)> callback = bind(flockCallback, _1, i);
        flockSubscriber.push_back(node.subscribe("turtle" + std::to_string(i) + "/pose", 100, callback));
    }
}

std::vector<swarmRobot::MissionNewConstPtr> missions;
void missionNewCallback(const swarmRobot::MissionNewConstPtr &mission, const unsigned int index) {
    const std::pair<unsigned int, unsigned int> range(mission->robot_index_from, mission->robot_index_to);

    Logger(INFO, "Got a new mission: Robots %d - %d", range.first, range.second)
            .newLine("Object: pos(%f, %f)", mission->object_position_x, mission->object_position_y)
            .newLine("Object: size(%f, %f)", mission->object_size_x, mission->object_size_y)
            .newLine("Target: (%f, %f)", mission->target_x, mission->target_y);


    if (alg::inRange<unsigned int>(index, range.first, range.second)) {
        missions.push_back(mission);

        mySwarmIndices.clear();
        for (unsigned int i = range.first; i < range.second; i++) {
            mySwarmIndices.push_back(i);
            robotsNotInPosition.push_back(i);
        }
    } else {
        auto newEnd = std::remove_if(mySwarmIndices.begin(), mySwarmIndices.end(), [range](unsigned int i) {
            return alg::inRange<unsigned int>(i, range.first, range.second);
        });
        mySwarmIndices.erase(newEnd, mySwarmIndices.end());
    }
}

void missionInPositionCallback(const swarmRobot::MissionFinishedConstPtr &index) {
    auto pos = std::find(robotsNotInPosition.begin(), robotsNotInPosition.end(), index->index);
    if (pos == robotsNotInPosition.end()) {
        return;
    }
    robotsNotInPosition.erase(pos);

    std::string left;
    for (unsigned int i : robotsNotInPosition) left += std::to_string(i) + ", ";
    Logger(INFO, "%d reports in position, left: %s", index->index, left.c_str());
}

void missionFinishedCallback(const swarmRobot::MissionFinishedConstPtr &finishedMission, const unsigned int myIndex) {
    Logger(INFO, "%d reports finished mission", finishedMission->index);

    if (finishedMission->index == myIndex) {
        mySwarmIndices.clear();
        for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
            mySwarmIndices.push_back(i);
        }
    } else {
        mySwarmIndices.push_back(finishedMission->index);
    }
}

Angle averageDegree(const std::vector<Robot> &robots) {
    std::vector<Angle> angles;

    for (const auto &i : robots) {
        angles.push_back(i.angle);
    }

    return meanAngle(angles);
}

bool flockIsReady() {
    for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
        const Position &current = globalSwarm[i].position;

        if ((current.x < 0.01f) and (current.y < 0.01f)) {
            Logger(DEEP_DEBUG, "Robot %d not ready yet", i);
            return false;
        }
    }

    return true;
}

void initFlockObservation() {
    for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
        globalSwarm[i] = Robot(i);
        mySwarmIndices.push_back(i);
    }
}