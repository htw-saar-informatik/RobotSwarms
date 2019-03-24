//
// Created by smanier on 30.09.18.
//

#pragma once

#include "ros/ros.h"
#include <ros/subscriber.h>
#include <turtlesim/Pose.h>
#include "../../msgs/RobotIndex.h"
#include "../../msgs/MissionNew.h"
#include "../../msgs/MissionFinished.h"
#include "../../msgs/UpdateShape.h"

#include "../robot_system/Robot.h"
#include "../../various/Constants.h"
#include "../../logger/Logger.h"
#include "../../basics/Range.h"

#include <cmath>

std::array<Robot, FLOCK_SIZE> globalSwarm;
std::vector<unsigned int> mySwarmIndices;
std::vector<unsigned int> missingOkays;
std::vector<Object> safeZones = {BORDER};

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
        flockSubscriber.push_back(node.subscribe("turtle" + std::to_string(i) + "/pose", ROS_QUEUE_SIZE, callback));
    }
}

std::vector<swarmRobot::MissionNewConstPtr> missions;

void missionNewCallback(const swarmRobot::MissionNewConstPtr &mission, const unsigned int index) {
    const Range<unsigned int> robotRange(mission->robot_index_from, mission->robot_index_to);

    Logger(INFO, "Got a new mission: Robots %d -> %d", robotRange.begin().get(), robotRange.end().get())
            .newLine("Object: pos(%f, %f)", mission->object_position_x, mission->object_position_y)
            .newLine("Object: size(%f, %f)", mission->object_size_x, mission->object_size_y)
            .newLine("Target: (%f, %f)", mission->target_x, mission->target_y);


    bool inverted = false;
    if (robotRange.valueInRange(index)) {
        missions.push_back(mission);

        mySwarmIndices.clear();
        for (unsigned int i : robotRange) {
            mySwarmIndices.push_back(i);
        }
    } else {
        auto newEnd = std::remove_if(mySwarmIndices.begin(), mySwarmIndices.end(), [robotRange](unsigned int i) {
            return robotRange.valueInRange(i);
        });
        mySwarmIndices.erase(newEnd, mySwarmIndices.end());
        inverted = true;
    }


    safeZones.emplace_back(new Rectangle(
            Position{mission->object_position_x, mission->object_position_y},
            Size{mission->object_size_x, mission->object_size_y},
            inverted),mission->id);
}

void missionInPositionCallback(const swarmRobot::RobotIndexConstPtr &index) {
    Logger(DEBUG, 0, "Incoming okay: %d", index->index);
    Logger(DEBUG, 0, "1Left: %s", myString::toString(missingOkays).c_str());
    auto pos = std::find(missingOkays.begin(), missingOkays.end(), index->index);
    if (pos == missingOkays.end()) {
        return;
    }
    missingOkays.erase(pos);

    Logger(DEBUG, 0, "2Left: %s", myString::toString(missingOkays).c_str());
}

void missionFinishedCallback(const swarmRobot::MissionFinishedConstPtr &finishedMission, const unsigned int myIndex) {
    Logger(DEBUG, "%d reports finished mission", finishedMission->robotIndex);

    if (finishedMission->robotIndex == myIndex) {
        mySwarmIndices.clear();
        for (unsigned int i = 0; i < FLOCK_SIZE; i++) {
            mySwarmIndices.push_back(i);
        }
    } else {
        mySwarmIndices.push_back(finishedMission->robotIndex);
    }

    const unsigned long id = finishedMission->missionId;
    auto pos = std::remove_if(safeZones.begin(), safeZones.end(), [=](const Object &o) {
        return id == o.getId();
    });

    safeZones.erase(pos, safeZones.end());
}

void updateObject(const swarmRobot::UpdateShapeConstPtr &update) {
    for (auto &object : safeZones) {
        if (object.getId() == update->mission_index) {
            object.move_absolute(Position(update->pos_x, update->pos_y));
        }
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