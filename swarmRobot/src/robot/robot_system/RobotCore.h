//
// Created by smanier on 26.11.18.
//

#pragma once


#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/TeleportRelative.h>
#include <turtlesim/SetPen.h>

#include <chrono>
#include <string>
#include <functional>

#include "../../various/Constants.h"
#include "Robot.h"
#include "../flock/FlockControl.h"

class RobotCore {
public:
    explicit RobotCore(int index, bool disablePen = true) : myIndex(index) {
        long seed = std::chrono::system_clock::now().time_since_epoch().count();
        rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * myIndex));
        rndDistributor = std::uniform_real_distribution<float>(-FLOCK_RANDOM_TURN_DEGREE / 2.0,
                                                               FLOCK_RANDOM_TURN_DEGREE / 2.0);

        auto spawnDistributor = std::uniform_real_distribution<float>(BORDER.getSize().x, BORDER.getSize().y);

        const Angle &spawnAngle = Angle(std::uniform_real_distribution<float>(0, 360)(rndGenerator));

        this->topic = "turtle" + std::to_string(myIndex);

        initNodes();
        const Position &spawnPosition{spawnDistributor(rndGenerator), spawnDistributor(rndGenerator)};
        spawn(spawnPosition, spawnAngle, disablePen);
    }

    explicit RobotCore(int index, const Position &spawnPosition, const Angle &spawnAngle, bool disablePen = true)
            : myIndex(index) {
        long seed = std::chrono::system_clock::now().time_since_epoch().count();
        rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * myIndex));
        rndDistributor = std::uniform_real_distribution<float>(-FLOCK_RANDOM_TURN_DEGREE / 2.0,
                                                               FLOCK_RANDOM_TURN_DEGREE / 2.0);

        this->topic = "turtle" + std::to_string(myIndex);

        initNodes();
        spawn(spawnPosition, spawnAngle, disablePen);
    }

    void turn_teleport_absolute(const Angle &degree) {
        turtlesim::TeleportAbsolute t;
        t.request.x = this->position().x;
        t.request.y = this->position().y;
        t.request.theta = degree.toRad();
        serviceClient_teleportAbsolute.call(t);
    }

    void teleport_absolute(const Position &pos, const Angle &teleportAngle) {
        turtlesim::TeleportAbsolute t;
        t.request.x = pos.x;
        t.request.y = pos.y;
        t.request.theta = teleportAngle.toRad();

        ros::service::waitForService(topic + "/teleport_absolute");
        serviceClient_teleportAbsolute.call(t);
    }

    void turn_teleport_relative(const Angle &degree) {
        turtlesim::TeleportRelative t;
        t.request.angular = degree.toRad();
        serviceClient_teleportRelative.call(t);
    }

    Position position() const {
        return self().position;
    }

    Robot self() const {
        return swarm[static_cast<unsigned long>(myIndex)];
    }

private:
    void spawn(const Position &spawnPosition, const Angle &spawnAngle, bool disablePen) {
        if (myIndex != 1) {
            turtlesim::Spawn s;
            s.request.x = spawnPosition.x;
            s.request.y = spawnPosition.y;
            s.request.theta = spawnAngle.toRad();
            s.request.name = this->topic;

            ros::service::waitForService("spawn");
            if (!node.serviceClient<turtlesim::Spawn>("spawn").call(s)) {
                Logger(ERROR, "Error spawning turtle: " + this->topic);
            }

            if (disablePen) setOffPen();
        } else {
            if (disablePen) setOffPen();
            teleport_absolute(spawnPosition, 0);
        }

        Logger(INFO, topic + " spawned");
    }

    void initNodes() {
        this->publisher = node.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1);
        this->serviceClient_teleportAbsolute = node.serviceClient<turtlesim::TeleportAbsolute>(
                topic + "/teleport_absolute");
        this->serviceClient_teleportRelative = node.serviceClient<turtlesim::TeleportRelative>(
                topic + "/teleport_relative");
        this->missionSubscriber = node.subscribe("flock", 1, missionCallback);
    }

    void setOffPen() {
        turtlesim::SetPen pen;
        pen.request.off = 1;
        ros::service::waitForService(topic + "/set_pen");

        if (!node.serviceClient<turtlesim::SetPen>(topic + "/set_pen").call(pen)) {
            Logger(ERROR, "Setting off pen failed!");
        }
    }

protected:
    ros::NodeHandle node;
    ros::ServiceClient serviceClient_teleportAbsolute;
    ros::ServiceClient serviceClient_teleportRelative;
    ros::Subscriber missionSubscriber;
    ros::Publisher publisher;
    ros::Rate sleeper = ros::Rate(0.9);

    std::string topic;
    int myIndex;

    std::default_random_engine rndGenerator;
    std::uniform_real_distribution<float> rndDistributor;

    void refreshPosition() {
        publisher.publish(geometry_msgs::Twist());
        ros::Rate(1).sleep();
        ros::spinOnce();
    }
};