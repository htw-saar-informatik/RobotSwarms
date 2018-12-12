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
    explicit RobotCore(unsigned int index, bool disablePen = true) : myIndex(index) {
        long seed = std::chrono::system_clock::now().time_since_epoch().count();
        rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * myIndex));
        auto spawnDistributor = std::uniform_real_distribution<float>(BORDER.getSize().x, BORDER.getSize().y);
        const Angle &spawnAngle = Angle(rndDistributor(rndGenerator));

        this->topic = "turtle" + std::to_string(myIndex);

        initNodes();
        const Position &spawnPosition{spawnDistributor(rndGenerator), spawnDistributor(rndGenerator)};
        spawn(spawnPosition, spawnAngle, disablePen);
    }

    explicit RobotCore(unsigned int index, const Position &spawnPosition, const Angle &spawnAngle, bool disablePen = true)
            : myIndex(index) {
        long seed = std::chrono::system_clock::now().time_since_epoch().count();
        rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * myIndex));

        this->topic = "turtle" + std::to_string(myIndex);

        initNodes();
        spawn(spawnPosition, spawnAngle, disablePen);
    }

    void turn_teleport_absolute(const Angle &degree) {
        turtlesim::TeleportAbsolute t;
        t.request.x = this->position().x;
        t.request.y = this->position().y;
        t.request.theta = degree.toRad();
        teleportAbsoluteService.call(t);
    }

    void teleport_absolute(const Position &pos, const Angle &teleportAngle) {
        turtlesim::TeleportAbsolute t;
        t.request.x = pos.x;
        t.request.y = pos.y;
        t.request.theta = teleportAngle.toRad();

        ros::service::waitForService(topic + "/teleport_absolute");
        teleportAbsoluteService.call(t);
    }

    void turn_teleport_relative(const Angle &degree) {
        turtlesim::TeleportRelative t;
        t.request.angular = degree.toRad();
        teleportRelativeService.call(t);
    }

    Position position() const {
        return self().position;
    }

    Robot self() const {
        return globalSwarm[myIndex];
    }

    unsigned int getIndex() const {
        return myIndex;
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
            alg::assertTrue(node.serviceClient<turtlesim::Spawn>("spawn").call(s),
                            "Error spawning turtle: " + this->topic);
            if (disablePen) setOffPen();
        } else {
            if (disablePen) setOffPen();
            teleport_absolute(spawnPosition, 0);
        }

        Logger(ALWAYS, topic + " spawned");
    }

    void initNodes() {
        movePublisher = node.advertise<geometry_msgs::Twist>(topic + "/cmd_vel", 1);
        alg::assertNotNull(movePublisher, "MovePublisher");

        teleportAbsoluteService = node.serviceClient<turtlesim::TeleportAbsolute>(topic + "/teleport_absolute");
        alg::assertNotNull(teleportAbsoluteService, "TeleportAbsoluteService");

        teleportRelativeService = node.serviceClient<turtlesim::TeleportRelative>(topic + "/teleport_relative");
        alg::assertNotNull(teleportRelativeService, "TeleportRelativeService");

        const boost::function<void(const swarmRobot::MissionNewConstPtr &)> &callback = bind(missionNewCallback, _1,
                                                                                          myIndex);
        missionSubscriber = node.subscribe(TOPIC_MISSION_NEW, 1, callback);
        alg::assertNotNull(missionSubscriber, "MissionSubscriber");

        missionInPositionSubscriber = node.subscribe(TOPIC_MISSION_IN_POSITION, 1, missionInPositionCallback);
        alg::assertNotNull(missionInPositionSubscriber, "MissionInPositionSubscriber");
    }

    void setOffPen() {
        turtlesim::SetPen pen;
        pen.request.off = 1;
        ros::service::waitForService(topic + "/set_pen");
        alg::assertTrue(node.serviceClient<turtlesim::SetPen>(topic + "/set_pen").call(pen),
                "Setting off pen failed!");
    }

protected:
    ros::NodeHandle node;
    ros::ServiceClient teleportAbsoluteService;
    ros::ServiceClient teleportRelativeService;
    ros::Subscriber missionSubscriber;
    ros::Subscriber missionInPositionSubscriber;
    ros::Publisher movePublisher;
    ros::Rate sleeper = ros::Rate(0.9);

    std::string topic;
    unsigned int myIndex;

    std::uniform_real_distribution<float> rndDistributor{0, 360};
    std::default_random_engine rndGenerator;

    void refreshPosition() {
        movePublisher.publish(geometry_msgs::Twist());
        sleeper.sleep();
        ros::spinOnce();
    }
};