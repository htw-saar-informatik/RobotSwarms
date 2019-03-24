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
#include "../../various/Statistical.h"
#include "../../various/Constants.h"
#include "../../basics/ToString.h"

bool missionFinished = false;
void sendMission(const ros::Publisher &missionPublisher, const swarmRobot::MissionNew &mission) {
    missionFinished = false;

    ros::Rate sleeper(0.9);
    sleeper.sleep();
    missionPublisher.publish(mission);
    Logger(ALWAYS, "Mission published");

    while (!missionFinished) {
        sleeper.sleep();
        ros::spinOnce();
    }
}


void missionFinishedCallback(const swarmRobot::MissionFinishedConstPtr &index) {
    if (index->robotIndex == 0) {
        missionFinished = true;
    }
}

int main(int argc, char **argv) {
    try {
        initFlockObservation();
        // ===== INITS =====
        ros::init(argc, argv, "Observer");
        ros::start();
        LOGGER_CAPTION = "Observer";
        subscribeToFlock();

        ros::NodeHandle node;
        ros::Publisher missionPublisher = node.advertise<swarmRobot::MissionNew>(TOPIC_MISSION_NEW, ROS_QUEUE_SIZE);
        ros::Subscriber missionFinishedSubscriber = node.subscribe(TOPIC_ROBOT_MISSION_FINISHED, ROS_QUEUE_SIZE,
                                                                   missionFinishedCallback);

        swarmRobot::MissionNew mission;
        mission.number_leaders = 4;
        mission.id = 10;//static_cast<unsigned long>(time(nullptr));
        mission.robot_index_from = 0;
        mission.robot_index_to = FLOCK_SIZE - 1;
        mission.object_position_x = 7;
        mission.object_position_y = 7;
        mission.object_size_x = 2;
        mission.object_size_y = 2;
        mission.target_x = 3;
        mission.target_y = 3;

        sendMission(missionPublisher, mission);

        Logger(ALWAYS, "Sending second mission in ... ");
        const int time = 10;
        for (int i = 0; i < time; i++) {
            Logger(ALWAYS, "%d ...", time - i);
            sleep(1);
        }

        mission.object_position_x = 3;
        mission.object_position_y = 7;
        mission.target_x = 7;
        mission.target_y = 3;

        sendMission(missionPublisher, mission);

    } catch (const char *message) {
        Logger(CRITICAL, "Catched error message: %s", message);
    } catch (const std::string &message) {
        Logger(CRITICAL, "Catched error message: %s", message.c_str());
    } catch (const int errorCode) {
        Logger(CRITICAL, "Catched errorCode: %d", errorCode);
    } catch (const std::exception &e) {
        Logger(CRITICAL, "Catched exception: %s", e.what());
    } catch (...) {
        Logger(CRITICAL, "Catched unknown object-exception");
    }

    Logger(ALWAYS, "Observer shutting down");
    return 0;
}