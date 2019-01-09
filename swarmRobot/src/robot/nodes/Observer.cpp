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

std::vector<int> indexes;
std::vector<std::vector<Robot>> positionHistory;
bool missionFinished = false;

std::vector<Robot> getRobots(const swarmRobot::MissionNew &mission);

void waitForRobotsInPosition(const swarmRobot::MissionNew &mission);

void robotsInPositionCallback(const swarmRobot::MissionFinishedConstPtr &index) {
    auto pos = std::remove(indexes.begin(), indexes.end(), index->index);
    indexes.erase(pos, indexes.end());

    std::string left;
    for (int i : indexes) left += std::to_string(i) + ", ";
    Logger(INFO, "%d reports in position, left: %s", index->index, left.c_str());
}

void missionFinishedCallback(const swarmRobot::MissionFinishedConstPtr &index) {
    if (index->index == 0) {
        missionFinished = true;
    }
}

std::string getFilePath() {
    const std::string filePostfix = "_localRange" + myString::toString(FLOCK_LOCALITY_RANGE, 2)
                                    + "_speed" + myString::toString(ROBOT_SPEED_PERCENT, 2)
                                    + "_angle" + myString::toString(FLOCK_RANDOM_TURN_DEGREE, 2)
                                    + "_toFlock" + myString::toString(FLOCK_TURN_TO_FLOCK_PERCENT, 2);
    return "/home/smanier/catkin_ws/src/swarmRobot/Statistics" + filePostfix + ".txt";
}

std::string exec(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
            result += buffer.data();
    }
    return result;
}

void testFailure() {
    if (positionHistory.size() > 4) {
        for (unsigned int i = 0; i < positionHistory.size(); i++) {
            int count = 0;
            auto currentPos = positionHistory[i].end();
            for (int j = 0; j < 4; j++) {
                currentPos--;
                if (currentPos->operator==(positionHistory[i].back())) {
                    count++;
                }
            }

            if (count == 4) {
                std::string out = std::string("Robot didnt move: ") + positionHistory[i].back().toString();
                exec(("say" + out).c_str());
                throw out;
            }
        }
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
        ros::Publisher missionPublisher = node.advertise<swarmRobot::MissionNew>(TOPIC_MISSION_NEW, 100);
        ros::Subscriber missionFinishedSubscriber = node.subscribe(TOPIC_ROBOT_MISSION_FINISHED, 100, missionFinishedCallback);

        swarmRobot::MissionNew mission;
        mission.robot_index_from = 0;
        mission.robot_index_to = FLOCK_SIZE - 1;
        mission.object_position_x = 7;
        mission.object_position_y = 7;
        mission.object_size_x = 2;
        mission.object_size_y = 2;
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

        sleep(10);

        // ===== START =====
        waitForRobotsInPosition(mission);
        sleeper.sleep();

        Statistical<FLOCK_SIZE-1> statistics(getFilePath());

        std::vector<Robot> missionRobots_lastPosition;
        bool firstRound = true;
        while (ros::ok() and !missionFinished /*and (object.getCentre().distance(destination) > 0.05)*/) {
            const std::vector<Robot> &missionRobots = getRobots(mission);

            for (const Robot &missionRobot : missionRobots) {
                if (!object.inBoundary(missionRobot.position)) {
                    Logger(CRITICAL, "Robot leaving object: %d!", missionRobot.index);
                }
            }

            const Position &flockPos = flockCentre(missionRobots);
            object.move_absolute(flockPos);
            const Position &centre = object.getCentre();
            Logger(INFO, "Obj: %s, Dest: %s, Dist: %f",
                   centre.toString().c_str(), destination.toString().c_str(), centre.distance(destination));

            // +++++ Statistics +++++

            if (firstRound) {
                firstRound = false;
                statistics.setFirstRound(missionRobots);
            } else {
                statistics.record(missionRobots);
            }

            positionHistory.push_back(missionRobots);
            if (positionHistory.size() > 4) {
                positionHistory.erase(positionHistory.begin());
            }
            testFailure();
            // ----- Statistics -----

            ros::spinOnce();
            sleeper.sleep();
        }

        if (missionFinished) {
            statistics.print();
            //exec("say \"Simulation finished\"\0");
        }
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

    return 0;
}

void waitForRobotsInPosition(const swarmRobot::MissionNew &mission) {
    ros::Rate sleeper(0.9);
    indexes.clear();
    for (int i = mission.robot_index_from; i < mission.robot_index_to; i++) {
        indexes.push_back(i);
    }

    ros::NodeHandle node;
    auto missionInPositionSubscriber = node.subscribe(TOPIC_MISSION_IN_POSITION, 100, robotsInPositionCallback);

    while (indexes.size() > 0 and ros::ok()) {
        sleeper.sleep();
        ros::spinOnce();
    }

    Logger(INFO, "Robots in position. Starting transport");
}

std::vector<Robot> getRobots(const swarmRobot::MissionNew &mission) {
    std::vector<Robot> missionRobots;
    for (unsigned int i = mission.robot_index_from; i < mission.robot_index_to; i++) {
        missionRobots.push_back(globalSwarm[i]);
    }
    return missionRobots;
}