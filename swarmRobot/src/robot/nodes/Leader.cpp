#include "../robot_system/RobotPilot.h"
#include <cstdlib>
#include <ctime>

#include "../flock/FlockControl.h"
#include "../../objects/Object.h"

void transport(RobotPilot &pilot, swarmRobot::MissionNewConstPtr mission, ros::NodeHandle &node) {
    auto missionInPositionAdvertiser = node.advertise<swarmRobot::MissionFinished>(TOPIC_MISSION_IN_POSITION, 10);

    Object object(new Rectangle(
            Position{mission->object_position_x, mission->object_position_y},
            Size{mission->object_size_x, mission->object_size_y}
    ));
    Position destination{mission->target_x, mission->target_y};

    // Move to object
    while (!object.inBoundary(pilot.position()) and ros::ok()) {
        Logger(WARNING, "Moving to object")
                .newLine("Object: %s, Self: %s", object.toString().c_str(), pilot.self().position.toString().c_str());
        pilot.moveTo(object.getCentre());
    }

    // Wait for other robots
    while (!robotsNotInPosition.empty() and ros::ok()) {
        swarmRobot::MissionFinished msg;
        msg.index = static_cast<uint8_t>(pilot.getIndex());
        missionInPositionAdvertiser.publish(msg);

        pilot.hold();
    }

    Logger(INFO, "All robots in position. Starting transport");

    // Transport
    while (ros::ok() and (destination.distance(object.getCentre()) > 0.05f)) {
        float currentSpeed = ROBOT_SPEED * 0.25f;

        std::vector<Robot> missionRobots;
        for (unsigned int i = mission->robot_index_from; i < mission->robot_index_to; i++) {
            missionRobots.push_back(globalSwarm[i]);
        }

        object.move_absolute(flockCentre(missionRobots));

        if (!object.inBoundary(pilot.position())) {
            Logger(WARNING, "Moving to object")
                    .newLine("Object: %s, Self: %s", object.toString().c_str(), pilot.self().position.toString().c_str());
            pilot.moveTo(object.getCentre(), false, currentSpeed);
        } else {
            //Logger(WARNING, "Object: %s, Self: %s", object.toString().c_str(), pilot.position().toString().c_str());
            const Robot &self = pilot.self();
            Angle turnAngle = object.getCentre().angleTo(destination);

            int triesLeft = 10;
            Position nextDest = self.simulateMovement(turnAngle, currentSpeed);
            while ((triesLeft > 0) and (!pilot.predictLegalMove(nextDest, object))) {
                currentSpeed *= 0.75f;
                nextDest = self.simulateMovement(turnAngle, currentSpeed);
                triesLeft--;
            }

            if (triesLeft > 0) {
                pilot.move(turnAngle, false, currentSpeed);
            } else {
                pilot.turn_teleport_absolute(object.getCentre().angleTo(destination));
                pilot.hold();
            }
        }

        Logger(WARNING, 4, object.getCentre().toString());
    }
}

void transport2(RobotPilot &pilot, swarmRobot::MissionNewConstPtr mission, ros::NodeHandle &node) {
    auto missionInPositionAdvertiser = node.advertise<swarmRobot::MissionFinished>(TOPIC_MISSION_IN_POSITION, 100);

    Object object(new Rectangle(
            Position{mission->object_position_x, mission->object_position_y},
            Size{mission->object_size_x, mission->object_size_y}
    ));
    Position destination{mission->target_x, mission->target_y};

    bool startTransport = false;
    while (ros::ok() and (destination.distance(object.getCentre()) > 0.05f)) {
        float currentSpeed = ROBOT_SPEED * 0.25f;

        std::vector<Robot> missionRobots;
        for (unsigned int i : mySwarmIndices) {
            missionRobots.push_back(globalSwarm[i]);
        }

        if (object.inBoundary(pilot.position())) { // Trigger for starting the transport
            swarmRobot::MissionFinished msg;
            msg.index = (uint8_t) pilot.getIndex();
            missionInPositionAdvertiser.publish(msg);
        }

        if (robotsNotInPosition.empty()) { // Moves only if robots are in position
            const Position &flockPos = flockCentre(missionRobots);
            object.move_absolute(flockPos);

            if (!startTransport) {
                Logger(INFO, "All robots in position. Starting transport");
                startTransport = true;
            }
        }

        if (!object.inBoundary(pilot.position())) {
            Logger(WARNING, "Moving to object")
                    .newLine("Object: %s, Self: %s", object.toString().c_str(), pilot.self().position.toString().c_str());
            pilot.moveTo(object.getCentre());
        } else {
            if (!startTransport) {
                pilot.hold();
            } else {
                const Robot &self = pilot.self();
                Angle turnAngle = object.getCentre().angleTo(destination);

                int triesLeft = 10;
                Position nextDest = self.simulateMovement(turnAngle, currentSpeed);
                while ((triesLeft > 0) and (!pilot.predictLegalMove(nextDest, object))) {
                    currentSpeed *= 0.75f;
                    nextDest = self.simulateMovement(turnAngle, currentSpeed);
                    triesLeft--;
                }

                if (triesLeft > 0) {
                    pilot.move(turnAngle, false, currentSpeed);
                } else {
                    pilot.hold();
                }
            }
        }
    }
}

void loop(RobotPilot &pilot) {
    pilot.hold();
    ros::NodeHandle node;
    swarmRobot::MissionFinished msg;
    msg.index = static_cast<uint8_t >(pilot.getIndex());
    auto missionFinishedAdvertiser = node.advertise<swarmRobot::MissionFinished>(TOPIC_ROBOT_MISSION_FINISHED, 100);

    while (ros::ok()) {
        Angle turnAngle = pilot.nextStep_withNoise();
        pilot.move(turnAngle);

        if (!missions.empty()) {
            transport(pilot, missions.back(), node);
            Logger(INFO, "Mission finished");
            missions.pop_back();
            missionFinishedAdvertiser.publish(msg);

            while (ros::ok()) {
                pilot.hold();
            }
        }
    }
}

int main(int argc, char **argv) {
    initFlockObservation();
    // ===== Main init =====
    const unsigned int index = (unsigned int) std::stoul(argv[1], nullptr);
    const string &robotName = RobotName(index);
    ros::init(argc, argv, "Leader");
    LOGGER_CAPTION = robotName;

    const long seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * index));
    auto rndDistributor = std::uniform_real_distribution<float>(TURTLESIM_BORDER_THICKNESS,
                                                                TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS);

    ros::NodeHandle nodeHandle;
    const boost::function<void(const swarmRobot::MissionFinishedConstPtr &)> &callback = bind(missionFinishedCallback,
                                                                                              _1, index);
    ros::Subscriber missionFinishedSubscriber = nodeHandle.subscribe(TOPIC_ROBOT_MISSION_FINISHED, 100, callback);
    // ===== Robot init =====
    ros::spinOnce();
    Position spawn{rndDistributor(rndGenerator), rndDistributor(rndGenerator)};
    RobotPilot pilot(index, spawn);
    subscribeToFlock();
    pilot.hold();
    pilot.waitForFlockReady();

    BORDER.print();

    try {
        loop(pilot);
    } catch (const char *message) {
        Logger(CRITICAL, "Catched error message: %s", message);
    } catch (const string &message) {
        Logger(CRITICAL, "Catched error message: %s", message.c_str());
    } catch (const int errorCode) {
        Logger(CRITICAL, "Catched errorCode: %d", errorCode);
    } catch (const exception &e) {
        Logger(CRITICAL, "Catched exception: %s", e.what());
    } catch (...) {
        Logger(CRITICAL, "Catched unknown object-exception");
    }

    Logger(CRITICAL, "Robot %d shutting down", index);
}