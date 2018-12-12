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

    bool startTransport = false;
    while (ros::ok() and (destination.distance(object.getCentre()) > 0.01f)) {
        float currentSpeed = ROBOT_SPEED * 0.25f;

        std::vector<Robot> missionRobots;
        //for (unsigned int i : mySwarmIndices) {
        for (unsigned int i = mission->robot_index_from; i < (mission->robot_index_to + 1); i++) {
            missionRobots.push_back(globalSwarm[i]);
        }

        if (object.inBoundary(pilot.self().position)) { // Trigger for starting the transport
            static bool didReport = false;
            if (!didReport) {
                swarmRobot::MissionFinished msg;
                msg.index = (uint8_t) pilot.getIndex();
                missionInPositionAdvertiser.publish(msg);
                Logger(INFO, "Im in position");
                didReport = true;
            }
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
            pilot.moveTo(object.getCentre());
        } else {
            if (!startTransport) {
                pilot.hold();
            } else {
                Angle turnAngle = pilot.nextStep_withNoise();

                while ((!pilot.makeMoveLegal(turnAngle, currentSpeed * 1.5f, object)) && ros::ok()) {
                    currentSpeed *= 0.9f;
                    usleep(1);
                    ros::spinOnce();
                }

                pilot.move(turnAngle, false, currentSpeed);
            }
        }
    }
}

void loop(RobotPilot &pilot) {
    pilot.hold();
    ros::NodeHandle node;
    swarmRobot::MissionFinished msg;
    msg.index = static_cast<uint8_t >(pilot.getIndex());
    auto missionFinishedAdvertiser = node.advertise<swarmRobot::MissionFinished>(TOPIC_ROBOT_MISSION_FINISHED, 10);

    while (ros::ok()) {
        Angle turnAngle = pilot.nextStep_withNoise();
        pilot.move(turnAngle);

        if (!missions.empty()) {
            transport(pilot, missions.back(), node);
            Logger(INFO, "Mission finished");
            missions.pop_back();
            missionFinishedAdvertiser.publish(msg);
        }
    }
}

int main(int argc, char **argv) {
    initFlockObservation();
    // ===== Main init =====
    const unsigned int index = (unsigned int) std::stoul(argv[1], nullptr, 10);
    const string &robotName = RobotName(index);
    ros::init(argc, argv, robotName);
    LOGGER_CAPTION = robotName;

    const long seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed * index));
    auto rndDistributor = std::uniform_real_distribution<float>(TURTLESIM_BORDER_THICKNESS,
                                                                TURTLESIM_SIZE - TURTLESIM_BORDER_THICKNESS);

    ros::NodeHandle nodeHandle;
    const boost::function<void(const swarmRobot::MissionFinishedConstPtr &)> &callback = bind(missionFinishedCallback,
                                                                                              _1, index);
    ros::Subscriber missionFinishedSubscriber = nodeHandle.subscribe(TOPIC_ROBOT_MISSION_FINISHED, 10, callback);
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