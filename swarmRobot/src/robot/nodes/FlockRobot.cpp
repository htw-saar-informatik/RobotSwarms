#include "../robot_system/RobotPilot.h"
#include <cstdlib>
#include <ctime>

#include "../flock/FlockControl.h"
#include "../../objects/Object.h"

void initiateWaitForRobots(Range<unsigned int> robotRange) {
    missingOkays.clear();
    for (unsigned int i : robotRange) {
        missingOkays.push_back(i);
    }
    Logger(DEBUG, 0 , "Setting for: %s", myString::toString(missingOkays).c_str());
}

void waitForRobots(RobotPilot &pilot, ros::Publisher &inPositionPublisher) {
    Logger(DEBUG, 0 , "Waiting for: %s", myString::toString(missingOkays).c_str());

    swarmRobot::RobotIndex msg;
    msg.index = static_cast<uint8_t>(pilot.getIndex());
    do {
        Logger(DEBUG, 0, "Sending my okay! %d", msg.index);
        pilot.hold();
        inPositionPublisher.publish(msg);
    } while (!missingOkays.empty() and ros::ok());

    if (missingOkays.empty()) Logger(DEBUG, 0, "Missing okays are empty");
    if (!ros::ok()) Logger(CRITICAL, 0, "Ros is not okay");
}

void transport(RobotPilot &pilot, swarmRobot::MissionNewConstPtr mission, ros::Publisher &inPositionPublisher, ros::Publisher &missionFinishedPublisher) {
    const Range<unsigned int> robotRange(mission->robot_index_from, mission->robot_index_to);
    const unsigned long missionId = mission->id;

    initiateWaitForRobots(robotRange);

    const bool isLeader = alg::inRange(pilot.getIndex(), robotRange.begin().get(), robotRange.begin().get() + mission->number_leaders);
    const bool isFirstLeader = pilot.getIndex() == robotRange.begin().get();

    if (isLeader) {
        Logger(INFO, "Im a leader");
    }

    if (isFirstLeader) {
        Logger(INFO, "Im the first leader");
    }

    Position destination{mission->target_x, mission->target_y};

    // Move to object
    Object &target = *std::find_if(safeZones.begin(), safeZones.end(), [=](const Object &o) {
        return o.getId() == missionId;
    });
    while (!target.inBoundary(pilot.position()) and ros::ok()) {
        Logger(DEBUG, "Moving to object")
                .newLine("Object: %s, Self: %s", target.toString().c_str(), pilot.self().position.toString().c_str());
        pilot.moveTo(target.getCentre());
    }

    waitForRobots(pilot, inPositionPublisher);
    initiateWaitForRobots(robotRange);

    if (isFirstLeader) {
        Logger(INFO, "All robots in position. Starting transport");
    }

    // Transport
    while (ros::ok() and (destination.distance(target.getCentre()) > 0.05f)) {
        float currentSpeed = ROBOT_SPEED * 0.25f;

        std::vector<Robot> missionRobots;
        for (unsigned int i : robotRange) {
            missionRobots.push_back(globalSwarm[i]);
        }

        target.move_absolute(flockCentre(missionRobots));

        if (!target.inBoundary(pilot.position())) {
            Logger(DEBUG, "Moving to object")
                    .newLine("Object: %s, Self: %s", target.toString().c_str(), pilot.position().toString().c_str());
            pilot.moveTo(target.getCentre(), false, currentSpeed);
        } else {
            if (!isLeader) {
                Angle turnAngle = pilot.nextStep_withNoise();
                while ((!pilot.makeMoveLegal(turnAngle, currentSpeed * 1.5f)) && ros::ok()) {
                    currentSpeed *= 0.9f;
                    usleep(1);
                    ros::spinOnce();
                }

                pilot.move(turnAngle, false, currentSpeed);
            } else {
                const Robot &self = pilot.self();
                Angle turnAngle = target.getCentre().angleTo(destination);

                int triesLeft = 10;
                Position nextDest = self.simulateMovement(turnAngle, currentSpeed);
                while ((triesLeft > 0) and (!pilot.predictLegalMove(nextDest))) {
                    currentSpeed *= 0.75f;
                    nextDest = self.simulateMovement(turnAngle, currentSpeed);
                    triesLeft--;
                }

                if (triesLeft > 0) {
                    pilot.move(turnAngle, false, currentSpeed);
                } else {
                    pilot.turn_teleport_absolute(target.getCentre().angleTo(destination));
                    pilot.hold();
                }
            }
        }
    }

    waitForRobots(pilot, inPositionPublisher);

    if (isFirstLeader) {
        Logger(INFO, "Removing mission. Robots are free");
        swarmRobot::MissionFinished msg;
        msg.missionId = mission.get()->id;
        msg.robotIndex = pilot.getIndex();
        missionFinishedPublisher.publish(msg);
    }
}

void loop(RobotPilot &pilot) {
    pilot.hold();
    ros::NodeHandle node;
    auto missionFinishedAdvertiser = node.advertise<const swarmRobot::MissionFinished>(TOPIC_ROBOT_MISSION_FINISHED, ROS_QUEUE_SIZE);
    auto inPositionPublisher = node.advertise<swarmRobot::RobotIndex>(TOPIC_MISSION_IN_POSITION, ROS_QUEUE_SIZE);
    
    while (ros::ok()) {
        Angle turnAngle = pilot.nextStep_withNoise();
        pilot.move(turnAngle);

        if (!missions.empty()) {
            const auto &mission = missions.back();
            transport(pilot, mission, inPositionPublisher, missionFinishedAdvertiser);
            Logger(INFO, "Mission finished");

            missions.pop_back();
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
    ros::Subscriber missionFinishedSubscriber = nodeHandle.subscribe(TOPIC_ROBOT_MISSION_FINISHED, ROS_QUEUE_SIZE, callback);
    // ===== Robot init =====
    ros::spinOnce();
    Position spawn{rndDistributor(rndGenerator), rndDistributor(rndGenerator)};
    RobotPilot pilot(index, spawn);
    subscribeToFlock();
    pilot.hold();
    pilot.waitForFlockReady();

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