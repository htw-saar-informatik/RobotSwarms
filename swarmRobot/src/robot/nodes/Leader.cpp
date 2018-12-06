//
// Created by smanier on 07.10.18.
//

#include <ros/init.h>
#include <ros/ros.h>
#include <fstream>
#include "../../various/Constants.h"
#include "../robot_system/RobotPilot.h"
#include "../flock/Statistics.h"

struct SimResult {
    int waitingTimes;
    int flockSize;
    float distance;
};

std::string exec(const char* cmd) {
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

int main(int argc, char **argv) {
    const time_t startTime = time(nullptr);

    ros::init(argc, argv, "Leader");
    LOGGER_CAPTION = "Leader";

    string filePostfix = "_localRange" + to_string(FLOCK_LOCALITY_RANGE)
                         + "_speed" + to_string(ROBOT_SPEED_PERCENT)
                         + "_angle" + to_string(FLOCK_RANDOM_TURN_DEGREE);
    const string fileName = "/home/smanier/catkin_ws/src/beginner_tutorials/Statistics" + filePostfix + ".txt";
    ofstream file;
    file.open(fileName, ios::app);
    file << "===== ===== ===== ===== ===== =====\n";
    file << "FlockSize: " << FLOCK_SIZE << "\n";
    file << "ToFlockTurn: " << FLOCK_TURN_TO_FLOCK_PERCENT * 100.0f << "%\n";
    file << "FreeWill: " << FLOCK_RANDOM_TURN_DEGREE << "°\n";
    file << "LocalityRange: " << FLOCK_LOCALITY_RANGE << "\n";
    file << "Speed: " << ROBOT_SPEED_PERCENT << "%\n";

    subscribeToFlock();

    Position spawn = {2, 2};
    const Position destination = {10, 10};
    RobotPilot pilot(0, spawn, false);
    ros::Rate sleeper(1);

    Angle moveAngle = 45.0f;
    long seed = std::chrono::system_clock::now().time_since_epoch().count();
    auto rndGenerator = std::default_random_engine(static_cast<unsigned long>(seed));
    float range = FLOCK_LOCALITY_RANGE - 0.1f;
    auto rndDistributor = std::uniform_real_distribution<float>(-range, range);
    ros::NodeHandle node;
    vector<SimResult> results;

    pilot.waitForFlockReady();
    const int MAX_ROUNDS = 10;
    for (int i = 0; (i < MAX_ROUNDS) and ros::ok(); i++) {
        int waitingCount = 0;
        float distance = 0;
        int flockSize = 0;
        float roundCount = 0;
        long END_TIME = time(nullptr) + 60 * 20;

        while (ros::ok() and (pilot.position().distance(destination) > 0.15) and (time(nullptr) < END_TIME)) {
            auto myFlock = findFlock(Robot(pilot.position(), 0));
            auto flockPos = flockCentre(myFlock);
            Angle relativeAngle = pilot.self().getRelativeAngle(flockPos);
            flockSize = static_cast<int>(myFlock.size()) - 1;

            relativeAngle.normalizeToCircularAngle();
            if (pilot.position().distance(flockPos) > (FLOCK_LOCALITY_RANGE * 0.5) and
                (relativeAngle.isBetween(90.0, 270.0))) {
                Logger(INFO, "Waiting... ");
                pilot.hold();
                waitingCount++;
                continue;
            }

            distance++;
            if (flockSize == (FLOCK_SIZE - 1)) roundCount++;

            Logger(INFO, "Round: %02d/%2d, FlockSize: %d", i + 1, MAX_ROUNDS, flockSize);

            pilot.move(moveAngle);
        }

        if (time(nullptr) > END_TIME) {
            Logger(INFO, "Timeout");
            flockSize = 0;
        }

        if (pilot.position().distance(destination) < 0.15) {
            Logger(INFO, "Target reached");
        }

        results.push_back(SimResult{waitingCount, flockSize, (roundCount / distance) * 100.0f});
        Logger(INFO, "").bar();
        for (const SimResult &j : results) {
            Logger(INFO, "[%d] Waiting times: %d, FlockSize: %d, Successful: %f", i, j.waitingTimes, j.flockSize,
                   j.distance);
        }

        for (int j = 1; j < FLOCK_SIZE; j++) {
            auto teleportService = node.serviceClient<turtlesim::TeleportAbsolute>(
                    "turtle" + to_string(j) + "/teleport_absolute");
            turtlesim::TeleportAbsolute t;
            t.request.x = 2 + rndDistributor(rndGenerator);
            t.request.y = 2 + rndDistributor(rndGenerator);
            t.request.theta = moveAngle.toRad();
            teleportService.call(t);
        }

        auto teleportService = node.serviceClient<turtlesim::TeleportAbsolute>(
                "turtle/teleport_absolute");
        turtlesim::TeleportAbsolute t;
        t.request.x = 2 + rndDistributor(rndGenerator);
        t.request.y = 2 + rndDistributor(rndGenerator);
        t.request.theta = moveAngle.toRad();
        teleportService.call(t);

        pilot.teleport_absolute(spawn, 45.0f);

        sleeper.sleep();
        ros::spinOnce();
        sleeper.sleep();
        ros::spinOnce();
    }

    string out[3] = {"FlockSize; ", "Distance; ", "WaitingTimes; "};
    for (const SimResult &i : results) {
        out[0] += to_string(i.flockSize) + ";";
        out[1] += to_string(i.distance) + ";";
        out[2] += to_string(i.waitingTimes) + ";";
    }

    file << out[0] << "\n" << out[1] << "\n" << out[2] << "\n";
    file.close();

    Logger(INFO, "").bar();
    for (const SimResult &i : results) {
        static int count = 0;
        Logger(INFO, "[%d] Waiting times: %d, FlockSize: %d, Successful: %f", count++, i.waitingTimes, i.flockSize,
               i.distance);
    }

    Logger(INFO, "FINISHED!").newLine("File written:").newLine("-> " + fileName);
    Time elapsedTime(time(nullptr) - startTime);
    Logger(INFO, "%2dh, %2dmin, %2ds", elapsedTime.h, elapsedTime.min, elapsedTime.sec);

    exec("spd-say \"Simulation finished!\"");
}

int main2(int argc, char **argv) {
    ros::init(argc, argv, "Leader");

    string filePostfix = "_localRange" + to_string(FLOCK_LOCALITY_RANGE)
                         + "_speed" + to_string(ROBOT_SPEED_PERCENT)
                         + "_angle" + to_string(FLOCK_RANDOM_TURN_DEGREE);
    const string fileName = "/home/smanier/catkin_ws/src/beginner_tutorials/Statistics" + filePostfix + ".txt";
    ofstream file;
    file.open(fileName);

    subscribeToFlock();

    Position spawn = {2, 2};
    const Position destination = {10, 10};
    RobotPilot pilot(0, spawn, false);
    ros::Rate sleeper(1);

    Angle moveAngle = 45.0f;

    int waitingCount = 0;
    float roundCount = 0;
    float distance = 0;
    const long END_TIME = time(nullptr) + 60 * 20;

    sleep(3);
    while (ros::ok() and (pilot.position().distance(destination) > 0.15) and (time(nullptr) < END_TIME)) {
        auto myFlock = findFlock(Robot(pilot.position(), 0));
        auto flockPos = flockCentre(myFlock);
        Angle relativeAngle = pilot.self().getRelativeAngle(flockPos); // MIGHT BE BROKEN! 'angleTo()'
        const unsigned long flockSize = myFlock.size() - 1;

        relativeAngle.normalizeToCircularAngle();
        if ((pilot.position().distance(flockPos) > (FLOCK_LOCALITY_RANGE * 0.5)) and
            relativeAngle.isBetween(135, 225)) { // NOT WORKING!
            // Wait
            Logger(INFO, "Leader", "Waiting... ");
            //ros::spinOnce();
            //sleeper.sleep();
            pilot.hold();
            waitingCount++;
            continue;
        }

        distance++;
        if (flockSize > 0) roundCount++;

        file << flockSize << "\n";
        Logger(INFO, "FlockSize: %d", flockSize);

        pilot.move(moveAngle);

    }

    file.close();

    if (pilot.position().distance(destination) < 0.15) {
        Logger(INFO, "Leader", "Target reached");
    }

    if (time(nullptr) > END_TIME) {
        Logger(INFO, "Leader", "Timeout");
    }

    Logger(INFO, "FINISHED!").newLine("File written:").newLine("-> " + fileName);
    Logger(INFO, "Waited %d times!", waitingCount);
    Logger(INFO, "Successful for %f/%", (roundCount / distance) * 100.0f);

    return 0;
}