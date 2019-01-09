//
// Created by smanier on 03.01.19.
//

#pragma once

#include <fstream>
#include <algorithm>
#include "../robot/robot_system/Robot.h"
#include "../robot/flock/Statistics.h"

template<int numberOfRobots>
class Statistical {
private:
    std::vector<Robot> lastPositions;
    std::ofstream file;
    const std::string fileName;

    int observations = 0;
    int leaderDidntMove = 0;
    double lengthOfDrivenWay = 0;
    double lengthOfSlack = 0;
    time_t startTime = 0;

    void copyThem(const std::vector<Robot> &positions) {
        lastPositions.clear();
        std::copy(positions.begin(), positions.end(), std::back_inserter(lastPositions));
    }

public:
    Statistical(const std::string &filePath)
            : fileName(filePath) {
        file.open(filePath, std::ios::app);

        if (!file.good()) {
            throw "Cant open file!";
        }
    }

    void record(const std::vector<Robot> &positions) {
        observations++;

        for (unsigned int i = 0; i < positions.size(); i++) {
            lengthOfDrivenWay += positions[i].distance(lastPositions[i]);
        }

        if (positions[0].distance(lastPositions[0]) < 0.001) {
            leaderDidntMove++;
        }

        const Position &lastCentre = flockCentre(lastPositions);
        const Position &currentCentre = flockCentre(positions);
        for (unsigned int i = 0; i < numberOfRobots; i++) {
            const Position &lastRelativePosition = lastCentre - lastPositions[i].position;
            const Position &currentRelativePosition = currentCentre - positions[i].position;
            lengthOfSlack += lastRelativePosition.distance(currentRelativePosition);
        }

        copyThem(positions);
    }

    void setFirstRound(const std::vector<Robot> &firstPositions) {
        copyThem(firstPositions);
        startTime = time(0);
    }

    void print() {
        file << "NumberOfRobots;"
             << "Observations;"
             << "Time(s);"
             << "LeaderDidntMove;"
             << "LengthOfDrivenWay;"
             << "LengthOfSlack;"
             << std::endl;
        file << numberOfRobots << ";"
             << observations << ";"
             << time(0) - startTime << ";"
             << leaderDidntMove << ";"
             << lengthOfDrivenWay << ";"
             << lengthOfSlack << ";"
             << std::endl;
        Logger(INFO, "FINISHED!").newLine("File written:").newLine("-> " + fileName);
    }

    ~Statistical() {
        file.close();
    }
};