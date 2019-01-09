//
// Created by smanier on 06.10.18.
//

#pragma once

#include <ros/ros.h>
#include <vector>

#include <iostream>
#include <fstream>

#include "FlockControl.h"
#include "../../various/Constants.h"
#include "../../logger/Logger.h"
#include "../../basics/ToString.h"

std::vector<Robot> getFlockRobots() {
    std::vector<Robot> swarm;
    for (unsigned int i : mySwarmIndices) {
        swarm.push_back(globalSwarm[i]);
    }
    return swarm;
}

void print(const std::vector<Robot> &flock, const std::vector<Robot> &left, std::string caption) {
    Logger(WARNING, 2, caption)
            .newLine("\n\tFlock(%d): %s", flock.size(), myString::toString(flock).c_str())
            .newLine("\tLeftovers(%d): %s", left.size(), myString::toString(left).c_str());
}

void findFlockGroup(std::vector<Robot> &flock, std::vector<Robot> &leftovers) {
    //print(flock, leftovers, "Begin");

    for (unsigned int i = 0; i < flock.size(); i++) {
        const Robot &currentRoboter = flock[i];

        auto partitionIterator = std::partition(leftovers.begin(), leftovers.end(), [&currentRoboter](Robot &other) {
            return currentRoboter.distance(other) <= FLOCK_LOCALITY_RANGE;
        });
        //print(flock, leftovers, "Partition");

        std::move(leftovers.begin(), partitionIterator, std::back_inserter(flock));
        //print(flock, leftovers, "Move");

        leftovers.erase(leftovers.begin(), partitionIterator);
        //print(flock, leftovers, "Erase");
    }

    //print(flock, leftovers, "End");
}

std::vector<std::vector<Robot>> flockGroups() {
    std::vector<Robot> leftovers = getFlockRobots();
    std::vector<std::vector<Robot>> flocks;

    while (!leftovers.empty()) {
        flocks.emplace_back();
        std::vector<Robot> &currentFlock = flocks.back();

        auto firstLeftover = leftovers.begin();
        currentFlock.push_back(*firstLeftover);
        leftovers.erase(firstLeftover);

        findFlockGroup(currentFlock, leftovers);
    }

    return flocks;
}

std::vector<Robot> findFlock(const Robot &self) {
    std::vector<Robot> robotFlock = getFlockRobots();
    std::vector<Robot> myFlock;
    myFlock.push_back(self);

    findFlockGroup(myFlock, robotFlock);
    myFlock.erase(find(myFlock.begin(), myFlock.end(), self));

    return myFlock;
}

Position flockCentre(const std::vector<Robot> &flock) {
    double x = 0, y = 0;

    for (const Robot &i : flock) {
        x += i.position.x;
        y += i.position.y;
    }

    return {(float) (x / flock.size()), (float) (y / flock.size())};
}

void statistics(std::ofstream &file, const std::vector<Robot> &missionRobots) {

}