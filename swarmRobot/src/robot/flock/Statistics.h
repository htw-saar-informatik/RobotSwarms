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

std::vector<Robot> getFlockRobots() {
    std::vector<Robot> swarm(mySwarmIndices.size());
    for (unsigned int i : mySwarmIndices) {
        swarm.push_back(globalSwarm[i]);
    }

    return swarm;
}

void findFlockGroup(std::vector<Robot> &flock, std::vector<Robot> &leftovers) {
    for (unsigned int i = 0; i < flock.size(); i++) {
        const Robot &currentRoboter = flock[i];

        auto partitionIterator = std::partition(leftovers.begin(), leftovers.end(), [&currentRoboter](Robot &other) {
            return currentRoboter.distance(other) <= FLOCK_LOCALITY_RANGE;
        });

        std::move(leftovers.begin(), partitionIterator, std::back_inserter(flock));
        leftovers.erase(leftovers.begin(), partitionIterator);
    }
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

void statistics(std::ofstream &file) {
    auto flocks = flockGroups();

    Logger(INFO, "Number of flocks: %ld", flocks.size());
    file << std::to_string(flocks.size()) << "\n";
}