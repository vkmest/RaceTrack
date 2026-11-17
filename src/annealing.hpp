// Metaheuristic: simulated annealing that re-plans short pieces of the path.
#pragma once

#include <cstdint>

#include "greedy.hpp"
#include "track.hpp"

namespace racetrack {

struct AnnealingParams {
    int iterations = 1000000;
    double startTemperature = 0.5;  // temperatures are measured in moves
    double endTemperature = 0.05;
    int maxWindow = 12;             // longest piece of the path that is re-planned
};

// Improves the greedy solution. The driver is used for the initial solution
// and to finish the race whenever the end of the path is re-planned.
Path solveAnnealing(const GreedyDriver& driver, const AnnealingParams& params, std::uint64_t seed);

}  // namespace racetrack
