// Construction heuristic: a greedy driver that never loses the ability to stop.
#pragma once

#include "track.hpp"

namespace racetrack {

class GreedyDriver {
public:
    explicit GreedyDriver(const Track& track);

    // Builds a complete solution from the start position
    // (empty if the finish line cannot be reached).
    Path solve() const;

    // Drives from the given state to the finish and appends the visited
    // positions to path. Fails only if the finish line cannot be reached.
    bool drive(State state, Path& path) const;

    // Estimated number of moves to the finish if the car makes this move,
    // then brakes to a halt and crawls the rest of the way at speed 1.
    // Returns nothing if the move is illegal or unsafe, i.e. if the car
    // would crash while braking.
    std::optional<int> estimate(State state, Vec acceleration) const;

    // The safe move with the lowest estimate. A car that got where it is by
    // safe moves always has one, unless the finish line cannot be reached.
    std::optional<Vec> bestAcceleration(const State& state) const;

    const Track& track() const { return track_; }

private:
    void buildDistanceMap();

    const Track& track_;
    std::vector<int> distance_;  // crawling distance from each cell to the finish
};

}  // namespace racetrack
