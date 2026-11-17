// Exact solver: breadth-first search over all (position, velocity) states.
#pragma once

#include "track.hpp"

namespace racetrack {

// Returns a path with the minimum possible number of moves,
// or an empty path if the finish line cannot be reached.
Path solveBfs(const Track& track);

}  // namespace racetrack
