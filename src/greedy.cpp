#include "greedy.hpp"

#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace racetrack {

namespace {

constexpr int kUnreachable = std::numeric_limits<int>::max();

// Distance-map cost of leaving a grass cell (a track cell costs 1).
// Makes the driver avoid coming to rest on grass.
constexpr int kGrassCost = 3;

// Estimated number of moves per unit of crawling distance. Must be at least
// 2, otherwise a standing car would see no benefit in starting to move.
constexpr int kCrawlCost = 2;

int sign(int v) { return (v > 0) - (v < 0); }

}  // namespace

GreedyDriver::GreedyDriver(const Track& track) : track_(track) { buildDistanceMap(); }

// Dijkstra from the finish line over single-cell moves (speed 1). The result
// tells the driver how far from the finish every cell is, around all corners.
void GreedyDriver::buildDistanceMap() {
    const int width = track_.width();
    distance_.assign(width * track_.height(), kUnreachable);

    using Entry = std::pair<int, int>;  // (distance, cell)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;

    for (int cell = 0; cell < static_cast<int>(distance_.size()); ++cell) {
        if (track_.at({cell % width, cell / width}) == 'F') {
            distance_[cell] = 0;
            open.push({0, cell});
        }
    }

    while (!open.empty()) {
        const auto [dist, cell] = open.top();
        open.pop();
        if (dist > distance_[cell]) continue;  // outdated queue entry

        // Single-cell moves are symmetric, so "pos -> neighbour is legal"
        // also means the car can crawl from the neighbour to pos.
        const Vec pos{cell % width, cell / width};
        for (const Vec direction : kAccelerations) {
            const auto step = track_.step(State{pos, {0, 0}}, direction);
            if (!step) continue;

            const Vec neighbourPos = step->state.pos;
            const int neighbour = neighbourPos.y * width + neighbourPos.x;
            const int cost = track_.at(neighbourPos) == 'G' ? kGrassCost : 1;
            if (dist + cost < distance_[neighbour]) {
                distance_[neighbour] = dist + cost;
                open.push({dist + cost, neighbour});
            }
        }
    }
}

std::optional<int> GreedyDriver::estimate(State state, Vec acceleration) const {
    for (int moves = 1;; ++moves) {
        const auto step = track_.step(state, acceleration);
        if (!step) return std::nullopt;
        if (step->finished) return moves;

        state = step->state;
        if (state.vel == Vec{0, 0}) {
            const int rest = distance_[state.pos.y * track_.width() + state.pos.x];
            if (rest == kUnreachable) return std::nullopt;
            return moves + kCrawlCost * rest;
        }
        acceleration = {-sign(state.vel.x), -sign(state.vel.y)};  // full braking
    }
}

std::optional<Vec> GreedyDriver::bestAcceleration(const State& state) const {
    std::optional<Vec> best;
    int bestEstimate = kUnreachable;

    for (const Vec acceleration : kAccelerations) {
        const auto moves = estimate(state, acceleration);
        if (moves && *moves < bestEstimate) {
            bestEstimate = *moves;
            best = acceleration;
        }
    }
    return best;
}

bool GreedyDriver::drive(State state, Path& path) const {
    while (true) {
        const auto acceleration = bestAcceleration(state);
        if (!acceleration) return false;

        const Step step = *track_.step(state, *acceleration);
        state = step.state;
        path.push_back(state.pos);
        if (step.finished) return true;
    }
}

Path GreedyDriver::solve() const {
    Path path{track_.start()};
    if (!drive(State{track_.start(), {0, 0}}, path)) return {};
    return path;
}

}  // namespace racetrack
