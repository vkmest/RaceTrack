#include "annealing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <random>

namespace racetrack {

namespace {

// A solution as the annealer sees it: the car's state after every move.
using Trajectory = std::vector<State>;

using Rng = std::mt19937_64;

int randomInt(Rng& rng, int low, int high) {
    return std::uniform_int_distribution<int>(low, high)(rng);
}

// One axis in isolation: can a car with velocity `from` make exactly `moves`
// moves that cover `distance` in total and end with velocity `to`?
//
// The farthest it can get is by accelerating as long as possible and braking
// just in time, the shortest by doing the opposite. Every distance between
// those two extremes is achievable as well.
bool reachable(int from, int to, int moves, int distance) {
    if (std::abs(to - from) > moves) return false;

    int farthest = 0;
    int shortest = 0;
    for (int t = 1; t <= moves; ++t) {
        farthest += std::min(from + t, to + (moves - t));
        shortest += std::max(from - t, to - (moves - t));
    }
    return shortest <= distance && distance <= farthest;
}

// Draws a random acceleration sequence for one axis that satisfies
// reachable(). Returns false if there is none.
bool sampleAxis(int from, int to, int moves, int distance, Rng& rng, std::vector<int>& out) {
    out.clear();
    int velocity = from;
    for (int left = moves - 1; left >= 0; --left) {
        int options[3];
        int count = 0;
        for (int a = -1; a <= 1; ++a) {
            if (reachable(velocity + a, to, left, distance - (velocity + a))) options[count++] = a;
        }
        if (count == 0) return false;

        const int a = options[randomInt(rng, 0, count - 1)];
        velocity += a;
        distance -= velocity;
        out.push_back(a);
    }
    return true;
}

class Annealer {
public:
    Annealer(const GreedyDriver& driver, const AnnealingParams& params, std::uint64_t seed)
        : driver_(driver), track_(driver.track()), params_(params), rng_(seed) {}

    Path run() {
        Path initial = driver_.solve();
        if (initial.empty()) return {};

        Trajectory current = toTrajectory(initial);
        Trajectory best = current;

        const double cooling =
            std::pow(params_.endTemperature / params_.startTemperature, 1.0 / params_.iterations);
        double temperature = params_.startTemperature;

        for (int i = 0; i < params_.iterations; ++i, temperature *= cooling) {
            Trajectory candidate;
            if (!neighbour(current, candidate)) continue;

            const int delta = static_cast<int>(candidate.size()) - static_cast<int>(current.size());
            if (delta <= 0 || uniform_(rng_) < std::exp(-delta / temperature)) {
                current = std::move(candidate);
                if (current.size() < best.size()) best = current;
            }
        }

        Path path;
        for (const State& state : best) path.push_back(state.pos);
        return path;
    }

private:
    Trajectory toTrajectory(const Path& path) const {
        Trajectory trajectory{State{path[0], {0, 0}}};
        for (std::size_t k = 1; k < path.size(); ++k) {
            const Vec vel{path[k].x - path[k - 1].x, path[k].y - path[k - 1].y};
            trajectory.push_back(State{path[k], vel});
        }
        return trajectory;
    }

    // Builds a random neighbour of `current`: a window of the trajectory is
    // replaced by a new piece. Returns false if the attempt did not produce
    // a valid trajectory.
    bool neighbour(const Trajectory& current, Trajectory& candidate) {
        const int last = static_cast<int>(current.size()) - 1;  // index of the finishing state
        const int begin = randomInt(rng_, 0, last - 1);
        const int end = begin + randomInt(rng_, 2, params_.maxWindow);

        candidate.assign(current.begin(), current.begin() + begin + 1);
        if (end >= last) return replanFinish(candidate);

        bool finished = false;
        const int moves = end - begin + randomInt(rng_, -1, 1);
        if (!replanWindow(current[end], moves, candidate, finished)) return false;

        // Unless the new piece already crossed the finish line, the rest of
        // the old trajectory follows unchanged.
        if (!finished) candidate.insert(candidate.end(), current.begin() + end + 1, current.end());
        return true;
    }

    // Extends the trajectory by `moves` random moves that lead exactly to
    // the state `target` (same position and same velocity). Sets `finished`
    // if the new piece happens to cross the finish line on its way.
    bool replanWindow(const State& target, int moves, Trajectory& trajectory, bool& finished) {
        const State from = trajectory.back();
        if (!sampleAxis(from.vel.x, target.vel.x, moves, target.pos.x - from.pos.x, rng_, accelX_) ||
            !sampleAxis(from.vel.y, target.vel.y, moves, target.pos.y - from.pos.y, rng_, accelY_))
            return false;

        for (int k = 0; k < moves; ++k) {
            const auto step = track_.step(trajectory.back(), {accelX_[k], accelY_[k]});
            if (!step) return false;
            trajectory.push_back(step->state);
            if (step->finished) {
                finished = true;
                break;
            }
        }
        return true;
    }

    // Finishes the race from the end of the trajectory: one random move,
    // then the greedy driver takes over.
    bool replanFinish(Trajectory& trajectory) {
        const Vec acceleration = kAccelerations[randomInt(rng_, 0, 8)];
        const auto step = track_.step(trajectory.back(), acceleration);
        if (!step) return false;
        trajectory.push_back(step->state);
        if (step->finished) return true;

        Path rest;
        if (!driver_.drive(step->state, rest)) return false;
        for (const Vec pos : rest) {
            const Vec previous = trajectory.back().pos;
            trajectory.push_back(State{pos, {pos.x - previous.x, pos.y - previous.y}});
        }
        return true;
    }

    const GreedyDriver& driver_;
    const Track& track_;
    AnnealingParams params_;
    Rng rng_;
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
    std::vector<int> accelX_, accelY_;  // scratch buffers of replanWindow
};

}  // namespace

Path solveAnnealing(const GreedyDriver& driver, const AnnealingParams& params, std::uint64_t seed) {
    return Annealer(driver, params, seed).run();
}

}  // namespace racetrack
