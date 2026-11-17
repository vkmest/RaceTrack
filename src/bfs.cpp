#include "bfs.hpp"

#include <algorithm>
#include <cstdint>

namespace racetrack {

namespace {

// Maps every state to a dense array index and back.
class StateIndex {
public:
    explicit StateIndex(const Track& track)
        : width_(track.width()),
          maxSpeed_(track.maxSpeed()),
          speeds_(2 * track.maxSpeed() + 1),
          size_(static_cast<std::size_t>(track.width()) * track.height() * speeds_ * speeds_) {}

    std::size_t size() const { return size_; }

    std::int32_t encode(const State& s) const {
        const int cell = s.pos.y * width_ + s.pos.x;
        return (cell * speeds_ + s.vel.x + maxSpeed_) * speeds_ + s.vel.y + maxSpeed_;
    }

    State decode(std::int32_t index) const {
        State s;
        s.vel.y = index % speeds_ - maxSpeed_;
        index /= speeds_;
        s.vel.x = index % speeds_ - maxSpeed_;
        index /= speeds_;
        s.pos = {index % width_, index / width_};
        return s;
    }

private:
    int width_;
    int maxSpeed_;
    int speeds_;  // number of distinct values of one velocity component
    std::size_t size_;
};

}  // namespace

Path solveBfs(const Track& track) {
    constexpr std::int32_t kUnvisited = -1;

    const StateIndex index(track);
    const State start{track.start(), {0, 0}};
    const std::int32_t startId = index.encode(start);

    // parent[s] is the state from which s was first reached. BFS reaches
    // every state with the fewest possible moves, so following the parents
    // back from the first finishing move gives an optimal path.
    std::vector<std::int32_t> parent(index.size(), kUnvisited);
    std::vector<std::int32_t> queue{startId};
    parent[startId] = startId;

    for (std::size_t head = 0; head < queue.size(); ++head) {
        const std::int32_t currentId = queue[head];
        const State current = index.decode(currentId);

        for (const Vec acceleration : kAccelerations) {
            const auto step = track.step(current, acceleration);
            if (!step) continue;

            if (step->finished) {
                Path path{step->state.pos};
                for (std::int32_t id = currentId;; id = parent[id]) {
                    path.push_back(index.decode(id).pos);
                    if (id == startId) break;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            const std::int32_t nextId = index.encode(step->state);
            if (parent[nextId] == kUnvisited) {
                parent[nextId] = currentId;
                queue.push_back(nextId);
            }
        }
    }
    return {};
}

}  // namespace racetrack
