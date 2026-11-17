#include "track.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace racetrack {

Track::Track(const std::string& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("cannot open track file: " + file);

    std::vector<std::string> rows;
    for (std::string line; std::getline(in, line);) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) rows.push_back(line);
    }
    if (rows.empty()) throw std::runtime_error("empty track file: " + file);

    height_ = static_cast<int>(rows.size());
    width_ = static_cast<int>(rows.front().size());
    std::reverse(rows.begin(), rows.end());  // the last line becomes y = 0

    bool hasStart = false;
    for (int y = 0; y < height_; ++y) {
        if (static_cast<int>(rows[y].size()) != width_)
            throw std::runtime_error("track is not rectangular: " + file);
        for (int x = 0; x < width_; ++x) {
            cells_.push_back(rows[y][x]);
            if (rows[y][x] == 'S') {
                start_ = {x, y};
                hasStart = true;
            }
        }
    }
    if (!hasStart) throw std::runtime_error("track has no start: " + file);

    const int longestSide = std::max(width_, height_);
    while ((maxSpeed_ + 1) * (maxSpeed_ + 2) / 2 <= longestSide) ++maxSpeed_;
}

char Track::at(Vec p) const {
    if (p.x < 0 || p.x >= width_ || p.y < 0 || p.y >= height_) return 'O';
    return cells_[p.y * width_ + p.x];
}

std::optional<Step> Track::step(const State& from, Vec acceleration) const {
    const Vec vel = from.vel + acceleration;

    // Grass rule (per axis): a speed of 2 or more must drop, and a speed of
    // 0 or 1 must not grow beyond 1.
    if (at(from.pos) == 'G') {
        if (std::abs(vel.x) > std::max(std::abs(from.vel.x) - 1, 1)) return std::nullopt;
        if (std::abs(vel.y) > std::max(std::abs(from.vel.y) - 1, 1)) return std::nullopt;
    }

    const Vec pos = from.pos + vel;
    const Contact contact = trace(from.pos, pos);
    if (contact == Contact::Obstacle) return std::nullopt;
    return Step{State{pos, vel}, contact == Contact::Finish};
}

// A cell is the unit square around its centre. The segment from -> to touches
// the square around c exactly if c lies in the bounding box of the segment
// and the line passes c closely enough:
//
//     2 * |d x (c - from)|  <=  |d.x| + |d.y|        with d = to - from
//
// (separating-axis test of a segment against a square, in integers only).
// Touching a corner counts, which matches the official validator.
Track::Contact Track::trace(Vec from, Vec to) const {
    if (at(to) == 'O') return Contact::Obstacle;  // also covers "off the board"

    const int dx = to.x - from.x;
    const int dy = to.y - from.y;
    const int reach = std::abs(dx) + std::abs(dy);

    bool touchesFinish = false;
    for (int y = std::min(from.y, to.y); y <= std::max(from.y, to.y); ++y) {
        for (int x = std::min(from.x, to.x); x <= std::max(from.x, to.x); ++x) {
            const int cross = dx * (y - from.y) - dy * (x - from.x);
            if (2 * std::abs(cross) > reach) continue;  // the segment misses this cell

            const char cell = cells_[y * width_ + x];
            if (cell == 'O') return Contact::Obstacle;
            if (cell == 'F') touchesFinish = true;
        }
    }
    return touchesFinish ? Contact::Finish : Contact::None;
}

}  // namespace racetrack
