// The RaceTrack board and the rules of the game.
//
// Every solver uses Track::step() as its only way to move the car, so the
// rules live in exactly one place.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace racetrack {

struct Vec {
    int x = 0;
    int y = 0;

    friend Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y}; }
    friend bool operator==(Vec a, Vec b) { return a.x == b.x && a.y == b.y; }
};

// The car: where it is and how far it travelled in its previous move.
struct State {
    Vec pos;
    Vec vel;
};

// A solution: every position the car visits, the start first.
// The number of moves is therefore path.size() - 1.
using Path = std::vector<Vec>;

// In each move the car may change each velocity component by -1, 0 or +1.
inline constexpr std::array<Vec, 9> kAccelerations{{
    {-1, -1}, {0, -1}, {1, -1},
    {-1,  0}, {0,  0}, {1,  0},
    {-1,  1}, {0,  1}, {1,  1},
}};

// The outcome of one legal move.
struct Step {
    State state;
    bool finished;  // the move touched the finish line
};

class Track {
public:
    // Reads a track file: a character raster of T(rack), S(tart), F(inish),
    // O(bstacle) and G(rass). The last line of the file is y = 0.
    explicit Track(const std::string& file);

    int width() const { return width_; }
    int height() const { return height_; }
    Vec start() const { return start_; }

    // No velocity component can ever exceed this: reaching speed v needs
    // 1 + 2 + ... + v cells of run-up, which must fit on the board.
    int maxSpeed() const { return maxSpeed_; }

    // Cell type at p; everything outside the board is an obstacle.
    char at(Vec p) const;

    // Performs one move. Returns nothing if the move is illegal, i.e. if it
    // breaks the grass rule, hits an obstacle or leaves the board.
    std::optional<Step> step(const State& from, Vec acceleration) const;

private:
    enum class Contact { None, Obstacle, Finish };

    // What the straight line between two cell centres touches.
    Contact trace(Vec from, Vec to) const;

    int width_ = 0;
    int height_ = 0;
    int maxSpeed_ = 0;
    Vec start_;
    std::vector<char> cells_;  // row-major, y = 0 first
};

}  // namespace racetrack
