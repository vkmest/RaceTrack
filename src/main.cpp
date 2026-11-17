// Benchmark: solves every given track with all three methods, writes the
// solutions as CSV files and prints / stores a comparison table.
//
//   racetrack [--runs N] [--iterations N] [--out DIR] <track file>...
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "annealing.hpp"
#include "bfs.hpp"
#include "greedy.hpp"
#include "track.hpp"

using namespace racetrack;

namespace {

struct Options {
    int runs = 10;  // repetitions per method and track
    AnnealingParams annealing;
    std::filesystem::path outDir = "results";
    std::vector<std::filesystem::path> trackFiles;
};

Options parseArguments(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool hasValue = i + 1 < argc;
        if (arg == "--runs" && hasValue) {
            options.runs = std::stoi(argv[++i]);
        } else if (arg == "--iterations" && hasValue) {
            options.annealing.iterations = std::stoi(argv[++i]);
        } else if (arg == "--out" && hasValue) {
            options.outDir = argv[++i];
        } else {
            options.trackFiles.emplace_back(arg);
        }
    }
    if (options.trackFiles.empty() || options.runs < 1)
        throw std::runtime_error("usage: racetrack [--runs N] [--iterations N] [--out DIR] <track file>...");
    return options;
}

int moveCount(const Path& path) { return static_cast<int>(path.size()) - 1; }

void writePath(const Path& path, const std::filesystem::path& file) {
    std::ofstream out(file);
    for (const Vec pos : path) out << pos.x << ',' << pos.y << '\n';
}

// Runs a solver and returns its result together with the run time in ms.
template <typename Solver>
Path timed(Solver solve, double& milliseconds) {
    const auto begin = std::chrono::steady_clock::now();
    Path path = solve();
    const auto end = std::chrono::steady_clock::now();
    milliseconds = std::chrono::duration<double, std::milli>(end - begin).count();
    return path;
}

// Statistics over the repeated runs of one method on one track.
struct Summary {
    Path bestPath;
    int best = 0;
    int worst = 0;
    double mean = 0;
    double deviation = 0;     // standard deviation of the number of moves
    double milliseconds = 0;  // mean run time
};

template <typename Solver>
Summary benchmark(int runs, Solver solve) {
    Summary summary;
    std::vector<int> moves;
    for (int run = 0; run < runs; ++run) {
        double milliseconds = 0;
        Path path = timed([&] { return solve(run); }, milliseconds);
        if (path.empty()) throw std::runtime_error("the finish line cannot be reached");

        summary.milliseconds += milliseconds / runs;
        moves.push_back(moveCount(path));
        if (run == 0 || moveCount(path) < moveCount(summary.bestPath)) summary.bestPath = path;
    }

    summary.best = *std::min_element(moves.begin(), moves.end());
    summary.worst = *std::max_element(moves.begin(), moves.end());
    double sum = 0;
    for (const int m : moves) sum += m;
    summary.mean = sum / runs;

    double squares = 0;
    for (const int m : moves) squares += (m - summary.mean) * (m - summary.mean);
    summary.deviation = std::sqrt(squares / runs);
    return summary;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArguments(argc, argv);
        std::filesystem::create_directories(options.outDir / "solutions");

        std::ofstream csv(options.outDir / "results.csv");
        csv << "track,width,height,bfs_moves,bfs_ms,greedy_moves,greedy_ms,"
               "sa_best,sa_mean,sa_dev,sa_worst,sa_ms\n";

        std::printf("%-10s | %5s %9s | %6s %9s | %4s %6s %5s %5s %9s\n", "track", "BFS", "ms",
                    "greedy", "ms", "SA", "mean", "dev", "worst", "ms");

        for (const auto& file : options.trackFiles) {
            const Track track(file.string());
            const std::string name = file.stem().string();

            // The two deterministic methods are repeated only to get a
            // stable run time. Each method is timed including all of its
            // own preprocessing (e.g. building the distance map).
            const Summary bfs = benchmark(options.runs, [&](int) { return solveBfs(track); });
            const Summary greedy =
                benchmark(options.runs, [&](int) { return GreedyDriver(track).solve(); });
            const Summary sa = benchmark(options.runs, [&](int run) {
                const GreedyDriver driver(track);
                return solveAnnealing(driver, options.annealing, run + 1);  // seeds 1, 2, ...
            });

            writePath(bfs.bestPath, options.outDir / "solutions" / (name + "_bfs.csv"));
            writePath(greedy.bestPath, options.outDir / "solutions" / (name + "_greedy.csv"));
            writePath(sa.bestPath, options.outDir / "solutions" / (name + "_sa.csv"));

            csv << name << ',' << track.width() << ',' << track.height() << ',' << bfs.best << ','
                << bfs.milliseconds << ',' << greedy.best << ',' << greedy.milliseconds << ','
                << sa.best << ',' << sa.mean << ',' << sa.deviation << ',' << sa.worst << ','
                << sa.milliseconds << '\n';

            std::printf("%-10s | %5d %9.2f | %6d %9.3f | %4d %6.2f %5.2f %5d %9.1f\n",
                        name.c_str(), bfs.best, bfs.milliseconds, greedy.best, greedy.milliseconds,
                        sa.best, sa.mean, sa.deviation, sa.worst, sa.milliseconds);
            std::fflush(stdout);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
