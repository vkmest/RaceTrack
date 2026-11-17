# RaceTrack

Solvers for the pencil-and-paper game RaceTrack: drive from the start to the
finish line in as few moves as possible, where every move may change the
velocity by at most one cell per axis. Rules and task are described in
[`RaceTrack ProjectDescription.pdf`](RaceTrack%20ProjectDescription.pdf).

Three methods are implemented in C++17 and compared on all benchmark tracks:

| Method | Kind | File |
|---|---|---|
| Breadth-first search | exact, the ground truth | `src/bfs.cpp` |
| Greedy driver | construction heuristic | `src/greedy.cpp` |
| Simulated annealing | metaheuristic | `src/annealing.cpp` |

## What is in this repository

- **Ready-made tracks:** the nine benchmark tracks are in [`tracks/`](tracks)
  (`track_02.t` to `track_10.t`). Nothing has to be created or downloaded.
- **The validator / visualiser:** [`visualise.pl`](visualise.pl), the script
  provided with the exercise. It checks a solution against the rules and draws
  it.
- **The solutions already computed:** [`results/`](results) holds the result
  table (`results.csv`) and one solution file per track and method, so you can
  look at them without building anything.

## Quick start

You need a C++17 compiler and CMake (3.16 or newer).

**Linux / macOS**

```bash
cmake -S . -B build
```

```bash
cmake --build build
```

```bash
./build/racetrack tracks/*.t
```

**Windows** (MinGW-w64, commands for Git Bash)

```bash
cmake -S . -B build -G "MinGW Makefiles"
```

```bash
cmake --build build
```

```bash
./build/racetrack.exe tracks/*.t
```

The program solves every track with all three methods (about one minute in
total), prints the comparison table and writes

- `results/results.csv` – the table, and
- `results/solutions/<track>_<bfs|greedy|sa>.csv` – the paths, one `x,y`
  position per line.

To solve a single track, pass just that file:

```bash
./build/racetrack tracks/track_05.t
```

Options:

| Option | Meaning | Default |
|---|---|---|
| `--runs N` | repetitions per method and track | 10 |
| `--iterations N` | simulated annealing iterations | 1,000,000 |
| `--out DIR` | output folder | `results` |

## Results

Number of moves (SA: 10 runs per track) and mean run time per run.

| Track | Size | BFS (optimum) | Greedy | SA best | SA mean | SA worst | BFS ms | Greedy ms | SA ms |
|---|---|---|---|---|---|---|---|---|---|
| track_02 | 24x21 | 13 | 15 | 13 | 13.0 | 13 | 1.1 | 0.06 | 1485 |
| track_03 | 76x58 | 41 | 45 | 41 | 41.2 | 42 | 94.9 | 1.15 | 1397 |
| track_04 | 76x58 | 39 | 48 | 39 | 39.2 | 41 | 55.9 | 1.02 | 1211 |
| track_05 | 113x59 | 88 | 104 | 88 | 88.0 | 88 | 29.6 | 0.51 | 824 |
| track_06 | 113x59 | 85 | 107 | 85 | 85.0 | 85 | 39.9 | 1.26 | 873 |
| track_07 | 113x38 | 32 | 33 | 32 | 32.0 | 32 | 90.7 | 0.80 | 748 |
| track_08 | 113x38 | 33 | 44 | 33 | 33.0 | 33 | 82.1 | 0.61 | 787 |
| track_09 | 113x38 | 32 | 35 | 32 | 32.0 | 32 | 74.8 | 0.81 | 686 |
| track_10 | 113x38 | 25 | 31 | 25 | 25.0 | 25 | 72.4 | 0.60 | 775 |

Every solution was checked with the official validator `visualise.pl`. BFS is provably
optimal; simulated annealing reaches that optimum on every track, the greedy heuristic
is about 19 % above it but needs only about a millisecond. Measured on an Intel
Core i7-12700H, single-threaded.

## Validate and draw a solution

`visualise.pl` needs Perl with the `Text::CSV` module, and LaTeX to turn its
output into a picture. It takes a track, a solution and the name of the output
file:

```bash
perl visualise.pl tracks/track_05.t results/solutions/track_05_sa.csv track_05.tex
```

```bash
pdflatex track_05.tex
```

The resulting `track_05.pdf` shows the track with the path; any rule violation
is marked with a red circle.

## Layout

```
src/track.*       board and rules of the game (the only place that knows them)
src/bfs.*         exact breadth-first search
src/greedy.*      construction heuristic
src/annealing.*   simulated annealing
src/main.cpp      benchmark program
tracks/           benchmark tracks
visualise.pl      validator and visualiser provided with the exercise
results/          solutions and results of the last benchmark run
```

## References

- M. Gardner. Mathematical games – Sim, Chomp and Race Track. *Scientific American* 228(1), 1973.
- M. A. Bekos et al. Algorithms and insights for RaceTrack. *Theoretical Computer Science* 748, 2018. <https://doi.org/10.1016/j.tcs.2018.04.028>
- M. Holzer, P. McKenzie. The computational complexity of RaceTrack. FUN 2010. <https://doi.org/10.1007/978-3-642-13122-6_26>
