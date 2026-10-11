# Fixed Hall-Call Assignments and Floor-Ordered Pair Costs in Elevator Dispatch

Research code and synthetic simulation data for the accompanying article. The repository includes the event-driven elevator simulator, corrected and uncorrected quadratic dispatch, floor-ordered coefficient assembly, sequential insertion, an adapted genetic algorithm, and the three disclosed learning studies.

## Requirements

- A C++20 compiler, CMake 3.16 or later, and Make or Ninja.
- Python 3.9 or later for experiment orchestration; no Python packages are required.
- Linux `taskset` is optional for a timing run pinned to a logical CPU.

The simulator, learning algorithms, numerical checks and statistical aggregation are implemented in C++. Python scripts invoke these programs and prepare their output tables.

## Build and verify

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
python3 scripts/reproduce.py smoke
```

To select a compiler, pass `-DCMAKE_CXX_COMPILER=g++` to the first command. Build with ordinary floating-point arithmetic; do not enable fast-math.

## Recreate the numerical summaries

```sh
python3 scripts/reproduce.py summarize
```

This reads the packaged records and writes main-service comparisons, learning results, peak results, the exclusion bound, timing aggregation and LaTeX tables under `outputs/`. It does not retrain models or rerun the traffic simulations. `outputs/tables/learning-tables.tex` orders the learning tables as in Appendix F.

## Rerun experiments

```sh
python3 scripts/reproduce.py main
python3 scripts/reproduce.py peak
python3 scripts/reproduce.py omission
python3 scripts/reproduce.py scale
python3 scripts/reproduce.py timing
python3 scripts/reproduce.py evaluate final
python3 scripts/reproduce.py diagnostic
```

`evaluate final` uses the packaged selected models and recomputes the final learning comparison and its separate RL-only sensitivity analysis. Earlier studies can be rerun with `evaluate initial` and `evaluate followup`. Output directories are separate from the packaged input data. Use `--jobs 1` for sequential execution or `--out PATH` for another output directory.

Timing should run while traffic simulation and training are idle. On Linux, `--cpu N` pins the timing program to logical CPU N. Frequency scaling and host core placement remain properties of the execution environment, and measured microsecond times will vary.

## Train learning baselines

```sh
python3 scripts/reproduce.py train initial
python3 scripts/reproduce.py train followup
python3 scripts/reproduce.py train final
python3 scripts/reproduce.py evaluate final --models outputs/learning/final
```

Each training command runs that study's fixed tuning candidates and five main initializations. The final study includes 90 route-pretraining episodes before 3,600 reinforcement-learning episodes. Its evaluation reevaluates the packaged follow-up models on the final test streams; `--previous-models PATH` selects another follow-up model directory.

## Contents

| Directory | Contents |
|---|---|
| `src/` | Simulator, passenger generation, route costs and dispatch policies |
| `research/` | Structured coefficient evaluation, experiment runners and C++ aggregators |
| `tests/` | Numerical, observation, routing and dispatch checks |
| `scripts/` | Experiment commands and table formatting |
| `data/` | Per-run results, saved decision inputs, model checkpoints and training/validation records |

[EXPERIMENTS.md](EXPERIMENTS.md) describes protocols and interpretation. [DATA.md](DATA.md) describes the records and policy identifiers.

## Scope of the learning comparison

On the final common test streams, the primary selected network has mean AWT 22.694 s, sequential insertion 22.610 s, and corrected search 19.716 s. All five primary selections are route-pretraining-only checkpoints. Actual waiting-reward learning did not improve the selected validation score; its separately disclosed RL-only selection has mean AWT 23.342 s. These are our common-engine baselines, not reproductions of published contemporary elevator DRL controllers. Different action and capacity treatments are described in the article and in EXPERIMENTS.md.

The experiments use synthetic traffic. Peak results and constructed larger-building checks do not establish real-building performance. Earlier learning studies and all their cells are retained in the data.
