# Data guide

All traffic records are synthetic. Passenger streams are paired across policies within a setting and seed label. Different building settings have different streams even when they share a label.

| Location | Records |
|---|---|
| `data/main/` | The 75-setting, 30-seed confirmation grid; thresholds 0, 2 and 5 s, GA and periodic comparisons |
| `data/timing/inputs/` | 75 compressed JSONL files containing saved decision states and coefficients |
| `data/timing/inputs.json` | Input filenames and row counts in the timing aggregation order |
| `data/timing/cells/` | Recorded 30-repeat timings and same-input equivalence results |
| `data/omission/inputs/` | 144 compressed traces from 48 audit settings and seeds 0–2 |
| `data/omission/` | Audit summary, excluded-decision counts and deterministic bound |
| `data/learning/initial/` | Initial DQN training, tuning, selected models and test records |
| `data/learning/followup/` | Route-feature follow-up DQN, full training curves and checkpoints |
| `data/learning/final/` | Literature-informed route-pretraining and Dueling Double DQN study, primary and RL-only evaluations |
| `data/diagnostic/` | Validation-only episode records and compressed selection-level records |
| `data/peak/` | Fixed-policy synthetic up/down profiles |
| `data/scale/` | Constructed 24/40-floor, 12-car coefficient and assignment checks |
| `data/pickup-convention/` | Exact alternative-timestamp verification summary |

## Policy identifiers

| Identifier | Policy |
|---|---|
| `paper2017` | Quadratic greedy assignment without the fixed-call correction |
| `paper2017_lk` | Quadratic greedy assignment with the fixed-call correction |
| `paper2017_ls` | Move-and-swap search without the correction |
| `paper2017_ls_lk` | Move-and-swap search with the correction |
| `sequential_greedy` | Sequential insertion retaining existing assignments |
| `ga2022_all` | Specified common-engine genetic-algorithm adaptation |
| `route_rule` | Minimum route-proxy rule used for final-study pretraining |

## Numerical fields

`awt`, `p95_wait`, `att` and `ajt` are in seconds. The reported P95 aggregates average within-run quantiles rather than pooling passengers across runs. `unserved` and `unfinished` describe the final drained state. Learning episode records also contain policy, model seed, traffic label, building setting and update counts.

The simulator records pickup at the serving car's arrival at its service stop, or immediate boarding into an already open door. AWT ends at this recorded event; it is not physical completion of boarding.

Training and checkpoint CSV files accompany the binary model files. `best.bin` is selected using validation AWT. Final-study `checkpoint-0.bin` follows route pretraining and precedes actual waiting-reward learning. The RL-only selections and their validation scores are listed in `data/learning/final/rl-only-test/selected-models.tsv`.

Files ending in `.gz` use standard gzip compression. The reproduction script decompresses them when needed. Model files use the C++ implementations' binary serialization and should be read with the corresponding study's runner.
