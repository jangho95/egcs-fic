# Experiment protocols

## Main service comparisons

The main grid uses 8, 10 and 12 floors; 2, 3, 4, 5 and 6 cars; and rates 10, 15, 20, 25 and 30%, giving 75 settings. Labels 50000–50029 supply 30 paired replications per setting. The generator seed is `1000003 * label + 101 * floors + 7 * cars + 10 * rate`. Traffic and learning random number generators are separate.

Each floor represents 30 occupants. One hour of arrivals is followed by full drainage, capped at four hours. Car capacity is 13, speed 1.6 m/s, acceleration 0.8 m/s², floor spacing 3.5 m, settling time 0.5 s, door opening/closing 2/3 s and passenger transfer time 1.2 s. The main policies observe direction buttons and permitted car state. Actual waiting group sizes and unregistered destinations are not policy inputs.

For each contrast, average paired differences across the 75 settings within each label, then use the 30 label means for a two-sided 95% t interval with 29 degrees of freedom. Cell results are descriptive or use their own pointwise intervals. The main threshold is 5 s; separate files preserve 0/2 s and periodic comparisons. GA uses population 30, 30 generations and collective-route evaluation; it is the adaptation specified in the article, with omitted source features disclosed there.

## Learning studies

All three studies mix nine 10-floor settings equally: 2/4/6 cars and rates 10/20/30%. Each uses five independently initialized models and a separate test block. A five-model mean is a summary across policies, not an ensemble controller. Confidence intervals condition on these five models and use 30 label-level means across the nine cells.

| Parameter | Initial | Follow-up | Final |
|---|---|---|---|
| Main model seeds | 82000–82004 | 84000–84004 | 86000–86004 |
| Validation labels | 90000–90009 | 95000–95009 | 97000–97009 |
| Test labels | 91000–91029 | 96000–96029 | 98000–98029 |
| Shared car network | 89–64–64–1 | 118–64–64–1 | 122–32–32–2, dueling |
| Adam learning rate | 0.0003 | 0.0003 | 0.0005 |
| Replay capacity | 50,000 | 100,000 | 50,000 |
| Batch size | 64 | 64 | 32 |
| Update every transitions | 8 | 8 | 4 |
| Target-copy interval | 1,000 updates | 2,000 updates | 1,000 updates |
| Exploration decay | 200,000 transitions | 2,000,000 transitions | 400,000 transitions |
| Waiting-integral reward divisor | 1,000 | 1,000 | 100 |
| Main RL episode budget | 1,800 or 100,000 updates | 3,600 | 3,600 after 90 pretraining episodes |

Adam uses β1 = 0.9, β2 = 0.999 and ε = 1e−8, with Huber threshold 1 and gradient clipping at norm 10. Exploration decreases from 1 to 0.05. Same-instant sequential choices have zero waiting reward and unit discount; terminal states have zero bootstrap. The earlier discount is elapsed-time based with a 600 s half-life; the final study uses 0.95 when time elapses. The discounted training objective is not identical to undiscounted test AWT.

The initial and follow-up policies sequentially reassign free calls while retaining near-pickup fixed calls and using the original capacity postprocessing. The follow-up adds route and call-age features. The final policy retains all existing assignments and assigns only currently unassigned calls. Observed load of at least 11 masks a car for a new assignment; existing assignments remain until physical full-car release. This differs from revocation of free assignments in the earlier controllers and is not a capacity-matched ablation.

The final route proxy uses target pickup time plus the delay to existing hall calls. One rider is assumed per lit button or hall call; four deterministic directional destination probes use permitted observations. The route-pretraining and dueling advantage targets are derived from this proxy without using the proposed quadratic correction. The network makes its own decisions at inference.

Initial tuning tests learning rates 0.001/0.0003 with seed 81000 for 180 episodes; follow-up tuning uses seed 83000 for 600 episodes. Both select 0.0003. Final tuning tests guidance weights 0/0.1 with seed 85500, 90 pretraining and 900 RL episodes. Both select episode 0 with the same validation score; the specified tie rule selects guidance weight 0. Main validation chooses minimum AWT, breaking ties with the earlier checkpoint.

All initial models hit their update cap after 300–311 episodes. The follow-up completes 3,600 episodes per model. All five final primary selections are episode 0, so the selected policy is a learned approximation of the route guide and there is no demonstrated additional RL gain. Initial/follow-up mean AWT is 32.206/29.823 s on their respective test blocks. These different blocks do not yield a paired improvement estimate.

The final primary AWT is 22.694 s versus sequential insertion 22.610 s, difference +0.084 s with interval [−0.290, +0.458]. Corrected search has 19.716 s. A separate RL-only rule excludes episode 0 and selects episodes 3600/720/1620/3420/3420; its AWT is 23.342 s. This sensitivity rule was added after examining validation and before the final test, and it does not replace the primary selection. Earlier studies, tuning candidates and validation-only diagnostics are retained. The development stages do not constitute one protocol specified before all results.

## Verification and supplementary experiments

The pickup-convention checker independently enumerates 333,504 pairs in 19,656 stationary initial states. It verifies both the service-arrival convention and the alternative after door opening and alighting. It does not verify moving states by enumeration.

The exclusion audit uses 48 settings and seeds 0–2, rather than the 75-setting confirmation grid. Of 2,789 excluded decisions, 437 have positive corrections. Identical objectives in the remaining excluded cases imply the 36.10–37.42% deterministic range for disjoint optimum sets among positive-correction decisions in this audit. This is not a confidence interval.

The timing study uses 36,285 saved states from 75 settings. Each input runs all six method orders five times, for 30 repetitions. Timing includes coefficient preparation, fallback, greedy/search and capacity postprocessing, and excludes parsing and engine execution. The recorded environment was an i9-12900K with WSL2, GCC 15.3.0 and `-O2`. A single process was pinned to logical CPU 0; host frequency policy and performance/efficiency core placement were not controlled. Recorded equal-cell dispatch means are 46.039 and 36.945 microseconds. New execution environments can produce different timings.

The peak experiment uses labels 92000–92029, nine settings and three fixed policies, for 1,620 runs. Up traffic has an 85/10/5% incoming/outgoing/interfloor mixture over one hour. Down traffic has a 15-minute low-rate interfloor prelude and a 30-minute 10/85/5% peak. The uniform-destination cost model remains unchanged. Both correction intervals include zero; insertion has lower AWT in up traffic, and correction increases mean within-run P95 waiting in down traffic.

Larger-building checks use 24/40 floors and 12 cars at 1.6/5.0 m/s in three stationary directions. These are constructed computational workloads, not traffic or field tests. At 5.0 m/s the sufficient structural guard rejects every pair and the path evaluator provides all fallback costs.

The post-hoc diagnostic uses follow-up validation labels 95000–95009 without training or model selection. Minimum raw ETA improves on that DQN in eight cells but remains worse than insertion in all nine. Selection agreement and Q gaps are dependent trajectory summaries; they do not identify reward noise or partial observation as the causal explanation.
