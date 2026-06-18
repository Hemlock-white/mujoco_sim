# MPC Loop Latency - Suite Report

Auto-generated from `logs/suite` (pooled across repeats per preset).

## 1. Config legend

| preset | configuration | #runs |
|---|---|---|
| 1 | 2-proc, Write ON, stand->move | 10 |
| 2 | 2-proc, Write OFF, stand->move | 10 |
| 3 | 2-proc, Write ON, stand-only | 10 |
| normal | 2-proc, Write ON, stand->move | 10 |
| 4 | standalone, Write OFF | 10 |
| 5 | standalone, Write ON, gc.disable | 10 |
| 6 | standalone, Write ON | 10 |

## 2. MPC loop dt - pooled statistics (ms)

| preset | samples | mean | median | p99 | p99.9 | max | %>10ms | %>20ms | %>50ms |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 211150 | 5.68 | 5.35 | 9.60 | 11.40 | 290.6 | 0.58 | 0.005 | 0.0024 |
| 2 | 232935 | 5.15 | 5.14 | 6.39 | 6.93 | 164.7 | 0.00 | 0.002 | 0.0013 |
| 3 | 232876 | 5.15 | 5.12 | 6.19 | 6.69 | 143.6 | 0.00 | 0.001 | 0.0004 |
| normal | 232983 | 5.15 | 5.14 | 6.37 | 6.93 | 104.3 | 0.00 | 0.001 | 0.0004 |
| 4 | 234283 | 5.12 | 5.12 | 6.13 | 6.43 | 86.3 | 0.01 | 0.002 | 0.0009 |
| 5 | 233785 | 5.13 | 5.14 | 6.13 | 6.43 | 35.6 | 0.01 | 0.001 | 0.0000 |
| 6 | 233782 | 5.13 | 5.14 | 6.13 | 6.42 | 39.0 | 0.00 | 0.000 | 0.0000 |

## 3. Spike (>50ms) rate

| preset | total spikes | total minutes | spikes / min | per-run counts |
|---|---|---|---|---|
| 1 | 5 | 20.0 | 0.250 | [0, 0, 2, 0, 1, 1, 0, 0, 0, 1] |
| 2 | 3 | 20.0 | 0.150 | [0, 0, 0, 0, 1, 1, 1, 0, 0, 0] |
| 3 | 1 | 20.0 | 0.050 | [1, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| normal | 1 | 20.0 | 0.050 | [0, 0, 0, 0, 0, 0, 1, 0, 0, 0] |
| 4 | 2 | 20.0 | 0.100 | [1, 0, 0, 1, 0, 0, 0, 0, 0, 0] |
| 5 | 0 | 20.0 | 0.000 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |
| 6 | 0 | 20.0 | 0.000 | [0, 0, 0, 0, 0, 0, 0, 0, 0, 0] |

## 4. Publish dt - pooled (ms)

| preset | mean | p99.9 | max | %>50ms |
|---|---|---|---|---|
| 1 | 5.68 | 11.42 | 294.2 | 0.0024 |
| 2 | 5.15 | 7.27 | 164.6 | 0.0013 |
| 3 | 5.15 | 6.99 | 143.5 | 0.0004 |
| normal | 5.15 | 7.27 | 106.6 | 0.0004 |
| 4 | 5.12 | 6.45 | 88.1 | 0.0009 |
| 5 | 5.13 | 6.45 | 38.4 | 0.0000 |
| 6 | 5.13 | 6.45 | 39.0 | 0.0000 |

## 5. Bridge lowcmd_age_ms (2-proc only)

| preset | mean | median | max |
|---|---|---|---|
| 1 | (standalone / n/a) |
| 2 | (standalone / n/a) |
| 3 | (standalone / n/a) |
| normal | (standalone / n/a) |
| 4 | (standalone / n/a) |
| 5 | (standalone / n/a) |
| 6 | (standalone / n/a) |

## 6. Auto-derived observations

- Spike rate with DDS Write ON = 0.070/min vs OFF = 0.125/min -> DDS Write is NOT the dominant cause.
- Jitter floor (%>10ms): 2-proc avg = 0.15% vs standalone avg = 0.01% -> the second process (MuJoCo sim) is the main driver of sub-spike jitter.

> Fill in narrative conclusions below after reviewing the tables.

## 7. Conclusions (manual)

- TODO
