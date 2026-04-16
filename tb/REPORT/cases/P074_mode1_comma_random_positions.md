# ✅ P074_mode1_comma_random_positions

**Bucket:** `PROF` &nbsp; **Method:** `R` &nbsp; **Build:** `CFG_B` &nbsp; **Effort:** `extensive` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `MODE_HALT=1`, random comma positions in 10k frames
- **Primary checks:** parser never deadlocks
- **Contract anchor:** mode1 hold

## Execution Evidence

<!-- fields (chief-architect legend)
  status                       = this case's overall health (legend: ✅ pass / ⚠️ partial / ❌ fail / ❓ pending)
  method                       = D = directed (1 txn); R = randomised (N txns)
  observed_txn                 = number of scoreboard-observed transactions driven by this case
  standalone_coverage          = code coverage measured from this case's own isolated UCDB
  isolated_cov_per_txn         = standalone_coverage averaged over observed_txn (useful for random cases)
  bucket_gain_by_case          = incremental code coverage this case added to the bucket's ordered merge
  bucket_merged_total_after    = the bucket's merged code coverage after this case was merged
  bucket_gain_per_txn          = bucket_gain_by_case averaged over observed_txn
-->

| status | field | value |
|:---:|---|---|
| ✅ | observed_txn | `25000` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P074_mode1_comma_random_positions_after_s1.log`](../../uvm/logs/P074_mode1_comma_random_positions_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P074_mode1_comma_random_positions_s1.ucdb`](../../uvm/cov_after/P074_mode1_comma_random_positions_s1.ucdb) |
| ℹ️ | log.headers | `25000` |
| ℹ️ | log.hits | `25000` |
| ℹ️ | log.real_eops | `25000` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.08 | 0.00 | 0.00 | 94.55 | 0.00 |
| branch | 64.24 | 0.00 | 0.00 | 87.50 | 0.00 |
| cond | 58.62 | 0.00 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 0.00 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 0.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 0.00 | 0.00 | 73.33 | 0.00 |
| toggle | 44.89 | 0.00 | 4.17 | 60.27 | 0.00 |

## Transaction Growth (checkpoint UCDBs)

ℹ️ See [`../txn_growth/P074_mode1_comma_random_positions.md`](../txn_growth/P074_mode1_comma_random_positions.md) for the log-spaced coverage curve.

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
