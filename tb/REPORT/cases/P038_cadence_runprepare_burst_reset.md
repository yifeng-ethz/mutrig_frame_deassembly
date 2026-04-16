# ✅ P038_cadence_runprepare_burst_reset

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** frequent `RUN_PREPARE` pulses during long regressions
- **Primary checks:** parser and counters always reinitialize cleanly
- **Contract anchor:** run-prepare reset

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
| ✅ | observed_txn | `1` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P038_cadence_runprepare_burst_reset_after_s1.log`](../../uvm/logs/P038_cadence_runprepare_burst_reset_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P038_cadence_runprepare_burst_reset_s1.ucdb`](../../uvm/cov_after/P038_cadence_runprepare_burst_reset_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 79.77 | 79.77 | 0.00 | 91.83 | 0.00 |
| branch | 63.58 | 63.58 | 0.00 | 83.44 | 0.00 |
| cond | 48.28 | 48.28 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 66.67 | 0.00 |
| toggle | 17.21 | 17.21 | 0.52 | 54.38 | 0.52 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
