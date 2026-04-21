# ✅ E120_running_to_run_prepare_same_cycle_resets_next_cycle

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Apply `RUN_PREPARE` during running
- **Primary checks:** reset effect happens on next registered cycle
- **Contract anchor:** control/reset edge

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
| ✅ | observed_txn | `0` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/E120_running_to_run_prepare_same_cycle_resets_next_cycle_after_s1.log`](../../uvm/logs/E120_running_to_run_prepare_same_cycle_resets_next_cycle_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E120_running_to_run_prepare_same_cycle_resets_next_cycle_s1.ucdb`](../../uvm/cov_after/E120_running_to_run_prepare_same_cycle_resets_next_cycle_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 69.76 | n/a | 0.00 | 95.19 | n/a |
| branch | 44.67 | n/a | 0.00 | 89.33 | n/a |
| cond | 37.50 | n/a | 0.00 | 70.00 | n/a |
| expr | 81.48 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 10.27 | n/a | 0.00 | 47.71 | n/a |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
