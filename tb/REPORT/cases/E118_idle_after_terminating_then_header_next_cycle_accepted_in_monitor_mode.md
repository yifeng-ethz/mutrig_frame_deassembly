# ✅ E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `TERMINATING -> IDLE`, then new header
- **Primary checks:** monitor mode reopens parser on next header
- **Contract anchor:** `IDLE` force-go recovery

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
| ℹ️ | log | [`uvm/logs/E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode_after_s1.log`](../../uvm/logs/E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode_s1.ucdb`](../../uvm/cov_after/E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 74.32 | 74.32 | 0.39 | 96.50 | 0.39 |
| branch | 58.94 | 58.94 | 0.66 | 90.79 | 0.66 |
| cond | 65.52 | 65.52 | 0.00 | 86.21 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 0.00 | 66.67 | 0.00 |
| toggle | 16.21 | 16.21 | 0.00 | 53.74 | 0.00 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
