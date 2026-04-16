# ✅ E004_terminating_command_same_cycle_open_frame_continues

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Apply `TERMINATING` exactly as a frame is already open
- **Primary checks:** current frame is not aborted
- **Contract anchor:** open-frame drain path

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
| ℹ️ | log | [`uvm/logs/E004_terminating_command_same_cycle_open_frame_continues_after_s1.log`](../../uvm/logs/E004_terminating_command_same_cycle_open_frame_continues_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E004_terminating_command_same_cycle_open_frame_continues_s1.ucdb`](../../uvm/cov_after/E004_terminating_command_same_cycle_open_frame_continues_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.93 | 73.93 | 2.72 | 82.49 | 2.72 |
| branch | 58.28 | 58.28 | 5.30 | 68.87 | 5.30 |
| cond | 65.52 | 65.52 | 13.79 | 65.52 | 13.79 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 0.00 | 46.67 | 0.00 |
| toggle | 15.75 | 15.75 | 0.83 | 17.36 | 0.83 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
