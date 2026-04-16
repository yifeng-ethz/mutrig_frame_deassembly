# ✅ E002_running_command_same_cycle_header_not_accepted

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Present `RUNNING` command and header on the same edge from a blocked state
- **Primary checks:** header is not accepted until the next cycle
- **Contract anchor:** one-cycle decode/effect lag

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
| ℹ️ | log | [`uvm/logs/E002_running_command_same_cycle_header_not_accepted_after_s1.log`](../../uvm/logs/E002_running_command_same_cycle_header_not_accepted_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E002_running_command_same_cycle_header_not_accepted_s1.ucdb`](../../uvm/cov_after/E002_running_command_same_cycle_header_not_accepted_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 71.21 | 71.21 | 0.00 | 79.77 | 0.00 |
| branch | 52.98 | 52.98 | 0.00 | 63.58 | 0.00 |
| cond | 51.72 | 51.72 | 3.45 | 51.72 | 3.45 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 6.67 | 46.67 | 6.67 |
| toggle | 14.81 | 14.81 | 0.63 | 16.53 | 0.63 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
