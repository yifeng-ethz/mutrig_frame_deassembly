# ✅ E001_ctrl_state_update_is_one_cycle_late_from_valid

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Apply a legal control word on one cycle and sample outputs immediately
- **Primary checks:** `run_state_cmd` updates after the clock edge, but decoded enable effect appears on the following cycle
- **Contract anchor:** sequential control process ordering

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
| ℹ️ | log | [`uvm/logs/E001_ctrl_state_update_is_one_cycle_late_from_valid_after_s1.log`](../../uvm/logs/E001_ctrl_state_update_is_one_cycle_late_from_valid_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E001_ctrl_state_update_is_one_cycle_late_from_valid_s1.ucdb`](../../uvm/cov_after/E001_ctrl_state_update_is_one_cycle_late_from_valid_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.44 | n/a | 81.44 | 81.44 | n/a |
| branch | 65.33 | n/a | 65.33 | 65.33 | n/a |
| cond | 45.00 | n/a | 45.00 | 45.00 | n/a |
| expr | 98.77 | n/a | 98.77 | 98.77 | n/a |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 12.94 | n/a | 12.94 | 12.94 | n/a |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
