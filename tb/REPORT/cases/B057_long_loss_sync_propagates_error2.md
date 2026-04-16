# ✅ B057_long_loss_sync_propagates_error2

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert loss-sync during a long frame
- **Primary checks:** every emitted hit shows `error(2)=1` while active
- **Contract anchor:** lane-corrupt propagation

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
| ℹ️ | log | [`uvm/logs/B057_long_loss_sync_propagates_error2_after_s1.log`](../../uvm/logs/B057_long_loss_sync_propagates_error2_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B057_long_loss_sync_propagates_error2_s1.ucdb`](../../uvm/cov_after/B057_long_loss_sync_propagates_error2_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 71.21 | 71.21 | 0.00 | 89.11 | 0.00 |
| branch | 52.98 | 52.98 | 0.00 | 81.46 | 0.00 |
| cond | 51.72 | 51.72 | 0.00 | 75.86 | 0.00 |
| expr | 92.59 | 92.59 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 0.00 | 60.00 | 0.00 |
| toggle | 15.95 | 15.95 | 0.21 | 28.73 | 0.21 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
