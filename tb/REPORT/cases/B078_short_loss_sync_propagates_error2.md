# ✅ B078_short_loss_sync_propagates_error2

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert loss-sync during short frame
- **Primary checks:** emitted hits carry `error(2)=1`
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
| ℹ️ | log | [`uvm/logs/B078_short_loss_sync_propagates_error2_after_s1.log`](../../uvm/logs/B078_short_loss_sync_propagates_error2_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B078_short_loss_sync_propagates_error2_s1.ucdb`](../../uvm/cov_after/B078_short_loss_sync_propagates_error2_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 74.23 | 74.23 | 0.00 | 97.94 | 0.00 |
| branch | 56.00 | 56.00 | 0.00 | 94.00 | 0.00 |
| cond | 45.00 | 45.00 | 0.00 | 77.50 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 14.16 | 14.16 | 0.23 | 38.07 | 0.23 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
