# ✅ E078_loss_sync_only_on_last_byte_still_sets_error2

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert loss-sync only at end of frame
- **Primary checks:** terminal hit still reports `error(2)=1`
- **Contract anchor:** lane error observation edge

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
| ℹ️ | log | [`uvm/logs/E078_loss_sync_only_on_last_byte_still_sets_error2_after_s1.log`](../../uvm/logs/E078_loss_sync_only_on_last_byte_still_sets_error2_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E078_loss_sync_only_on_last_byte_still_sets_error2_s1.ucdb`](../../uvm/cov_after/E078_loss_sync_only_on_last_byte_still_sets_error2_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.88 | n/a | 0.00 | 93.13 | n/a |
| branch | 54.00 | n/a | 0.00 | 85.33 | n/a |
| cond | 47.50 | n/a | 0.00 | 65.00 | n/a |
| expr | 98.77 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 12.88 | n/a | 0.00 | 46.78 | n/a |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
