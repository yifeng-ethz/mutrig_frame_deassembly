# ✅ X126_hw_tcl_maxchannel_mismatch_negative

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Use stale maxChannel assumption after width rebuild
- **Primary checks:** negative check demonstrates `myelaborate` owns maxChannel
- **Contract anchor:** packaging contract

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
| ℹ️ | log | [`uvm/logs/X126_hw_tcl_maxchannel_mismatch_negative_after_s1.log`](../../uvm/logs/X126_hw_tcl_maxchannel_mismatch_negative_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X126_hw_tcl_maxchannel_mismatch_negative_s1.ucdb`](../../uvm/cov_after/X126_hw_tcl_maxchannel_mismatch_negative_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1023` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 79.38 | 79.38 | 0.00 | 96.11 | 0.00 |
| branch | 65.56 | 65.56 | 0.00 | 90.13 | 0.00 |
| cond | 51.72 | 51.72 | 3.45 | 89.66 | 3.45 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 80.00 | 0.00 |
| toggle | 42.39 | 42.39 | 3.07 | 55.07 | 3.07 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
