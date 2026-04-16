# ✅ X001_reset_in_fs_idle

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert reset while parser idle
- **Primary checks:** outputs and counters clear immediately, no hang
- **Contract anchor:** reset behavior

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
| ℹ️ | log | [`uvm/logs/X001_reset_in_fs_idle_after_s1.log`](../../uvm/logs/X001_reset_in_fs_idle_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X001_reset_in_fs_idle_s1.ucdb`](../../uvm/cov_after/X001_reset_in_fs_idle_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 79.77 | 79.77 | 79.77 | 79.77 | 79.77 |
| branch | 63.58 | 63.58 | 63.58 | 63.58 | 63.58 |
| cond | 48.28 | 48.28 | 48.28 | 48.28 | 48.28 |
| expr | 98.77 | 98.77 | 98.77 | 98.77 | 98.77 |
| fsm_state | 85.71 | 85.71 | 85.71 | 85.71 | 85.71 |
| fsm_trans | 40.00 | 40.00 | 40.00 | 40.00 | 40.00 |
| toggle | 16.84 | 16.84 | 16.84 | 16.84 | 16.84 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
