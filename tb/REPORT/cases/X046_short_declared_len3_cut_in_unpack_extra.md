# ✅ X046_short_declared_len3_cut_in_unpack_extra

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Stop in `FS_UNPACK_EXTRA`
- **Primary checks:** saved nibble does not leak into next frame
- **Contract anchor:** truncation

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
| ℹ️ | log | [`uvm/logs/X046_short_declared_len3_cut_in_unpack_extra_after_s1.log`](../../uvm/logs/X046_short_declared_len3_cut_in_unpack_extra_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X046_short_declared_len3_cut_in_unpack_extra_s1.ucdb`](../../uvm/cov_after/X046_short_declared_len3_cut_in_unpack_extra_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `3` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 83.27 | 83.27 | 0.78 | 92.61 | 0.78 |
| branch | 72.19 | 72.19 | 0.66 | 84.21 | 0.66 |
| cond | 48.28 | 48.28 | 0.00 | 75.86 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 53.33 | 53.33 | 0.00 | 73.33 | 0.00 |
| toggle | 25.76 | 25.76 | 0.36 | 49.69 | 0.36 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
