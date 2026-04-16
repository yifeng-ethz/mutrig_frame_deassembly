# ✅ X005_reset_in_short_unpack

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert reset during short-hit unpack
- **Primary checks:** partial hit is dropped, no false valid
- **Contract anchor:** parser reset

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
| ℹ️ | log | [`uvm/logs/X005_reset_in_short_unpack_after_s1.log`](../../uvm/logs/X005_reset_in_short_unpack_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X005_reset_in_short_unpack_s1.ucdb`](../../uvm/cov_after/X005_reset_in_short_unpack_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 80.16 | 4.28 | 84.05 | 4.28 |
| branch | 66.23 | 66.23 | 5.96 | 69.54 | 5.96 |
| cond | 48.28 | 48.28 | 3.45 | 51.72 | 3.45 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 14.29 | 100.00 | 14.29 |
| fsm_trans | 46.67 | 46.67 | 13.33 | 53.33 | 13.33 |
| toggle | 19.03 | 19.03 | 1.62 | 19.34 | 1.62 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
