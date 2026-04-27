# ✅ X004_reset_in_long_unpack

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert reset during long-hit unpack
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
| ℹ️ | log | [`uvm/logs/X004_reset_in_long_unpack_after_s1.log`](../../uvm/logs/X004_reset_in_long_unpack_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X004_reset_in_long_unpack_s1.ucdb`](../../uvm/cov_after/X004_reset_in_long_unpack_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.88 | 73.88 | 1.72 | 73.88 | 1.72 |
| branch | 54.00 | 54.00 | 6.67 | 54.00 | 6.67 |
| cond | 42.50 | 42.50 | 5.00 | 42.50 | 5.00 |
| expr | 88.89 | 88.89 | 0.00 | 92.59 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 11.95 | 11.95 | 1.88 | 13.24 | 1.88 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
