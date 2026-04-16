# ✅ X045_short_declared_len2_halfword_only

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Declare two short hits, stop after incomplete even/odd pair
- **Primary checks:** no false extra hit
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
| ℹ️ | log | [`uvm/logs/X045_short_declared_len2_halfword_only_after_s1.log`](../../uvm/logs/X045_short_declared_len2_halfword_only_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X045_short_declared_len2_halfword_only_s1.ucdb`](../../uvm/cov_after/X045_short_declared_len2_halfword_only_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `2` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 82.49 | 82.49 | 3.50 | 91.83 | 3.50 |
| branch | 71.52 | 71.52 | 4.61 | 83.55 | 4.61 |
| cond | 48.28 | 48.28 | 0.00 | 75.86 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 53.33 | 53.33 | 6.67 | 73.33 | 6.67 |
| toggle | 22.94 | 22.94 | 0.63 | 49.32 | 0.63 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
