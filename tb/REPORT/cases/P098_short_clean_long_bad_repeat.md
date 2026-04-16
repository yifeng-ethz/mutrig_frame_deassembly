# ✅ P098_short_clean_long_bad_repeat

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** alternate clean short and bad long frames
- **Primary checks:** error1 isolated to bad long frames
- **Contract anchor:** mixed-mode error isolation

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
| ✅ | observed_txn | `16` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P098_short_clean_long_bad_repeat_after_s1.log`](../../uvm/logs/P098_short_clean_long_bad_repeat_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P098_short_clean_long_bad_repeat_s1.ucdb`](../../uvm/cov_after/P098_short_clean_long_bad_repeat_s1.ucdb) |
| ℹ️ | log.headers | `16` |
| ℹ️ | log.hits | `16` |
| ℹ️ | log.real_eops | `16` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.54 | 5.03 | 0.00 | 94.94 | 0.00 |
| branch | 66.89 | 4.18 | 0.00 | 88.82 | 0.00 |
| cond | 55.17 | 3.45 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 6.17 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 6.25 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 46.67 | 2.92 | 0.00 | 73.33 | 0.00 |
| toggle | 34.78 | 2.17 | 0.00 | 61.46 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
