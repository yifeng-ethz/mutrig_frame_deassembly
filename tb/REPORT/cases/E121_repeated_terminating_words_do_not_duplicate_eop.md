# ✅ E121_repeated_terminating_words_do_not_duplicate_eop

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Hold `TERMINATING` high-equivalent word across multiple cycles
- **Primary checks:** no duplicate `eop` or duplicated frame completion
- **Contract anchor:** drain idempotence

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
| ✅ | observed_txn | `32` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/E121_repeated_terminating_words_do_not_duplicate_eop_after_s1.log`](../../uvm/logs/E121_repeated_terminating_words_do_not_duplicate_eop_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E121_repeated_terminating_words_do_not_duplicate_eop_s1.ucdb`](../../uvm/cov_after/E121_repeated_terminating_words_do_not_duplicate_eop_s1.ucdb) |
| ℹ️ | log.headers | `32` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.93 | 2.31 | 0.00 | 96.50 | 0.00 |
| branch | 58.28 | 1.82 | 0.00 | 90.79 | 0.00 |
| cond | 68.97 | 2.16 | 3.45 | 89.66 | 0.11 |
| expr | 98.77 | 3.09 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 1.79 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 26.67 | 0.83 | 0.00 | 66.67 | 0.00 |
| toggle | 23.04 | 0.72 | 0.00 | 53.89 | 0.00 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
