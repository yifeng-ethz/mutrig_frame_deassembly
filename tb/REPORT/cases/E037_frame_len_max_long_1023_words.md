# ✅ E037_frame_len_max_long_1023_words

**Bucket:** `EDGE` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Long-mode frame length `1023`
- **Primary checks:** parser completes without counter overflow
- **Contract anchor:** 10-bit frame length boundary

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
| ℹ️ | log | [`uvm/logs/E037_frame_len_max_long_1023_words_after_s1.log`](../../uvm/logs/E037_frame_len_max_long_1023_words_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E037_frame_len_max_long_1023_words_s1.ucdb`](../../uvm/cov_after/E037_frame_len_max_long_1023_words_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1023` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.10 | 81.10 | 0.34 | 85.91 | 0.34 |
| branch | 67.33 | 67.33 | 2.67 | 74.67 | 2.67 |
| cond | 47.50 | 47.50 | 7.50 | 67.50 | 7.50 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 40.41 | 40.41 | 26.58 | 44.70 | 26.58 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
