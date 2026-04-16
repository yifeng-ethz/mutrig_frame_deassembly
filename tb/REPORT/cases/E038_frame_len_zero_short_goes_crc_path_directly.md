# ✅ E038_frame_len_zero_short_goes_crc_path_directly

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Short-mode frame length 0
- **Primary checks:** zero-hit short frame behaves like clean metadata-only frame
- **Contract anchor:** short zero-length edge

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
| ℹ️ | log | [`uvm/logs/E038_frame_len_zero_short_goes_crc_path_directly_after_s1.log`](../../uvm/logs/E038_frame_len_zero_short_goes_crc_path_directly_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E038_frame_len_zero_short_goes_crc_path_directly_s1.ucdb`](../../uvm/cov_after/E038_frame_len_zero_short_goes_crc_path_directly_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 80.16 | 4.28 | 89.49 | 4.28 |
| branch | 66.23 | 66.23 | 5.92 | 80.92 | 5.92 |
| cond | 48.28 | 48.28 | 3.45 | 79.31 | 3.45 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 14.29 | 100.00 | 14.29 |
| fsm_trans | 46.67 | 46.67 | 13.33 | 60.00 | 13.33 |
| toggle | 17.62 | 17.62 | 0.78 | 49.90 | 0.78 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
