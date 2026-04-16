# ✅ P012_long_len767_32_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 32 767-hit long frames
- **Primary checks:** no late CRC or word-count corruption
- **Contract anchor:** long near-max size

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
| ℹ️ | log | [`uvm/logs/P012_long_len767_32_frames_after_s1.log`](../../uvm/logs/P012_long_len767_32_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P012_long_len767_32_frames_s1.ucdb`](../../uvm/cov_after/P012_long_len767_32_frames_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `767` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 79.38 | 79.38 | 0.00 | 80.16 | 0.00 |
| branch | 65.56 | 65.56 | 0.00 | 66.23 | 0.00 |
| cond | 51.72 | 51.72 | 0.00 | 62.07 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 46.67 | 0.00 |
| toggle | 41.87 | 41.87 | 0.31 | 49.84 | 0.31 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
