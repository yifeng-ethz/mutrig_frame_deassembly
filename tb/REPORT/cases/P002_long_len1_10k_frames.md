# ✅ P002_long_len1_10k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 10k one-hit long frames
- **Primary checks:** stable `sop/eop` coincidence, counters remain coherent
- **Contract anchor:** long min nonzero steady state

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
| ✅ | observed_txn | `64` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P002_long_len1_10k_frames_after_s1.log`](../../uvm/logs/P002_long_len1_10k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P002_long_len1_10k_frames_s1.ucdb`](../../uvm/cov_after/P002_long_len1_10k_frames_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `64` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 1.25 | 7.39 | 80.16 | 0.12 |
| branch | 64.24 | 1.00 | 13.91 | 64.24 | 0.22 |
| cond | 58.62 | 0.92 | 13.79 | 58.62 | 0.22 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 1.34 | 14.29 | 85.71 | 0.22 |
| fsm_trans | 40.00 | 0.62 | 13.33 | 46.67 | 0.21 |
| toggle | 42.91 | 0.67 | 24.30 | 43.59 | 0.38 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
