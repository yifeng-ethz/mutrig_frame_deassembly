# ✅ P006_long_len15_2k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 2k fifteen-hit long frames
- **Primary checks:** counters and metadata remain aligned
- **Contract anchor:** long medium size

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
| ✅ | observed_txn | `42` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P006_long_len15_2k_frames_after_s1.log`](../../uvm/logs/P006_long_len15_2k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P006_long_len15_2k_frames_s1.ucdb`](../../uvm/cov_after/P006_long_len15_2k_frames_s1.ucdb) |
| ℹ️ | log.headers | `42` |
| ℹ️ | log.hits | `630` |
| ℹ️ | log.real_eops | `42` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 1.91 | 0.00 | 80.16 | 0.00 |
| branch | 66.23 | 1.58 | 0.00 | 66.23 | 0.00 |
| cond | 58.62 | 1.40 | 0.00 | 58.62 | 0.00 |
| expr | 98.77 | 2.35 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 2.04 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 40.00 | 0.95 | 0.00 | 46.67 | 0.00 |
| toggle | 44.73 | 1.07 | 0.63 | 45.93 | 0.01 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
