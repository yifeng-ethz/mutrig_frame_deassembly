# ✅ P014_short_len0_10k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 10k zero-hit short frames
- **Primary checks:** metadata-only path stays stable
- **Contract anchor:** short zero-hit steady state

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
| ℹ️ | log | [`uvm/logs/P014_short_len0_10k_frames_after_s1.log`](../../uvm/logs/P014_short_len0_10k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P014_short_len0_10k_frames_s1.ucdb`](../../uvm/cov_after/P014_short_len0_10k_frames_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 74.32 | 1.16 | 0.78 | 80.93 | 0.01 |
| branch | 52.98 | 0.83 | 1.32 | 67.55 | 0.02 |
| cond | 48.28 | 0.75 | 0.00 | 62.07 | 0.00 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 71.43 | 1.12 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 33.33 | 0.52 | 0.00 | 46.67 | 0.00 |
| toggle | 20.07 | 0.31 | 0.47 | 50.31 | 0.01 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
