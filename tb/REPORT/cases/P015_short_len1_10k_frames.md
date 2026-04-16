# ✅ P015_short_len1_10k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 10k one-hit short frames
- **Primary checks:** one-hit `sop/eop` remains stable
- **Contract anchor:** short min nonzero steady state

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
| ℹ️ | log | [`uvm/logs/P015_short_len1_10k_frames_after_s1.log`](../../uvm/logs/P015_short_len1_10k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P015_short_len1_10k_frames_s1.ucdb`](../../uvm/cov_after/P015_short_len1_10k_frames_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `64` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.54 | 1.26 | 3.50 | 84.44 | 0.05 |
| branch | 66.89 | 1.05 | 4.64 | 72.19 | 0.07 |
| cond | 55.17 | 0.86 | 3.45 | 65.52 | 0.05 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 1.56 | 14.29 | 100.00 | 0.22 |
| fsm_trans | 46.67 | 0.73 | 13.33 | 60.00 | 0.21 |
| toggle | 37.38 | 0.58 | 1.15 | 51.46 | 0.02 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
