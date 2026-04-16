# ✅ P022_short_len127_256_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 256 127-hit short frames
- **Primary checks:** no short-packer deadlock
- **Contract anchor:** short large size

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
| ✅ | observed_txn | `9` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P022_short_len127_256_frames_after_s1.log`](../../uvm/logs/P022_short_len127_256_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P022_short_len127_256_frames_s1.ucdb`](../../uvm/cov_after/P022_short_len127_256_frames_s1.ucdb) |
| ℹ️ | log.headers | `9` |
| ℹ️ | log.hits | `1143` |
| ℹ️ | log.real_eops | `9` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 83.66 | 9.30 | 0.00 | 88.72 | 0.00 |
| branch | 72.85 | 8.09 | 0.00 | 77.48 | 0.00 |
| cond | 55.17 | 6.13 | 0.00 | 65.52 | 0.00 |
| expr | 98.77 | 10.97 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 11.11 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 53.33 | 5.93 | 0.00 | 66.67 | 0.00 |
| toggle | 39.68 | 4.41 | 0.00 | 52.97 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
