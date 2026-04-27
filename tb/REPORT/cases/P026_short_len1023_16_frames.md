# ✅ P026_short_len1023_16_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 16 max-length short frames
- **Primary checks:** parser survives full 10-bit short length range
- **Contract anchor:** short max size boundary

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
| ✅ | observed_txn | `2` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P026_short_len1023_16_frames_after_s1.log`](../../uvm/logs/P026_short_len1023_16_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P026_short_len1023_16_frames_s1.ucdb`](../../uvm/cov_after/P026_short_len1023_16_frames_s1.ucdb) |
| ℹ️ | log.headers | `2` |
| ℹ️ | log.hits | `2046` |
| ℹ️ | log.real_eops | `2` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 84.88 | 42.44 | 0.00 | 89.35 | 0.00 |
| branch | 74.67 | 37.33 | 0.00 | 79.33 | 0.00 |
| cond | 47.50 | 23.75 | 0.00 | 55.00 | 0.00 |
| expr | 98.77 | 49.38 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 37.07 | 18.54 | 0.00 | 49.46 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
