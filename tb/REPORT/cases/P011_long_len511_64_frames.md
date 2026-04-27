# ✅ P011_long_len511_64_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 64 511-hit long frames
- **Primary checks:** sustained large-frame drain remains clean
- **Contract anchor:** long large size

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
| ℹ️ | log | [`uvm/logs/P011_long_len511_64_frames_after_s1.log`](../../uvm/logs/P011_long_len511_64_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P011_long_len511_64_frames_s1.ucdb`](../../uvm/cov_after/P011_long_len511_64_frames_s1.ucdb) |
| ℹ️ | log.headers | `2` |
| ℹ️ | log.hits | `1022` |
| ℹ️ | log.real_eops | `2` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.10 | 40.55 | 0.00 | 81.79 | 0.00 |
| branch | 67.33 | 33.67 | 0.00 | 68.00 | 0.00 |
| cond | 47.50 | 23.75 | 2.50 | 52.50 | 1.25 |
| expr | 98.77 | 49.38 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 43.94 | 21.97 | 0.83 | 47.38 | 0.41 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
