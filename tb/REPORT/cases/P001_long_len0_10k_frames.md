# ✅ P001_long_len0_10k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 10k zero-hit long frames
- **Primary checks:** no hit output, one metadata pulse per frame, no deadlock
- **Contract anchor:** long zero-hit steady state

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
| ✅ | observed_txn | `8` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P001_long_len0_10k_frames_after_s1.log`](../../uvm/logs/P001_long_len0_10k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P001_long_len0_10k_frames_s1.ucdb`](../../uvm/cov_after/P001_long_len0_10k_frames_s1.ucdb) |
| ℹ️ | log.headers | `8` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 75.26 | 9.41 | 75.26 | 75.26 | 9.41 |
| branch | 52.00 | 6.50 | 52.00 | 52.00 | 6.50 |
| cond | 40.00 | 5.00 | 40.00 | 40.00 | 5.00 |
| expr | 98.77 | 12.35 | 98.77 | 98.77 | 12.35 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 12.81 | 1.60 | 12.81 | 12.81 | 1.60 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
