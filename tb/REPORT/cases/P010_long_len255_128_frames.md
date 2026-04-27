# ✅ P010_long_len255_128_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 128 255-hit long frames
- **Primary checks:** no counter rollover or timeout
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
| ✅ | observed_txn | `5` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P010_long_len255_128_frames_after_s1.log`](../../uvm/logs/P010_long_len255_128_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P010_long_len255_128_frames_s1.ucdb`](../../uvm/cov_after/P010_long_len255_128_frames_s1.ucdb) |
| ℹ️ | log.headers | `5` |
| ℹ️ | log.hits | `1275` |
| ℹ️ | log.real_eops | `5` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.79 | 16.36 | 0.00 | 81.79 | 0.00 |
| branch | 68.00 | 13.60 | 0.00 | 68.00 | 0.00 |
| cond | 50.00 | 10.00 | 0.00 | 50.00 | 0.00 |
| expr | 98.77 | 19.75 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 45.06 | 9.01 | 0.79 | 46.55 | 0.16 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
