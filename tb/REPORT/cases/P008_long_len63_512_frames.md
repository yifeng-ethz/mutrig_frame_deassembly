# ✅ P008_long_len63_512_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 512 sixty-three-hit long frames
- **Primary checks:** CRC and EOP remain aligned at scale
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
| ✅ | observed_txn | `11` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P008_long_len63_512_frames_after_s1.log`](../../uvm/logs/P008_long_len63_512_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P008_long_len63_512_frames_s1.ucdb`](../../uvm/cov_after/P008_long_len63_512_frames_s1.ucdb) |
| ℹ️ | log.headers | `11` |
| ℹ️ | log.hits | `693` |
| ℹ️ | log.real_eops | `11` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 7.29 | 0.00 | 80.16 | 0.00 |
| branch | 66.23 | 6.02 | 0.00 | 66.23 | 0.00 |
| cond | 58.62 | 5.33 | 0.00 | 58.62 | 0.00 |
| expr | 98.77 | 8.98 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 7.79 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 40.00 | 3.64 | 0.00 | 46.67 | 0.00 |
| toggle | 44.42 | 4.04 | 1.04 | 47.60 | 0.09 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
