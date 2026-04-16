# ✅ P005_long_len7_5k_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** 5k seven-hit long frames
- **Primary checks:** sustained unpack throughput is stable
- **Contract anchor:** long unpack cadence

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
| ℹ️ | log | [`uvm/logs/P005_long_len7_5k_frames_after_s1.log`](../../uvm/logs/P005_long_len7_5k_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P005_long_len7_5k_frames_s1.ucdb`](../../uvm/cov_after/P005_long_len7_5k_frames_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `448` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 1.25 | 0.00 | 80.16 | 0.00 |
| branch | 66.23 | 1.03 | 0.00 | 66.23 | 0.00 |
| cond | 58.62 | 0.92 | 0.00 | 58.62 | 0.00 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 1.34 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 40.00 | 0.62 | 0.00 | 46.67 | 0.00 |
| toggle | 45.15 | 0.71 | 0.63 | 45.31 | 0.01 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
