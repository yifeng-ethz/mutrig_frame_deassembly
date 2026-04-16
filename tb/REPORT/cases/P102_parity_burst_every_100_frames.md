# ✅ P102_parity_burst_every_100_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** inject parity error burst every 100th frame
- **Primary checks:** error0 appears only on affected hits
- **Contract anchor:** parity burst stress

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
| ✅ | observed_txn | `50` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P102_parity_burst_every_100_frames_after_s1.log`](../../uvm/logs/P102_parity_burst_every_100_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P102_parity_burst_every_100_frames_s1.ucdb`](../../uvm/cov_after/P102_parity_burst_every_100_frames_s1.ucdb) |
| ℹ️ | log.headers | `50` |
| ℹ️ | log.hits | `50` |
| ℹ️ | log.real_eops | `50` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.54 | 1.61 | 0.39 | 95.33 | 0.01 |
| branch | 64.90 | 1.30 | 0.66 | 89.47 | 0.01 |
| cond | 62.07 | 1.24 | 3.45 | 82.76 | 0.07 |
| expr | 98.77 | 1.98 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 1.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 0.80 | 0.00 | 73.33 | 0.00 |
| toggle | 43.33 | 0.87 | 0.20 | 61.67 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
