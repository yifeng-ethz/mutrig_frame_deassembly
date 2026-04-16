# ✅ P044_mask_toggle_every_16_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** flip mask every 16 frames
- **Primary checks:** long-running mask control stable
- **Contract anchor:** CSR gating

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
| ℹ️ | log | [`uvm/logs/P044_mask_toggle_every_16_frames_after_s1.log`](../../uvm/logs/P044_mask_toggle_every_16_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P044_mask_toggle_every_16_frames_s1.ucdb`](../../uvm/cov_after/P044_mask_toggle_every_16_frames_s1.ucdb) |
| ℹ️ | log.headers | `8` |
| ℹ️ | log.hits | `8` |
| ℹ️ | log.real_eops | `8` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 10.02 | 0.00 | 91.83 | 0.00 |
| branch | 64.24 | 8.03 | 0.00 | 83.44 | 0.00 |
| cond | 58.62 | 7.33 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 12.35 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 10.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 5.00 | 0.00 | 73.33 | 0.00 |
| toggle | 39.94 | 4.99 | 0.00 | 54.38 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
