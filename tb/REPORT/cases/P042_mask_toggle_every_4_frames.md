# ✅ P042_mask_toggle_every_4_frames

**Bucket:** `PROF` &nbsp; **Method:** `S` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** flip mask every 4 frames
- **Primary checks:** no counter drift across windows
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
| ✅ | observed_txn | `4` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P042_mask_toggle_every_4_frames_after_s1.log`](../../uvm/logs/P042_mask_toggle_every_4_frames_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P042_mask_toggle_every_4_frames_s1.ucdb`](../../uvm/cov_after/P042_mask_toggle_every_4_frames_s1.ucdb) |
| ℹ️ | log.headers | `4` |
| ℹ️ | log.hits | `4` |
| ℹ️ | log.real_eops | `4` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.79 | 20.45 | 0.00 | 92.44 | 0.00 |
| branch | 66.00 | 16.50 | 0.00 | 84.67 | 0.00 |
| cond | 47.50 | 11.88 | 0.00 | 60.00 | 0.00 |
| expr | 98.77 | 24.69 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 30.04 | 7.51 | 0.00 | 50.54 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
