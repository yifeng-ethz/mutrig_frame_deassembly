# ✅ X099_csr_mask_low_then_illegal_ctrl

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Mask off parser and then send illegal control words
- **Primary checks:** illegal control handling still works
- **Contract anchor:** independent control planes

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
| ✅ | observed_txn | `0` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/X099_csr_mask_low_then_illegal_ctrl_after_s1.log`](../../uvm/logs/X099_csr_mask_low_then_illegal_ctrl_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X099_csr_mask_low_then_illegal_ctrl_s1.ucdb`](../../uvm/cov_after/X099_csr_mask_low_then_illegal_ctrl_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 63.81 | n/a | 1.17 | 95.33 | n/a |
| branch | 42.38 | n/a | 1.97 | 89.47 | n/a |
| cond | 37.93 | n/a | 0.00 | 86.21 | n/a |
| expr | 46.91 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | 14.29 | n/a | 0.00 | 100.00 | n/a |
| fsm_trans | 0.00 | n/a | 0.00 | 80.00 | n/a |
| toggle | 7.04 | n/a | 0.16 | 51.82 | n/a |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
