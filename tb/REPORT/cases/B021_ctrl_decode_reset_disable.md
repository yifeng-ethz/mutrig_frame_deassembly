# ✅ B021_ctrl_decode_reset_disable

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Send `RESET` word
- **Primary checks:** parser stays blocked
- **Contract anchor:** non-running state decode

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
| ✅ | observed_txn | `1` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/B021_ctrl_decode_reset_disable_after_s1.log`](../../uvm/logs/B021_ctrl_decode_reset_disable_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B021_ctrl_decode_reset_disable_s1.ucdb`](../../uvm/cov_after/B021_ctrl_decode_reset_disable_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 79.77 | 79.77 | 0.00 | 86.38 | 0.00 |
| branch | 63.58 | 63.58 | 0.00 | 74.83 | 0.00 |
| cond | 48.28 | 48.28 | 0.00 | 68.97 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 46.67 | 0.00 |
| toggle | 17.73 | 17.73 | 0.63 | 20.33 | 0.63 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
