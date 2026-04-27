# ✅ E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `MODE_HALT=1`, inject comma and then header-like byte
- **Primary checks:** parser continues old frame semantics, not new frame start
- **Contract anchor:** mode1 hold edge

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
| ℹ️ | log | [`uvm/logs/E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header_after_s1.log`](../../uvm/logs/E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header_s1.ucdb`](../../uvm/cov_after/E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.88 | 73.88 | 0.00 | 85.57 | 0.00 |
| branch | 54.00 | 54.00 | 0.00 | 72.00 | 0.00 |
| cond | 47.50 | 47.50 | 0.00 | 57.50 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 12.61 | 12.61 | 0.00 | 17.83 | 0.00 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
