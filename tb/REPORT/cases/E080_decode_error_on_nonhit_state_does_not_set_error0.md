# ✅ E080_decode_error_on_nonhit_state_does_not_set_error0

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert decode error during CRC bytes only
- **Primary checks:** no earlier hit is falsely marked
- **Contract anchor:** hit-error state gating

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
| ℹ️ | log | [`uvm/logs/E080_decode_error_on_nonhit_state_does_not_set_error0_after_s1.log`](../../uvm/logs/E080_decode_error_on_nonhit_state_does_not_set_error0_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E080_decode_error_on_nonhit_state_does_not_set_error0_s1.ucdb`](../../uvm/cov_after/E080_decode_error_on_nonhit_state_does_not_set_error0_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 80.16 | 0.00 | 94.16 | 0.00 |
| branch | 64.24 | 64.24 | 0.00 | 86.84 | 0.00 |
| cond | 51.72 | 51.72 | 3.45 | 86.21 | 3.45 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 66.67 | 0.00 |
| toggle | 16.11 | 16.11 | 0.10 | 52.55 | 0.10 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
