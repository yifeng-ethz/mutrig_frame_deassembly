# ✅ E081_mode_halt0_comma_on_frame_counter_byte_aborts

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `MODE_HALT=0`, comma in frame-number field
- **Primary checks:** parser returns to idle
- **Contract anchor:** abort timing edge

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
| ℹ️ | log | [`uvm/logs/E081_mode_halt0_comma_on_frame_counter_byte_aborts_after_s1.log`](../../uvm/logs/E081_mode_halt0_comma_on_frame_counter_byte_aborts_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E081_mode_halt0_comma_on_frame_counter_byte_aborts_s1.ucdb`](../../uvm/cov_after/E081_mode_halt0_comma_on_frame_counter_byte_aborts_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 71.98 | 71.98 | 0.78 | 94.94 | 0.78 |
| branch | 54.30 | 54.30 | 1.32 | 88.16 | 1.32 |
| cond | 51.72 | 51.72 | 0.00 | 86.21 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 0.00 | 66.67 | 0.00 |
| toggle | 15.90 | 15.90 | 0.21 | 52.76 | 0.21 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
