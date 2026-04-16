# ✅ B065_short_zero_hit_frame_headerinfo_pulse

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Short frame with event count 0
- **Primary checks:** metadata pulse occurs, no hit output
- **Contract anchor:** zero-hit short path

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
| ℹ️ | log | [`uvm/logs/B065_short_zero_hit_frame_headerinfo_pulse_after_s1.log`](../../uvm/logs/B065_short_zero_hit_frame_headerinfo_pulse_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B065_short_zero_hit_frame_headerinfo_pulse_s1.ucdb`](../../uvm/cov_after/B065_short_zero_hit_frame_headerinfo_pulse_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 74.32 | 74.32 | 0.78 | 89.88 | 0.78 |
| branch | 52.98 | 52.98 | 1.32 | 82.78 | 1.32 |
| cond | 44.83 | 44.83 | 0.00 | 75.86 | 0.00 |
| expr | 92.59 | 92.59 | 0.00 | 98.77 | 0.00 |
| fsm_state | 71.43 | 71.43 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 33.33 | 33.33 | 0.00 | 60.00 | 0.00 |
| toggle | 14.60 | 14.60 | 0.47 | 29.51 | 0.47 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
