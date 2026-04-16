# ✅ B039_long_zero_hit_frame_headerinfo_pulse

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Long frame with event count 0
- **Primary checks:** one `headerinfo_valid` pulse, no hit output
- **Contract anchor:** `FS_EVENT_COUNTER -> FS_CRC_CALC` zero-hit path

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
| ℹ️ | log | [`uvm/logs/B039_long_zero_hit_frame_headerinfo_pulse_after_s1.log`](../../uvm/logs/B039_long_zero_hit_frame_headerinfo_pulse_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B039_long_zero_hit_frame_headerinfo_pulse_s1.ucdb`](../../uvm/cov_after/B039_long_zero_hit_frame_headerinfo_pulse_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 72.76 | 72.76 | 0.00 | 88.72 | 0.00 |
| branch | 50.33 | 50.33 | 0.00 | 78.81 | 0.00 |
| cond | 41.38 | 41.38 | 0.00 | 68.97 | 0.00 |
| expr | 92.59 | 92.59 | 0.00 | 98.77 | 0.00 |
| fsm_state | 71.43 | 71.43 | 0.00 | 85.71 | 0.00 |
| fsm_trans | 33.33 | 33.33 | 6.67 | 60.00 | 6.67 |
| toggle | 14.75 | 14.75 | 0.00 | 21.32 | 0.00 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
