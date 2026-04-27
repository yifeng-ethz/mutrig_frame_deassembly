# ✅ X119_cfgD_channel_width8_random_wide_values

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_D` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Build `CHANNEL_WIDTH=8`, spray wide channel values during errors
- **Primary checks:** no truncation beyond configured width
- **Contract anchor:** `_hw.tcl` width contract

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
| ✅ | observed_txn | `64` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/X119_cfgD_channel_width8_random_wide_values_after_s1.log`](../../uvm/logs/X119_cfgD_channel_width8_random_wide_values_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X119_cfgD_channel_width8_random_wide_values_s1.ucdb`](../../uvm/cov_after/X119_cfgD_channel_width8_random_wide_values_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `64` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.79 | 1.28 | 0.00 | 95.19 | 0.00 |
| branch | 66.00 | 1.03 | 0.00 | 89.33 | 0.00 |
| cond | 50.00 | 0.78 | 2.50 | 67.50 | 0.04 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 41.99 | 0.66 | 12.15 | 49.14 | 0.19 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
