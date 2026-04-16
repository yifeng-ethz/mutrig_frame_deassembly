# ✅ B104_error_state_keeps_outputs_quiet

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Enter `ERROR` with illegal control word and drive bytes
- **Primary checks:** no new header start
- **Contract anchor:** control decode error state

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
| ℹ️ | log | [`uvm/logs/B104_error_state_keeps_outputs_quiet_after_s1.log`](../../uvm/logs/B104_error_state_keeps_outputs_quiet_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B104_error_state_keeps_outputs_quiet_s1.ucdb`](../../uvm/cov_after/B104_error_state_keeps_outputs_quiet_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 67.70 | n/a | 0.00 | 97.67 | n/a |
| branch | 45.03 | n/a | 0.00 | 92.76 | n/a |
| cond | 41.38 | n/a | 0.00 | 79.31 | n/a |
| expr | 87.65 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | 42.86 | n/a | 0.00 | 100.00 | n/a |
| fsm_trans | 20.00 | n/a | 0.00 | 80.00 | n/a |
| toggle | 13.19 | n/a | 0.16 | 39.42 | n/a |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
