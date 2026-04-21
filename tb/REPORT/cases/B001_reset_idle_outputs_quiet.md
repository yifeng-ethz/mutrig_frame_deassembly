# ✅ B001_reset_idle_outputs_quiet

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Assert reset, hold clock, release into default state
- **Primary checks:** `hit_type0_valid=0`, `headerinfo_valid=0`, CSR counters clear
- **Contract anchor:** reset branch of parser, outputs, CSR

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
| ℹ️ | log | [`uvm/logs/B001_reset_idle_outputs_quiet_after_s1.log`](../../uvm/logs/B001_reset_idle_outputs_quiet_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B001_reset_idle_outputs_quiet_s1.ucdb`](../../uvm/cov_after/B001_reset_idle_outputs_quiet_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 59.11 | n/a | 59.11 | 59.11 | n/a |
| branch | 30.67 | n/a | 30.67 | 30.67 | n/a |
| cond | 7.50 | n/a | 7.50 | 7.50 | n/a |
| expr | 0.00 | n/a | 0.00 | 0.00 | n/a |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 0.73 | n/a | 0.73 | 0.73 | n/a |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
