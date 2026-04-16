# ✅ B014_ctrl_decode_idle_force_go

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Send `IDLE` word
- **Primary checks:** `receiver_force_go=1`, `receiver_go=0`
- **Contract anchor:** control decode table

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
| ℹ️ | log | [`uvm/logs/B014_ctrl_decode_idle_force_go_after_s1.log`](../../uvm/logs/B014_ctrl_decode_idle_force_go_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B014_ctrl_decode_idle_force_go_s1.ucdb`](../../uvm/cov_after/B014_ctrl_decode_idle_force_go_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 58.37 | n/a | 0.00 | 85.21 | n/a |
| branch | 34.44 | n/a | 0.00 | 72.85 | n/a |
| cond | 20.69 | n/a | 0.00 | 65.52 | n/a |
| expr | 46.91 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | 14.29 | n/a | 0.00 | 85.71 | n/a |
| fsm_trans | 0.00 | n/a | 0.00 | 46.67 | n/a |
| toggle | 6.47 | n/a | 0.10 | 19.03 | n/a |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
