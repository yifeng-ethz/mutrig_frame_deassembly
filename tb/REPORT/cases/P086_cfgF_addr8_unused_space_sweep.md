# ✅ P086_cfgF_addr8_unused_space_sweep

**Bucket:** `PROF` &nbsp; **Method:** `K` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `CFG_F`, random high-address reads
- **Primary checks:** wide address build keeps unused space harmless
- **Contract anchor:** `CSR_ADDR_WIDTH=8`

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
| ℹ️ | log | [`uvm/logs/P086_cfgF_addr8_unused_space_sweep_after_s1.log`](../../uvm/logs/P086_cfgF_addr8_unused_space_sweep_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P086_cfgF_addr8_unused_space_sweep_s1.ucdb`](../../uvm/cov_after/P086_cfgF_addr8_unused_space_sweep_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 81.79 | 81.79 | 0.00 | 94.16 | 0.00 |
| branch | 66.67 | 66.67 | 0.66 | 88.08 | 0.66 |
| cond | 45.00 | 45.00 | 0.00 | 67.50 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 13.54 | 13.54 | 0.00 | 53.95 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
