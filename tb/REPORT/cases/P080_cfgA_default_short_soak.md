# ✅ P080_cfgA_default_short_soak

**Bucket:** `PROF` &nbsp; **Method:** `K` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `CFG_A`, 20k clean short frames
- **Primary checks:** packaged default stays stable
- **Contract anchor:** `_hw.tcl` default config

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
| ℹ️ | log | [`uvm/logs/P080_cfgA_default_short_soak_after_s1.log`](../../uvm/logs/P080_cfgA_default_short_soak_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P080_cfgA_default_short_soak_s1.ucdb`](../../uvm/cov_after/P080_cfgA_default_short_soak_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `64` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.54 | 1.26 | 0.00 | 94.55 | 0.00 |
| branch | 66.89 | 1.05 | 0.00 | 87.50 | 0.00 |
| cond | 55.17 | 0.86 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 1.56 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 46.67 | 0.73 | 0.00 | 73.33 | 0.00 |
| toggle | 36.55 | 0.57 | 0.00 | 60.27 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
