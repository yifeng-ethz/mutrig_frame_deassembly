# ✅ P087_cfgG_debuglv2_functional_soak

**Bucket:** `PROF` &nbsp; **Method:** `K` &nbsp; **Build:** `CFG_G` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `CFG_G`, default traffic
- **Primary checks:** `DEBUG_LV=2` functional behavior matches cfg A
- **Contract anchor:** generation parameter

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
| ℹ️ | log | [`uvm/logs/P087_cfgG_debuglv2_functional_soak_after_s1.log`](../../uvm/logs/P087_cfgG_debuglv2_functional_soak_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P087_cfgG_debuglv2_functional_soak_s1.ucdb`](../../uvm/cov_after/P087_cfgG_debuglv2_functional_soak_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `64` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 1.25 | 0.00 | 94.94 | 0.00 |
| branch | 64.24 | 1.00 | 0.00 | 88.82 | 0.00 |
| cond | 58.62 | 0.92 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 1.34 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 0.62 | 0.00 | 73.33 | 0.00 |
| toggle | 44.06 | 0.69 | 0.00 | 61.46 | 0.00 |

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
