# ✅ P069_mode0_comma_random_positions

**Bucket:** `PROF` &nbsp; **Method:** `R` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `MODE_HALT=0`, random comma positions in 10k frames
- **Primary checks:** parser never wedges after abort
- **Contract anchor:** mode0 recovery

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
| ✅ | observed_txn | `4` |
| ℹ️ | implementation_mode | `doc_case_engine_v2` |
| ℹ️ | log | [`uvm/logs/P069_mode0_comma_random_positions_after_s1.log`](../../uvm/logs/P069_mode0_comma_random_positions_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/P069_mode0_comma_random_positions_s1.ucdb`](../../uvm/cov_after/P069_mode0_comma_random_positions_s1.ucdb) |
| ℹ️ | log.headers | `4` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 73.88 | 18.47 | 0.00 | 93.81 | 0.00 |
| branch | 54.00 | 13.50 | 0.00 | 86.67 | 0.00 |
| cond | 47.50 | 11.88 | 0.00 | 67.50 | 0.00 |
| expr | 98.77 | 24.69 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 16.28 | 4.07 | 0.00 | 51.44 | 0.00 |

## Transaction Growth (checkpoint UCDBs)

ℹ️ See [`../txn_growth/P069_mode0_comma_random_positions.md`](../txn_growth/P069_mode0_comma_random_positions.md) for the log-spaced coverage curve.

---
_Back to [bucket](../buckets/PROF.md) &middot; [dashboard](../../DV_REPORT.md)_
