# ✅ X050_short_comma_inside_payload_mode0

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** `MODE_HALT=0`, comma in short payload
- **Primary checks:** frame aborts
- **Contract anchor:** comma abort

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
| ℹ️ | log | [`uvm/logs/X050_short_comma_inside_payload_mode0_after_s1.log`](../../uvm/logs/X050_short_comma_inside_payload_mode0_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X050_short_comma_inside_payload_mode0_s1.ucdb`](../../uvm/cov_after/X050_short_comma_inside_payload_mode0_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 71.60 | 71.60 | 0.00 | 92.61 | 0.00 |
| branch | 54.97 | 54.97 | 0.00 | 84.21 | 0.00 |
| cond | 48.28 | 48.28 | 0.00 | 75.86 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 57.14 | 57.14 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 26.67 | 26.67 | 0.00 | 73.33 | 0.00 |
| toggle | 16.48 | 16.48 | 0.21 | 49.90 | 0.21 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
