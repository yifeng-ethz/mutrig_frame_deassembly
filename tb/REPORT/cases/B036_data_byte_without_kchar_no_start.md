# ✅ B036_data_byte_without_kchar_no_start

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Drive `0x1C` as data with `i_byteisk=0`
- **Primary checks:** no frame start
- **Contract anchor:** start condition requires `i_byteisk=1`

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
| ℹ️ | log | [`uvm/logs/B036_data_byte_without_kchar_no_start_after_s1.log`](../../uvm/logs/B036_data_byte_without_kchar_no_start_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B036_data_byte_without_kchar_no_start_s1.ucdb`](../../uvm/cov_after/B036_data_byte_without_kchar_no_start_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 67.70 | n/a | 0.00 | 88.72 | n/a |
| branch | 45.03 | n/a | 0.00 | 78.81 | n/a |
| cond | 41.38 | n/a | 0.00 | 68.97 | n/a |
| expr | 81.48 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | 42.86 | n/a | 0.00 | 85.71 | n/a |
| fsm_trans | 20.00 | n/a | 6.67 | 53.33 | n/a |
| toggle | 12.88 | n/a | 0.16 | 21.32 | n/a |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
