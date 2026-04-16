# ✅ X057_short_bad_crc_onebit_flip

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Flip one CRC-related byte in short frame
- **Primary checks:** `error(1)` on final hit, counter increments once
- **Contract anchor:** CRC bad path

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
| ℹ️ | log | [`uvm/logs/X057_short_bad_crc_onebit_flip_after_s1.log`](../../uvm/logs/X057_short_bad_crc_onebit_flip_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X057_short_bad_crc_onebit_flip_s1.ucdb`](../../uvm/cov_after/X057_short_bad_crc_onebit_flip_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 80.16 | 0.00 | 92.61 | 0.00 |
| branch | 66.23 | 66.23 | 0.00 | 84.21 | 0.00 |
| cond | 48.28 | 48.28 | 0.00 | 75.86 | 0.00 |
| expr | 92.59 | 92.59 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 46.67 | 46.67 | 0.00 | 73.33 | 0.00 |
| toggle | 17.57 | 17.57 | 0.00 | 49.90 | 0.00 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
