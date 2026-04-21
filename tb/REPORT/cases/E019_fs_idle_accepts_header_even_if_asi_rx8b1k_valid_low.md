# ✅ E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Hold `asi_rx8b1k_valid=0` while presenting a legal header
- **Primary checks:** no frame start or metadata emission; regression guard for the old valid-ignore bug
- **Contract anchor:** `proc_input_wrapper_comb` valid gate

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
| ℹ️ | log | [`uvm/logs/E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low_after_s1.log`](../../uvm/logs/E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low_s1.ucdb`](../../uvm/cov_after/E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low_s1.ucdb) |
| ℹ️ | log.headers | `0` |
| ℹ️ | log.hits | `0` |
| ℹ️ | log.real_eops | `0` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 63.92 | n/a | 0.00 | 85.22 | n/a |
| branch | 38.67 | n/a | 0.00 | 71.33 | n/a |
| cond | 20.00 | n/a | 0.00 | 50.00 | n/a |
| expr | 0.00 | n/a | 0.00 | 98.77 | n/a |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 2.38 | n/a | 0.00 | 17.07 | n/a |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
