# ✅ E040_frame_len_two_short_even_pair

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Short-mode frame length 2
- **Primary checks:** even pairing completes without stray nibble
- **Contract anchor:** short even-count edge

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
| ℹ️ | log | [`uvm/logs/E040_frame_len_two_short_even_pair_after_s1.log`](../../uvm/logs/E040_frame_len_two_short_even_pair_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E040_frame_len_two_short_even_pair_s1.ucdb`](../../uvm/cov_after/E040_frame_len_two_short_even_pair_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `2` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 82.49 | 82.49 | 3.50 | 93.00 | 3.50 |
| branch | 71.52 | 71.52 | 4.61 | 85.53 | 4.61 |
| cond | 48.28 | 48.28 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 53.33 | 53.33 | 6.67 | 66.67 | 6.67 |
| toggle | 22.11 | 22.11 | 0.52 | 50.42 | 0.52 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
