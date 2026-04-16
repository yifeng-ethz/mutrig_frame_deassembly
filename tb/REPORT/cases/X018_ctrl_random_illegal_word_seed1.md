# ✅ X018_ctrl_random_illegal_word_seed1

**Bucket:** `ERROR` &nbsp; **Method:** `F` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Random non-one-hot word set #1
- **Primary checks:** no crash, `ERROR` reached
- **Contract anchor:** illegal control stability

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
| ℹ️ | log | [`uvm/logs/X018_ctrl_random_illegal_word_seed1_after_s1.log`](../../uvm/logs/X018_ctrl_random_illegal_word_seed1_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/X018_ctrl_random_illegal_word_seed1_s1.ucdb`](../../uvm/cov_after/X018_ctrl_random_illegal_word_seed1_s1.ucdb) |
| ℹ️ | log.headers | `64` |
| ℹ️ | log.hits | `256` |
| ℹ️ | log.real_eops | `64` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 1.25 | 0.39 | 88.33 | 0.01 |
| branch | 66.23 | 1.03 | 2.65 | 78.81 | 0.04 |
| cond | 58.62 | 0.92 | 6.90 | 75.86 | 0.11 |
| expr | 98.77 | 1.54 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 1.34 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 0.62 | 0.00 | 60.00 | 0.00 |
| toggle | 44.42 | 0.69 | 26.64 | 48.54 | 0.42 |

---
_Back to [bucket](../buckets/ERROR.md) &middot; [dashboard](../../DV_REPORT.md)_
