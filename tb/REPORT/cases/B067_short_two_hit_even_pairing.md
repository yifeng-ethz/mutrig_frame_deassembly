# ✅ B067_short_two_hit_even_pairing

**Bucket:** `BASIC` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Short frame length 2
- **Primary checks:** two hits unpack correctly over `UNPACK/UNPACK_EXTRA`
- **Contract anchor:** short packer even case

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
| ℹ️ | log | [`uvm/logs/B067_short_two_hit_even_pairing_after_s1.log`](../../uvm/logs/B067_short_two_hit_even_pairing_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/B067_short_two_hit_even_pairing_s1.ucdb`](../../uvm/cov_after/B067_short_two_hit_even_pairing_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `2` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 82.49 | 82.49 | 3.50 | 96.89 | 3.50 |
| branch | 71.52 | 71.52 | 4.64 | 92.05 | 4.64 |
| cond | 48.28 | 48.28 | 0.00 | 79.31 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 100.00 | 100.00 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 53.33 | 53.33 | 6.67 | 80.00 | 6.67 |
| toggle | 22.00 | 22.00 | 3.13 | 33.68 | 3.13 |

---
_Back to [bucket](../buckets/BASIC.md) &middot; [dashboard](../../DV_REPORT.md)_
