# ✅ E093_csr_addr_width8_high_unused_reads_zero

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_F` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Build with `CSR_ADDR_WIDTH=8`, read high addresses
- **Primary checks:** unused space reads default zero
- **Contract anchor:** `_hw.tcl` generic width

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
| ℹ️ | log | [`uvm/logs/E093_csr_addr_width8_high_unused_reads_zero_after_s1.log`](../../uvm/logs/E093_csr_addr_width8_high_unused_reads_zero_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E093_csr_addr_width8_high_unused_reads_zero_s1.ucdb`](../../uvm/cov_after/E093_csr_addr_width8_high_unused_reads_zero_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 80.16 | 80.16 | 0.00 | 94.94 | 0.00 |
| branch | 64.90 | 64.90 | 0.66 | 88.82 | 0.66 |
| cond | 48.28 | 48.28 | 0.00 | 86.21 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | 85.71 | 85.71 | 0.00 | 100.00 | 0.00 |
| fsm_trans | 40.00 | 40.00 | 0.00 | 66.67 | 0.00 |
| toggle | 17.41 | 17.41 | 0.73 | 53.16 | 0.73 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
