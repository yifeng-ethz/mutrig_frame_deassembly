# ✅ E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating

**Bucket:** `EDGE` &nbsp; **Method:** `D` &nbsp; **Build:** `CFG_A` &nbsp; **Effort:** `practical` &nbsp; **Result:** `pass`

## Intent

- **Scenario:** Emulate fresh post-stop frame arrival
- **Primary checks:** current DUT drops that frame at `FS_IDLE`
- **Contract anchor:** exact failing hole in upgrade plan

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
| ℹ️ | log | [`uvm/logs/E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating_after_s1.log`](../../uvm/logs/E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating_after_s1.log) |
| ℹ️ | ucdb | [`uvm/cov_after/E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating_s1.ucdb`](../../uvm/cov_after/E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating_s1.ucdb) |
| ℹ️ | log.headers | `1` |
| ℹ️ | log.hits | `1` |
| ℹ️ | log.real_eops | `1` |
| ℹ️ | log.synth_eops | `0` |
| ℹ️ | log.endofruns | `0` |

## Coverage

<!-- code coverage vectors: stmt/branch/cond/expr/fsm_state/fsm_trans/toggle (percent) -->

| metric | standalone | isolated_per_txn | bucket_gain | bucket_merged_after | bucket_gain_per_txn |
|---|---|---|---|---|---|
| stmt | 83.85 | 83.85 | 0.00 | 95.53 | 0.00 |
| branch | 69.33 | 69.33 | 0.00 | 90.00 | 0.00 |
| cond | 47.50 | 47.50 | 0.00 | 75.00 | 0.00 |
| expr | 98.77 | 98.77 | 0.00 | 98.77 | 0.00 |
| fsm_state | n/a | n/a | n/a | n/a | n/a |
| fsm_trans | n/a | n/a | n/a | n/a | n/a |
| toggle | 13.04 | 13.04 | 0.00 | 48.53 | 0.00 |

---
_Back to [bucket](../buckets/EDGE.md) &middot; [dashboard](../../DV_REPORT.md)_
