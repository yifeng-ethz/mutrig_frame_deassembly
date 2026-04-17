# ⚠️ bucket_frame_edge_cfg_a

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `EDGE` &nbsp; **Sequence:** `EDGE_CFG_A`

## Summary

<!-- field legend:
  case_count              = number of plan cases composed into this run
  effort                  = practical (capped per case) or extensive (full planned stress)
  iter_cap, payload_cap   = practical-mode budget caps
  txns                    = total transactions driven through the DUT in this run
  functional_cross_pct    = functional coverage against DV_CROSS.md (percent)
  queued_overlap          = transactions enqueued before the previous drained
  counter_checks_failed   = scoreboard counter mismatches observed (0 is required for pass)
  unexpected_outputs      = outputs the scoreboard did not predict
-->

| status | field | value |
|:---:|---|---|
| ℹ️ | case_count | `119` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `None` |
| ℹ️ | payload_cap | `None` |
| ℹ️ | txns | `24` |
| ⚠️ | functional_cross_pct | `40.62` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 91.83 |
| branch | 82.12 |
| cond | 79.31 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 60.00 |
| toggle | 53.39 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `E001_ctrl_state_update_is_one_cycle_late_from_valid` | `` | 34.38 | 11 | hit_eop |
| 2 | `E002_running_command_same_cycle_header_not_accepted` | `` | 37.50 | 1 | hit_eop |
| 3 | `E003_idle_command_same_cycle_header_still_sees_previous_running` | `` | 37.50 | 0 | hit_eop |
| 4 | `E004_terminating_command_same_cycle_open_frame_continues` | `` | 37.50 | 0 | hit_eop |
| 5 | `E005_back_to_back_running_and_terminating_words` | `` | 40.62 | 1 | hit_eop |
| 6 | `E006_duplicate_running_words_idempotent` | `` | 40.62 | 0 | hit_eop |
| 7 | `E007_duplicate_idle_words_idempotent` | `` | 40.62 | 0 | hit_eop |
| 8 | `E008_ctrl_valid_pulse_one_cycle_only_still_latches_state` | `` | 40.62 | 0 | hit_eop |
| 9 | `E013_ctrl_valid_low_keeps_previous_state` | `` | 40.62 | 0 | hit_eop |
| 10 | `E014_ctrl_ready_does_not_backpressure_illegal_word` | `` | 40.62 | 0 | hit_eop |
| 11 | `E016_fs_idle_ignores_header_when_byteisk0_data1c` | `` | 40.62 | 0 | hit_eop |
| 12 | `E017_fs_idle_ignores_kchar_non_header_9c` | `` | 40.62 | 0 | hit_eop |
| 13 | `E018_fs_idle_ignores_kchar_nonheader_other` | `` | 40.62 | 0 | hit_eop |
| 14 | `E021_midframe_channel_change_not_tracked_internally` | `` | 40.62 | 0 | hit_eop |
| 15 | `E024_header_immediately_after_run_prepare_blocked` | `` | 40.62 | 0 | hit_eop |
| 16 | `E025_header_immediately_after_sync_blocked` | `` | 40.62 | 0 | hit_eop |

## Issue reference

| status | field | value |
|:---:|---|---|
| ℹ️ | bug_ref | [FRCV-2026-04-17-003](../../BUG_HISTORY.md) |
| ✅ | rerun_date | `2026-04-18` |
| ✅ | rerun_verdict | Before fix on 2026-04-17 this run reproduced unexpected_outputs=3 with counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. After fix on 2026-04-18 the rerun is clean: unexpected_outputs=0, counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. |
| ℹ️ | rerun_log | [uvm/logs/bucket_frame_edge_cfg_a_after_s1.log](../../uvm/logs/bucket_frame_edge_cfg_a_after_s1.log) |

---
_Back to [dashboard](../../DV_REPORT.md)_
