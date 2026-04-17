# ✅ bucket_frame_basic_cfg_a

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `BASIC` &nbsp; **Sequence:** `BASIC_CFG_A`

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
| ℹ️ | case_count | `129` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `None` |
| ℹ️ | payload_cap | `None` |
| ℹ️ | txns | `84` |
| ✅ | functional_cross_pct | `59.38` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 98.83 |
| branch | 94.70 |
| cond | 82.76 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 80.00 |
| toggle | 51.72 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `B002_idle_force_go_monitor_path_open` | `` | 34.38 | 11 | hit_eop |
| 2 | `B003_idle_force_go_ignores_csr_mask` | `` | 37.50 | 1 | hit_eop |
| 3 | `B004_running_needs_csr_enable_high` | `` | 37.50 | 0 | hit_eop |
| 4 | `B006_run_prepare_clears_parser_state` | `` | 37.50 | 0 | hit_eop |
| 5 | `B007_sync_keeps_parser_closed` | `` | 37.50 | 0 | hit_eop |
| 6 | `B008_running_opens_header_detection` | `` | 37.50 | 0 | hit_eop |
| 7 | `B009_terminating_blocks_new_header_from_idle` | `` | 40.62 | 1 | hit_eop |
| 8 | `B010_terminating_allows_open_frame_to_finish` | `` | 40.62 | 0 | hit_eop |
| 9 | `B011_ctrl_unknown_word_enters_error` | `` | 40.62 | 0 | hit_eop |
| 10 | `B012_ctrl_ready_high_with_valid` | `` | 40.62 | 0 | hit_eop |
| 11 | `B013_ctrl_ready_high_without_valid` | `` | 40.62 | 0 | hit_eop |
| 12 | `B014_ctrl_decode_idle_force_go` | `` | 40.62 | 0 | hit_eop |
| 13 | `B015_ctrl_decode_run_prepare_disable` | `` | 40.62 | 0 | hit_eop |
| 14 | `B016_ctrl_decode_sync_disable` | `` | 40.62 | 0 | hit_eop |
| 15 | `B017_ctrl_decode_running_go` | `` | 40.62 | 0 | hit_eop |
| 16 | `B018_ctrl_decode_terminating_stop_new_headers` | `` | 40.62 | 0 | hit_eop |
| 32 | `B035_fs_idle_requires_k28_0_and_kchar` | `` | 40.62 | 0 | hit_eop |
| 34 | `B039_long_zero_hit_frame_headerinfo_pulse` | `` | 43.75 | 1 | header_only_or_zero_hit |
| 37 | `B042_long_two_hit_sop_first_only` | `` | 46.88 | 1 | hit_eop |
| 59 | `B065_short_zero_hit_frame_headerinfo_pulse` | `` | 56.25 | 3 | header_only_or_zero_hit |
| 62 | `B068_short_three_hit_odd_pairing` | `` | 59.38 | 1 | hit_eop |
| 64 | `B070_short_mode_selected_by_flags_100` | `` | 59.38 | 0 | hit_eop |

## Issue reference

| status | field | value |
|:---:|---|---|
| ℹ️ | bug_ref | [FRCV-2026-04-17-003](../../BUG_HISTORY.md) |
| ✅ | rerun_date | `2026-04-18` |
| ✅ | rerun_verdict | Before fix on 2026-04-17 this run reproduced unexpected_outputs=2 with counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. After fix on 2026-04-18 the rerun is clean: unexpected_outputs=0, counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. |
| ℹ️ | rerun_log | [uvm/logs/bucket_frame_basic_cfg_a_after_s1.log](../../uvm/logs/bucket_frame_basic_cfg_a_after_s1.log) |

---
_Back to [dashboard](../../DV_REPORT.md)_
