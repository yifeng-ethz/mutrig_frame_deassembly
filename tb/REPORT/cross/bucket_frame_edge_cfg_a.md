# ❌ bucket_frame_edge_cfg_a

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
| ℹ️ | txns | `61` |
| ❌ | functional_cross_pct | `40.62` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ❌ | unexpected_outputs | `3` |

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
| 2 | `E005_back_to_back_running_and_terminating_words` | `` | 40.62 | 2 | hit_eop |
| 3 | `E006_duplicate_running_words_idempotent` | `` | 40.62 | 0 | hit_eop |
| 4 | `E007_duplicate_idle_words_idempotent` | `` | 40.62 | 0 | hit_eop |
| 5 | `E008_ctrl_valid_pulse_one_cycle_only_still_latches_state` | `` | 40.62 | 0 | hit_eop |
| 6 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 7 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 8 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 9 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 10 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 11 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 12 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 13 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 14 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 15 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 16 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 17 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 18 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 19 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 20 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 21 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 22 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 23 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 24 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 25 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 26 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 27 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 28 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 29 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 30 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 31 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 32 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 33 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 34 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 35 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 36 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 37 | `E009_ctrl_valid_held_high_repeated_word_stable` | `` | 40.62 | 0 | hit_eop |
| 38 | `E010_ctrl_illegal_zero_word_maps_error` | `` | 40.62 | 0 | hit_eop |
| 39 | `E011_ctrl_twohot_word_maps_error` | `` | 40.62 | 0 | hit_eop |
| 40 | `E012_ctrl_allones_word_maps_error` | `` | 40.62 | 0 | hit_eop |
| 41 | `E013_ctrl_valid_low_keeps_previous_state` | `` | 40.62 | 0 | hit_eop |
| 42 | `E014_ctrl_ready_does_not_backpressure_illegal_word` | `` | 40.62 | 0 | hit_eop |
| 43 | `E016_fs_idle_ignores_header_when_byteisk0_data1c` | `` | 40.62 | 0 | hit_eop |
| 44 | `E017_fs_idle_ignores_kchar_non_header_9c` | `` | 40.62 | 0 | hit_eop |
| 45 | `E018_fs_idle_ignores_kchar_nonheader_other` | `` | 40.62 | 0 | hit_eop |
| 46 | `E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low` | `` | 40.62 | 0 | hit_eop |
| 47 | `E020_midframe_valid_low_does_not_pause_parser` | `` | 40.62 | 0 | hit_eop |
| 48 | `E021_midframe_channel_change_not_tracked_internally` | `` | 40.62 | 0 | hit_eop |
| 49 | `E024_header_immediately_after_run_prepare_blocked` | `` | 40.62 | 0 | hit_eop |
| 50 | `E025_header_immediately_after_sync_blocked` | `` | 40.62 | 0 | hit_eop |
| 51 | `E026_header_immediately_after_running_opened_next_cycle` | `` | 40.62 | 0 | hit_eop |
| 52 | `E027_header_immediately_after_terminating_blocked` | `` | 40.62 | 0 | hit_eop |
| 53 | `E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable` | `` | 40.62 | 0 | hit_eop |
| 54 | `E029_header_following_write_zero_then_one_requires_clocked_enable` | `` | 40.62 | 0 | hit_eop |
| 55 | `E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt` | `` | 40.62 | 0 | hit_eop |
| 56 | `E031_frame_number_0000_propagates` | `` | 40.62 | 0 | hit_eop |
| 57 | `E032_frame_number_ffff_propagates` | `` | 40.62 | 0 | hit_eop |
| 58 | `E033_frame_number_wrap_ffff_to_0000` | `` | 40.62 | 0 | hit_eop |
| 59 | `E034_frame_len_zero_long_goes_crc_path_directly` | `` | 40.62 | 0 | hit_eop |
| 60 | `E035_frame_len_one_long_single_word` | `` | 40.62 | 0 | hit_eop |
| 61 | `E036_frame_len_two_long_two_words` | `` | 40.62 | 0 | hit_eop |

## Issue reference

| status | field | value |
|:---:|---|---|
| ℹ️ | bug_ref | [FRCV-2026-04-17-003](../../BUG_HISTORY.md) |
| ❌ | rerun_date | `2026-04-17` |
| ❌ | rerun_verdict | Rerun reproduced unexpected_outputs=3 with counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. |
| ℹ️ | rerun_log | [uvm/logs/bucket_frame_edge_cfg_a_after_s1.log](../../uvm/logs/bucket_frame_edge_cfg_a_after_s1.log) |

---
_Back to [dashboard](../../DV_REPORT.md)_
