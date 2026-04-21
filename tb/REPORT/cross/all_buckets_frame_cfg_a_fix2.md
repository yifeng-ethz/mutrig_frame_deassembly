# ✅ all_buckets_frame_cfg_a_fix2

**Kind:** `all_buckets_frame` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `ALL_CFG_A`

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
| ℹ️ | case_count | `474` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `775` |
| ✅ | functional_cross_pct | `68.75` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 96.54 |
| branch | 91.33 |
| cond | 79.49 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 100.00 |
| toggle | 59.53 |

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
| 128 | `E018_fs_idle_ignores_kchar_nonheader_other` | `` | 59.38 | 0 | hit_eop |
| 143 | `E037_frame_len_max_long_1023_words` | `` | 62.50 | 1 | hit_eop |
| 221 | `E124_truncated_long_frame_never_asserts_eop` | `` | 65.62 | 1 | header_only_or_zero_hit |
| 256 | `P001_long_len0_10k_frames` | `` | 65.62 | 0 | header_only_or_zero_hit |
| 356 | `P005_long_len7_5k_frames` | `` | 68.75 | 1 | hit_eop |
| 512 | `P016_short_len2_10k_frames` | `` | 68.75 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
