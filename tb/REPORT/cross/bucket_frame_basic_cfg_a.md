# ❌ bucket_frame_basic_cfg_a

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
| ℹ️ | txns | `87` |
| ❌ | functional_cross_pct | `59.38` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ❌ | unexpected_outputs | `2` |

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
| 17 | `B019_ctrl_decode_link_test_disable` | `` | 40.62 | 0 | hit_eop |
| 18 | `B020_ctrl_decode_sync_test_disable` | `` | 40.62 | 0 | hit_eop |
| 19 | `B022_ctrl_decode_out_of_daq_disable` | `` | 40.62 | 0 | hit_eop |
| 20 | `B023_csr_control_default_enable` | `` | 40.62 | 0 | hit_eop |
| 21 | `B024_csr_control_write_zero_masks_running` | `` | 40.62 | 0 | hit_eop |
| 22 | `B025_csr_control_write_one_unmasks_running` | `` | 40.62 | 0 | hit_eop |
| 23 | `B026_csr_read_addr0_control_status` | `` | 40.62 | 0 | hit_eop |
| 24 | `B027_csr_read_addr1_crc_counter` | `` | 40.62 | 0 | hit_eop |
| 25 | `B028_csr_read_addr2_frame_counter` | `` | 40.62 | 0 | hit_eop |
| 26 | `B029_csr_read_unused_word_zero` | `` | 40.62 | 0 | hit_eop |
| 27 | `B030_csr_write_unused_word_no_side_effect` | `` | 40.62 | 0 | hit_eop |
| 28 | `B031_csr_waitrequest_low_on_read` | `` | 40.62 | 0 | hit_eop |
| 29 | `B032_csr_waitrequest_low_on_write` | `` | 40.62 | 0 | hit_eop |
| 30 | `B033_csr_waitrequest_high_when_idle` | `` | 40.62 | 0 | hit_eop |
| 31 | `B034_csr_reserved_control_bits_roundtrip` | `` | 40.62 | 0 | hit_eop |
| 32 | `B035_fs_idle_requires_k28_0_and_kchar` | `` | 40.62 | 0 | hit_eop |
| 33 | `B038_header_byte_with_enable_high_starts_frame` | `` | 40.62 | 0 | hit_eop |
| 34 | `B039_long_zero_hit_frame_headerinfo_pulse` | `` | 43.75 | 1 | header_only_or_zero_hit |
| 35 | `B040_long_zero_hit_frame_no_hit_valid` | `` | 43.75 | 0 | hit_eop |
| 36 | `B041_long_one_hit_sop_and_eop_same_transfer` | `` | 43.75 | 0 | hit_eop |
| 37 | `B042_long_two_hit_sop_first_only` | `` | 46.88 | 1 | hit_eop |
| 38 | `B043_long_two_hit_eop_last_only` | `` | 46.88 | 0 | hit_eop |
| 39 | `B044_long_frame_number_lsb_capture` | `` | 46.88 | 0 | hit_eop |
| 40 | `B045_long_frame_number_msb_capture` | `` | 46.88 | 0 | hit_eop |
| 41 | `B046_long_frame_flags_capture` | `` | 46.88 | 0 | hit_eop |
| 42 | `B047_long_frame_len_capture` | `` | 46.88 | 0 | hit_eop |
| 43 | `B048_long_hit_channel_unpack` | `` | 46.88 | 0 | hit_eop |
| 44 | `B049_long_hit_tcc_unpack` | `` | 46.88 | 0 | hit_eop |
| 45 | `B050_long_hit_tfine_unpack` | `` | 46.88 | 0 | hit_eop |
| 46 | `B051_long_hit_ecc_unpack` | `` | 46.88 | 0 | hit_eop |
| 47 | `B052_long_hit_eflag_unpack` | `` | 46.88 | 0 | hit_eop |
| 48 | `B053_long_t_badhit_raises_error0` | `` | 46.88 | 0 | hit_eop |
| 49 | `B054_long_e_badhit_raises_error0` | `` | 46.88 | 0 | hit_eop |
| 50 | `B055_long_parity_error_raises_error0` | `` | 46.88 | 0 | hit_eop |
| 51 | `B056_long_decode_error_raises_error0` | `` | 46.88 | 0 | hit_eop |
| 52 | `B057_long_loss_sync_propagates_error2` | `` | 46.88 | 0 | hit_eop |
| 53 | `B058_long_crc_good_keeps_error1_low` | `` | 46.88 | 0 | hit_eop |
| 54 | `B059_long_crc_bad_raises_error1_on_eop` | `` | 46.88 | 0 | hit_eop |
| 55 | `B060_long_crc_bad_increments_crc_counter` | `` | 46.88 | 0 | hit_eop |
| 56 | `B061_sop_increments_frame_counter_head` | `` | 46.88 | 0 | hit_eop |
| 57 | `B062_eop_increments_frame_counter_tail` | `` | 46.88 | 0 | hit_eop |
| 58 | `B063_frame_counter_register_is_head_minus_tail` | `` | 46.88 | 0 | hit_eop |
| 59 | `B065_short_zero_hit_frame_headerinfo_pulse` | `` | 56.25 | 3 | header_only_or_zero_hit |
| 60 | `B066_short_one_hit_sop_eop_same_transfer` | `` | 56.25 | 0 | hit_eop |
| 61 | `B067_short_two_hit_even_pairing` | `` | 56.25 | 0 | hit_eop |
| 62 | `B068_short_three_hit_odd_pairing` | `` | 59.38 | 1 | hit_eop |
| 63 | `B069_short_four_hit_even_followup` | `` | 59.38 | 0 | hit_eop |
| 64 | `B070_short_mode_selected_by_flags_100` | `` | 59.38 | 0 | hit_eop |
| 65 | `B071_short_path_zeroes_ecc_field` | `` | 59.38 | 0 | hit_eop |
| 66 | `B072_short_path_uses_bit0_as_eflag` | `` | 59.38 | 0 | hit_eop |
| 67 | `B073_short_hit_channel_unpack` | `` | 59.38 | 0 | hit_eop |
| 68 | `B074_short_hit_tcc_unpack` | `` | 59.38 | 0 | hit_eop |
| 69 | `B075_short_hit_tfine_unpack` | `` | 59.38 | 0 | hit_eop |
| 70 | `B076_short_parity_error_raises_error0` | `` | 59.38 | 0 | hit_eop |
| 71 | `B077_short_decode_error_raises_error0` | `` | 59.38 | 0 | hit_eop |
| 72 | `B078_short_loss_sync_propagates_error2` | `` | 59.38 | 0 | hit_eop |
| 73 | `B079_short_crc_bad_raises_error1_on_last_hit` | `` | 59.38 | 0 | hit_eop |
| 74 | `B080_short_crc_counter_accumulates` | `` | 59.38 | 0 | hit_eop |
| 75 | `B081_short_frame_counter_updates` | `` | 59.38 | 0 | hit_eop |
| 76 | `B082_headerinfo_one_pulse_per_frame` | `` | 59.38 | 0 | hit_eop |
| 77 | `B083_headerinfo_channel_matches_input` | `` | 59.38 | 0 | hit_eop |
| 78 | `B084_headerinfo_frame_number_field_map` | `` | 59.38 | 0 | hit_eop |
| 79 | `B085_headerinfo_word_count_field_map` | `` | 59.38 | 0 | hit_eop |
| 80 | `B086_headerinfo_frame_len_field_map` | `` | 59.38 | 0 | hit_eop |
| 81 | `B087_headerinfo_flags_field_map` | `` | 59.38 | 0 | hit_eop |
| 82 | `B088_output_channel_matches_input_channel` | `` | 59.38 | 0 | hit_eop |
| 83 | `B089_hit_valid_only_on_complete_word` | `` | 59.38 | 0 | hit_eop |
| 84 | `B090_no_hit_valid_in_crc_states` | `` | 59.38 | 0 | hit_eop |
| 85 | `B091_no_sop_without_valid` | `` | 59.38 | 0 | hit_eop |
| 86 | `B092_no_eop_without_valid` | `` | 59.38 | 0 | hit_eop |
| 87 | `B096_running_mask_toggle_between_frames` | `` | 59.38 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
