# ⚠️ EDGE bucket

**Planned:** `130` &nbsp; **Evidenced:** `130` &nbsp; **Status:** ⚠️

## Merged code coverage (this bucket)

<!-- column legend:
  metric          = code-coverage category (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle)
  merged_pct      = bucket-local ordered-merge percentage across all evidenced cases
  target          = workflow coverage target (blank = no hard target for that category)
  status          = target check vs merged_pct
-->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 96.50 | 95.0 |
| ✅ | branch | 90.79 | 90.0 |
| ℹ️ | cond | 89.66 | - |
| ℹ️ | expr | 98.77 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 66.67 | 90.0 |
| ⚠️ | toggle | 53.89 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `E001_ctrl_state_update_is_one_cycle_late_from_valid` | stmt=79.77, branch=63.58, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=15.90 | [case](../cases/E001_ctrl_state_update_is_one_cycle_late_from_valid.md) |
| ✅ | 2 | `E002_running_command_same_cycle_header_not_accepted` | stmt=79.77, branch=63.58, cond=51.72, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=16.53 | [case](../cases/E002_running_command_same_cycle_header_not_accepted.md) |
| ✅ | 3 | `E003_idle_command_same_cycle_header_still_sees_previous_running` | stmt=79.77, branch=63.58, cond=51.72, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=16.53 | [case](../cases/E003_idle_command_same_cycle_header_still_sees_previous_running.md) |
| ✅ | 4 | `E004_terminating_command_same_cycle_open_frame_continues` | stmt=82.49, branch=68.87, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.36 | [case](../cases/E004_terminating_command_same_cycle_open_frame_continues.md) |
| ✅ | 5 | `E005_back_to_back_running_and_terminating_words` | stmt=82.49, branch=68.87, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.36 | [case](../cases/E005_back_to_back_running_and_terminating_words.md) |
| ✅ | 6 | `E006_duplicate_running_words_idempotent` | stmt=82.49, branch=68.87, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.36 | [case](../cases/E006_duplicate_running_words_idempotent.md) |
| ✅ | 7 | `E007_duplicate_idle_words_idempotent` | stmt=82.49, branch=68.87, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.36 | [case](../cases/E007_duplicate_idle_words_idempotent.md) |
| ✅ | 8 | `E008_ctrl_valid_pulse_one_cycle_only_still_latches_state` | stmt=82.49, branch=68.87, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.78 | [case](../cases/E008_ctrl_valid_pulse_one_cycle_only_still_latches_state.md) |
| ✅ | 9 | `E009_ctrl_valid_held_high_repeated_word_stable` | stmt=82.88, branch=69.54, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.17 | [case](../cases/E009_ctrl_valid_held_high_repeated_word_stable.md) |
| ✅ | 10 | `E010_ctrl_illegal_zero_word_maps_error` | stmt=82.88, branch=69.54, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.17 | [case](../cases/E010_ctrl_illegal_zero_word_maps_error.md) |
| ✅ | 11 | `E011_ctrl_twohot_word_maps_error` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.22 | [case](../cases/E011_ctrl_twohot_word_maps_error.md) |
| ✅ | 12 | `E012_ctrl_allones_word_maps_error` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.64 | [case](../cases/E012_ctrl_allones_word_maps_error.md) |
| ✅ | 13 | `E013_ctrl_valid_low_keeps_previous_state` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.64 | [case](../cases/E013_ctrl_valid_low_keeps_previous_state.md) |
| ✅ | 14 | `E014_ctrl_ready_does_not_backpressure_illegal_word` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.64 | [case](../cases/E014_ctrl_ready_does_not_backpressure_illegal_word.md) |
| ✅ | 15 | `E015_ctrl_ready_high_during_reset_release` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.47 | [case](../cases/E015_ctrl_ready_high_during_reset_release.md) |
| ✅ | 16 | `E016_fs_idle_ignores_header_when_byteisk0_data1c` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E016_fs_idle_ignores_header_when_byteisk0_data1c.md) |
| ✅ | 17 | `E017_fs_idle_ignores_kchar_non_header_9c` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E017_fs_idle_ignores_kchar_non_header_9c.md) |
| ✅ | 18 | `E018_fs_idle_ignores_kchar_nonheader_other` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E018_fs_idle_ignores_kchar_nonheader_other.md) |
| ✅ | 19 | `E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low.md) |
| ✅ | 20 | `E020_midframe_valid_low_does_not_pause_parser` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E020_midframe_valid_low_does_not_pause_parser.md) |
| ✅ | 21 | `E021_midframe_channel_change_not_tracked_internally` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E021_midframe_channel_change_not_tracked_internally.md) |
| ✅ | 22 | `E022_header_after_comma_recovery_mode0_restarts_cleanly` | stmt=84.05, branch=70.86, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E022_header_after_comma_recovery_mode0_restarts_cleanly.md) |
| ✅ | 23 | `E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header.md) |
| ✅ | 24 | `E024_header_immediately_after_run_prepare_blocked` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E024_header_immediately_after_run_prepare_blocked.md) |
| ✅ | 25 | `E025_header_immediately_after_sync_blocked` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E025_header_immediately_after_sync_blocked.md) |
| ✅ | 26 | `E026_header_immediately_after_running_opened_next_cycle` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E026_header_immediately_after_running_opened_next_cycle.md) |
| ✅ | 27 | `E027_header_immediately_after_terminating_blocked` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E027_header_immediately_after_terminating_blocked.md) |
| ✅ | 28 | `E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable` | stmt=84.05, branch=71.05, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.63 | [case](../cases/E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable.md) |
| ✅ | 29 | `E029_header_following_write_zero_then_one_requires_clocked_enable` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.79 | [case](../cases/E029_header_following_write_zero_then_one_requires_clocked_enable.md) |
| ✅ | 30 | `E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.79 | [case](../cases/E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt.md) |
| ✅ | 31 | `E031_frame_number_0000_propagates` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.79 | [case](../cases/E031_frame_number_0000_propagates.md) |
| ✅ | 32 | `E032_frame_number_ffff_propagates` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.94 | [case](../cases/E032_frame_number_ffff_propagates.md) |
| ✅ | 33 | `E033_frame_number_wrap_ffff_to_0000` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.94 | [case](../cases/E033_frame_number_wrap_ffff_to_0000.md) |
| ✅ | 34 | `E034_frame_len_zero_long_goes_crc_path_directly` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.94 | [case](../cases/E034_frame_len_zero_long_goes_crc_path_directly.md) |
| ✅ | 35 | `E035_frame_len_one_long_single_word` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.94 | [case](../cases/E035_frame_len_one_long_single_word.md) |
| ✅ | 36 | `E036_frame_len_two_long_two_words` | stmt=85.21, branch=73.03, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.94 | [case](../cases/E036_frame_len_two_long_two_words.md) |
| ✅ | 37 | `E037_frame_len_max_long_1023_words` | stmt=85.21, branch=75.00, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=49.11 | [case](../cases/E037_frame_len_max_long_1023_words.md) |
| ✅ | 38 | `E038_frame_len_zero_short_goes_crc_path_directly` | stmt=89.49, branch=80.92, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=60.00, toggle=49.90 | [case](../cases/E038_frame_len_zero_short_goes_crc_path_directly.md) |
| ✅ | 39 | `E039_frame_len_one_short_single_word` | stmt=89.49, branch=80.92, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=60.00, toggle=49.90 | [case](../cases/E039_frame_len_one_short_single_word.md) |
| ✅ | 40 | `E040_frame_len_two_short_even_pair` | stmt=93.00, branch=85.53, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=50.42 | [case](../cases/E040_frame_len_two_short_even_pair.md) |
| ✅ | 41 | `E041_frame_len_three_short_odd_pair_plus_residual` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=50.89 | [case](../cases/E041_frame_len_three_short_odd_pair_plus_residual.md) |
| ✅ | 42 | `E042_frame_len_max_short_1023_words` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E042_frame_len_max_short_1023_words.md) |
| ✅ | 43 | `E043_frame_flags_short_bit_exact_100` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E043_frame_flags_short_bit_exact_100.md) |
| ✅ | 44 | `E044_frame_flags_nonshort_000_long` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E044_frame_flags_nonshort_000_long.md) |
| ✅ | 45 | `E045_frame_flags_cec_101_uses_long_output_path` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E045_frame_flags_cec_101_uses_long_output_path.md) |
| ✅ | 46 | `E046_word_count_headerinfo_for_zero_length_frame` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E046_word_count_headerinfo_for_zero_length_frame.md) |
| ✅ | 47 | `E047_word_count_headerinfo_for_single_hit_frame` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E047_word_count_headerinfo_for_single_hit_frame.md) |
| ✅ | 48 | `E048_word_count_headerinfo_for_max_length_frame` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E048_word_count_headerinfo_for_max_length_frame.md) |
| ✅ | 49 | `E049_startofpacket_asserts_when_n_word_cnt_becomes1` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E049_startofpacket_asserts_when_n_word_cnt_becomes1.md) |
| ✅ | 50 | `E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len.md) |
| ✅ | 51 | `E051_long_hit_channel_00000_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E051_long_hit_channel_00000_boundary.md) |
| ✅ | 52 | `E052_long_hit_channel_11111_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E052_long_hit_channel_11111_boundary.md) |
| ✅ | 53 | `E053_long_tcc_zero_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E053_long_tcc_zero_boundary.md) |
| ✅ | 54 | `E054_long_tcc_allones_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E054_long_tcc_allones_boundary.md) |
| ✅ | 55 | `E055_long_tfine_zero_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E055_long_tfine_zero_boundary.md) |
| ✅ | 56 | `E056_long_tfine_allones_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E056_long_tfine_allones_boundary.md) |
| ✅ | 57 | `E057_long_ecc_zero_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E057_long_ecc_zero_boundary.md) |
| ✅ | 58 | `E058_long_ecc_allones_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E058_long_ecc_allones_boundary.md) |
| ✅ | 59 | `E059_short_hit_channel_00000_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E059_short_hit_channel_00000_boundary.md) |
| ✅ | 60 | `E060_short_hit_channel_11111_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E060_short_hit_channel_11111_boundary.md) |
| ✅ | 61 | `E061_short_tcc_zero_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E061_short_tcc_zero_boundary.md) |
| ✅ | 62 | `E062_short_tcc_allones_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E062_short_tcc_allones_boundary.md) |
| ✅ | 63 | `E063_short_tfine_zero_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=51.88 | [case](../cases/E063_short_tfine_zero_boundary.md) |
| ✅ | 64 | `E064_short_tfine_allones_boundary` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E064_short_tfine_allones_boundary.md) |
| ✅ | 65 | `E065_short_even_hit_extra_nibble_boundary_0` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E065_short_even_hit_extra_nibble_boundary_0.md) |
| ✅ | 66 | `E066_short_even_hit_extra_nibble_boundary_f` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E066_short_even_hit_extra_nibble_boundary_f.md) |
| ✅ | 67 | `E067_short_odd_hit_saved_nibble_boundary_0` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E067_short_odd_hit_saved_nibble_boundary_0.md) |
| ✅ | 68 | `E068_short_odd_hit_saved_nibble_boundary_f` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E068_short_odd_hit_saved_nibble_boundary_f.md) |
| ✅ | 69 | `E069_short_last_odd_hit_goes_direct_to_crc` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E069_short_last_odd_hit_goes_direct_to_crc.md) |
| ✅ | 70 | `E070_short_last_even_hit_goes_from_unpack_extra_to_crc` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E070_short_last_even_hit_goes_from_unpack_extra_to_crc.md) |
| ✅ | 71 | `E071_crc_magic_match_7ff2_only_case` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E071_crc_magic_match_7ff2_only_case.md) |
| ✅ | 72 | `E072_crc_magic_onebit_off_is_error` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E072_crc_magic_onebit_off_is_error.md) |
| ✅ | 73 | `E073_crc_error_only_asserts_on_eop_cycle` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E073_crc_error_only_asserts_on_eop_cycle.md) |
| ✅ | 74 | `E074_hit_error_latches_across_multi_byte_long_word` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E074_hit_error_latches_across_multi_byte_long_word.md) |
| ✅ | 75 | `E075_hit_error_clears_after_word_commit` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E075_hit_error_clears_after_word_commit.md) |
| ✅ | 76 | `E076_hit_error_latches_across_multi_byte_short_word` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.03 | [case](../cases/E076_hit_error_latches_across_multi_byte_short_word.md) |
| ✅ | 77 | `E077_loss_sync_on_header_byte_sets_error2_entire_frame` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.24 | [case](../cases/E077_loss_sync_on_header_byte_sets_error2_entire_frame.md) |
| ✅ | 78 | `E078_loss_sync_only_on_last_byte_still_sets_error2` | stmt=93.77, branch=86.18, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.24 | [case](../cases/E078_loss_sync_only_on_last_byte_still_sets_error2.md) |
| ✅ | 79 | `E079_parity_error_on_nonhit_state_does_not_set_error0` | stmt=94.16, branch=86.84, cond=82.76, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.45 | [case](../cases/E079_parity_error_on_nonhit_state_does_not_set_error0.md) |
| ✅ | 80 | `E080_decode_error_on_nonhit_state_does_not_set_error0` | stmt=94.16, branch=86.84, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.55 | [case](../cases/E080_decode_error_on_nonhit_state_does_not_set_error0.md) |
| ✅ | 81 | `E081_mode_halt0_comma_on_frame_counter_byte_aborts` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E081_mode_halt0_comma_on_frame_counter_byte_aborts.md) |
| ✅ | 82 | `E082_mode_halt0_comma_on_event_counter_byte_aborts` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E082_mode_halt0_comma_on_event_counter_byte_aborts.md) |
| ✅ | 83 | `E083_mode_halt0_comma_on_unpack_byte_aborts` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E083_mode_halt0_comma_on_unpack_byte_aborts.md) |
| ✅ | 84 | `E084_mode_halt1_comma_on_frame_counter_byte_holds_state` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E084_mode_halt1_comma_on_frame_counter_byte_holds_state.md) |
| ✅ | 85 | `E085_mode_halt1_comma_on_event_counter_byte_holds_state` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E085_mode_halt1_comma_on_event_counter_byte_holds_state.md) |
| ✅ | 86 | `E086_mode_halt1_comma_on_unpack_byte_holds_state` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E086_mode_halt1_comma_on_unpack_byte_holds_state.md) |
| ✅ | 87 | `E087_recovery_after_mode0_abort_starts_next_clean_frame` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E087_recovery_after_mode0_abort_starts_next_clean_frame.md) |
| ✅ | 88 | `E088_mode1_resume_with_error_bits_preserved` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E088_mode1_resume_with_error_bits_preserved.md) |
| ✅ | 89 | `E089_clean_frame_after_bad_crc_clears_error1` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E089_clean_frame_after_bad_crc_clears_error1.md) |
| ✅ | 90 | `E090_clean_frame_after_hit_error_clears_error0` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E090_clean_frame_after_hit_error_clears_error0.md) |
| ✅ | 91 | `E091_csr_addr_width1_aliases_words0_1_only` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E091_csr_addr_width1_aliases_words0_1_only.md) |
| ✅ | 92 | `E092_csr_addr_width2_supports_words0_2_exactly` | stmt=94.94, branch=88.16, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.76 | [case](../cases/E092_csr_addr_width2_supports_words0_2_exactly.md) |
| ✅ | 93 | `E093_csr_addr_width8_high_unused_reads_zero` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.16 | [case](../cases/E093_csr_addr_width8_high_unused_reads_zero.md) |
| ✅ | 94 | `E094_channel_width1_headerinfo_maxchannel1` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.16 | [case](../cases/E094_channel_width1_headerinfo_maxchannel1.md) |
| ✅ | 95 | `E095_channel_width1_output_channel_singlebit` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.16 | [case](../cases/E095_channel_width1_output_channel_singlebit.md) |
| ✅ | 96 | `E096_channel_width4_default_maxchannel15` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.16 | [case](../cases/E096_channel_width4_default_maxchannel15.md) |
| ✅ | 97 | `E097_channel_width8_large_channel_passthrough` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.81 | [case](../cases/E097_channel_width8_large_channel_passthrough.md) |
| ✅ | 98 | `E098_channel_width_change_between_builds_no_scoreboard_rewrite` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.81 | [case](../cases/E098_channel_width_change_between_builds_no_scoreboard_rewrite.md) |
| ✅ | 99 | `E099_csr_read_and_write_same_cycle_prioritizes_read` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.81 | [case](../cases/E099_csr_read_and_write_same_cycle_prioritizes_read.md) |
| ✅ | 100 | `E100_write_control_zero_midframe_affects_next_fs_idle_only` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.81 | [case](../cases/E100_write_control_zero_midframe_affects_next_fs_idle_only.md) |
| ✅ | 101 | `E101_write_control_one_midframe_does_not_create_spurious_start` | stmt=94.94, branch=88.82, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.81 | [case](../cases/E101_write_control_one_midframe_does_not_create_spurious_start.md) |
| ✅ | 102 | `E102_control_status_read_during_frame_updates_after_headerinfo` | stmt=95.72, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.92 | [case](../cases/E102_control_status_read_during_frame_updates_after_headerinfo.md) |
| ✅ | 103 | `E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.92 | [case](../cases/E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value.md) |
| ✅ | 104 | `E104_frame_counter_read_on_sop_cycle_returns_preincremented_value` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.92 | [case](../cases/E104_frame_counter_read_on_sop_cycle_returns_preincremented_value.md) |
| ✅ | 105 | `E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.92 | [case](../cases/E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference.md) |
| ✅ | 106 | `E106_waitrequest_release_during_reset` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E106_waitrequest_release_during_reset.md) |
| ✅ | 107 | `E107_headerinfo_channel_width_tracks_generic` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E107_headerinfo_channel_width_tracks_generic.md) |
| ✅ | 108 | `E108_hit_type0_channel_width_tracks_generic` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E108_hit_type0_channel_width_tracks_generic.md) |
| ✅ | 109 | `E109_rx_channel_width_tracks_generic` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E109_rx_channel_width_tracks_generic.md) |
| ✅ | 110 | `E110_debug_lv0_and_debug_lv2_behave_identically_functionally` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E110_debug_lv0_and_debug_lv2_behave_identically_functionally.md) |
| ✅ | 111 | `E111_terminating_before_header_byte_blocks_frame` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E111_terminating_before_header_byte_blocks_frame.md) |
| ✅ | 112 | `E112_terminating_on_header_byte_same_cycle_depends_on_old_state` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E112_terminating_on_header_byte_same_cycle_depends_on_old_state.md) |
| ✅ | 113 | `E113_terminating_after_header_before_len_allows_completion` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E113_terminating_after_header_before_len_allows_completion.md) |
| ✅ | 114 | `E114_terminating_after_first_long_hit_before_second_hit` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.53 | [case](../cases/E114_terminating_after_first_long_hit_before_second_hit.md) |
| ✅ | 115 | `E115_terminating_after_first_short_hit_before_second_hit` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.74 | [case](../cases/E115_terminating_after_first_short_hit_before_second_hit.md) |
| ✅ | 116 | `E116_terminating_during_crc_calc` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.74 | [case](../cases/E116_terminating_during_crc_calc.md) |
| ✅ | 117 | `E117_terminating_during_crc_check` | stmt=96.11, branch=90.13, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.74 | [case](../cases/E117_terminating_during_crc_check.md) |
| ✅ | 118 | `E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode` | stmt=96.50, branch=90.79, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.74 | [case](../cases/E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode.md) |
| ✅ | 119 | `E119_running_to_idle_same_cycle_header_follows_old_state_running` | stmt=96.50, branch=90.79, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.74 | [case](../cases/E119_running_to_idle_same_cycle_header_follows_old_state_running.md) |
| ✅ | 120 | `E120_running_to_run_prepare_same_cycle_resets_next_cycle` | stmt=96.50, branch=90.79, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E120_running_to_run_prepare_same_cycle_resets_next_cycle.md) |
| ✅ | 121 | `E121_repeated_terminating_words_do_not_duplicate_eop` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E121_repeated_terminating_words_do_not_duplicate_eop.md) |
| ✅ | 122 | `E122_repeated_idle_words_do_not_force_extra_output` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E122_repeated_idle_words_do_not_force_extra_output.md) |
| ✅ | 123 | `E123_header_after_error_state_needs_new_running_or_idle_monitoring` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E123_header_after_error_state_needs_new_running_or_idle_monitoring.md) |
| ✅ | 124 | `E124_truncated_long_frame_never_asserts_eop` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E124_truncated_long_frame_never_asserts_eop.md) |
| ✅ | 125 | `E125_truncated_short_frame_never_asserts_eop` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E125_truncated_short_frame_never_asserts_eop.md) |
| ✅ | 126 | `E126_excess_bytes_after_declared_long_frame_ignored_until_next_header` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E126_excess_bytes_after_declared_long_frame_ignored_until_next_header.md) |
| ✅ | 127 | `E127_excess_bytes_after_declared_short_frame_ignored_until_next_header` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E127_excess_bytes_after_declared_short_frame_ignored_until_next_header.md) |
| ✅ | 128 | `E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating.md) |
| ✅ | 129 | `E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut.md) |
| ✅ | 130 | `E130_upgrade_plan_current_ready_does_not_wait_for_drain` | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | [case](../cases/E130_upgrade_plan_current_ready_does_not_wait_for_drain.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_
