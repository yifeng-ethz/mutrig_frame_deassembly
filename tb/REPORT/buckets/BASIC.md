# ⚠️ BASIC bucket

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
| ✅ | stmt | 97.67 | 95.0 |
| ✅ | branch | 92.76 | 90.0 |
| ℹ️ | cond | 79.31 | - |
| ℹ️ | expr | 98.77 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ⚠️ | toggle | 39.73 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `B001_reset_idle_outputs_quiet` | stmt=79.77, branch=63.58, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.84 | [case](../cases/B001_reset_idle_outputs_quiet.md) |
| ✅ | 2 | `B002_idle_force_go_monitor_path_open` | stmt=80.16, branch=64.24, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.84 | [case](../cases/B002_idle_force_go_monitor_path_open.md) |
| ✅ | 3 | `B003_idle_force_go_ignores_csr_mask` | stmt=80.16, branch=64.24, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.84 | [case](../cases/B003_idle_force_go_ignores_csr_mask.md) |
| ✅ | 4 | `B004_running_needs_csr_enable_high` | stmt=80.16, branch=64.24, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.26 | [case](../cases/B004_running_needs_csr_enable_high.md) |
| ✅ | 5 | `B005_running_mask_low_blocks_header_start` | stmt=81.32, branch=66.23, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.41 | [case](../cases/B005_running_mask_low_blocks_header_start.md) |
| ✅ | 6 | `B006_run_prepare_clears_parser_state` | stmt=81.32, branch=66.23, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.57 | [case](../cases/B006_run_prepare_clears_parser_state.md) |
| ✅ | 7 | `B007_sync_keeps_parser_closed` | stmt=81.32, branch=66.23, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.57 | [case](../cases/B007_sync_keeps_parser_closed.md) |
| ✅ | 8 | `B008_running_opens_header_detection` | stmt=81.32, branch=66.23, cond=48.28, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.99 | [case](../cases/B008_running_opens_header_detection.md) |
| ✅ | 9 | `B009_terminating_blocks_new_header_from_idle` | stmt=84.05, branch=71.52, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=18.87 | [case](../cases/B009_terminating_blocks_new_header_from_idle.md) |
| ✅ | 10 | `B010_terminating_allows_open_frame_to_finish` | stmt=84.05, branch=71.52, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=18.87 | [case](../cases/B010_terminating_allows_open_frame_to_finish.md) |
| ✅ | 11 | `B011_ctrl_unknown_word_enters_error` | stmt=85.21, branch=72.85, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=18.93 | [case](../cases/B011_ctrl_unknown_word_enters_error.md) |
| ✅ | 12 | `B012_ctrl_ready_high_with_valid` | stmt=85.21, branch=72.85, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=18.93 | [case](../cases/B012_ctrl_ready_high_with_valid.md) |
| ✅ | 13 | `B013_ctrl_ready_high_without_valid` | stmt=85.21, branch=72.85, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=18.93 | [case](../cases/B013_ctrl_ready_high_without_valid.md) |
| ✅ | 14 | `B014_ctrl_decode_idle_force_go` | stmt=85.21, branch=72.85, cond=65.52, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.03 | [case](../cases/B014_ctrl_decode_idle_force_go.md) |
| ✅ | 15 | `B015_ctrl_decode_run_prepare_disable` | stmt=85.60, branch=73.51, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.24 | [case](../cases/B015_ctrl_decode_run_prepare_disable.md) |
| ✅ | 16 | `B016_ctrl_decode_sync_disable` | stmt=85.60, branch=73.51, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.24 | [case](../cases/B016_ctrl_decode_sync_disable.md) |
| ✅ | 17 | `B017_ctrl_decode_running_go` | stmt=85.60, branch=73.51, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.40 | [case](../cases/B017_ctrl_decode_running_go.md) |
| ✅ | 18 | `B018_ctrl_decode_terminating_stop_new_headers` | stmt=85.60, branch=73.51, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.40 | [case](../cases/B018_ctrl_decode_terminating_stop_new_headers.md) |
| ✅ | 19 | `B019_ctrl_decode_link_test_disable` | stmt=85.99, branch=74.17, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.55 | [case](../cases/B019_ctrl_decode_link_test_disable.md) |
| ✅ | 20 | `B020_ctrl_decode_sync_test_disable` | stmt=86.38, branch=74.83, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=19.71 | [case](../cases/B020_ctrl_decode_sync_test_disable.md) |
| ✅ | 21 | `B021_ctrl_decode_reset_disable` | stmt=86.38, branch=74.83, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.33 | [case](../cases/B021_ctrl_decode_reset_disable.md) |
| ✅ | 22 | `B022_ctrl_decode_out_of_daq_disable` | stmt=86.77, branch=75.50, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.49 | [case](../cases/B022_ctrl_decode_out_of_daq_disable.md) |
| ✅ | 23 | `B023_csr_control_default_enable` | stmt=86.77, branch=75.50, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.49 | [case](../cases/B023_csr_control_default_enable.md) |
| ✅ | 24 | `B024_csr_control_write_zero_masks_running` | stmt=86.77, branch=75.50, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.49 | [case](../cases/B024_csr_control_write_zero_masks_running.md) |
| ✅ | 25 | `B025_csr_control_write_one_unmasks_running` | stmt=86.77, branch=75.50, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.59 | [case](../cases/B025_csr_control_write_one_unmasks_running.md) |
| ✅ | 26 | `B026_csr_read_addr0_control_status` | stmt=87.94, branch=76.82, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.80 | [case](../cases/B026_csr_read_addr0_control_status.md) |
| ✅ | 27 | `B027_csr_read_addr1_crc_counter` | stmt=88.33, branch=77.48, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=20.91 | [case](../cases/B027_csr_read_addr1_crc_counter.md) |
| ✅ | 28 | `B028_csr_read_addr2_frame_counter` | stmt=88.72, branch=78.15, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.01 | [case](../cases/B028_csr_read_addr2_frame_counter.md) |
| ✅ | 29 | `B029_csr_read_unused_word_zero` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.01 | [case](../cases/B029_csr_read_unused_word_zero.md) |
| ✅ | 30 | `B030_csr_write_unused_word_no_side_effect` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.01 | [case](../cases/B030_csr_write_unused_word_no_side_effect.md) |
| ✅ | 31 | `B031_csr_waitrequest_low_on_read` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.01 | [case](../cases/B031_csr_waitrequest_low_on_read.md) |
| ✅ | 32 | `B032_csr_waitrequest_low_on_write` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.17 | [case](../cases/B032_csr_waitrequest_low_on_write.md) |
| ✅ | 33 | `B033_csr_waitrequest_high_when_idle` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.17 | [case](../cases/B033_csr_waitrequest_high_when_idle.md) |
| ✅ | 34 | `B034_csr_reserved_control_bits_roundtrip` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.17 | [case](../cases/B034_csr_reserved_control_bits_roundtrip.md) |
| ✅ | 35 | `B035_fs_idle_requires_k28_0_and_kchar` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=21.17 | [case](../cases/B035_fs_idle_requires_k28_0_and_kchar.md) |
| ✅ | 36 | `B036_data_byte_without_kchar_no_start` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=21.32 | [case](../cases/B036_data_byte_without_kchar_no_start.md) |
| ✅ | 37 | `B037_header_byte_with_enable_low_no_start` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=21.32 | [case](../cases/B037_header_byte_with_enable_low_no_start.md) |
| ✅ | 38 | `B038_header_byte_with_enable_high_starts_frame` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=21.32 | [case](../cases/B038_header_byte_with_enable_high_starts_frame.md) |
| ✅ | 39 | `B039_long_zero_hit_frame_headerinfo_pulse` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=21.32 | [case](../cases/B039_long_zero_hit_frame_headerinfo_pulse.md) |
| ✅ | 40 | `B040_long_zero_hit_frame_no_hit_valid` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=21.32 | [case](../cases/B040_long_zero_hit_frame_no_hit_valid.md) |
| ✅ | 41 | `B041_long_one_hit_sop_and_eop_same_transfer` | stmt=88.72, branch=78.81, cond=68.97, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=21.32 | [case](../cases/B041_long_one_hit_sop_and_eop_same_transfer.md) |
| ✅ | 42 | `B042_long_two_hit_sop_first_only` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B042_long_two_hit_sop_first_only.md) |
| ✅ | 43 | `B043_long_two_hit_eop_last_only` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B043_long_two_hit_eop_last_only.md) |
| ✅ | 44 | `B044_long_frame_number_lsb_capture` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B044_long_frame_number_lsb_capture.md) |
| ✅ | 45 | `B045_long_frame_number_msb_capture` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B045_long_frame_number_msb_capture.md) |
| ✅ | 46 | `B046_long_frame_flags_capture` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B046_long_frame_flags_capture.md) |
| ✅ | 47 | `B047_long_frame_len_capture` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B047_long_frame_len_capture.md) |
| ✅ | 48 | `B048_long_hit_channel_unpack` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B048_long_hit_channel_unpack.md) |
| ✅ | 49 | `B049_long_hit_tcc_unpack` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B049_long_hit_tcc_unpack.md) |
| ✅ | 50 | `B050_long_hit_tfine_unpack` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B050_long_hit_tfine_unpack.md) |
| ✅ | 51 | `B051_long_hit_ecc_unpack` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B051_long_hit_ecc_unpack.md) |
| ✅ | 52 | `B052_long_hit_eflag_unpack` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B052_long_hit_eflag_unpack.md) |
| ✅ | 53 | `B053_long_t_badhit_raises_error0` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B053_long_t_badhit_raises_error0.md) |
| ✅ | 54 | `B054_long_e_badhit_raises_error0` | stmt=89.11, branch=81.46, cond=72.41, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.42 | [case](../cases/B054_long_e_badhit_raises_error0.md) |
| ✅ | 55 | `B055_long_parity_error_raises_error0` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.52 | [case](../cases/B055_long_parity_error_raises_error0.md) |
| ✅ | 56 | `B056_long_decode_error_raises_error0` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.52 | [case](../cases/B056_long_decode_error_raises_error0.md) |
| ✅ | 57 | `B057_long_loss_sync_propagates_error2` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B057_long_loss_sync_propagates_error2.md) |
| ✅ | 58 | `B058_long_crc_good_keeps_error1_low` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B058_long_crc_good_keeps_error1_low.md) |
| ✅ | 59 | `B059_long_crc_bad_raises_error1_on_eop` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B059_long_crc_bad_raises_error1_on_eop.md) |
| ✅ | 60 | `B060_long_crc_bad_increments_crc_counter` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B060_long_crc_bad_increments_crc_counter.md) |
| ✅ | 61 | `B061_sop_increments_frame_counter_head` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B061_sop_increments_frame_counter_head.md) |
| ✅ | 62 | `B062_eop_increments_frame_counter_tail` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B062_eop_increments_frame_counter_tail.md) |
| ✅ | 63 | `B063_frame_counter_register_is_head_minus_tail` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=28.73 | [case](../cases/B063_frame_counter_register_is_head_minus_tail.md) |
| ✅ | 64 | `B064_run_prepare_resets_head_tail_counters` | stmt=89.11, branch=81.46, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=29.04 | [case](../cases/B064_run_prepare_resets_head_tail_counters.md) |
| ✅ | 65 | `B065_short_zero_hit_frame_headerinfo_pulse` | stmt=89.88, branch=82.78, cond=75.86, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=29.51 | [case](../cases/B065_short_zero_hit_frame_headerinfo_pulse.md) |
| ✅ | 66 | `B066_short_one_hit_sop_eop_same_transfer` | stmt=93.39, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=30.55 | [case](../cases/B066_short_one_hit_sop_eop_same_transfer.md) |
| ✅ | 67 | `B067_short_two_hit_even_pairing` | stmt=96.89, branch=92.05, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=33.68 | [case](../cases/B067_short_two_hit_even_pairing.md) |
| ✅ | 68 | `B068_short_three_hit_odd_pairing` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=35.77 | [case](../cases/B068_short_three_hit_odd_pairing.md) |
| ✅ | 69 | `B069_short_four_hit_even_followup` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B069_short_four_hit_even_followup.md) |
| ✅ | 70 | `B070_short_mode_selected_by_flags_100` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B070_short_mode_selected_by_flags_100.md) |
| ✅ | 71 | `B071_short_path_zeroes_ecc_field` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B071_short_path_zeroes_ecc_field.md) |
| ✅ | 72 | `B072_short_path_uses_bit0_as_eflag` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B072_short_path_uses_bit0_as_eflag.md) |
| ✅ | 73 | `B073_short_hit_channel_unpack` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B073_short_hit_channel_unpack.md) |
| ✅ | 74 | `B074_short_hit_tcc_unpack` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B074_short_hit_tcc_unpack.md) |
| ✅ | 75 | `B075_short_hit_tfine_unpack` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B075_short_hit_tfine_unpack.md) |
| ✅ | 76 | `B076_short_parity_error_raises_error0` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B076_short_parity_error_raises_error0.md) |
| ✅ | 77 | `B077_short_decode_error_raises_error0` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.58 | [case](../cases/B077_short_decode_error_raises_error0.md) |
| ✅ | 78 | `B078_short_loss_sync_propagates_error2` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B078_short_loss_sync_propagates_error2.md) |
| ✅ | 79 | `B079_short_crc_bad_raises_error1_on_last_hit` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B079_short_crc_bad_raises_error1_on_last_hit.md) |
| ✅ | 80 | `B080_short_crc_counter_accumulates` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B080_short_crc_counter_accumulates.md) |
| ✅ | 81 | `B081_short_frame_counter_updates` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B081_short_frame_counter_updates.md) |
| ✅ | 82 | `B082_headerinfo_one_pulse_per_frame` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B082_headerinfo_one_pulse_per_frame.md) |
| ✅ | 83 | `B083_headerinfo_channel_matches_input` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B083_headerinfo_channel_matches_input.md) |
| ✅ | 84 | `B084_headerinfo_frame_number_field_map` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B084_headerinfo_frame_number_field_map.md) |
| ✅ | 85 | `B085_headerinfo_word_count_field_map` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B085_headerinfo_word_count_field_map.md) |
| ✅ | 86 | `B086_headerinfo_frame_len_field_map` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B086_headerinfo_frame_len_field_map.md) |
| ✅ | 87 | `B087_headerinfo_flags_field_map` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B087_headerinfo_flags_field_map.md) |
| ✅ | 88 | `B088_output_channel_matches_input_channel` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B088_output_channel_matches_input_channel.md) |
| ✅ | 89 | `B089_hit_valid_only_on_complete_word` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B089_hit_valid_only_on_complete_word.md) |
| ✅ | 90 | `B090_no_hit_valid_in_crc_states` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B090_no_hit_valid_in_crc_states.md) |
| ✅ | 91 | `B091_no_sop_without_valid` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B091_no_sop_without_valid.md) |
| ✅ | 92 | `B092_no_eop_without_valid` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B092_no_eop_without_valid.md) |
| ✅ | 93 | `B093_idle_comma_does_not_create_frame` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B093_idle_comma_does_not_create_frame.md) |
| ✅ | 94 | `B094_mode_halt0_comma_inside_frame_drops_to_idle` | stmt=97.67, branch=92.72, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B094_mode_halt0_comma_inside_frame_drops_to_idle.md) |
| ✅ | 95 | `B095_mode_halt1_comma_inside_frame_holds_state` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B095_mode_halt1_comma_inside_frame_holds_state.md) |
| ✅ | 96 | `B096_running_mask_toggle_between_frames` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B096_running_mask_toggle_between_frames.md) |
| ✅ | 97 | `B097_mask_toggle_midframe_does_not_abort_current_frame` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B097_mask_toggle_midframe_does_not_abort_current_frame.md) |
| ✅ | 98 | `B098_idle_monitoring_accepts_header_without_running` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B098_idle_monitoring_accepts_header_without_running.md) |
| ✅ | 99 | `B099_idle_monitoring_still_ignores_csr_zero` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B099_idle_monitoring_still_ignores_csr_zero.md) |
| ✅ | 100 | `B100_link_test_keeps_outputs_quiet` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B100_link_test_keeps_outputs_quiet.md) |
| ✅ | 101 | `B101_sync_test_keeps_outputs_quiet` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=38.79 | [case](../cases/B101_sync_test_keeps_outputs_quiet.md) |
| ✅ | 102 | `B102_reset_state_keeps_outputs_quiet` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.26 | [case](../cases/B102_reset_state_keeps_outputs_quiet.md) |
| ✅ | 103 | `B103_out_of_daq_keeps_outputs_quiet` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.26 | [case](../cases/B103_out_of_daq_keeps_outputs_quiet.md) |
| ✅ | 104 | `B104_error_state_keeps_outputs_quiet` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B104_error_state_keeps_outputs_quiet.md) |
| ✅ | 105 | `B105_run_prepare_sync_running_spine` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B105_run_prepare_sync_running_spine.md) |
| ✅ | 106 | `B106_running_to_terminating_during_header_bytes` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B106_running_to_terminating_during_header_bytes.md) |
| ✅ | 107 | `B107_running_to_terminating_during_unpack_finishes_frame` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B107_running_to_terminating_during_unpack_finishes_frame.md) |
| ✅ | 108 | `B108_terminating_to_idle_returns_quiescent` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B108_terminating_to_idle_returns_quiescent.md) |
| ✅ | 109 | `B109_terminating_blocks_second_header_after_first_eop` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B109_terminating_blocks_second_header_after_first_eop.md) |
| ✅ | 110 | `B110_idle_after_terminating_restores_monitoring` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B110_idle_after_terminating_restores_monitoring.md) |
| ✅ | 111 | `B111_system_run_sequence_spine_matches_upgrade_plan` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B111_system_run_sequence_spine_matches_upgrade_plan.md) |
| ✅ | 112 | `B112_long_good_frames_leave_crc_counter_stable` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B112_long_good_frames_leave_crc_counter_stable.md) |
| ✅ | 113 | `B113_bad_frames_accumulate_crc_counter` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B113_bad_frames_accumulate_crc_counter.md) |
| ✅ | 114 | `B114_mixed_good_bad_crc_count_matches_bad_frames` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B114_mixed_good_bad_crc_count_matches_bad_frames.md) |
| ✅ | 115 | `B115_frame_counter_zero_after_complete_frame` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B115_frame_counter_zero_after_complete_frame.md) |
| ✅ | 116 | `B116_frame_counter_nonzero_during_open_frame` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B116_frame_counter_nonzero_during_open_frame.md) |
| ✅ | 117 | `B117_status_updates_on_new_frame_flags` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B117_status_updates_on_new_frame_flags.md) |
| ✅ | 118 | `B118_status_holds_last_flags_between_frames` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B118_status_holds_last_flags_between_frames.md) |
| ✅ | 119 | `B119_clean_short_hit_clears_all_error_bits` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B119_clean_short_hit_clears_all_error_bits.md) |
| ✅ | 120 | `B120_clean_long_hit_clears_all_error_bits` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B120_clean_long_hit_clears_all_error_bits.md) |
| ✅ | 121 | `B121_control_write_does_not_clear_crc_counter` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B121_control_write_does_not_clear_crc_counter.md) |
| ✅ | 122 | `B122_control_write_does_not_clear_frame_counter` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B122_control_write_does_not_clear_frame_counter.md) |
| ✅ | 123 | `B123_channel_change_between_frames_is_observed` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B123_channel_change_between_frames_is_observed.md) |
| ✅ | 124 | `B124_channel_change_midframe_is_prohibited_in_harness` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B124_channel_change_midframe_is_prohibited_in_harness.md) |
| ✅ | 125 | `B125_headerinfo_and_hit_channel_track_same_asic` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B125_headerinfo_and_hit_channel_track_same_asic.md) |
| ✅ | 126 | `B126_zero_hit_long_then_zero_hit_short_sequence` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B126_zero_hit_long_then_zero_hit_short_sequence.md) |
| ✅ | 127 | `B127_good_long_then_good_short_sequence` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.42 | [case](../cases/B127_good_long_then_good_short_sequence.md) |
| ✅ | 128 | `B128_run_prepare_after_crc_error_resets_crc_counter` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.73 | [case](../cases/B128_run_prepare_after_crc_error_resets_crc_counter.md) |
| ✅ | 129 | `B129_upgrade_contract_no_fresh_header_after_terminating` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.73 | [case](../cases/B129_upgrade_contract_no_fresh_header_after_terminating.md) |
| ✅ | 130 | `B130_upgrade_contract_terminal_frame_finishes_once` | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.73 | [case](../cases/B130_upgrade_contract_terminal_frame_finishes_once.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_
