# DV Coverage Tracking — mutrig_frame_deassembly

**DUT:** `frame_rcv_ip`  
**Date:** 2026-04-16  
**Execution baseline tracked here:** `after` RTL, seed `1`, isolated doc-case runs merged in bucket order, then documented case order

Execution-mode status:
- `isolated`: tracked in this file as ordered incremental gains from passing case-keyed UCDBs merged in bucket order, then documented case order
- `bucket_frame`: pending refresh from doc-case list runners
- `all_buckets_frame`: pending refresh from doc-case list runners

## Bucket Summary

| bucket | planned_cases | evidenced_cases | execution_mode_baseline | current_code_coverage_total | current_functional_coverage_total |
|---|---:|---:|---|---|---|
| BASIC | 130 | 130 | isolated ordered-merge | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 | 100.00% (130/130) |
| EDGE | 130 | 130 | isolated ordered-merge | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.25 | 100.00% (130/130) |
| PROF | 130 | 130 | isolated ordered-merge | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 | 100.00% (130/130) |
| ERROR | 130 | 130 | isolated ordered-merge | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.99 | 100.00% (130/130) |
| CROSS | n/a | 0 | none yet | n/a | 0.00% (no instrumented covergroups yet) |

## BASIC Bucket

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---|---|
| B001_reset_idle_outputs_quiet | d | stmt=55.25, branch=31.13, cond=11.11, expr=0.00, fsm_state=14.29, fsm_trans=0.00, toggle=0.52 | 0 | stmt=55.25, branch=31.13, cond=11.11, expr=0.00, fsm_state=14.29, fsm_trans=0.00, toggle=0.52 |
| B002_idle_force_go_monitor_path_open | d | stmt=26.46, branch=35.10, cond=37.04, expr=98.77, fsm_state=71.43, fsm_trans=40.00, toggle=15.48 | 0 | stmt=26.46, branch=35.10, cond=37.04, expr=98.77, fsm_state=71.43, fsm_trans=40.00, toggle=15.48 |
| B003_idle_force_go_ignores_csr_mask | d | stmt=3.89, branch=6.62, cond=11.11, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.99 | 0 | stmt=3.89, branch=6.62, cond=11.11, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.99 |
| B004_running_needs_csr_enable_high | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 |
| B005_running_mask_low_blocks_header_start | d | stmt=0.39, branch=0.66, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.05 | 0 | stmt=0.39, branch=0.66, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.05 |
| B006_run_prepare_clears_parser_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.31 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.31 |
| B007_sync_keeps_parser_closed | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B008_running_opens_header_detection | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 |
| B009_terminating_blocks_new_header_from_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B010_terminating_allows_open_frame_to_finish | d | stmt=0.39, branch=1.32, cond=7.41, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.39, branch=1.32, cond=7.41, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B011_ctrl_unknown_word_enters_error | d | stmt=1.17, branch=1.32, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.05 | 0 | stmt=1.17, branch=1.32, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.05 |
| B012_ctrl_ready_high_with_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B013_ctrl_ready_high_without_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B014_ctrl_decode_idle_force_go | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B015_ctrl_decode_run_prepare_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B016_ctrl_decode_sync_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.16 |
| B017_ctrl_decode_running_go | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B018_ctrl_decode_terminating_stop_new_headers | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B019_ctrl_decode_link_test_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B020_ctrl_decode_sync_test_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B021_ctrl_decode_reset_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B022_ctrl_decode_out_of_daq_disable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B023_csr_control_default_enable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B024_csr_control_write_zero_masks_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B025_csr_control_write_one_unmasks_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B026_csr_read_addr0_control_status | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B027_csr_read_addr1_crc_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B028_csr_read_addr2_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B029_csr_read_unused_word_zero | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B030_csr_write_unused_word_no_side_effect | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B031_csr_waitrequest_low_on_read | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B032_csr_waitrequest_low_on_write | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 |
| B033_csr_waitrequest_high_when_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B034_csr_reserved_control_bits_roundtrip | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B035_fs_idle_requires_k28_0_and_kchar | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B036_data_byte_without_kchar_no_start | d | stmt=0.00, branch=0.00, cond=3.70, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=3.70, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B037_header_byte_with_enable_low_no_start | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B038_header_byte_with_enable_high_starts_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B039_long_zero_hit_frame_headerinfo_pulse | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B040_long_zero_hit_frame_no_hit_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B041_long_one_hit_sop_and_eop_same_transfer | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 |
| B042_long_two_hit_sop_first_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B043_long_two_hit_eop_last_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B044_long_frame_number_lsb_capture | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B045_long_frame_number_msb_capture | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B046_long_frame_flags_capture | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B047_long_frame_len_capture | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B048_long_hit_channel_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B049_long_hit_tcc_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B050_long_hit_tfine_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B051_long_hit_ecc_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B052_long_hit_eflag_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B053_long_t_badhit_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B054_long_e_badhit_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B055_long_parity_error_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B056_long_decode_error_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B057_long_loss_sync_propagates_error2 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B058_long_crc_good_keeps_error1_low | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B059_long_crc_bad_raises_error1_on_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B060_long_crc_bad_increments_crc_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B061_sop_increments_frame_counter_head | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B062_eop_increments_frame_counter_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B063_frame_counter_register_is_head_minus_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B064_run_prepare_resets_head_tail_counters | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B065_short_zero_hit_frame_headerinfo_pulse | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B066_short_one_hit_sop_eop_same_transfer | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B067_short_two_hit_even_pairing | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B068_short_three_hit_odd_pairing | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B069_short_four_hit_even_followup | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B070_short_mode_selected_by_flags_100 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B071_short_path_zeroes_ecc_field | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B072_short_path_uses_bit0_as_eflag | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B073_short_hit_channel_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B074_short_hit_tcc_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B075_short_hit_tfine_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B076_short_parity_error_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B077_short_decode_error_raises_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B078_short_loss_sync_propagates_error2 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B079_short_crc_bad_raises_error1_on_last_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B080_short_crc_counter_accumulates | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B081_short_frame_counter_updates | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B082_headerinfo_one_pulse_per_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B083_headerinfo_channel_matches_input | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B084_headerinfo_frame_number_field_map | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B085_headerinfo_word_count_field_map | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B086_headerinfo_frame_len_field_map | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B087_headerinfo_flags_field_map | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B088_output_channel_matches_input_channel | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B089_hit_valid_only_on_complete_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B090_no_hit_valid_in_crc_states | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B091_no_sop_without_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B092_no_eop_without_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B093_idle_comma_does_not_create_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B094_mode_halt0_comma_inside_frame_drops_to_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B095_mode_halt1_comma_inside_frame_holds_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B096_running_mask_toggle_between_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B097_mask_toggle_midframe_does_not_abort_current_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B098_idle_monitoring_accepts_header_without_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B099_idle_monitoring_still_ignores_csr_zero | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B100_link_test_keeps_outputs_quiet | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B101_sync_test_keeps_outputs_quiet | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B102_reset_state_keeps_outputs_quiet | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B103_out_of_daq_keeps_outputs_quiet | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B104_error_state_keeps_outputs_quiet | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B105_run_prepare_sync_running_spine | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B106_running_to_terminating_during_header_bytes | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B107_running_to_terminating_during_unpack_finishes_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B108_terminating_to_idle_returns_quiescent | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B109_terminating_blocks_second_header_after_first_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B110_idle_after_terminating_restores_monitoring | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B111_system_run_sequence_spine_matches_upgrade_plan | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B112_long_good_frames_leave_crc_counter_stable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B113_bad_frames_accumulate_crc_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B114_mixed_good_bad_crc_count_matches_bad_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B115_frame_counter_zero_after_complete_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B116_frame_counter_nonzero_during_open_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B117_status_updates_on_new_frame_flags | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B118_status_holds_last_flags_between_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B119_clean_short_hit_clears_all_error_bits | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B120_clean_long_hit_clears_all_error_bits | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B121_control_write_does_not_clear_crc_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B122_control_write_does_not_clear_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B123_channel_change_between_frames_is_observed | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B124_channel_change_midframe_is_prohibited_in_harness | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B125_headerinfo_and_hit_channel_track_same_asic | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B126_zero_hit_long_then_zero_hit_short_sequence | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B127_good_long_then_good_short_sequence | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B128_run_prepare_after_crc_error_resets_crc_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.16 |
| B129_upgrade_contract_no_fresh_header_after_terminating | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| B130_upgrade_contract_terminal_frame_finishes_once | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |

### Bucket-Local Ordered Isolated Merge Trace

| step | case_id | merged_total_after_case |
|---:|---|---|
| 1 | B001_reset_idle_outputs_quiet | stmt=55.25, branch=31.13, cond=11.11, expr=0.00, fsm_state=14.29, fsm_trans=0.00, toggle=0.52 |
| 2 | B002_idle_force_go_monitor_path_open | stmt=81.71, branch=66.23, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.01 |
| 3 | B003_idle_force_go_ignores_csr_mask | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.00 |
| 4 | B004_running_needs_csr_enable_high | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.15 |
| 5 | B005_running_mask_low_blocks_header_start | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.21 |
| 6 | B006_run_prepare_clears_parser_state | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.52 |
| 7 | B007_sync_keeps_parser_closed | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.52 |
| 8 | B008_running_opens_header_detection | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 9 | B009_terminating_blocks_new_header_from_idle | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 10 | B010_terminating_allows_open_frame_to_finish | stmt=86.38, branch=74.83, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 11 | B011_ctrl_unknown_word_enters_error | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 12 | B012_ctrl_ready_high_with_valid | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 13 | B013_ctrl_ready_high_without_valid | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 14 | B014_ctrl_decode_idle_force_go | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 15 | B015_ctrl_decode_run_prepare_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 16 | B016_ctrl_decode_sync_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 17 | B017_ctrl_decode_running_go | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 18 | B018_ctrl_decode_terminating_stop_new_headers | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 19 | B019_ctrl_decode_link_test_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 20 | B020_ctrl_decode_sync_test_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 21 | B021_ctrl_decode_reset_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 22 | B022_ctrl_decode_out_of_daq_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 23 | B023_csr_control_default_enable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 24 | B024_csr_control_write_zero_masks_running | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 25 | B025_csr_control_write_one_unmasks_running | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 26 | B026_csr_read_addr0_control_status | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 27 | B027_csr_read_addr1_crc_counter | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 28 | B028_csr_read_addr2_frame_counter | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 29 | B029_csr_read_unused_word_zero | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 30 | B030_csr_write_unused_word_no_side_effect | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 31 | B031_csr_waitrequest_low_on_read | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 32 | B032_csr_waitrequest_low_on_write | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 33 | B033_csr_waitrequest_high_when_idle | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 34 | B034_csr_reserved_control_bits_roundtrip | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 35 | B035_fs_idle_requires_k28_0_and_kchar | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 36 | B036_data_byte_without_kchar_no_start | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 37 | B037_header_byte_with_enable_low_no_start | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 38 | B038_header_byte_with_enable_high_starts_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 39 | B039_long_zero_hit_frame_headerinfo_pulse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 40 | B040_long_zero_hit_frame_no_hit_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 41 | B041_long_one_hit_sop_and_eop_same_transfer | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 42 | B042_long_two_hit_sop_first_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 43 | B043_long_two_hit_eop_last_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 44 | B044_long_frame_number_lsb_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 45 | B045_long_frame_number_msb_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 46 | B046_long_frame_flags_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 47 | B047_long_frame_len_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 48 | B048_long_hit_channel_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 49 | B049_long_hit_tcc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 50 | B050_long_hit_tfine_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 51 | B051_long_hit_ecc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 52 | B052_long_hit_eflag_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 53 | B053_long_t_badhit_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 54 | B054_long_e_badhit_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 55 | B055_long_parity_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 56 | B056_long_decode_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 57 | B057_long_loss_sync_propagates_error2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 58 | B058_long_crc_good_keeps_error1_low | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 59 | B059_long_crc_bad_raises_error1_on_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 60 | B060_long_crc_bad_increments_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 61 | B061_sop_increments_frame_counter_head | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 62 | B062_eop_increments_frame_counter_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 63 | B063_frame_counter_register_is_head_minus_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 64 | B064_run_prepare_resets_head_tail_counters | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 65 | B065_short_zero_hit_frame_headerinfo_pulse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 66 | B066_short_one_hit_sop_eop_same_transfer | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 67 | B067_short_two_hit_even_pairing | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 68 | B068_short_three_hit_odd_pairing | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 69 | B069_short_four_hit_even_followup | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 70 | B070_short_mode_selected_by_flags_100 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 71 | B071_short_path_zeroes_ecc_field | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 72 | B072_short_path_uses_bit0_as_eflag | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 73 | B073_short_hit_channel_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 74 | B074_short_hit_tcc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 75 | B075_short_hit_tfine_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 76 | B076_short_parity_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 77 | B077_short_decode_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 78 | B078_short_loss_sync_propagates_error2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 79 | B079_short_crc_bad_raises_error1_on_last_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 80 | B080_short_crc_counter_accumulates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 81 | B081_short_frame_counter_updates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 82 | B082_headerinfo_one_pulse_per_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 83 | B083_headerinfo_channel_matches_input | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 84 | B084_headerinfo_frame_number_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 85 | B085_headerinfo_word_count_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 86 | B086_headerinfo_frame_len_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 87 | B087_headerinfo_flags_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 88 | B088_output_channel_matches_input_channel | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 89 | B089_hit_valid_only_on_complete_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 90 | B090_no_hit_valid_in_crc_states | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 91 | B091_no_sop_without_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 92 | B092_no_eop_without_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 93 | B093_idle_comma_does_not_create_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 94 | B094_mode_halt0_comma_inside_frame_drops_to_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 95 | B095_mode_halt1_comma_inside_frame_holds_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 96 | B096_running_mask_toggle_between_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 97 | B097_mask_toggle_midframe_does_not_abort_current_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 98 | B098_idle_monitoring_accepts_header_without_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 99 | B099_idle_monitoring_still_ignores_csr_zero | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 100 | B100_link_test_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 101 | B101_sync_test_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 102 | B102_reset_state_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 103 | B103_out_of_daq_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 104 | B104_error_state_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 105 | B105_run_prepare_sync_running_spine | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 106 | B106_running_to_terminating_during_header_bytes | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 107 | B107_running_to_terminating_during_unpack_finishes_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 108 | B108_terminating_to_idle_returns_quiescent | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 109 | B109_terminating_blocks_second_header_after_first_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 110 | B110_idle_after_terminating_restores_monitoring | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 111 | B111_system_run_sequence_spine_matches_upgrade_plan | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 112 | B112_long_good_frames_leave_crc_counter_stable | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 113 | B113_bad_frames_accumulate_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 114 | B114_mixed_good_bad_crc_count_matches_bad_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 115 | B115_frame_counter_zero_after_complete_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 116 | B116_frame_counter_nonzero_during_open_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 117 | B117_status_updates_on_new_frame_flags | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 118 | B118_status_holds_last_flags_between_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 119 | B119_clean_short_hit_clears_all_error_bits | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 120 | B120_clean_long_hit_clears_all_error_bits | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 121 | B121_control_write_does_not_clear_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 122 | B122_control_write_does_not_clear_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 123 | B123_channel_change_between_frames_is_observed | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 124 | B124_channel_change_midframe_is_prohibited_in_harness | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 125 | B125_headerinfo_and_hit_channel_track_same_asic | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 126 | B126_zero_hit_long_then_zero_hit_short_sequence | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 127 | B127_good_long_then_good_short_sequence | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 128 | B128_run_prepare_after_crc_error_resets_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 129 | B129_upgrade_contract_no_fresh_header_after_terminating | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 130 | B130_upgrade_contract_terminal_frame_finishes_once | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |

## EDGE Bucket

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---|---|
| E001_ctrl_state_update_is_one_cycle_late_from_valid | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E002_running_command_same_cycle_header_not_accepted | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E003_idle_command_same_cycle_header_still_sees_previous_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E004_terminating_command_same_cycle_open_frame_continues | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E005_back_to_back_running_and_terminating_words | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E006_duplicate_running_words_idempotent | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E007_duplicate_idle_words_idempotent | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E008_ctrl_valid_pulse_one_cycle_only_still_latches_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E009_ctrl_valid_held_high_repeated_word_stable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E010_ctrl_illegal_zero_word_maps_error | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E011_ctrl_twohot_word_maps_error | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E012_ctrl_allones_word_maps_error | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E013_ctrl_valid_low_keeps_previous_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E014_ctrl_ready_does_not_backpressure_illegal_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E015_ctrl_ready_high_during_reset_release | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E016_fs_idle_ignores_header_when_byteisk0_data1c | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E017_fs_idle_ignores_kchar_non_header_9c | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E018_fs_idle_ignores_kchar_nonheader_other | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E020_midframe_valid_low_does_not_pause_parser | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.31 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.31 |
| E021_midframe_channel_change_not_tracked_internally | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E022_header_after_comma_recovery_mode0_restarts_cleanly | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E024_header_immediately_after_run_prepare_blocked | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E025_header_immediately_after_sync_blocked | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E026_header_immediately_after_running_opened_next_cycle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E027_header_immediately_after_terminating_blocked | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E029_header_following_write_zero_then_one_requires_clocked_enable | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E031_frame_number_0000_propagates | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E032_frame_number_ffff_propagates | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E033_frame_number_wrap_ffff_to_0000 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E034_frame_len_zero_long_goes_crc_path_directly | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E035_frame_len_one_long_single_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E036_frame_len_two_long_two_words | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E037_frame_len_max_long_1023_words | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E038_frame_len_zero_short_goes_crc_path_directly | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E039_frame_len_one_short_single_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E040_frame_len_two_short_even_pair | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E041_frame_len_three_short_odd_pair_plus_residual | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E042_frame_len_max_short_1023_words | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E043_frame_flags_short_bit_exact_100 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E044_frame_flags_nonshort_000_long | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E045_frame_flags_cec_101_uses_long_output_path | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E046_word_count_headerinfo_for_zero_length_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E047_word_count_headerinfo_for_single_hit_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E048_word_count_headerinfo_for_max_length_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E049_startofpacket_asserts_when_n_word_cnt_becomes1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E051_long_hit_channel_00000_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E052_long_hit_channel_11111_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E053_long_tcc_zero_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E054_long_tcc_allones_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E055_long_tfine_zero_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E056_long_tfine_allones_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E057_long_ecc_zero_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E058_long_ecc_allones_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E059_short_hit_channel_00000_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E060_short_hit_channel_11111_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E061_short_tcc_zero_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E062_short_tcc_allones_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E063_short_tfine_zero_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E064_short_tfine_allones_boundary | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E065_short_even_hit_extra_nibble_boundary_0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E066_short_even_hit_extra_nibble_boundary_f | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E067_short_odd_hit_saved_nibble_boundary_0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E068_short_odd_hit_saved_nibble_boundary_f | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E069_short_last_odd_hit_goes_direct_to_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E070_short_last_even_hit_goes_from_unpack_extra_to_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E071_crc_magic_match_7ff2_only_case | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E072_crc_magic_onebit_off_is_error | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E073_crc_error_only_asserts_on_eop_cycle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E074_hit_error_latches_across_multi_byte_long_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E075_hit_error_clears_after_word_commit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E076_hit_error_latches_across_multi_byte_short_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E077_loss_sync_on_header_byte_sets_error2_entire_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E078_loss_sync_only_on_last_byte_still_sets_error2 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E079_parity_error_on_nonhit_state_does_not_set_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E080_decode_error_on_nonhit_state_does_not_set_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E081_mode_halt0_comma_on_frame_counter_byte_aborts | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E082_mode_halt0_comma_on_event_counter_byte_aborts | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E083_mode_halt0_comma_on_unpack_byte_aborts | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E084_mode_halt1_comma_on_frame_counter_byte_holds_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E085_mode_halt1_comma_on_event_counter_byte_holds_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E086_mode_halt1_comma_on_unpack_byte_holds_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E087_recovery_after_mode0_abort_starts_next_clean_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E088_mode1_resume_with_error_bits_preserved | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E089_clean_frame_after_bad_crc_clears_error1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E090_clean_frame_after_hit_error_clears_error0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E091_csr_addr_width1_aliases_words0_1_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E092_csr_addr_width2_supports_words0_2_exactly | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E093_csr_addr_width8_high_unused_reads_zero | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E094_channel_width1_headerinfo_maxchannel1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E095_channel_width1_output_channel_singlebit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E096_channel_width4_default_maxchannel15 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E097_channel_width8_large_channel_passthrough | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E098_channel_width_change_between_builds_no_scoreboard_rewrite | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E099_csr_read_and_write_same_cycle_prioritizes_read | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E100_write_control_zero_midframe_affects_next_fs_idle_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E101_write_control_one_midframe_does_not_create_spurious_start | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E102_control_status_read_during_frame_updates_after_headerinfo | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E104_frame_counter_read_on_sop_cycle_returns_preincremented_value | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E106_waitrequest_release_during_reset | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E107_headerinfo_channel_width_tracks_generic | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E108_hit_type0_channel_width_tracks_generic | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E109_rx_channel_width_tracks_generic | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E110_debug_lv0_and_debug_lv2_behave_identically_functionally | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E111_terminating_before_header_byte_blocks_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E112_terminating_on_header_byte_same_cycle_depends_on_old_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E113_terminating_after_header_before_len_allows_completion | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E114_terminating_after_first_long_hit_before_second_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E115_terminating_after_first_short_hit_before_second_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E116_terminating_during_crc_calc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E117_terminating_during_crc_check | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E119_running_to_idle_same_cycle_header_follows_old_state_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E120_running_to_run_prepare_same_cycle_resets_next_cycle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E121_repeated_terminating_words_do_not_duplicate_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E122_repeated_idle_words_do_not_force_extra_output | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E123_header_after_error_state_needs_new_running_or_idle_monitoring | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E124_truncated_long_frame_never_asserts_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E125_truncated_short_frame_never_asserts_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E126_excess_bytes_after_declared_long_frame_ignored_until_next_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E127_excess_bytes_after_declared_short_frame_ignored_until_next_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| E130_upgrade_plan_current_ready_does_not_wait_for_drain | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |

### Bucket-Local Ordered Isolated Merge Trace

| step | case_id | merged_total_after_case |
|---:|---|---|
| 1 | E001_ctrl_state_update_is_one_cycle_late_from_valid | stmt=79.77, branch=63.58, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=15.54 |
| 2 | E002_running_command_same_cycle_header_not_accepted | stmt=81.71, branch=66.23, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.16 |
| 3 | E003_idle_command_same_cycle_header_still_sees_previous_running | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.00 |
| 4 | E004_terminating_command_same_cycle_open_frame_continues | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.15 |
| 5 | E005_back_to_back_running_and_terminating_words | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 6 | E006_duplicate_running_words_idempotent | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 7 | E007_duplicate_idle_words_idempotent | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 8 | E008_ctrl_valid_pulse_one_cycle_only_still_latches_state | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 9 | E009_ctrl_valid_held_high_repeated_word_stable | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 10 | E010_ctrl_illegal_zero_word_maps_error | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 11 | E011_ctrl_twohot_word_maps_error | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 12 | E012_ctrl_allones_word_maps_error | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 13 | E013_ctrl_valid_low_keeps_previous_state | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 14 | E014_ctrl_ready_does_not_backpressure_illegal_word | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 15 | E015_ctrl_ready_high_during_reset_release | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 16 | E016_fs_idle_ignores_header_when_byteisk0_data1c | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 17 | E017_fs_idle_ignores_kchar_non_header_9c | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 18 | E018_fs_idle_ignores_kchar_nonheader_other | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 19 | E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 20 | E020_midframe_valid_low_does_not_pause_parser | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 21 | E021_midframe_channel_change_not_tracked_internally | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 22 | E022_header_after_comma_recovery_mode0_restarts_cleanly | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 23 | E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 24 | E024_header_immediately_after_run_prepare_blocked | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 25 | E025_header_immediately_after_sync_blocked | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 26 | E026_header_immediately_after_running_opened_next_cycle | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 27 | E027_header_immediately_after_terminating_blocked | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 28 | E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 29 | E029_header_following_write_zero_then_one_requires_clocked_enable | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 30 | E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 31 | E031_frame_number_0000_propagates | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 32 | E032_frame_number_ffff_propagates | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 33 | E033_frame_number_wrap_ffff_to_0000 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 34 | E034_frame_len_zero_long_goes_crc_path_directly | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 35 | E035_frame_len_one_long_single_word | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 36 | E036_frame_len_two_long_two_words | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 37 | E037_frame_len_max_long_1023_words | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 38 | E038_frame_len_zero_short_goes_crc_path_directly | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 39 | E039_frame_len_one_short_single_word | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 40 | E040_frame_len_two_short_even_pair | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 41 | E041_frame_len_three_short_odd_pair_plus_residual | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 42 | E042_frame_len_max_short_1023_words | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 43 | E043_frame_flags_short_bit_exact_100 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 44 | E044_frame_flags_nonshort_000_long | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 45 | E045_frame_flags_cec_101_uses_long_output_path | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 46 | E046_word_count_headerinfo_for_zero_length_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 47 | E047_word_count_headerinfo_for_single_hit_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 48 | E048_word_count_headerinfo_for_max_length_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 49 | E049_startofpacket_asserts_when_n_word_cnt_becomes1 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 50 | E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 51 | E051_long_hit_channel_00000_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 52 | E052_long_hit_channel_11111_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 53 | E053_long_tcc_zero_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 54 | E054_long_tcc_allones_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 55 | E055_long_tfine_zero_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 56 | E056_long_tfine_allones_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 57 | E057_long_ecc_zero_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 58 | E058_long_ecc_allones_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 59 | E059_short_hit_channel_00000_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 60 | E060_short_hit_channel_11111_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 61 | E061_short_tcc_zero_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 62 | E062_short_tcc_allones_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 63 | E063_short_tfine_zero_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 64 | E064_short_tfine_allones_boundary | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 65 | E065_short_even_hit_extra_nibble_boundary_0 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 66 | E066_short_even_hit_extra_nibble_boundary_f | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 67 | E067_short_odd_hit_saved_nibble_boundary_0 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 68 | E068_short_odd_hit_saved_nibble_boundary_f | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 69 | E069_short_last_odd_hit_goes_direct_to_crc | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 70 | E070_short_last_even_hit_goes_from_unpack_extra_to_crc | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 71 | E071_crc_magic_match_7ff2_only_case | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 72 | E072_crc_magic_onebit_off_is_error | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 73 | E073_crc_error_only_asserts_on_eop_cycle | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 74 | E074_hit_error_latches_across_multi_byte_long_word | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 75 | E075_hit_error_clears_after_word_commit | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 76 | E076_hit_error_latches_across_multi_byte_short_word | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 77 | E077_loss_sync_on_header_byte_sets_error2_entire_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 78 | E078_loss_sync_only_on_last_byte_still_sets_error2 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 79 | E079_parity_error_on_nonhit_state_does_not_set_error0 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 80 | E080_decode_error_on_nonhit_state_does_not_set_error0 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 81 | E081_mode_halt0_comma_on_frame_counter_byte_aborts | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 82 | E082_mode_halt0_comma_on_event_counter_byte_aborts | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 83 | E083_mode_halt0_comma_on_unpack_byte_aborts | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 84 | E084_mode_halt1_comma_on_frame_counter_byte_holds_state | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 85 | E085_mode_halt1_comma_on_event_counter_byte_holds_state | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 86 | E086_mode_halt1_comma_on_unpack_byte_holds_state | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 87 | E087_recovery_after_mode0_abort_starts_next_clean_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 88 | E088_mode1_resume_with_error_bits_preserved | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 89 | E089_clean_frame_after_bad_crc_clears_error1 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 90 | E090_clean_frame_after_hit_error_clears_error0 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 91 | E091_csr_addr_width1_aliases_words0_1_only | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 92 | E092_csr_addr_width2_supports_words0_2_exactly | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 93 | E093_csr_addr_width8_high_unused_reads_zero | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 94 | E094_channel_width1_headerinfo_maxchannel1 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 95 | E095_channel_width1_output_channel_singlebit | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 96 | E096_channel_width4_default_maxchannel15 | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 97 | E097_channel_width8_large_channel_passthrough | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 98 | E098_channel_width_change_between_builds_no_scoreboard_rewrite | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 99 | E099_csr_read_and_write_same_cycle_prioritizes_read | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 100 | E100_write_control_zero_midframe_affects_next_fs_idle_only | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 101 | E101_write_control_one_midframe_does_not_create_spurious_start | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 102 | E102_control_status_read_during_frame_updates_after_headerinfo | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 103 | E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 104 | E104_frame_counter_read_on_sop_cycle_returns_preincremented_value | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 105 | E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 106 | E106_waitrequest_release_during_reset | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 107 | E107_headerinfo_channel_width_tracks_generic | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 108 | E108_hit_type0_channel_width_tracks_generic | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 109 | E109_rx_channel_width_tracks_generic | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 110 | E110_debug_lv0_and_debug_lv2_behave_identically_functionally | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 111 | E111_terminating_before_header_byte_blocks_frame | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 112 | E112_terminating_on_header_byte_same_cycle_depends_on_old_state | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 113 | E113_terminating_after_header_before_len_allows_completion | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 114 | E114_terminating_after_first_long_hit_before_second_hit | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 115 | E115_terminating_after_first_short_hit_before_second_hit | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 116 | E116_terminating_during_crc_calc | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 117 | E117_terminating_during_crc_check | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 118 | E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 119 | E119_running_to_idle_same_cycle_header_follows_old_state_running | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 120 | E120_running_to_run_prepare_same_cycle_resets_next_cycle | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 121 | E121_repeated_terminating_words_do_not_duplicate_eop | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 122 | E122_repeated_idle_words_do_not_force_extra_output | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 123 | E123_header_after_error_state_needs_new_running_or_idle_monitoring | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 124 | E124_truncated_long_frame_never_asserts_eop | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 125 | E125_truncated_short_frame_never_asserts_eop | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 126 | E126_excess_bytes_after_declared_long_frame_ignored_until_next_header | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 127 | E127_excess_bytes_after_declared_short_frame_ignored_until_next_header | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.09 |
| 128 | E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.25 |
| 129 | E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.25 |
| 130 | E130_upgrade_plan_current_ready_does_not_wait_for_drain | stmt=85.60, branch=72.85, cond=62.96, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.25 |

## PROF Bucket

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---|---|
| P001_long_len0_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P002_long_len1_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P003_long_len2_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P004_long_len3_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P005_long_len7_5k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P006_long_len15_2k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P007_long_len31_1k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P008_long_len63_512_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P009_long_len127_256_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P010_long_len255_128_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P011_long_len511_64_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P012_long_len767_32_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P013_long_len1023_16_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P014_short_len0_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P015_short_len1_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P016_short_len2_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P017_short_len3_10k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P018_short_len7_5k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P019_short_len15_2k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P020_short_len31_1k_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P021_short_len63_512_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P022_short_len127_256_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P023_short_len255_128_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P024_short_len511_64_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P025_short_len767_32_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P026_short_len1023_16_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P027_cadence_1_1_16_1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P028_cadence_1_1_64_1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P029_cadence_2_2_64_2 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P030_cadence_4_4_128_4 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P031_cadence_8_8_256_8 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P032_cadence_16_16_512_16 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P033_cadence_running_only_1024 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P034_cadence_idle_monitoring_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P035_cadence_stop_after_every_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P036_cadence_stop_every_other_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P037_cadence_legal_and_illegal_ctrl_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P038_cadence_runprepare_burst_reset | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P039_cadence_terminating_hold_open_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P040_mask_toggle_every_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P041_mask_toggle_every_2_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P042_mask_toggle_every_4_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P043_mask_toggle_every_8_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P044_mask_toggle_every_16_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P045_mask_low_long_idle_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P046_mask_high_long_running_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P047_midframe_mask_drop_repeated | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P048_midframe_mask_raise_repeated | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P049_counter_poll_during_open_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P050_counter_poll_during_bad_crc_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P051_status_poll_after_every_headerinfo | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P052_control_write_soak_no_counter_side_effect | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P053_all_clean_long_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P054_all_clean_short_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P055_bad_every_32nd_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P056_bad_every_16th_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P057_bad_every_8th_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P058_bad_every_4th_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P059_bad_every_other_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P060_all_bad_long_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P061_all_bad_short_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P062_mixed_long_short_bad_ratio10 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P063_mixed_long_short_bad_ratio50 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P064_bad_crc_plus_loss_sync_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P065_bad_crc_plus_hit_error_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P066_mode0_comma_every_64th_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P067_mode0_comma_every_64th_event_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P068_mode0_comma_every_64th_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P069_mode0_comma_random_positions | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P070_mode0_abort_then_clean_restart | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P071_mode1_comma_every_64th_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P072_mode1_comma_every_64th_event_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P073_mode1_comma_every_64th_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P074_mode1_comma_random_positions | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P075_mode1_hold_with_loss_sync_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P076_mode0_abort_with_loss_sync_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P077_mode0_to_mode1_build_comparison | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P078_mode1_to_mode0_build_comparison | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P079_cfgA_default_long_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P080_cfgA_default_short_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P081_cfgB_modehalt1_clean_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P082_cfgB_modehalt1_comma_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P083_cfgC_channel1_alternating_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P084_cfgD_channel8_wide_sweep | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P085_cfgE_addr1_poll_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P086_cfgF_addr8_unused_space_sweep | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P087_cfgG_debuglv2_functional_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P088_cfgC_termination_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P089_cfgD_zerohit_onehit_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P090_cfgE_control_churn_soak | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P091_crossbuild_reference_trace_equivalence | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P092_long_short_alternate_every_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P093_long_long_short_short_repeat | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P094_zerohit_to_fullhit_alternate | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P095_channel_rotate_0_to_15 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P096_channel_rotate_sparse | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P097_long_clean_short_bad_repeat | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P098_short_clean_long_bad_repeat | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P099_long_hiterror_short_clean_repeat | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P100_short_hiterror_long_clean_repeat | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P101_losssync_burst_every_100_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P102_parity_burst_every_100_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P103_decode_burst_every_100_frames | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P104_metadata_poll_under_mode_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P105_stop_before_header_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P106_stop_on_header_start_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P107_stop_after_event_counter_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P108_stop_after_first_long_hit_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P109_stop_after_first_short_hit_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P110_stop_during_crc_calc_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P111_stop_during_crc_check_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P112_stop_then_immediate_idle_every_run | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P113_stop_every_other_frame_mixed_modes | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P114_stop_under_bad_crc_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P115_stop_under_hiterror_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P116_stop_under_losssync_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P117_stop_soak_10k_cycles_no_fresh_poststop_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P118_seed01_clean_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P119_seed02_clean_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P120_seed03_error_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P121_seed04_stop_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P122_seed05_mask_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P123_seed06_mode_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P124_seed07_channel_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P125_seed08_badcrc_mix_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P126_seed09_comma_mix_mode0_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P127_seed10_comma_mix_mode1_100k_cycles | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P128_seed11_cfgD_wide_channel_100k_cycles | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P129_seed12_cfgE_narrow_addr_100k_cycles | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| P130_seed13_upgrade_gap_observation_bundle | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |

### Bucket-Local Ordered Isolated Merge Trace

| step | case_id | merged_total_after_case |
|---:|---|---|
| 1 | P001_long_len0_10k_frames | stmt=79.77, branch=63.58, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=15.54 |
| 2 | P002_long_len1_10k_frames | stmt=81.71, branch=66.23, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.16 |
| 3 | P003_long_len2_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.00 |
| 4 | P004_long_len3_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.15 |
| 5 | P005_long_len7_5k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 6 | P006_long_len15_2k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 7 | P007_long_len31_1k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 8 | P008_long_len63_512_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 9 | P009_long_len127_256_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 10 | P010_long_len255_128_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 11 | P011_long_len511_64_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 12 | P012_long_len767_32_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 13 | P013_long_len1023_16_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 14 | P014_short_len0_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 15 | P015_short_len1_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 16 | P016_short_len2_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 17 | P017_short_len3_10k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 18 | P018_short_len7_5k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 19 | P019_short_len15_2k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 20 | P020_short_len31_1k_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 21 | P021_short_len63_512_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 22 | P022_short_len127_256_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 23 | P023_short_len255_128_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 24 | P024_short_len511_64_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 25 | P025_short_len767_32_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 26 | P026_short_len1023_16_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 27 | P027_cadence_1_1_16_1 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 28 | P028_cadence_1_1_64_1 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 29 | P029_cadence_2_2_64_2 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 30 | P030_cadence_4_4_128_4 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 31 | P031_cadence_8_8_256_8 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.47 |
| 32 | P032_cadence_16_16_512_16 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 33 | P033_cadence_running_only_1024 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 34 | P034_cadence_idle_monitoring_only | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 35 | P035_cadence_stop_after_every_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 36 | P036_cadence_stop_every_other_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 37 | P037_cadence_legal_and_illegal_ctrl_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 38 | P038_cadence_runprepare_burst_reset | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 39 | P039_cadence_terminating_hold_open_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 40 | P040_mask_toggle_every_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 41 | P041_mask_toggle_every_2_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 42 | P042_mask_toggle_every_4_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 43 | P043_mask_toggle_every_8_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 44 | P044_mask_toggle_every_16_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 45 | P045_mask_low_long_idle_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 46 | P046_mask_high_long_running_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 47 | P047_midframe_mask_drop_repeated | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 48 | P048_midframe_mask_raise_repeated | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 49 | P049_counter_poll_during_open_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 50 | P050_counter_poll_during_bad_crc_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 51 | P051_status_poll_after_every_headerinfo | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 52 | P052_control_write_soak_no_counter_side_effect | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 53 | P053_all_clean_long_crc | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 54 | P054_all_clean_short_crc | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 55 | P055_bad_every_32nd_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 56 | P056_bad_every_16th_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 57 | P057_bad_every_8th_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 58 | P058_bad_every_4th_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 59 | P059_bad_every_other_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 60 | P060_all_bad_long_crc | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 61 | P061_all_bad_short_crc | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 62 | P062_mixed_long_short_bad_ratio10 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 63 | P063_mixed_long_short_bad_ratio50 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.62 |
| 64 | P064_bad_crc_plus_loss_sync_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 65 | P065_bad_crc_plus_hit_error_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 66 | P066_mode0_comma_every_64th_frame_counter | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 67 | P067_mode0_comma_every_64th_event_counter | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 68 | P068_mode0_comma_every_64th_unpack | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 69 | P069_mode0_comma_random_positions | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 70 | P070_mode0_abort_then_clean_restart | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 71 | P071_mode1_comma_every_64th_frame_counter | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 72 | P072_mode1_comma_every_64th_event_counter | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 73 | P073_mode1_comma_every_64th_unpack | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 74 | P074_mode1_comma_random_positions | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 75 | P075_mode1_hold_with_loss_sync_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 76 | P076_mode0_abort_with_loss_sync_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 77 | P077_mode0_to_mode1_build_comparison | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 78 | P078_mode1_to_mode0_build_comparison | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 79 | P079_cfgA_default_long_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 80 | P080_cfgA_default_short_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 81 | P081_cfgB_modehalt1_clean_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 82 | P082_cfgB_modehalt1_comma_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 83 | P083_cfgC_channel1_alternating_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 84 | P084_cfgD_channel8_wide_sweep | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 85 | P085_cfgE_addr1_poll_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 86 | P086_cfgF_addr8_unused_space_sweep | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 87 | P087_cfgG_debuglv2_functional_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 88 | P088_cfgC_termination_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 89 | P089_cfgD_zerohit_onehit_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 90 | P090_cfgE_control_churn_soak | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 91 | P091_crossbuild_reference_trace_equivalence | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 92 | P092_long_short_alternate_every_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 93 | P093_long_long_short_short_repeat | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 94 | P094_zerohit_to_fullhit_alternate | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 95 | P095_channel_rotate_0_to_15 | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 96 | P096_channel_rotate_sparse | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 97 | P097_long_clean_short_bad_repeat | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 98 | P098_short_clean_long_bad_repeat | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 99 | P099_long_hiterror_short_clean_repeat | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 100 | P100_short_hiterror_long_clean_repeat | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 101 | P101_losssync_burst_every_100_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 102 | P102_parity_burst_every_100_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 103 | P103_decode_burst_every_100_frames | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 104 | P104_metadata_poll_under_mode_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 105 | P105_stop_before_header_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 106 | P106_stop_on_header_start_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 107 | P107_stop_after_event_counter_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 108 | P108_stop_after_first_long_hit_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 109 | P109_stop_after_first_short_hit_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 110 | P110_stop_during_crc_calc_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 111 | P111_stop_during_crc_check_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 112 | P112_stop_then_immediate_idle_every_run | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 113 | P113_stop_every_other_frame_mixed_modes | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 114 | P114_stop_under_bad_crc_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 115 | P115_stop_under_hiterror_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 116 | P116_stop_under_losssync_mix | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 117 | P117_stop_soak_10k_cycles_no_fresh_poststop_header | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 118 | P118_seed01_clean_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 119 | P119_seed02_clean_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 120 | P120_seed03_error_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 121 | P121_seed04_stop_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 122 | P122_seed05_mask_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 123 | P123_seed06_mode_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 124 | P124_seed07_channel_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 125 | P125_seed08_badcrc_mix_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 126 | P126_seed09_comma_mix_mode0_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 127 | P127_seed10_comma_mix_mode1_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.78 |
| 128 | P128_seed11_cfgD_wide_channel_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 129 | P129_seed12_cfgE_narrow_addr_100k_cycles | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |
| 130 | P130_seed13_upgrade_gap_observation_bundle | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.94 |

## ERROR Bucket

| case_id | type (d/r) | coverage_by_this_case | executed random txn | coverage_incr_per_txn |
|---|---|---|---|---|
| X001_reset_in_fs_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X002_reset_in_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X003_reset_in_event_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X004_reset_in_long_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X005_reset_in_short_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X006_reset_in_unpack_extra | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X007_reset_in_crc_calc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X008_reset_in_crc_check | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X009_reset_during_bad_crc_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X010_reset_during_hit_error_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X011_reset_during_losssync_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X012_reset_while_csr_read_active | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X013_reset_while_ctrl_valid_high | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X014_ctrl_allzero_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X015_ctrl_twohot_idle_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X016_ctrl_twohot_running_terminating | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X017_ctrl_allones_word | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X018_ctrl_random_illegal_word_seed1 | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X019_ctrl_random_illegal_word_seed2 | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X020_ctrl_valid_glitch_no_word_change | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X021_ctrl_payload_changes_while_valid_high | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X022_ctrl_illegal_then_legal_recovery_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X023_ctrl_illegal_then_legal_recovery_running | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X024_ctrl_ready_not_indicating_recovery | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X025_ctrl_valid_low_payload_noise | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 1 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X026_ctrl_fast_oscillation_legal_illegal_mix | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X027_long_header_only_no_counters | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=6.67, toggle=0.00 |
| X028_long_missing_frame_number_byte1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X029_long_missing_event_counter_msb | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X030_long_missing_event_counter_lsb | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X031_long_declared_len1_missing_payload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X032_long_declared_len2_only_one_hit_present | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X033_long_declared_len_large_cut_midpayload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X034_long_bad_trailer_symbol_replaced_with_data | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X035_long_extra_payload_after_declared_len | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X036_long_new_header_inside_payload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X037_long_comma_inside_payload_mode0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X038_long_comma_inside_payload_mode1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X039_long_valid_low_entire_malformed_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X040_short_header_only_no_counters | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X041_short_missing_frame_number_byte1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X042_short_missing_event_counter_msb | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X043_short_missing_event_counter_lsb | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X044_short_declared_len1_missing_payload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X045_short_declared_len2_halfword_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X046_short_declared_len3_cut_in_unpack_extra | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X047_short_declared_len_large_cut_midpayload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X048_short_new_header_inside_payload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X049_short_extra_payload_after_declared_len | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X050_short_comma_inside_payload_mode0 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X051_short_comma_inside_payload_mode1 | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X052_short_valid_low_entire_malformed_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X053_long_bad_crc_onebit_flip | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X054_long_bad_crc_twobit_flip | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X055_long_bad_crc_allzero_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X056_long_bad_crc_allones_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X057_short_bad_crc_onebit_flip | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X058_short_bad_crc_twobit_flip | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X059_short_bad_crc_allzero_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X060_short_bad_crc_allones_tail | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X061_clean_after_bad_crc_long | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X062_clean_after_bad_crc_short | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X063_multiple_bad_crc_reads_midupdate | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X064_bad_crc_during_terminating | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X065_bad_crc_during_runprepare_reset | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X066_long_parity_error_first_payload_byte | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X067_long_parity_error_last_payload_byte | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X068_long_decode_error_middle_payload_byte | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X069_short_parity_error_even_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X070_short_parity_error_odd_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X071_short_decode_error_unpack_extra | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X072_hiterror_then_clean_hit_same_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X073_losssync_on_header_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X074_losssync_midpayload_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X075_losssync_on_crc_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X076_parity_and_decode_same_hit | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X077_hiterror_plus_losssync_same_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X078_clean_frame_after_combined_errors | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X079_mode0_comma_in_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X080_mode0_comma_in_event_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X081_mode0_comma_in_long_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X082_mode0_comma_in_short_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X083_mode0_abort_then_postabort_garbage | r | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X084_mode0_abort_then_immediate_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X085_mode1_comma_in_frame_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X086_mode1_comma_in_event_counter | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X087_mode1_comma_in_long_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X088_mode1_comma_in_short_unpack | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X089_mode1_hold_then_bad_crc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X090_mode1_hold_then_runprepare | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X091_mode_compare_same_corrupt_stream | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X092_csr_read_unused_address_flood | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X093_csr_write_unused_address_flood | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X094_csr_read_write_same_cycle_overlap | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X095_csr_control_write_during_header_accept | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X096_csr_control_write_during_payload | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X097_csr_poll_word2_every_cycle_open_frame | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X098_csr_poll_word1_every_cycle_badcrc | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X099_csr_mask_low_then_illegal_ctrl | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X100_csr_mask_high_after_error_state | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X101_waitrequest_expected_high_idle_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X102_waitrequest_stuck_low_detection | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X103_word0_status_read_before_headerinfo | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X104_word0_status_read_after_runprepare | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X105_postterminate_fresh_frame_attempt_from_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X106_postterminate_header_same_cycle_as_ctrl_edge | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X107_stop_midframe_without_terminal_marker | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X108_stop_midframe_ready_never_drops | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X109_stop_during_long_frame_then_second_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X110_stop_during_short_frame_then_second_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X111_stop_then_illegal_ctrl_then_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X112_stop_then_runprepare_before_eop | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X113_stop_then_sync_without_idle | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X114_stop_then_running_reopen_window | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X115_emulator_mismatch_repro_openframe_only | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X116_emulator_mismatch_repro_postedge_header | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X117_termination_gap_signature_regression | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X118_cfgC_channel_width1_invalid_channel_drive | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X119_cfgD_channel_width8_random_wide_values | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X120_cfgE_addrwidth1_highaddr_alias_misuse | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X121_cfgF_addrwidth8_highaddr_abuse | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X122_cfgB_modehalt1_abort_assumption_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X123_cfgA_modehalt0_hold_assumption_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X124_cfgG_debuglv2_functional_delta_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X125_hw_tcl_readlatency_mismatch_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X126_hw_tcl_maxchannel_mismatch_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X127_hw_tcl_ctrl_readylatency_assumption_negative | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X128_hw_tcl_headerinfo_channelwidth_mismatch | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X129_build_matrix_compile_smoke_failfast | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |
| X130_known_gap_bundle_current_contract | d | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 | 0 | stmt=0.00, branch=0.00, cond=0.00, expr=0.00, fsm_state=0.00, fsm_trans=0.00, toggle=0.00 |

### Bucket-Local Ordered Isolated Merge Trace

| step | case_id | merged_total_after_case |
|---:|---|---|
| 1 | X001_reset_in_fs_idle | stmt=79.77, branch=63.58, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=15.54 |
| 2 | X002_reset_in_frame_counter | stmt=81.71, branch=66.23, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.16 |
| 3 | X003_reset_in_event_counter | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.00 |
| 4 | X004_reset_in_long_unpack | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.15 |
| 5 | X005_reset_in_short_unpack | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 6 | X006_reset_in_unpack_extra | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 7 | X007_reset_in_crc_calc | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.15 |
| 8 | X008_reset_in_crc_check | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 9 | X009_reset_during_bad_crc_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 10 | X010_reset_during_hit_error_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 11 | X011_reset_during_losssync_frame | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 12 | X012_reset_while_csr_read_active | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 13 | X013_reset_while_ctrl_valid_high | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.31 |
| 14 | X014_ctrl_allzero_word | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.36 |
| 15 | X015_ctrl_twohot_idle_running | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.36 |
| 16 | X016_ctrl_twohot_running_terminating | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 17 | X017_ctrl_allones_word | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 18 | X018_ctrl_random_illegal_word_seed1 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 19 | X019_ctrl_random_illegal_word_seed2 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 20 | X020_ctrl_valid_glitch_no_word_change | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 21 | X021_ctrl_payload_changes_while_valid_high | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 22 | X022_ctrl_illegal_then_legal_recovery_idle | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 23 | X023_ctrl_illegal_then_legal_recovery_running | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 24 | X024_ctrl_ready_not_indicating_recovery | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 25 | X025_ctrl_valid_low_payload_noise | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 26 | X026_ctrl_fast_oscillation_legal_illegal_mix | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.52 |
| 27 | X027_long_header_only_no_counters | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.52 |
| 28 | X028_long_missing_frame_number_byte1 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.52 |
| 29 | X029_long_missing_event_counter_msb | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.52 |
| 30 | X030_long_missing_event_counter_lsb | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.52 |
| 31 | X031_long_declared_len1_missing_payload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.52 |
| 32 | X032_long_declared_len2_only_one_hit_present | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 33 | X033_long_declared_len_large_cut_midpayload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 34 | X034_long_bad_trailer_symbol_replaced_with_data | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 35 | X035_long_extra_payload_after_declared_len | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 36 | X036_long_new_header_inside_payload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 37 | X037_long_comma_inside_payload_mode0 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 38 | X038_long_comma_inside_payload_mode1 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 39 | X039_long_valid_low_entire_malformed_frame | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 40 | X040_short_header_only_no_counters | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 41 | X041_short_missing_frame_number_byte1 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 42 | X042_short_missing_event_counter_msb | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 43 | X043_short_missing_event_counter_lsb | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 44 | X044_short_declared_len1_missing_payload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 45 | X045_short_declared_len2_halfword_only | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 46 | X046_short_declared_len3_cut_in_unpack_extra | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 47 | X047_short_declared_len_large_cut_midpayload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 48 | X048_short_new_header_inside_payload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 49 | X049_short_extra_payload_after_declared_len | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 50 | X050_short_comma_inside_payload_mode0 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 51 | X051_short_comma_inside_payload_mode1 | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 52 | X052_short_valid_low_entire_malformed_frame | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 53 | X053_long_bad_crc_onebit_flip | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 54 | X054_long_bad_crc_twobit_flip | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 55 | X055_long_bad_crc_allzero_tail | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 56 | X056_long_bad_crc_allones_tail | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 57 | X057_short_bad_crc_onebit_flip | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 58 | X058_short_bad_crc_twobit_flip | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 59 | X059_short_bad_crc_allzero_tail | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 60 | X060_short_bad_crc_allones_tail | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 61 | X061_clean_after_bad_crc_long | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 62 | X062_clean_after_bad_crc_short | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 63 | X063_multiple_bad_crc_reads_midupdate | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.67 |
| 64 | X064_bad_crc_during_terminating | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 65 | X065_bad_crc_during_runprepare_reset | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 66 | X066_long_parity_error_first_payload_byte | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 67 | X067_long_parity_error_last_payload_byte | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 68 | X068_long_decode_error_middle_payload_byte | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 69 | X069_short_parity_error_even_hit | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 70 | X070_short_parity_error_odd_hit | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 71 | X071_short_decode_error_unpack_extra | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 72 | X072_hiterror_then_clean_hit_same_frame | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 73 | X073_losssync_on_header_only | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 74 | X074_losssync_midpayload_only | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 75 | X075_losssync_on_crc_only | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 76 | X076_parity_and_decode_same_hit | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 77 | X077_hiterror_plus_losssync_same_frame | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 78 | X078_clean_frame_after_combined_errors | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 79 | X079_mode0_comma_in_frame_counter | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 80 | X080_mode0_comma_in_event_counter | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 81 | X081_mode0_comma_in_long_unpack | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 82 | X082_mode0_comma_in_short_unpack | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 83 | X083_mode0_abort_then_postabort_garbage | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 84 | X084_mode0_abort_then_immediate_header | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 85 | X085_mode1_comma_in_frame_counter | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 86 | X086_mode1_comma_in_event_counter | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 87 | X087_mode1_comma_in_long_unpack | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 88 | X088_mode1_comma_in_short_unpack | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 89 | X089_mode1_hold_then_bad_crc | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 90 | X090_mode1_hold_then_runprepare | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 91 | X091_mode_compare_same_corrupt_stream | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 92 | X092_csr_read_unused_address_flood | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 93 | X093_csr_write_unused_address_flood | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 94 | X094_csr_read_write_same_cycle_overlap | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 95 | X095_csr_control_write_during_header_accept | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 96 | X096_csr_control_write_during_payload | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 97 | X097_csr_poll_word2_every_cycle_open_frame | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 98 | X098_csr_poll_word1_every_cycle_badcrc | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 99 | X099_csr_mask_low_then_illegal_ctrl | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 100 | X100_csr_mask_high_after_error_state | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 101 | X101_waitrequest_expected_high_idle_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 102 | X102_waitrequest_stuck_low_detection | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 103 | X103_word0_status_read_before_headerinfo | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 104 | X104_word0_status_read_after_runprepare | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 105 | X105_postterminate_fresh_frame_attempt_from_idle | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 106 | X106_postterminate_header_same_cycle_as_ctrl_edge | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 107 | X107_stop_midframe_without_terminal_marker | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 108 | X108_stop_midframe_ready_never_drops | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 109 | X109_stop_during_long_frame_then_second_header | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 110 | X110_stop_during_short_frame_then_second_header | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 111 | X111_stop_then_illegal_ctrl_then_idle | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 112 | X112_stop_then_runprepare_before_eop | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 113 | X113_stop_then_sync_without_idle | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 114 | X114_stop_then_running_reopen_window | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 115 | X115_emulator_mismatch_repro_openframe_only | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 116 | X116_emulator_mismatch_repro_postedge_header | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 117 | X117_termination_gap_signature_regression | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 118 | X118_cfgC_channel_width1_invalid_channel_drive | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 119 | X119_cfgD_channel_width8_random_wide_values | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 120 | X120_cfgE_addrwidth1_highaddr_alias_misuse | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 121 | X121_cfgF_addrwidth8_highaddr_abuse | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 122 | X122_cfgB_modehalt1_abort_assumption_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 123 | X123_cfgA_modehalt0_hold_assumption_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 124 | X124_cfgG_debuglv2_functional_delta_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 125 | X125_hw_tcl_readlatency_mismatch_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 126 | X126_hw_tcl_maxchannel_mismatch_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 127 | X127_hw_tcl_ctrl_readylatency_assumption_negative | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.83 |
| 128 | X128_hw_tcl_headerinfo_channelwidth_mismatch | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.99 |
| 129 | X129_build_matrix_compile_smoke_failfast | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.99 |
| 130 | X130_known_gap_bundle_current_contract | stmt=86.77, branch=74.17, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=17.99 |

## All-Buckets Ordered Isolated Merge Trace

| step | bucket | case_id | merged_total_after_case |
|---:|---|---|---|
| 1 | BASIC | B001_reset_idle_outputs_quiet | stmt=55.25, branch=31.13, cond=11.11, expr=0.00, fsm_state=14.29, fsm_trans=0.00, toggle=0.52 |
| 2 | BASIC | B002_idle_force_go_monitor_path_open | stmt=81.71, branch=66.23, cond=48.15, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=16.01 |
| 3 | BASIC | B003_idle_force_go_ignores_csr_mask | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.00 |
| 4 | BASIC | B004_running_needs_csr_enable_high | stmt=85.60, branch=72.85, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.15 |
| 5 | BASIC | B005_running_mask_low_blocks_header_start | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=40.00, toggle=17.21 |
| 6 | BASIC | B006_run_prepare_clears_parser_state | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.52 |
| 7 | BASIC | B007_sync_keeps_parser_closed | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.52 |
| 8 | BASIC | B008_running_opens_header_detection | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 9 | BASIC | B009_terminating_blocks_new_header_from_idle | stmt=85.99, branch=73.51, cond=59.26, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 10 | BASIC | B010_terminating_allows_open_frame_to_finish | stmt=86.38, branch=74.83, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.67 |
| 11 | BASIC | B011_ctrl_unknown_word_enters_error | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 12 | BASIC | B012_ctrl_ready_high_with_valid | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 13 | BASIC | B013_ctrl_ready_high_without_valid | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 14 | BASIC | B014_ctrl_decode_idle_force_go | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 15 | BASIC | B015_ctrl_decode_run_prepare_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=17.73 |
| 16 | BASIC | B016_ctrl_decode_sync_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 17 | BASIC | B017_ctrl_decode_running_go | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 18 | BASIC | B018_ctrl_decode_terminating_stop_new_headers | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 19 | BASIC | B019_ctrl_decode_link_test_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 20 | BASIC | B020_ctrl_decode_sync_test_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 21 | BASIC | B021_ctrl_decode_reset_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 22 | BASIC | B022_ctrl_decode_out_of_daq_disable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 23 | BASIC | B023_csr_control_default_enable | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 24 | BASIC | B024_csr_control_write_zero_masks_running | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 25 | BASIC | B025_csr_control_write_one_unmasks_running | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 26 | BASIC | B026_csr_read_addr0_control_status | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 27 | BASIC | B027_csr_read_addr1_crc_counter | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 28 | BASIC | B028_csr_read_addr2_frame_counter | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 29 | BASIC | B029_csr_read_unused_word_zero | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 30 | BASIC | B030_csr_write_unused_word_no_side_effect | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 31 | BASIC | B031_csr_waitrequest_low_on_read | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=17.88 |
| 32 | BASIC | B032_csr_waitrequest_low_on_write | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 33 | BASIC | B033_csr_waitrequest_high_when_idle | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 34 | BASIC | B034_csr_reserved_control_bits_roundtrip | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 35 | BASIC | B035_fs_idle_requires_k28_0_and_kchar | stmt=87.55, branch=76.16, cond=66.67, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 36 | BASIC | B036_data_byte_without_kchar_no_start | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 37 | BASIC | B037_header_byte_with_enable_low_no_start | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 38 | BASIC | B038_header_byte_with_enable_high_starts_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 39 | BASIC | B039_long_zero_hit_frame_headerinfo_pulse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 40 | BASIC | B040_long_zero_hit_frame_no_hit_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.04 |
| 41 | BASIC | B041_long_one_hit_sop_and_eop_same_transfer | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 42 | BASIC | B042_long_two_hit_sop_first_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 43 | BASIC | B043_long_two_hit_eop_last_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 44 | BASIC | B044_long_frame_number_lsb_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 45 | BASIC | B045_long_frame_number_msb_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 46 | BASIC | B046_long_frame_flags_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 47 | BASIC | B047_long_frame_len_capture | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 48 | BASIC | B048_long_hit_channel_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 49 | BASIC | B049_long_hit_tcc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 50 | BASIC | B050_long_hit_tfine_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 51 | BASIC | B051_long_hit_ecc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 52 | BASIC | B052_long_hit_eflag_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 53 | BASIC | B053_long_t_badhit_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 54 | BASIC | B054_long_e_badhit_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 55 | BASIC | B055_long_parity_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 56 | BASIC | B056_long_decode_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 57 | BASIC | B057_long_loss_sync_propagates_error2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 58 | BASIC | B058_long_crc_good_keeps_error1_low | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 59 | BASIC | B059_long_crc_bad_raises_error1_on_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 60 | BASIC | B060_long_crc_bad_increments_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 61 | BASIC | B061_sop_increments_frame_counter_head | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 62 | BASIC | B062_eop_increments_frame_counter_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 63 | BASIC | B063_frame_counter_register_is_head_minus_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 64 | BASIC | B064_run_prepare_resets_head_tail_counters | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 65 | BASIC | B065_short_zero_hit_frame_headerinfo_pulse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 66 | BASIC | B066_short_one_hit_sop_eop_same_transfer | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 67 | BASIC | B067_short_two_hit_even_pairing | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 68 | BASIC | B068_short_three_hit_odd_pairing | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 69 | BASIC | B069_short_four_hit_even_followup | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 70 | BASIC | B070_short_mode_selected_by_flags_100 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 71 | BASIC | B071_short_path_zeroes_ecc_field | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 72 | BASIC | B072_short_path_uses_bit0_as_eflag | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 73 | BASIC | B073_short_hit_channel_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 74 | BASIC | B074_short_hit_tcc_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 75 | BASIC | B075_short_hit_tfine_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 76 | BASIC | B076_short_parity_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 77 | BASIC | B077_short_decode_error_raises_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 78 | BASIC | B078_short_loss_sync_propagates_error2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 79 | BASIC | B079_short_crc_bad_raises_error1_on_last_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 80 | BASIC | B080_short_crc_counter_accumulates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 81 | BASIC | B081_short_frame_counter_updates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 82 | BASIC | B082_headerinfo_one_pulse_per_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 83 | BASIC | B083_headerinfo_channel_matches_input | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 84 | BASIC | B084_headerinfo_frame_number_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 85 | BASIC | B085_headerinfo_word_count_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 86 | BASIC | B086_headerinfo_frame_len_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 87 | BASIC | B087_headerinfo_flags_field_map | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 88 | BASIC | B088_output_channel_matches_input_channel | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 89 | BASIC | B089_hit_valid_only_on_complete_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 90 | BASIC | B090_no_hit_valid_in_crc_states | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 91 | BASIC | B091_no_sop_without_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 92 | BASIC | B092_no_eop_without_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 93 | BASIC | B093_idle_comma_does_not_create_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 94 | BASIC | B094_mode_halt0_comma_inside_frame_drops_to_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 95 | BASIC | B095_mode_halt1_comma_inside_frame_holds_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 96 | BASIC | B096_running_mask_toggle_between_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 97 | BASIC | B097_mask_toggle_midframe_does_not_abort_current_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 98 | BASIC | B098_idle_monitoring_accepts_header_without_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 99 | BASIC | B099_idle_monitoring_still_ignores_csr_zero | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 100 | BASIC | B100_link_test_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 101 | BASIC | B101_sync_test_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 102 | BASIC | B102_reset_state_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 103 | BASIC | B103_out_of_daq_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 104 | BASIC | B104_error_state_keeps_outputs_quiet | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 105 | BASIC | B105_run_prepare_sync_running_spine | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 106 | BASIC | B106_running_to_terminating_during_header_bytes | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 107 | BASIC | B107_running_to_terminating_during_unpack_finishes_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 108 | BASIC | B108_terminating_to_idle_returns_quiescent | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 109 | BASIC | B109_terminating_blocks_second_header_after_first_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 110 | BASIC | B110_idle_after_terminating_restores_monitoring | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 111 | BASIC | B111_system_run_sequence_spine_matches_upgrade_plan | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 112 | BASIC | B112_long_good_frames_leave_crc_counter_stable | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 113 | BASIC | B113_bad_frames_accumulate_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 114 | BASIC | B114_mixed_good_bad_crc_count_matches_bad_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 115 | BASIC | B115_frame_counter_zero_after_complete_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 116 | BASIC | B116_frame_counter_nonzero_during_open_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 117 | BASIC | B117_status_updates_on_new_frame_flags | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 118 | BASIC | B118_status_holds_last_flags_between_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 119 | BASIC | B119_clean_short_hit_clears_all_error_bits | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 120 | BASIC | B120_clean_long_hit_clears_all_error_bits | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 121 | BASIC | B121_control_write_does_not_clear_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 122 | BASIC | B122_control_write_does_not_clear_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 123 | BASIC | B123_channel_change_between_frames_is_observed | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 124 | BASIC | B124_channel_change_midframe_is_prohibited_in_harness | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 125 | BASIC | B125_headerinfo_and_hit_channel_track_same_asic | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 126 | BASIC | B126_zero_hit_long_then_zero_hit_short_sequence | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 127 | BASIC | B127_good_long_then_good_short_sequence | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.20 |
| 128 | BASIC | B128_run_prepare_after_crc_error_resets_crc_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 129 | BASIC | B129_upgrade_contract_no_fresh_header_after_terminating | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 130 | BASIC | B130_upgrade_contract_terminal_frame_finishes_once | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 131 | EDGE | E001_ctrl_state_update_is_one_cycle_late_from_valid | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 132 | EDGE | E002_running_command_same_cycle_header_not_accepted | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 133 | EDGE | E003_idle_command_same_cycle_header_still_sees_previous_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 134 | EDGE | E004_terminating_command_same_cycle_open_frame_continues | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 135 | EDGE | E005_back_to_back_running_and_terminating_words | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 136 | EDGE | E006_duplicate_running_words_idempotent | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 137 | EDGE | E007_duplicate_idle_words_idempotent | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 138 | EDGE | E008_ctrl_valid_pulse_one_cycle_only_still_latches_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 139 | EDGE | E009_ctrl_valid_held_high_repeated_word_stable | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 140 | EDGE | E010_ctrl_illegal_zero_word_maps_error | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 141 | EDGE | E011_ctrl_twohot_word_maps_error | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 142 | EDGE | E012_ctrl_allones_word_maps_error | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 143 | EDGE | E013_ctrl_valid_low_keeps_previous_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 144 | EDGE | E014_ctrl_ready_does_not_backpressure_illegal_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 145 | EDGE | E015_ctrl_ready_high_during_reset_release | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 146 | EDGE | E016_fs_idle_ignores_header_when_byteisk0_data1c | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 147 | EDGE | E017_fs_idle_ignores_kchar_non_header_9c | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 148 | EDGE | E018_fs_idle_ignores_kchar_nonheader_other | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 149 | EDGE | E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.35 |
| 150 | EDGE | E020_midframe_valid_low_does_not_pause_parser | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 151 | EDGE | E021_midframe_channel_change_not_tracked_internally | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 152 | EDGE | E022_header_after_comma_recovery_mode0_restarts_cleanly | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 153 | EDGE | E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 154 | EDGE | E024_header_immediately_after_run_prepare_blocked | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 155 | EDGE | E025_header_immediately_after_sync_blocked | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 156 | EDGE | E026_header_immediately_after_running_opened_next_cycle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 157 | EDGE | E027_header_immediately_after_terminating_blocked | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 158 | EDGE | E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 159 | EDGE | E029_header_following_write_zero_then_one_requires_clocked_enable | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 160 | EDGE | E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 161 | EDGE | E031_frame_number_0000_propagates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 162 | EDGE | E032_frame_number_ffff_propagates | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 163 | EDGE | E033_frame_number_wrap_ffff_to_0000 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 164 | EDGE | E034_frame_len_zero_long_goes_crc_path_directly | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 165 | EDGE | E035_frame_len_one_long_single_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 166 | EDGE | E036_frame_len_two_long_two_words | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 167 | EDGE | E037_frame_len_max_long_1023_words | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 168 | EDGE | E038_frame_len_zero_short_goes_crc_path_directly | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 169 | EDGE | E039_frame_len_one_short_single_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 170 | EDGE | E040_frame_len_two_short_even_pair | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 171 | EDGE | E041_frame_len_three_short_odd_pair_plus_residual | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 172 | EDGE | E042_frame_len_max_short_1023_words | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 173 | EDGE | E043_frame_flags_short_bit_exact_100 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 174 | EDGE | E044_frame_flags_nonshort_000_long | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 175 | EDGE | E045_frame_flags_cec_101_uses_long_output_path | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 176 | EDGE | E046_word_count_headerinfo_for_zero_length_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 177 | EDGE | E047_word_count_headerinfo_for_single_hit_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 178 | EDGE | E048_word_count_headerinfo_for_max_length_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 179 | EDGE | E049_startofpacket_asserts_when_n_word_cnt_becomes1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 180 | EDGE | E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 181 | EDGE | E051_long_hit_channel_00000_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 182 | EDGE | E052_long_hit_channel_11111_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 183 | EDGE | E053_long_tcc_zero_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 184 | EDGE | E054_long_tcc_allones_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 185 | EDGE | E055_long_tfine_zero_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 186 | EDGE | E056_long_tfine_allones_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 187 | EDGE | E057_long_ecc_zero_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 188 | EDGE | E058_long_ecc_allones_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 189 | EDGE | E059_short_hit_channel_00000_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 190 | EDGE | E060_short_hit_channel_11111_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 191 | EDGE | E061_short_tcc_zero_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 192 | EDGE | E062_short_tcc_allones_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 193 | EDGE | E063_short_tfine_zero_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 194 | EDGE | E064_short_tfine_allones_boundary | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 195 | EDGE | E065_short_even_hit_extra_nibble_boundary_0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 196 | EDGE | E066_short_even_hit_extra_nibble_boundary_f | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 197 | EDGE | E067_short_odd_hit_saved_nibble_boundary_0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 198 | EDGE | E068_short_odd_hit_saved_nibble_boundary_f | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 199 | EDGE | E069_short_last_odd_hit_goes_direct_to_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 200 | EDGE | E070_short_last_even_hit_goes_from_unpack_extra_to_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 201 | EDGE | E071_crc_magic_match_7ff2_only_case | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 202 | EDGE | E072_crc_magic_onebit_off_is_error | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 203 | EDGE | E073_crc_error_only_asserts_on_eop_cycle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 204 | EDGE | E074_hit_error_latches_across_multi_byte_long_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 205 | EDGE | E075_hit_error_clears_after_word_commit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 206 | EDGE | E076_hit_error_latches_across_multi_byte_short_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 207 | EDGE | E077_loss_sync_on_header_byte_sets_error2_entire_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 208 | EDGE | E078_loss_sync_only_on_last_byte_still_sets_error2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 209 | EDGE | E079_parity_error_on_nonhit_state_does_not_set_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 210 | EDGE | E080_decode_error_on_nonhit_state_does_not_set_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 211 | EDGE | E081_mode_halt0_comma_on_frame_counter_byte_aborts | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 212 | EDGE | E082_mode_halt0_comma_on_event_counter_byte_aborts | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 213 | EDGE | E083_mode_halt0_comma_on_unpack_byte_aborts | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 214 | EDGE | E084_mode_halt1_comma_on_frame_counter_byte_holds_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 215 | EDGE | E085_mode_halt1_comma_on_event_counter_byte_holds_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 216 | EDGE | E086_mode_halt1_comma_on_unpack_byte_holds_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 217 | EDGE | E087_recovery_after_mode0_abort_starts_next_clean_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 218 | EDGE | E088_mode1_resume_with_error_bits_preserved | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 219 | EDGE | E089_clean_frame_after_bad_crc_clears_error1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 220 | EDGE | E090_clean_frame_after_hit_error_clears_error0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 221 | EDGE | E091_csr_addr_width1_aliases_words0_1_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 222 | EDGE | E092_csr_addr_width2_supports_words0_2_exactly | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 223 | EDGE | E093_csr_addr_width8_high_unused_reads_zero | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 224 | EDGE | E094_channel_width1_headerinfo_maxchannel1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 225 | EDGE | E095_channel_width1_output_channel_singlebit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 226 | EDGE | E096_channel_width4_default_maxchannel15 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 227 | EDGE | E097_channel_width8_large_channel_passthrough | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 228 | EDGE | E098_channel_width_change_between_builds_no_scoreboard_rewrite | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 229 | EDGE | E099_csr_read_and_write_same_cycle_prioritizes_read | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 230 | EDGE | E100_write_control_zero_midframe_affects_next_fs_idle_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 231 | EDGE | E101_write_control_one_midframe_does_not_create_spurious_start | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 232 | EDGE | E102_control_status_read_during_frame_updates_after_headerinfo | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 233 | EDGE | E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 234 | EDGE | E104_frame_counter_read_on_sop_cycle_returns_preincremented_value | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 235 | EDGE | E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 236 | EDGE | E106_waitrequest_release_during_reset | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 237 | EDGE | E107_headerinfo_channel_width_tracks_generic | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 238 | EDGE | E108_hit_type0_channel_width_tracks_generic | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 239 | EDGE | E109_rx_channel_width_tracks_generic | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 240 | EDGE | E110_debug_lv0_and_debug_lv2_behave_identically_functionally | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 241 | EDGE | E111_terminating_before_header_byte_blocks_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 242 | EDGE | E112_terminating_on_header_byte_same_cycle_depends_on_old_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 243 | EDGE | E113_terminating_after_header_before_len_allows_completion | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 244 | EDGE | E114_terminating_after_first_long_hit_before_second_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 245 | EDGE | E115_terminating_after_first_short_hit_before_second_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 246 | EDGE | E116_terminating_during_crc_calc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 247 | EDGE | E117_terminating_during_crc_check | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 248 | EDGE | E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 249 | EDGE | E119_running_to_idle_same_cycle_header_follows_old_state_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 250 | EDGE | E120_running_to_run_prepare_same_cycle_resets_next_cycle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 251 | EDGE | E121_repeated_terminating_words_do_not_duplicate_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 252 | EDGE | E122_repeated_idle_words_do_not_force_extra_output | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 253 | EDGE | E123_header_after_error_state_needs_new_running_or_idle_monitoring | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 254 | EDGE | E124_truncated_long_frame_never_asserts_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 255 | EDGE | E125_truncated_short_frame_never_asserts_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 256 | EDGE | E126_excess_bytes_after_declared_long_frame_ignored_until_next_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 257 | EDGE | E127_excess_bytes_after_declared_short_frame_ignored_until_next_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 258 | EDGE | E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 259 | EDGE | E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 260 | EDGE | E130_upgrade_plan_current_ready_does_not_wait_for_drain | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 261 | PROF | P001_long_len0_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 262 | PROF | P002_long_len1_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 263 | PROF | P003_long_len2_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 264 | PROF | P004_long_len3_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 265 | PROF | P005_long_len7_5k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 266 | PROF | P006_long_len15_2k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 267 | PROF | P007_long_len31_1k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 268 | PROF | P008_long_len63_512_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 269 | PROF | P009_long_len127_256_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 270 | PROF | P010_long_len255_128_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 271 | PROF | P011_long_len511_64_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 272 | PROF | P012_long_len767_32_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 273 | PROF | P013_long_len1023_16_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 274 | PROF | P014_short_len0_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 275 | PROF | P015_short_len1_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 276 | PROF | P016_short_len2_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 277 | PROF | P017_short_len3_10k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 278 | PROF | P018_short_len7_5k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 279 | PROF | P019_short_len15_2k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 280 | PROF | P020_short_len31_1k_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 281 | PROF | P021_short_len63_512_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 282 | PROF | P022_short_len127_256_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 283 | PROF | P023_short_len255_128_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 284 | PROF | P024_short_len511_64_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 285 | PROF | P025_short_len767_32_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 286 | PROF | P026_short_len1023_16_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 287 | PROF | P027_cadence_1_1_16_1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 288 | PROF | P028_cadence_1_1_64_1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 289 | PROF | P029_cadence_2_2_64_2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 290 | PROF | P030_cadence_4_4_128_4 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 291 | PROF | P031_cadence_8_8_256_8 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 292 | PROF | P032_cadence_16_16_512_16 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 293 | PROF | P033_cadence_running_only_1024 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 294 | PROF | P034_cadence_idle_monitoring_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 295 | PROF | P035_cadence_stop_after_every_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 296 | PROF | P036_cadence_stop_every_other_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 297 | PROF | P037_cadence_legal_and_illegal_ctrl_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 298 | PROF | P038_cadence_runprepare_burst_reset | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 299 | PROF | P039_cadence_terminating_hold_open_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 300 | PROF | P040_mask_toggle_every_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 301 | PROF | P041_mask_toggle_every_2_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 302 | PROF | P042_mask_toggle_every_4_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 303 | PROF | P043_mask_toggle_every_8_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 304 | PROF | P044_mask_toggle_every_16_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 305 | PROF | P045_mask_low_long_idle_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 306 | PROF | P046_mask_high_long_running_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 307 | PROF | P047_midframe_mask_drop_repeated | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 308 | PROF | P048_midframe_mask_raise_repeated | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 309 | PROF | P049_counter_poll_during_open_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 310 | PROF | P050_counter_poll_during_bad_crc_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 311 | PROF | P051_status_poll_after_every_headerinfo | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 312 | PROF | P052_control_write_soak_no_counter_side_effect | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 313 | PROF | P053_all_clean_long_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 314 | PROF | P054_all_clean_short_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 315 | PROF | P055_bad_every_32nd_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 316 | PROF | P056_bad_every_16th_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 317 | PROF | P057_bad_every_8th_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 318 | PROF | P058_bad_every_4th_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 319 | PROF | P059_bad_every_other_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 320 | PROF | P060_all_bad_long_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 321 | PROF | P061_all_bad_short_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 322 | PROF | P062_mixed_long_short_bad_ratio10 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 323 | PROF | P063_mixed_long_short_bad_ratio50 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 324 | PROF | P064_bad_crc_plus_loss_sync_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 325 | PROF | P065_bad_crc_plus_hit_error_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 326 | PROF | P066_mode0_comma_every_64th_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 327 | PROF | P067_mode0_comma_every_64th_event_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 328 | PROF | P068_mode0_comma_every_64th_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 329 | PROF | P069_mode0_comma_random_positions | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 330 | PROF | P070_mode0_abort_then_clean_restart | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 331 | PROF | P071_mode1_comma_every_64th_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 332 | PROF | P072_mode1_comma_every_64th_event_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 333 | PROF | P073_mode1_comma_every_64th_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 334 | PROF | P074_mode1_comma_random_positions | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 335 | PROF | P075_mode1_hold_with_loss_sync_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 336 | PROF | P076_mode0_abort_with_loss_sync_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 337 | PROF | P077_mode0_to_mode1_build_comparison | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 338 | PROF | P078_mode1_to_mode0_build_comparison | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 339 | PROF | P079_cfgA_default_long_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 340 | PROF | P080_cfgA_default_short_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 341 | PROF | P081_cfgB_modehalt1_clean_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 342 | PROF | P082_cfgB_modehalt1_comma_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 343 | PROF | P083_cfgC_channel1_alternating_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 344 | PROF | P084_cfgD_channel8_wide_sweep | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 345 | PROF | P085_cfgE_addr1_poll_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 346 | PROF | P086_cfgF_addr8_unused_space_sweep | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 347 | PROF | P087_cfgG_debuglv2_functional_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 348 | PROF | P088_cfgC_termination_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 349 | PROF | P089_cfgD_zerohit_onehit_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 350 | PROF | P090_cfgE_control_churn_soak | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 351 | PROF | P091_crossbuild_reference_trace_equivalence | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 352 | PROF | P092_long_short_alternate_every_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 353 | PROF | P093_long_long_short_short_repeat | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 354 | PROF | P094_zerohit_to_fullhit_alternate | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 355 | PROF | P095_channel_rotate_0_to_15 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 356 | PROF | P096_channel_rotate_sparse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 357 | PROF | P097_long_clean_short_bad_repeat | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 358 | PROF | P098_short_clean_long_bad_repeat | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 359 | PROF | P099_long_hiterror_short_clean_repeat | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 360 | PROF | P100_short_hiterror_long_clean_repeat | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 361 | PROF | P101_losssync_burst_every_100_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 362 | PROF | P102_parity_burst_every_100_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 363 | PROF | P103_decode_burst_every_100_frames | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 364 | PROF | P104_metadata_poll_under_mode_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 365 | PROF | P105_stop_before_header_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 366 | PROF | P106_stop_on_header_start_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 367 | PROF | P107_stop_after_event_counter_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 368 | PROF | P108_stop_after_first_long_hit_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 369 | PROF | P109_stop_after_first_short_hit_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 370 | PROF | P110_stop_during_crc_calc_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 371 | PROF | P111_stop_during_crc_check_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 372 | PROF | P112_stop_then_immediate_idle_every_run | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 373 | PROF | P113_stop_every_other_frame_mixed_modes | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 374 | PROF | P114_stop_under_bad_crc_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 375 | PROF | P115_stop_under_hiterror_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 376 | PROF | P116_stop_under_losssync_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 377 | PROF | P117_stop_soak_10k_cycles_no_fresh_poststop_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 378 | PROF | P118_seed01_clean_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 379 | PROF | P119_seed02_clean_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 380 | PROF | P120_seed03_error_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 381 | PROF | P121_seed04_stop_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 382 | PROF | P122_seed05_mask_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 383 | PROF | P123_seed06_mode_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 384 | PROF | P124_seed07_channel_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 385 | PROF | P125_seed08_badcrc_mix_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 386 | PROF | P126_seed09_comma_mix_mode0_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 387 | PROF | P127_seed10_comma_mix_mode1_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 388 | PROF | P128_seed11_cfgD_wide_channel_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 389 | PROF | P129_seed12_cfgE_narrow_addr_100k_cycles | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 390 | PROF | P130_seed13_upgrade_gap_observation_bundle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 391 | ERROR | X001_reset_in_fs_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 392 | ERROR | X002_reset_in_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 393 | ERROR | X003_reset_in_event_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 394 | ERROR | X004_reset_in_long_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 395 | ERROR | X005_reset_in_short_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 396 | ERROR | X006_reset_in_unpack_extra | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 397 | ERROR | X007_reset_in_crc_calc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 398 | ERROR | X008_reset_in_crc_check | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 399 | ERROR | X009_reset_during_bad_crc_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 400 | ERROR | X010_reset_during_hit_error_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 401 | ERROR | X011_reset_during_losssync_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 402 | ERROR | X012_reset_while_csr_read_active | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 403 | ERROR | X013_reset_while_ctrl_valid_high | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 404 | ERROR | X014_ctrl_allzero_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 405 | ERROR | X015_ctrl_twohot_idle_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 406 | ERROR | X016_ctrl_twohot_running_terminating | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 407 | ERROR | X017_ctrl_allones_word | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 408 | ERROR | X018_ctrl_random_illegal_word_seed1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 409 | ERROR | X019_ctrl_random_illegal_word_seed2 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 410 | ERROR | X020_ctrl_valid_glitch_no_word_change | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 411 | ERROR | X021_ctrl_payload_changes_while_valid_high | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 412 | ERROR | X022_ctrl_illegal_then_legal_recovery_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 413 | ERROR | X023_ctrl_illegal_then_legal_recovery_running | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 414 | ERROR | X024_ctrl_ready_not_indicating_recovery | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 415 | ERROR | X025_ctrl_valid_low_payload_noise | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 416 | ERROR | X026_ctrl_fast_oscillation_legal_illegal_mix | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=53.33, toggle=18.67 |
| 417 | ERROR | X027_long_header_only_no_counters | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 418 | ERROR | X028_long_missing_frame_number_byte1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 419 | ERROR | X029_long_missing_event_counter_msb | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 420 | ERROR | X030_long_missing_event_counter_lsb | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 421 | ERROR | X031_long_declared_len1_missing_payload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 422 | ERROR | X032_long_declared_len2_only_one_hit_present | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 423 | ERROR | X033_long_declared_len_large_cut_midpayload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 424 | ERROR | X034_long_bad_trailer_symbol_replaced_with_data | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 425 | ERROR | X035_long_extra_payload_after_declared_len | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 426 | ERROR | X036_long_new_header_inside_payload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 427 | ERROR | X037_long_comma_inside_payload_mode0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 428 | ERROR | X038_long_comma_inside_payload_mode1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 429 | ERROR | X039_long_valid_low_entire_malformed_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 430 | ERROR | X040_short_header_only_no_counters | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 431 | ERROR | X041_short_missing_frame_number_byte1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 432 | ERROR | X042_short_missing_event_counter_msb | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 433 | ERROR | X043_short_missing_event_counter_lsb | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 434 | ERROR | X044_short_declared_len1_missing_payload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 435 | ERROR | X045_short_declared_len2_halfword_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 436 | ERROR | X046_short_declared_len3_cut_in_unpack_extra | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 437 | ERROR | X047_short_declared_len_large_cut_midpayload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 438 | ERROR | X048_short_new_header_inside_payload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 439 | ERROR | X049_short_extra_payload_after_declared_len | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 440 | ERROR | X050_short_comma_inside_payload_mode0 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 441 | ERROR | X051_short_comma_inside_payload_mode1 | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 442 | ERROR | X052_short_valid_low_entire_malformed_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 443 | ERROR | X053_long_bad_crc_onebit_flip | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 444 | ERROR | X054_long_bad_crc_twobit_flip | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 445 | ERROR | X055_long_bad_crc_allzero_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 446 | ERROR | X056_long_bad_crc_allones_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 447 | ERROR | X057_short_bad_crc_onebit_flip | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 448 | ERROR | X058_short_bad_crc_twobit_flip | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 449 | ERROR | X059_short_bad_crc_allzero_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 450 | ERROR | X060_short_bad_crc_allones_tail | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 451 | ERROR | X061_clean_after_bad_crc_long | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 452 | ERROR | X062_clean_after_bad_crc_short | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 453 | ERROR | X063_multiple_bad_crc_reads_midupdate | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 454 | ERROR | X064_bad_crc_during_terminating | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 455 | ERROR | X065_bad_crc_during_runprepare_reset | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 456 | ERROR | X066_long_parity_error_first_payload_byte | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 457 | ERROR | X067_long_parity_error_last_payload_byte | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 458 | ERROR | X068_long_decode_error_middle_payload_byte | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 459 | ERROR | X069_short_parity_error_even_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 460 | ERROR | X070_short_parity_error_odd_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 461 | ERROR | X071_short_decode_error_unpack_extra | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 462 | ERROR | X072_hiterror_then_clean_hit_same_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 463 | ERROR | X073_losssync_on_header_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 464 | ERROR | X074_losssync_midpayload_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 465 | ERROR | X075_losssync_on_crc_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 466 | ERROR | X076_parity_and_decode_same_hit | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 467 | ERROR | X077_hiterror_plus_losssync_same_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 468 | ERROR | X078_clean_frame_after_combined_errors | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 469 | ERROR | X079_mode0_comma_in_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 470 | ERROR | X080_mode0_comma_in_event_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 471 | ERROR | X081_mode0_comma_in_long_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 472 | ERROR | X082_mode0_comma_in_short_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 473 | ERROR | X083_mode0_abort_then_postabort_garbage | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 474 | ERROR | X084_mode0_abort_then_immediate_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 475 | ERROR | X085_mode1_comma_in_frame_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 476 | ERROR | X086_mode1_comma_in_event_counter | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 477 | ERROR | X087_mode1_comma_in_long_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 478 | ERROR | X088_mode1_comma_in_short_unpack | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 479 | ERROR | X089_mode1_hold_then_bad_crc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 480 | ERROR | X090_mode1_hold_then_runprepare | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 481 | ERROR | X091_mode_compare_same_corrupt_stream | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 482 | ERROR | X092_csr_read_unused_address_flood | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 483 | ERROR | X093_csr_write_unused_address_flood | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 484 | ERROR | X094_csr_read_write_same_cycle_overlap | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 485 | ERROR | X095_csr_control_write_during_header_accept | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 486 | ERROR | X096_csr_control_write_during_payload | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 487 | ERROR | X097_csr_poll_word2_every_cycle_open_frame | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 488 | ERROR | X098_csr_poll_word1_every_cycle_badcrc | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 489 | ERROR | X099_csr_mask_low_then_illegal_ctrl | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 490 | ERROR | X100_csr_mask_high_after_error_state | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 491 | ERROR | X101_waitrequest_expected_high_idle_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 492 | ERROR | X102_waitrequest_stuck_low_detection | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 493 | ERROR | X103_word0_status_read_before_headerinfo | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 494 | ERROR | X104_word0_status_read_after_runprepare | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 495 | ERROR | X105_postterminate_fresh_frame_attempt_from_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 496 | ERROR | X106_postterminate_header_same_cycle_as_ctrl_edge | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 497 | ERROR | X107_stop_midframe_without_terminal_marker | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 498 | ERROR | X108_stop_midframe_ready_never_drops | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 499 | ERROR | X109_stop_during_long_frame_then_second_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 500 | ERROR | X110_stop_during_short_frame_then_second_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 501 | ERROR | X111_stop_then_illegal_ctrl_then_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 502 | ERROR | X112_stop_then_runprepare_before_eop | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 503 | ERROR | X113_stop_then_sync_without_idle | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 504 | ERROR | X114_stop_then_running_reopen_window | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 505 | ERROR | X115_emulator_mismatch_repro_openframe_only | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 506 | ERROR | X116_emulator_mismatch_repro_postedge_header | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 507 | ERROR | X117_termination_gap_signature_regression | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 508 | ERROR | X118_cfgC_channel_width1_invalid_channel_drive | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 509 | ERROR | X119_cfgD_channel_width8_random_wide_values | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 510 | ERROR | X120_cfgE_addrwidth1_highaddr_alias_misuse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 511 | ERROR | X121_cfgF_addrwidth8_highaddr_abuse | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 512 | ERROR | X122_cfgB_modehalt1_abort_assumption_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 513 | ERROR | X123_cfgA_modehalt0_hold_assumption_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 514 | ERROR | X124_cfgG_debuglv2_functional_delta_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 515 | ERROR | X125_hw_tcl_readlatency_mismatch_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 516 | ERROR | X126_hw_tcl_maxchannel_mismatch_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 517 | ERROR | X127_hw_tcl_ctrl_readylatency_assumption_negative | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 518 | ERROR | X128_hw_tcl_headerinfo_channelwidth_mismatch | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 519 | ERROR | X129_build_matrix_compile_smoke_failfast | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |
| 520 | ERROR | X130_known_gap_bundle_current_contract | stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67 |

## All-Buckets Totals

- merged isolated total code coverage: `stmt=87.55, branch=76.16, cond=70.37, expr=98.77, fsm_state=85.71, fsm_trans=60.00, toggle=18.67`
- total final functional coverage: `100.00% (520/520 planned cases evidenced)`
- `bucket_frame` merged total code coverage: pending refresh
- `all_buckets_frame` merged total code coverage: pending refresh

