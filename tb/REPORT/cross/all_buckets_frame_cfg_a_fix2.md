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
| ℹ️ | txns | `1462` |
| ✅ | functional_cross_pct | `96.88` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 97.94 |
| branch | 94.00 |
| cond | 85.00 |
| expr | 98.77 |
| fsm_state | n/a |
| fsm_trans | n/a |
| toggle | 60.48 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `B004_running_needs_csr_enable_high` | `` | 34.38 | 11 | hit_eop |
| 2 | `B008_running_opens_header_detection` | `` | 34.38 | 0 | hit_eop |
| 3 | `B010_terminating_allows_open_frame_to_finish` | `` | 34.38 | 0 | hit_eop |
| 4 | `B025_csr_control_write_one_unmasks_running` | `` | 37.50 | 1 | hit_eop |
| 5 | `B027_csr_read_addr1_crc_counter` | `` | 37.50 | 0 | hit_eop |
| 6 | `B028_csr_read_addr2_frame_counter` | `` | 37.50 | 0 | hit_eop |
| 7 | `B030_csr_write_unused_word_no_side_effect` | `` | 37.50 | 0 | hit_eop |
| 8 | `B035_fs_idle_requires_k28_0_and_kchar` | `` | 37.50 | 0 | hit_eop |
| 9 | `B038_header_byte_with_enable_high_starts_frame` | `` | 37.50 | 0 | hit_eop |
| 10 | `B039_long_zero_hit_frame_headerinfo_pulse` | `` | 40.62 | 1 | header_only_or_zero_hit |
| 11 | `B040_long_zero_hit_frame_no_hit_valid` | `` | 40.62 | 0 | header_only_or_zero_hit |
| 12 | `B041_long_one_hit_sop_and_eop_same_transfer` | `` | 40.62 | 0 | hit_eop |
| 13 | `B042_long_two_hit_sop_first_only` | `` | 43.75 | 1 | hit_eop |
| 14 | `B043_long_two_hit_eop_last_only` | `` | 43.75 | 0 | hit_eop |
| 15 | `B044_long_frame_number_lsb_capture` | `` | 43.75 | 0 | hit_eop |
| 16 | `B045_long_frame_number_msb_capture` | `` | 43.75 | 0 | hit_eop |
| 32 | `B061_sop_increments_frame_counter_head` | `` | 43.75 | 0 | hit_eop |
| 35 | `B065_short_zero_hit_frame_headerinfo_pulse` | `` | 53.12 | 3 | header_only_or_zero_hit |
| 38 | `B068_short_three_hit_odd_pairing` | `` | 56.25 | 1 | hit_eop |
| 64 | `B093_idle_comma_does_not_create_frame` | `` | 59.38 | 1 | header_only_or_zero_hit |
| 69 | `B106_running_to_terminating_during_header_bytes` | `` | 62.50 | 1 | hit_eop |
| 128 | `E035_frame_len_one_long_single_word` | `` | 62.50 | 0 | hit_eop |
| 130 | `E037_frame_len_max_long_1023_words` | `` | 65.62 | 1 | hit_eop |
| 204 | `E124_truncated_long_frame_never_asserts_eop` | `` | 68.75 | 1 | header_only_or_zero_hit |
| 256 | `P002_long_len1_10k_frames` | `` | 68.75 | 0 | hit_eop |
| 339 | `P005_long_len7_5k_frames` | `` | 71.88 | 1 | hit_eop |
| 512 | `P017_short_len3_10k_frames` | `` | 71.88 | 0 | hit_eop |
| 780 | `P069_mode0_comma_random_positions` | `` | 75.00 | 1 | header_only_or_zero_hit |
| 1024 | `P117_stop_soak_10k_cycles_no_fresh_poststop_header` | `` | 75.00 | 0 | hit_eop |
| 1368 | `X004_reset_in_long_unpack` | `` | 78.12 | 1 | header_only_or_zero_hit |
| 1371 | `X007_reset_in_crc_calc` | `` | 81.25 | 1 | hit_eop |
| 1374 | `X010_reset_during_hit_error_frame` | `` | 84.38 | 1 | header_only_or_zero_hit |
| 1375 | `X010_reset_during_hit_error_frame` | `` | 87.50 | 1 | hit_eop |
| 1386 | `X034_long_bad_trailer_symbol_replaced_with_data` | `` | 90.62 | 1 | hit_eop |
| 1387 | `X035_long_extra_payload_after_declared_len` | `` | 93.75 | 1 | hit_eop |
| 1388 | `X036_long_new_header_inside_payload` | `` | 96.88 | 1 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
