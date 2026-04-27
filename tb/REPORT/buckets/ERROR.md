# ⚠️ ERROR bucket

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
| ✅ | stmt | 95.19 | 95.0 |
| ⚠️ | branch | 89.33 | 90.0 |
| ℹ️ | cond | 70.00 | - |
| ℹ️ | expr | 98.77 | - |
| ❓ | fsm_state | n/a | 95.0 |
| ❓ | fsm_trans | n/a | 90.0 |
| ⚠️ | toggle | 53.62 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `X001_reset_in_fs_idle` | stmt=59.11, branch=30.67, cond=7.50, expr=0.00, fsm_state=n/a, fsm_trans=n/a, toggle=0.73 | [case](../cases/X001_reset_in_fs_idle.md) |
| ✅ | 2 | `X002_reset_in_frame_counter` | stmt=69.76, branch=44.67, cond=37.50, expr=81.48, fsm_state=n/a, fsm_trans=n/a, toggle=9.67 | [case](../cases/X002_reset_in_frame_counter.md) |
| ✅ | 3 | `X003_reset_in_event_counter` | stmt=72.16, branch=47.33, cond=37.50, expr=92.59, fsm_state=n/a, fsm_trans=n/a, toggle=11.36 | [case](../cases/X003_reset_in_event_counter.md) |
| ✅ | 4 | `X004_reset_in_long_unpack` | stmt=73.88, branch=54.00, cond=42.50, expr=92.59, fsm_state=n/a, fsm_trans=n/a, toggle=13.24 | [case](../cases/X004_reset_in_long_unpack.md) |
| ✅ | 5 | `X005_reset_in_short_unpack` | stmt=75.26, branch=57.33, cond=42.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=14.36 | [case](../cases/X005_reset_in_short_unpack.md) |
| ✅ | 6 | `X006_reset_in_unpack_extra` | stmt=77.66, branch=60.00, cond=42.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=15.32 | [case](../cases/X006_reset_in_unpack_extra.md) |
| ✅ | 7 | `X007_reset_in_crc_calc` | stmt=83.16, branch=69.33, cond=42.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=16.84 | [case](../cases/X007_reset_in_crc_calc.md) |
| ✅ | 8 | `X008_reset_in_crc_check` | stmt=83.16, branch=69.33, cond=42.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=17.56 | [case](../cases/X008_reset_in_crc_check.md) |
| ✅ | 9 | `X009_reset_during_bad_crc_frame` | stmt=83.16, branch=69.33, cond=42.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=17.63 | [case](../cases/X009_reset_during_bad_crc_frame.md) |
| ✅ | 10 | `X010_reset_during_hit_error_frame` | stmt=85.22, branch=71.33, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=19.51 | [case](../cases/X010_reset_during_hit_error_frame.md) |
| ✅ | 11 | `X011_reset_during_losssync_frame` | stmt=85.22, branch=71.33, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=19.81 | [case](../cases/X011_reset_during_losssync_frame.md) |
| ✅ | 12 | `X012_reset_while_csr_read_active` | stmt=86.25, branch=72.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.07 | [case](../cases/X012_reset_while_csr_read_active.md) |
| ✅ | 13 | `X013_reset_while_ctrl_valid_high` | stmt=86.25, branch=72.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.07 | [case](../cases/X013_reset_while_ctrl_valid_high.md) |
| ✅ | 14 | `X014_ctrl_allzero_word` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.11 | [case](../cases/X014_ctrl_allzero_word.md) |
| ✅ | 15 | `X015_ctrl_twohot_idle_running` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.11 | [case](../cases/X015_ctrl_twohot_idle_running.md) |
| ✅ | 16 | `X016_ctrl_twohot_running_terminating` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.24 | [case](../cases/X016_ctrl_twohot_running_terminating.md) |
| ✅ | 17 | `X017_ctrl_allones_word` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.77 | [case](../cases/X017_ctrl_allones_word.md) |
| ✅ | 18 | `X018_ctrl_random_illegal_word_seed1` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.77 | [case](../cases/X018_ctrl_random_illegal_word_seed1.md) |
| ✅ | 19 | `X019_ctrl_random_illegal_word_seed2` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.77 | [case](../cases/X019_ctrl_random_illegal_word_seed2.md) |
| ✅ | 20 | `X020_ctrl_valid_glitch_no_word_change` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=20.77 | [case](../cases/X020_ctrl_valid_glitch_no_word_change.md) |
| ✅ | 21 | `X021_ctrl_payload_changes_while_valid_high` | stmt=87.29, branch=74.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.10 | [case](../cases/X021_ctrl_payload_changes_while_valid_high.md) |
| ✅ | 22 | `X022_ctrl_illegal_then_legal_recovery_idle` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.10 | [case](../cases/X022_ctrl_illegal_then_legal_recovery_idle.md) |
| ✅ | 23 | `X023_ctrl_illegal_then_legal_recovery_running` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.10 | [case](../cases/X023_ctrl_illegal_then_legal_recovery_running.md) |
| ✅ | 24 | `X024_ctrl_ready_not_indicating_recovery` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.10 | [case](../cases/X024_ctrl_ready_not_indicating_recovery.md) |
| ✅ | 25 | `X025_ctrl_valid_low_payload_noise` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.10 | [case](../cases/X025_ctrl_valid_low_payload_noise.md) |
| ✅ | 26 | `X026_ctrl_fast_oscillation_legal_illegal_mix` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.13 | [case](../cases/X026_ctrl_fast_oscillation_legal_illegal_mix.md) |
| ✅ | 27 | `X027_long_header_only_no_counters` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.13 | [case](../cases/X027_long_header_only_no_counters.md) |
| ✅ | 28 | `X028_long_missing_frame_number_byte1` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.29 | [case](../cases/X028_long_missing_frame_number_byte1.md) |
| ✅ | 29 | `X029_long_missing_event_counter_msb` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.29 | [case](../cases/X029_long_missing_event_counter_msb.md) |
| ✅ | 30 | `X030_long_missing_event_counter_lsb` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.29 | [case](../cases/X030_long_missing_event_counter_lsb.md) |
| ✅ | 31 | `X031_long_declared_len1_missing_payload` | stmt=87.63, branch=74.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.29 | [case](../cases/X031_long_declared_len1_missing_payload.md) |
| ✅ | 32 | `X032_long_declared_len2_only_one_hit_present` | stmt=87.63, branch=76.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.46 | [case](../cases/X032_long_declared_len2_only_one_hit_present.md) |
| ✅ | 33 | `X033_long_declared_len_large_cut_midpayload` | stmt=87.63, branch=76.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=21.66 | [case](../cases/X033_long_declared_len_large_cut_midpayload.md) |
| ✅ | 34 | `X034_long_bad_trailer_symbol_replaced_with_data` | stmt=87.63, branch=76.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=22.32 | [case](../cases/X034_long_bad_trailer_symbol_replaced_with_data.md) |
| ✅ | 35 | `X035_long_extra_payload_after_declared_len` | stmt=87.63, branch=76.00, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=22.32 | [case](../cases/X035_long_extra_payload_after_declared_len.md) |
| ✅ | 36 | `X036_long_new_header_inside_payload` | stmt=87.63, branch=76.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.56 | [case](../cases/X036_long_new_header_inside_payload.md) |
| ✅ | 37 | `X037_long_comma_inside_payload_mode0` | stmt=87.63, branch=76.67, cond=45.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.56 | [case](../cases/X037_long_comma_inside_payload_mode0.md) |
| ✅ | 38 | `X038_long_comma_inside_payload_mode1` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X038_long_comma_inside_payload_mode1.md) |
| ✅ | 39 | `X039_long_valid_low_entire_malformed_frame` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X039_long_valid_low_entire_malformed_frame.md) |
| ✅ | 40 | `X040_short_header_only_no_counters` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X040_short_header_only_no_counters.md) |
| ✅ | 41 | `X041_short_missing_frame_number_byte1` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X041_short_missing_frame_number_byte1.md) |
| ✅ | 42 | `X042_short_missing_event_counter_msb` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X042_short_missing_event_counter_msb.md) |
| ✅ | 43 | `X043_short_missing_event_counter_lsb` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X043_short_missing_event_counter_lsb.md) |
| ✅ | 44 | `X044_short_declared_len1_missing_payload` | stmt=87.63, branch=76.67, cond=50.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=28.92 | [case](../cases/X044_short_declared_len1_missing_payload.md) |
| ✅ | 45 | `X045_short_declared_len2_halfword_only` | stmt=89.69, branch=80.67, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=31.13 | [case](../cases/X045_short_declared_len2_halfword_only.md) |
| ✅ | 46 | `X046_short_declared_len3_cut_in_unpack_extra` | stmt=91.41, branch=82.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=33.31 | [case](../cases/X046_short_declared_len3_cut_in_unpack_extra.md) |
| ✅ | 47 | `X047_short_declared_len_large_cut_midpayload` | stmt=91.41, branch=82.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=33.31 | [case](../cases/X047_short_declared_len_large_cut_midpayload.md) |
| ✅ | 48 | `X048_short_new_header_inside_payload` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X048_short_new_header_inside_payload.md) |
| ✅ | 49 | `X049_short_extra_payload_after_declared_len` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X049_short_extra_payload_after_declared_len.md) |
| ✅ | 50 | `X050_short_comma_inside_payload_mode0` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X050_short_comma_inside_payload_mode0.md) |
| ✅ | 51 | `X051_short_comma_inside_payload_mode1` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X051_short_comma_inside_payload_mode1.md) |
| ✅ | 52 | `X052_short_valid_low_entire_malformed_frame` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X052_short_valid_low_entire_malformed_frame.md) |
| ✅ | 53 | `X053_long_bad_crc_onebit_flip` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X053_long_bad_crc_onebit_flip.md) |
| ✅ | 54 | `X054_long_bad_crc_twobit_flip` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X054_long_bad_crc_twobit_flip.md) |
| ✅ | 55 | `X055_long_bad_crc_allzero_tail` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X055_long_bad_crc_allzero_tail.md) |
| ✅ | 56 | `X056_long_bad_crc_allones_tail` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X056_long_bad_crc_allones_tail.md) |
| ✅ | 57 | `X057_short_bad_crc_onebit_flip` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X057_short_bad_crc_onebit_flip.md) |
| ✅ | 58 | `X058_short_bad_crc_twobit_flip` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X058_short_bad_crc_twobit_flip.md) |
| ✅ | 59 | `X059_short_bad_crc_allzero_tail` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X059_short_bad_crc_allzero_tail.md) |
| ✅ | 60 | `X060_short_bad_crc_allones_tail` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X060_short_bad_crc_allones_tail.md) |
| ✅ | 61 | `X061_clean_after_bad_crc_long` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X061_clean_after_bad_crc_long.md) |
| ✅ | 62 | `X062_clean_after_bad_crc_short` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X062_clean_after_bad_crc_short.md) |
| ✅ | 63 | `X063_multiple_bad_crc_reads_midupdate` | stmt=91.75, branch=82.67, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.42 | [case](../cases/X063_multiple_bad_crc_reads_midupdate.md) |
| ✅ | 64 | `X064_bad_crc_during_terminating` | stmt=94.16, branch=86.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.69 | [case](../cases/X064_bad_crc_during_terminating.md) |
| ✅ | 65 | `X065_bad_crc_during_runprepare_reset` | stmt=94.16, branch=86.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.69 | [case](../cases/X065_bad_crc_during_runprepare_reset.md) |
| ✅ | 66 | `X066_long_parity_error_first_payload_byte` | stmt=94.50, branch=87.33, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.89 | [case](../cases/X066_long_parity_error_first_payload_byte.md) |
| ✅ | 67 | `X067_long_parity_error_last_payload_byte` | stmt=94.50, branch=87.33, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=35.89 | [case](../cases/X067_long_parity_error_last_payload_byte.md) |
| ✅ | 68 | `X068_long_decode_error_middle_payload_byte` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X068_long_decode_error_middle_payload_byte.md) |
| ✅ | 69 | `X069_short_parity_error_even_hit` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X069_short_parity_error_even_hit.md) |
| ✅ | 70 | `X070_short_parity_error_odd_hit` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X070_short_parity_error_odd_hit.md) |
| ✅ | 71 | `X071_short_decode_error_unpack_extra` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X071_short_decode_error_unpack_extra.md) |
| ✅ | 72 | `X072_hiterror_then_clean_hit_same_frame` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X072_hiterror_then_clean_hit_same_frame.md) |
| ✅ | 73 | `X073_losssync_on_header_only` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.02 | [case](../cases/X073_losssync_on_header_only.md) |
| ✅ | 74 | `X074_losssync_midpayload_only` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.28 | [case](../cases/X074_losssync_midpayload_only.md) |
| ✅ | 75 | `X075_losssync_on_crc_only` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.28 | [case](../cases/X075_losssync_on_crc_only.md) |
| ✅ | 76 | `X076_parity_and_decode_same_hit` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.28 | [case](../cases/X076_parity_and_decode_same_hit.md) |
| ✅ | 77 | `X077_hiterror_plus_losssync_same_frame` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.28 | [case](../cases/X077_hiterror_plus_losssync_same_frame.md) |
| ✅ | 78 | `X078_clean_frame_after_combined_errors` | stmt=94.50, branch=87.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.28 | [case](../cases/X078_clean_frame_after_combined_errors.md) |
| ✅ | 79 | `X079_mode0_comma_in_frame_counter` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.41 | [case](../cases/X079_mode0_comma_in_frame_counter.md) |
| ✅ | 80 | `X080_mode0_comma_in_event_counter` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.41 | [case](../cases/X080_mode0_comma_in_event_counter.md) |
| ✅ | 81 | `X081_mode0_comma_in_long_unpack` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.41 | [case](../cases/X081_mode0_comma_in_long_unpack.md) |
| ✅ | 82 | `X082_mode0_comma_in_short_unpack` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X082_mode0_comma_in_short_unpack.md) |
| ✅ | 83 | `X083_mode0_abort_then_postabort_garbage` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X083_mode0_abort_then_postabort_garbage.md) |
| ✅ | 84 | `X084_mode0_abort_then_immediate_header` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X084_mode0_abort_then_immediate_header.md) |
| ✅ | 85 | `X085_mode1_comma_in_frame_counter` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X085_mode1_comma_in_frame_counter.md) |
| ✅ | 86 | `X086_mode1_comma_in_event_counter` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X086_mode1_comma_in_event_counter.md) |
| ✅ | 87 | `X087_mode1_comma_in_long_unpack` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X087_mode1_comma_in_long_unpack.md) |
| ✅ | 88 | `X088_mode1_comma_in_short_unpack` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X088_mode1_comma_in_short_unpack.md) |
| ✅ | 89 | `X089_mode1_hold_then_bad_crc` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X089_mode1_hold_then_bad_crc.md) |
| ✅ | 90 | `X090_mode1_hold_then_runprepare` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X090_mode1_hold_then_runprepare.md) |
| ✅ | 91 | `X091_mode_compare_same_corrupt_stream` | stmt=94.85, branch=88.00, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.65 | [case](../cases/X091_mode_compare_same_corrupt_stream.md) |
| ✅ | 92 | `X092_csr_read_unused_address_flood` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X092_csr_read_unused_address_flood.md) |
| ✅ | 93 | `X093_csr_write_unused_address_flood` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X093_csr_write_unused_address_flood.md) |
| ✅ | 94 | `X094_csr_read_write_same_cycle_overlap` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X094_csr_read_write_same_cycle_overlap.md) |
| ✅ | 95 | `X095_csr_control_write_during_header_accept` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X095_csr_control_write_during_header_accept.md) |
| ✅ | 96 | `X096_csr_control_write_during_payload` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X096_csr_control_write_during_payload.md) |
| ✅ | 97 | `X097_csr_poll_word2_every_cycle_open_frame` | stmt=94.85, branch=88.67, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X097_csr_poll_word2_every_cycle_open_frame.md) |
| ✅ | 98 | `X098_csr_poll_word1_every_cycle_badcrc` | stmt=95.19, branch=89.33, cond=62.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=36.78 | [case](../cases/X098_csr_poll_word1_every_cycle_badcrc.md) |
| ✅ | 99 | `X099_csr_mask_low_then_illegal_ctrl` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.07 | [case](../cases/X099_csr_mask_low_then_illegal_ctrl.md) |
| ✅ | 100 | `X100_csr_mask_high_after_error_state` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.07 | [case](../cases/X100_csr_mask_high_after_error_state.md) |
| ✅ | 101 | `X101_waitrequest_expected_high_idle_negative` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.07 | [case](../cases/X101_waitrequest_expected_high_idle_negative.md) |
| ✅ | 102 | `X102_waitrequest_stuck_low_detection` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.07 | [case](../cases/X102_waitrequest_stuck_low_detection.md) |
| ✅ | 103 | `X103_word0_status_read_before_headerinfo` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.07 | [case](../cases/X103_word0_status_read_before_headerinfo.md) |
| ✅ | 104 | `X104_word0_status_read_after_runprepare` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X104_word0_status_read_after_runprepare.md) |
| ✅ | 105 | `X105_postterminate_fresh_frame_attempt_from_idle` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X105_postterminate_fresh_frame_attempt_from_idle.md) |
| ✅ | 106 | `X106_postterminate_header_same_cycle_as_ctrl_edge` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X106_postterminate_header_same_cycle_as_ctrl_edge.md) |
| ✅ | 107 | `X107_stop_midframe_without_terminal_marker` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X107_stop_midframe_without_terminal_marker.md) |
| ✅ | 108 | `X108_stop_midframe_ready_never_drops` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X108_stop_midframe_ready_never_drops.md) |
| ✅ | 109 | `X109_stop_during_long_frame_then_second_header` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X109_stop_during_long_frame_then_second_header.md) |
| ✅ | 110 | `X110_stop_during_short_frame_then_second_header` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X110_stop_during_short_frame_then_second_header.md) |
| ✅ | 111 | `X111_stop_then_illegal_ctrl_then_idle` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X111_stop_then_illegal_ctrl_then_idle.md) |
| ✅ | 112 | `X112_stop_then_runprepare_before_eop` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X112_stop_then_runprepare_before_eop.md) |
| ✅ | 113 | `X113_stop_then_sync_without_idle` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X113_stop_then_sync_without_idle.md) |
| ✅ | 114 | `X114_stop_then_running_reopen_window` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X114_stop_then_running_reopen_window.md) |
| ✅ | 115 | `X115_emulator_mismatch_repro_openframe_only` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X115_emulator_mismatch_repro_openframe_only.md) |
| ✅ | 116 | `X116_emulator_mismatch_repro_postedge_header` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X116_emulator_mismatch_repro_postedge_header.md) |
| ✅ | 117 | `X117_termination_gap_signature_regression` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X117_termination_gap_signature_regression.md) |
| ✅ | 118 | `X118_cfgC_channel_width1_invalid_channel_drive` | stmt=95.19, branch=89.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=37.57 | [case](../cases/X118_cfgC_channel_width1_invalid_channel_drive.md) |
| ✅ | 119 | `X119_cfgD_channel_width8_random_wide_values` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X119_cfgD_channel_width8_random_wide_values.md) |
| ✅ | 120 | `X120_cfgE_addrwidth1_highaddr_alias_misuse` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X120_cfgE_addrwidth1_highaddr_alias_misuse.md) |
| ✅ | 121 | `X121_cfgF_addrwidth8_highaddr_abuse` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X121_cfgF_addrwidth8_highaddr_abuse.md) |
| ✅ | 122 | `X122_cfgB_modehalt1_abort_assumption_negative` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X122_cfgB_modehalt1_abort_assumption_negative.md) |
| ✅ | 123 | `X123_cfgA_modehalt0_hold_assumption_negative` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X123_cfgA_modehalt0_hold_assumption_negative.md) |
| ✅ | 124 | `X124_cfgG_debuglv2_functional_delta_negative` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X124_cfgG_debuglv2_functional_delta_negative.md) |
| ✅ | 125 | `X125_hw_tcl_readlatency_mismatch_negative` | stmt=95.19, branch=89.33, cond=67.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=49.14 | [case](../cases/X125_hw_tcl_readlatency_mismatch_negative.md) |
| ✅ | 126 | `X126_hw_tcl_maxchannel_mismatch_negative` | stmt=95.19, branch=89.33, cond=70.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=53.62 | [case](../cases/X126_hw_tcl_maxchannel_mismatch_negative.md) |
| ✅ | 127 | `X127_hw_tcl_ctrl_readylatency_assumption_negative` | stmt=95.19, branch=89.33, cond=70.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=53.62 | [case](../cases/X127_hw_tcl_ctrl_readylatency_assumption_negative.md) |
| ✅ | 128 | `X128_hw_tcl_headerinfo_channelwidth_mismatch` | stmt=95.19, branch=89.33, cond=70.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=53.62 | [case](../cases/X128_hw_tcl_headerinfo_channelwidth_mismatch.md) |
| ✅ | 129 | `X129_build_matrix_compile_smoke_failfast` | stmt=95.19, branch=89.33, cond=70.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=53.62 | [case](../cases/X129_build_matrix_compile_smoke_failfast.md) |
| ✅ | 130 | `X130_known_gap_bundle_current_contract` | stmt=95.19, branch=89.33, cond=70.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=53.62 | [case](../cases/X130_known_gap_bundle_current_contract.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_
