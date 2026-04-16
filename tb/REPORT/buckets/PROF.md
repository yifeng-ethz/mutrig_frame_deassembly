# ⚠️ PROF bucket

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
| ✅ | stmt | 95.33 | 95.0 |
| ⚠️ | branch | 89.47 | 90.0 |
| ℹ️ | cond | 86.21 | - |
| ℹ️ | expr | 98.77 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ⚠️ | fsm_trans | 80.00 | 90.0 |
| ⚠️ | toggle | 62.03 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `P001_long_len0_10k_frames` | stmt=72.76, branch=50.33, cond=44.83, expr=98.77, fsm_state=71.43, fsm_trans=33.33, toggle=19.29 | [case](../cases/P001_long_len0_10k_frames.md) |
| ✅ | 2 | `P002_long_len1_10k_frames` | stmt=80.16, branch=64.24, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=43.59 | [case](../cases/P002_long_len1_10k_frames.md) |
| ✅ | 3 | `P003_long_len2_10k_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.26 | [case](../cases/P003_long_len2_10k_frames.md) |
| ✅ | 4 | `P004_long_len3_10k_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=44.68 | [case](../cases/P004_long_len3_10k_frames.md) |
| ✅ | 5 | `P005_long_len7_5k_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=45.31 | [case](../cases/P005_long_len7_5k_frames.md) |
| ✅ | 6 | `P006_long_len15_2k_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=45.93 | [case](../cases/P006_long_len15_2k_frames.md) |
| ✅ | 7 | `P007_long_len31_1k_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=46.56 | [case](../cases/P007_long_len31_1k_frames.md) |
| ✅ | 8 | `P008_long_len63_512_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=47.60 | [case](../cases/P008_long_len63_512_frames.md) |
| ✅ | 9 | `P009_long_len127_256_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=48.23 | [case](../cases/P009_long_len127_256_frames.md) |
| ✅ | 10 | `P010_long_len255_128_frames` | stmt=80.16, branch=66.23, cond=58.62, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=48.85 | [case](../cases/P010_long_len255_128_frames.md) |
| ✅ | 11 | `P011_long_len511_64_frames` | stmt=80.16, branch=66.23, cond=62.07, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=49.53 | [case](../cases/P011_long_len511_64_frames.md) |
| ✅ | 12 | `P012_long_len767_32_frames` | stmt=80.16, branch=66.23, cond=62.07, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=49.84 | [case](../cases/P012_long_len767_32_frames.md) |
| ✅ | 13 | `P013_long_len1023_16_frames` | stmt=80.16, branch=66.23, cond=62.07, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=49.84 | [case](../cases/P013_long_len1023_16_frames.md) |
| ✅ | 14 | `P014_short_len0_10k_frames` | stmt=80.93, branch=67.55, cond=62.07, expr=98.77, fsm_state=85.71, fsm_trans=46.67, toggle=50.31 | [case](../cases/P014_short_len0_10k_frames.md) |
| ✅ | 15 | `P015_short_len1_10k_frames` | stmt=84.44, branch=72.19, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=60.00, toggle=51.46 | [case](../cases/P015_short_len1_10k_frames.md) |
| ✅ | 16 | `P016_short_len2_10k_frames` | stmt=87.94, branch=76.82, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P016_short_len2_10k_frames.md) |
| ✅ | 17 | `P017_short_len3_10k_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P017_short_len3_10k_frames.md) |
| ✅ | 18 | `P018_short_len7_5k_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P018_short_len7_5k_frames.md) |
| ✅ | 19 | `P019_short_len15_2k_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P019_short_len15_2k_frames.md) |
| ✅ | 20 | `P020_short_len31_1k_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P020_short_len31_1k_frames.md) |
| ✅ | 21 | `P021_short_len63_512_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P021_short_len63_512_frames.md) |
| ✅ | 22 | `P022_short_len127_256_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P022_short_len127_256_frames.md) |
| ✅ | 23 | `P023_short_len255_128_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P023_short_len255_128_frames.md) |
| ✅ | 24 | `P024_short_len511_64_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=52.97 | [case](../cases/P024_short_len511_64_frames.md) |
| ✅ | 25 | `P025_short_len767_32_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P025_short_len767_32_frames.md) |
| ✅ | 26 | `P026_short_len1023_16_frames` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P026_short_len1023_16_frames.md) |
| ✅ | 27 | `P027_cadence_1_1_16_1` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P027_cadence_1_1_16_1.md) |
| ✅ | 28 | `P028_cadence_1_1_64_1` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P028_cadence_1_1_64_1.md) |
| ✅ | 29 | `P029_cadence_2_2_64_2` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P029_cadence_2_2_64_2.md) |
| ✅ | 30 | `P030_cadence_4_4_128_4` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P030_cadence_4_4_128_4.md) |
| ✅ | 31 | `P031_cadence_8_8_256_8` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.28 | [case](../cases/P031_cadence_8_8_256_8.md) |
| ✅ | 32 | `P032_cadence_16_16_512_16` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.44 | [case](../cases/P032_cadence_16_16_512_16.md) |
| ✅ | 33 | `P033_cadence_running_only_1024` | stmt=88.72, branch=77.48, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.44 | [case](../cases/P033_cadence_running_only_1024.md) |
| ✅ | 34 | `P034_cadence_idle_monitoring_only` | stmt=89.11, branch=78.15, cond=65.52, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.44 | [case](../cases/P034_cadence_idle_monitoring_only.md) |
| ✅ | 35 | `P035_cadence_stop_after_every_frame` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.86 | [case](../cases/P035_cadence_stop_after_every_frame.md) |
| ✅ | 36 | `P036_cadence_stop_every_other_frame` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.86 | [case](../cases/P036_cadence_stop_every_other_frame.md) |
| ✅ | 37 | `P037_cadence_legal_and_illegal_ctrl_mix` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.86 | [case](../cases/P037_cadence_legal_and_illegal_ctrl_mix.md) |
| ✅ | 38 | `P038_cadence_runprepare_burst_reset` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=54.38 | [case](../cases/P038_cadence_runprepare_burst_reset.md) |
| ✅ | 39 | `P039_cadence_terminating_hold_open_frame` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P039_cadence_terminating_hold_open_frame.md) |
| ✅ | 40 | `P040_mask_toggle_every_frame` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P040_mask_toggle_every_frame.md) |
| ✅ | 41 | `P041_mask_toggle_every_2_frames` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P041_mask_toggle_every_2_frames.md) |
| ✅ | 42 | `P042_mask_toggle_every_4_frames` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P042_mask_toggle_every_4_frames.md) |
| ✅ | 43 | `P043_mask_toggle_every_8_frames` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P043_mask_toggle_every_8_frames.md) |
| ✅ | 44 | `P044_mask_toggle_every_16_frames` | stmt=91.83, branch=83.44, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.38 | [case](../cases/P044_mask_toggle_every_16_frames.md) |
| ✅ | 45 | `P045_mask_low_long_idle_soak` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.54 | [case](../cases/P045_mask_low_long_idle_soak.md) |
| ✅ | 46 | `P046_mask_high_long_running_soak` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.64 | [case](../cases/P046_mask_high_long_running_soak.md) |
| ✅ | 47 | `P047_midframe_mask_drop_repeated` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.64 | [case](../cases/P047_midframe_mask_drop_repeated.md) |
| ✅ | 48 | `P048_midframe_mask_raise_repeated` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.64 | [case](../cases/P048_midframe_mask_raise_repeated.md) |
| ✅ | 49 | `P049_counter_poll_during_open_frame` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.64 | [case](../cases/P049_counter_poll_during_open_frame.md) |
| ✅ | 50 | `P050_counter_poll_during_bad_crc_mix` | stmt=93.00, branch=85.43, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.64 | [case](../cases/P050_counter_poll_during_bad_crc_mix.md) |
| ✅ | 51 | `P051_status_poll_after_every_headerinfo` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P051_status_poll_after_every_headerinfo.md) |
| ✅ | 52 | `P052_control_write_soak_no_counter_side_effect` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P052_control_write_soak_no_counter_side_effect.md) |
| ✅ | 53 | `P053_all_clean_long_crc` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P053_all_clean_long_crc.md) |
| ✅ | 54 | `P054_all_clean_short_crc` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P054_all_clean_short_crc.md) |
| ✅ | 55 | `P055_bad_every_32nd_frame` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P055_bad_every_32nd_frame.md) |
| ✅ | 56 | `P056_bad_every_16th_frame` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P056_bad_every_16th_frame.md) |
| ✅ | 57 | `P057_bad_every_8th_frame` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P057_bad_every_8th_frame.md) |
| ✅ | 58 | `P058_bad_every_4th_frame` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P058_bad_every_4th_frame.md) |
| ✅ | 59 | `P059_bad_every_other_frame` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P059_bad_every_other_frame.md) |
| ✅ | 60 | `P060_all_bad_long_crc` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P060_all_bad_long_crc.md) |
| ✅ | 61 | `P061_all_bad_short_crc` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P061_all_bad_short_crc.md) |
| ✅ | 62 | `P062_mixed_long_short_bad_ratio10` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P062_mixed_long_short_bad_ratio10.md) |
| ✅ | 63 | `P063_mixed_long_short_bad_ratio50` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=54.85 | [case](../cases/P063_mixed_long_short_bad_ratio50.md) |
| ✅ | 64 | `P064_bad_crc_plus_loss_sync_mix` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=55.21 | [case](../cases/P064_bad_crc_plus_loss_sync_mix.md) |
| ✅ | 65 | `P065_bad_crc_plus_hit_error_mix` | stmt=94.16, branch=86.75, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=55.21 | [case](../cases/P065_bad_crc_plus_hit_error_mix.md) |
| ✅ | 66 | `P066_mode0_comma_every_64th_frame_counter` | stmt=94.55, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=55.32 | [case](../cases/P066_mode0_comma_every_64th_frame_counter.md) |
| ✅ | 67 | `P067_mode0_comma_every_64th_event_counter` | stmt=94.55, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=55.32 | [case](../cases/P067_mode0_comma_every_64th_event_counter.md) |
| ✅ | 68 | `P068_mode0_comma_every_64th_unpack` | stmt=94.55, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=55.32 | [case](../cases/P068_mode0_comma_every_64th_unpack.md) |
| ✅ | 69 | `P069_mode0_comma_random_positions` | stmt=94.55, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=56.10 | [case](../cases/P069_mode0_comma_random_positions.md) |
| ✅ | 70 | `P070_mode0_abort_then_clean_restart` | stmt=94.55, branch=87.42, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=56.10 | [case](../cases/P070_mode0_abort_then_clean_restart.md) |
| ✅ | 71 | `P071_mode1_comma_every_64th_frame_counter` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=56.10 | [case](../cases/P071_mode1_comma_every_64th_frame_counter.md) |
| ✅ | 72 | `P072_mode1_comma_every_64th_event_counter` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=56.10 | [case](../cases/P072_mode1_comma_every_64th_event_counter.md) |
| ✅ | 73 | `P073_mode1_comma_every_64th_unpack` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=56.10 | [case](../cases/P073_mode1_comma_every_64th_unpack.md) |
| ✅ | 74 | `P074_mode1_comma_random_positions` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P074_mode1_comma_random_positions.md) |
| ✅ | 75 | `P075_mode1_hold_with_loss_sync_mix` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P075_mode1_hold_with_loss_sync_mix.md) |
| ✅ | 76 | `P076_mode0_abort_with_loss_sync_mix` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P076_mode0_abort_with_loss_sync_mix.md) |
| ✅ | 77 | `P077_mode0_to_mode1_build_comparison` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P077_mode0_to_mode1_build_comparison.md) |
| ✅ | 78 | `P078_mode1_to_mode0_build_comparison` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P078_mode1_to_mode0_build_comparison.md) |
| ✅ | 79 | `P079_cfgA_default_long_soak` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P079_cfgA_default_long_soak.md) |
| ✅ | 80 | `P080_cfgA_default_short_soak` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P080_cfgA_default_short_soak.md) |
| ✅ | 81 | `P081_cfgB_modehalt1_clean_soak` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P081_cfgB_modehalt1_clean_soak.md) |
| ✅ | 82 | `P082_cfgB_modehalt1_comma_soak` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P082_cfgB_modehalt1_comma_soak.md) |
| ✅ | 83 | `P083_cfgC_channel1_alternating_soak` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=60.27 | [case](../cases/P083_cfgC_channel1_alternating_soak.md) |
| ✅ | 84 | `P084_cfgD_channel8_wide_sweep` | stmt=94.55, branch=87.50, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=59.68 | [case](../cases/P084_cfgD_channel8_wide_sweep.md) |
| ✅ | 85 | `P085_cfgE_addr1_poll_soak` | stmt=94.94, branch=88.16, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.23 | [case](../cases/P085_cfgE_addr1_poll_soak.md) |
| ✅ | 86 | `P086_cfgF_addr8_unused_space_sweep` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P086_cfgF_addr8_unused_space_sweep.md) |
| ✅ | 87 | `P087_cfgG_debuglv2_functional_soak` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P087_cfgG_debuglv2_functional_soak.md) |
| ✅ | 88 | `P088_cfgC_termination_soak` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P088_cfgC_termination_soak.md) |
| ✅ | 89 | `P089_cfgD_zerohit_onehit_mix` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P089_cfgD_zerohit_onehit_mix.md) |
| ✅ | 90 | `P090_cfgE_control_churn_soak` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P090_cfgE_control_churn_soak.md) |
| ✅ | 91 | `P091_crossbuild_reference_trace_equivalence` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P091_crossbuild_reference_trace_equivalence.md) |
| ✅ | 92 | `P092_long_short_alternate_every_frame` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P092_long_short_alternate_every_frame.md) |
| ✅ | 93 | `P093_long_long_short_short_repeat` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P093_long_long_short_short_repeat.md) |
| ✅ | 94 | `P094_zerohit_to_fullhit_alternate` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P094_zerohit_to_fullhit_alternate.md) |
| ✅ | 95 | `P095_channel_rotate_0_to_15` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P095_channel_rotate_0_to_15.md) |
| ✅ | 96 | `P096_channel_rotate_sparse` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P096_channel_rotate_sparse.md) |
| ✅ | 97 | `P097_long_clean_short_bad_repeat` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P097_long_clean_short_bad_repeat.md) |
| ✅ | 98 | `P098_short_clean_long_bad_repeat` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P098_short_clean_long_bad_repeat.md) |
| ✅ | 99 | `P099_long_hiterror_short_clean_repeat` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P099_long_hiterror_short_clean_repeat.md) |
| ✅ | 100 | `P100_short_hiterror_long_clean_repeat` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P100_short_hiterror_long_clean_repeat.md) |
| ✅ | 101 | `P101_losssync_burst_every_100_frames` | stmt=94.94, branch=88.82, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.46 | [case](../cases/P101_losssync_burst_every_100_frames.md) |
| ✅ | 102 | `P102_parity_burst_every_100_frames` | stmt=95.33, branch=89.47, cond=82.76, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.67 | [case](../cases/P102_parity_burst_every_100_frames.md) |
| ✅ | 103 | `P103_decode_burst_every_100_frames` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P103_decode_burst_every_100_frames.md) |
| ✅ | 104 | `P104_metadata_poll_under_mode_mix` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P104_metadata_poll_under_mode_mix.md) |
| ✅ | 105 | `P105_stop_before_header_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P105_stop_before_header_every_run.md) |
| ✅ | 106 | `P106_stop_on_header_start_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P106_stop_on_header_start_every_run.md) |
| ✅ | 107 | `P107_stop_after_event_counter_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P107_stop_after_event_counter_every_run.md) |
| ✅ | 108 | `P108_stop_after_first_long_hit_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P108_stop_after_first_long_hit_every_run.md) |
| ✅ | 109 | `P109_stop_after_first_short_hit_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P109_stop_after_first_short_hit_every_run.md) |
| ✅ | 110 | `P110_stop_during_crc_calc_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P110_stop_during_crc_calc_every_run.md) |
| ✅ | 111 | `P111_stop_during_crc_check_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P111_stop_during_crc_check_every_run.md) |
| ✅ | 112 | `P112_stop_then_immediate_idle_every_run` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P112_stop_then_immediate_idle_every_run.md) |
| ✅ | 113 | `P113_stop_every_other_frame_mixed_modes` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P113_stop_every_other_frame_mixed_modes.md) |
| ✅ | 114 | `P114_stop_under_bad_crc_mix` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P114_stop_under_bad_crc_mix.md) |
| ✅ | 115 | `P115_stop_under_hiterror_mix` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P115_stop_under_hiterror_mix.md) |
| ✅ | 116 | `P116_stop_under_losssync_mix` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P116_stop_under_losssync_mix.md) |
| ✅ | 117 | `P117_stop_soak_10k_cycles_no_fresh_poststop_header` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.77 | [case](../cases/P117_stop_soak_10k_cycles_no_fresh_poststop_header.md) |
| ✅ | 118 | `P118_seed01_clean_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P118_seed01_clean_mix_100k_cycles.md) |
| ✅ | 119 | `P119_seed02_clean_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P119_seed02_clean_mix_100k_cycles.md) |
| ✅ | 120 | `P120_seed03_error_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P120_seed03_error_mix_100k_cycles.md) |
| ✅ | 121 | `P121_seed04_stop_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P121_seed04_stop_mix_100k_cycles.md) |
| ✅ | 122 | `P122_seed05_mask_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P122_seed05_mask_mix_100k_cycles.md) |
| ✅ | 123 | `P123_seed06_mode_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P123_seed06_mode_mix_100k_cycles.md) |
| ✅ | 124 | `P124_seed07_channel_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P124_seed07_channel_mix_100k_cycles.md) |
| ✅ | 125 | `P125_seed08_badcrc_mix_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=73.33, toggle=61.82 | [case](../cases/P125_seed08_badcrc_mix_100k_cycles.md) |
| ✅ | 126 | `P126_seed09_comma_mix_mode0_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=61.87 | [case](../cases/P126_seed09_comma_mix_mode0_100k_cycles.md) |
| ✅ | 127 | `P127_seed10_comma_mix_mode1_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=61.87 | [case](../cases/P127_seed10_comma_mix_mode1_100k_cycles.md) |
| ✅ | 128 | `P128_seed11_cfgD_wide_channel_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=62.03 | [case](../cases/P128_seed11_cfgD_wide_channel_100k_cycles.md) |
| ✅ | 129 | `P129_seed12_cfgE_narrow_addr_100k_cycles` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=62.03 | [case](../cases/P129_seed12_cfgE_narrow_addr_100k_cycles.md) |
| ✅ | 130 | `P130_seed13_upgrade_gap_observation_bundle` | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=62.03 | [case](../cases/P130_seed13_upgrade_gap_observation_bundle.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_
