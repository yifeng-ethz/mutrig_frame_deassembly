# ⚠️ PROF bucket

**Planned:** `130` &nbsp; **Evidenced:** `70` &nbsp; **Status:** ⚠️

## Merged code coverage (this bucket)

<!-- column legend:
  metric          = code-coverage category (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle)
  merged_pct      = bucket-local ordered-merge percentage across all evidenced cases
  target          = workflow coverage target (blank = no hard target for that category)
  status          = target check vs merged_pct
-->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ⚠️ | stmt | 92.78 | 95.0 |
| ⚠️ | branch | 86.67 | 90.0 |
| ℹ️ | cond | 65.00 | - |
| ℹ️ | expr | 98.77 | - |
| ❓ | fsm_state | n/a | 95.0 |
| ❓ | fsm_trans | n/a | 90.0 |
| ⚠️ | toggle | 52.29 | 80.0 |

## Ordered merge trace

<!-- each row is the merged coverage total after that case was added to the bucket in case-id order. -->

| status | step | case_id | merged_total (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | detail |
|:---:|---:|---|---|---|
| ✅ | 1 | `P012_long_len767_32_frames` | stmt=81.10, branch=67.33, cond=47.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=40.05 | [case](../cases/P012_long_len767_32_frames.md) |
| ✅ | 2 | `P013_long_len1023_16_frames` | stmt=81.10, branch=67.33, cond=47.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=40.71 | [case](../cases/P013_long_len1023_16_frames.md) |
| ✅ | 3 | `P027_cadence_1_1_16_1` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.04 | [case](../cases/P027_cadence_1_1_16_1.md) |
| ✅ | 4 | `P028_cadence_1_1_64_1` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.04 | [case](../cases/P028_cadence_1_1_64_1.md) |
| ✅ | 5 | `P029_cadence_2_2_64_2` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.04 | [case](../cases/P029_cadence_2_2_64_2.md) |
| ✅ | 6 | `P030_cadence_4_4_128_4` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.04 | [case](../cases/P030_cadence_4_4_128_4.md) |
| ✅ | 7 | `P031_cadence_8_8_256_8` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.04 | [case](../cases/P031_cadence_8_8_256_8.md) |
| ✅ | 8 | `P032_cadence_16_16_512_16` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.20 | [case](../cases/P032_cadence_16_16_512_16.md) |
| ✅ | 9 | `P033_cadence_running_only_1024` | stmt=81.79, branch=68.00, cond=52.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=41.20 | [case](../cases/P033_cadence_running_only_1024.md) |
| ✅ | 10 | `P035_cadence_stop_after_every_frame` | stmt=84.19, branch=72.00, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.03 | [case](../cases/P035_cadence_stop_after_every_frame.md) |
| ✅ | 11 | `P036_cadence_stop_every_other_frame` | stmt=84.19, branch=72.00, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.03 | [case](../cases/P036_cadence_stop_every_other_frame.md) |
| ✅ | 12 | `P037_cadence_legal_and_illegal_ctrl_mix` | stmt=84.19, branch=72.00, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.03 | [case](../cases/P037_cadence_legal_and_illegal_ctrl_mix.md) |
| ✅ | 13 | `P038_cadence_runprepare_burst_reset` | stmt=84.19, branch=72.00, cond=55.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.52 | [case](../cases/P038_cadence_runprepare_burst_reset.md) |
| ✅ | 14 | `P039_cadence_terminating_hold_open_frame` | stmt=84.54, branch=72.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.59 | [case](../cases/P039_cadence_terminating_hold_open_frame.md) |
| ✅ | 15 | `P040_mask_toggle_every_frame` | stmt=84.54, branch=72.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.59 | [case](../cases/P040_mask_toggle_every_frame.md) |
| ✅ | 16 | `P049_counter_poll_during_open_frame` | stmt=84.54, branch=72.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.59 | [case](../cases/P049_counter_poll_during_open_frame.md) |
| ✅ | 17 | `P050_counter_poll_during_bad_crc_mix` | stmt=84.54, branch=72.67, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.59 | [case](../cases/P050_counter_poll_during_bad_crc_mix.md) |
| ✅ | 18 | `P051_status_poll_after_every_headerinfo` | stmt=85.57, branch=74.00, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.89 | [case](../cases/P051_status_poll_after_every_headerinfo.md) |
| ✅ | 19 | `P053_all_clean_long_crc` | stmt=85.57, branch=74.00, cond=57.50, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=42.89 | [case](../cases/P053_all_clean_long_crc.md) |
| ✅ | 20 | `P054_all_clean_short_crc` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P054_all_clean_short_crc.md) |
| ✅ | 21 | `P055_bad_every_32nd_frame` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P055_bad_every_32nd_frame.md) |
| ✅ | 22 | `P056_bad_every_16th_frame` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P056_bad_every_16th_frame.md) |
| ✅ | 23 | `P057_bad_every_8th_frame` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P057_bad_every_8th_frame.md) |
| ✅ | 24 | `P058_bad_every_4th_frame` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P058_bad_every_4th_frame.md) |
| ✅ | 25 | `P059_bad_every_other_frame` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P059_bad_every_other_frame.md) |
| ✅ | 26 | `P060_all_bad_long_crc` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P060_all_bad_long_crc.md) |
| ✅ | 27 | `P061_all_bad_short_crc` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P061_all_bad_short_crc.md) |
| ✅ | 28 | `P062_mixed_long_short_bad_ratio10` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P062_mixed_long_short_bad_ratio10.md) |
| ✅ | 29 | `P063_mixed_long_short_bad_ratio50` | stmt=89.35, branch=80.00, cond=60.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=43.55 | [case](../cases/P063_mixed_long_short_bad_ratio50.md) |
| ✅ | 30 | `P064_bad_crc_plus_loss_sync_mix` | stmt=89.35, branch=80.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=44.17 | [case](../cases/P064_bad_crc_plus_loss_sync_mix.md) |
| ✅ | 31 | `P065_bad_crc_plus_hit_error_mix` | stmt=89.35, branch=80.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=44.17 | [case](../cases/P065_bad_crc_plus_hit_error_mix.md) |
| ✅ | 32 | `P066_mode0_comma_every_64th_frame_counter` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=44.31 | [case](../cases/P066_mode0_comma_every_64th_frame_counter.md) |
| ✅ | 33 | `P067_mode0_comma_every_64th_event_counter` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=44.31 | [case](../cases/P067_mode0_comma_every_64th_event_counter.md) |
| ✅ | 34 | `P068_mode0_comma_every_64th_unpack` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=44.31 | [case](../cases/P068_mode0_comma_every_64th_unpack.md) |
| ✅ | 35 | `P069_mode0_comma_random_positions` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P069_mode0_comma_random_positions.md) |
| ✅ | 36 | `P070_mode0_abort_then_clean_restart` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P070_mode0_abort_then_clean_restart.md) |
| ✅ | 37 | `P071_mode1_comma_every_64th_frame_counter` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P071_mode1_comma_every_64th_frame_counter.md) |
| ✅ | 38 | `P072_mode1_comma_every_64th_event_counter` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P072_mode1_comma_every_64th_event_counter.md) |
| ✅ | 39 | `P073_mode1_comma_every_64th_unpack` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P073_mode1_comma_every_64th_unpack.md) |
| ✅ | 40 | `P074_mode1_comma_random_positions` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P074_mode1_comma_random_positions.md) |
| ✅ | 41 | `P075_mode1_hold_with_loss_sync_mix` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P075_mode1_hold_with_loss_sync_mix.md) |
| ✅ | 42 | `P076_mode0_abort_with_loss_sync_mix` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P076_mode0_abort_with_loss_sync_mix.md) |
| ✅ | 43 | `P077_mode0_to_mode1_build_comparison` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P077_mode0_to_mode1_build_comparison.md) |
| ✅ | 44 | `P078_mode1_to_mode0_build_comparison` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.62 | [case](../cases/P078_mode1_to_mode0_build_comparison.md) |
| ✅ | 45 | `P082_cfgB_modehalt1_comma_soak` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.95 | [case](../cases/P082_cfgB_modehalt1_comma_soak.md) |
| ✅ | 46 | `P084_cfgD_channel8_wide_sweep` | stmt=89.69, branch=80.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=46.95 | [case](../cases/P084_cfgD_channel8_wide_sweep.md) |
| ✅ | 47 | `P085_cfgE_addr1_poll_soak` | stmt=90.03, branch=81.33, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P085_cfgE_addr1_poll_soak.md) |
| ✅ | 48 | `P086_cfgF_addr8_unused_space_sweep` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P086_cfgF_addr8_unused_space_sweep.md) |
| ✅ | 49 | `P089_cfgD_zerohit_onehit_mix` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P089_cfgD_zerohit_onehit_mix.md) |
| ✅ | 50 | `P091_crossbuild_reference_trace_equivalence` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P091_crossbuild_reference_trace_equivalence.md) |
| ✅ | 51 | `P092_long_short_alternate_every_frame` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P092_long_short_alternate_every_frame.md) |
| ✅ | 52 | `P094_zerohit_to_fullhit_alternate` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P094_zerohit_to_fullhit_alternate.md) |
| ✅ | 53 | `P095_channel_rotate_0_to_15` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P095_channel_rotate_0_to_15.md) |
| ✅ | 54 | `P096_channel_rotate_sparse` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P096_channel_rotate_sparse.md) |
| ✅ | 55 | `P101_losssync_burst_every_100_frames` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P101_losssync_burst_every_100_frames.md) |
| ✅ | 56 | `P104_metadata_poll_under_mode_mix` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P104_metadata_poll_under_mode_mix.md) |
| ✅ | 57 | `P105_stop_before_header_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P105_stop_before_header_every_run.md) |
| ✅ | 58 | `P106_stop_on_header_start_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P106_stop_on_header_start_every_run.md) |
| ✅ | 59 | `P107_stop_after_event_counter_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P107_stop_after_event_counter_every_run.md) |
| ✅ | 60 | `P108_stop_after_first_long_hit_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P108_stop_after_first_long_hit_every_run.md) |
| ✅ | 61 | `P109_stop_after_first_short_hit_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P109_stop_after_first_short_hit_every_run.md) |
| ✅ | 62 | `P110_stop_during_crc_calc_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P110_stop_during_crc_calc_every_run.md) |
| ✅ | 63 | `P111_stop_during_crc_check_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P111_stop_during_crc_check_every_run.md) |
| ✅ | 64 | `P112_stop_then_immediate_idle_every_run` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P112_stop_then_immediate_idle_every_run.md) |
| ✅ | 65 | `P113_stop_every_other_frame_mixed_modes` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P113_stop_every_other_frame_mixed_modes.md) |
| ✅ | 66 | `P114_stop_under_bad_crc_mix` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P114_stop_under_bad_crc_mix.md) |
| ✅ | 67 | `P115_stop_under_hiterror_mix` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P115_stop_under_hiterror_mix.md) |
| ✅ | 68 | `P116_stop_under_losssync_mix` | stmt=90.03, branch=82.00, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=51.34 | [case](../cases/P116_stop_under_losssync_mix.md) |
| ✅ | 69 | `P126_seed09_comma_mix_mode0_100k_cycles` | stmt=92.78, branch=86.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=52.29 | [case](../cases/P126_seed09_comma_mix_mode0_100k_cycles.md) |
| ✅ | 70 | `P127_seed10_comma_mix_mode1_100k_cycles` | stmt=92.78, branch=86.67, cond=65.00, expr=98.77, fsm_state=n/a, fsm_trans=n/a, toggle=52.29 | [case](../cases/P127_seed10_comma_mix_mode1_100k_cycles.md) |

---
_Back to [dashboard](../../DV_REPORT.md)_
