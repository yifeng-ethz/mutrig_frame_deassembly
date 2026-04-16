# mutrig_frame_deassembly — REPORT index

**DUT:** `frame_rcv_ip` &nbsp; **Date:** `2026-04-16` &nbsp;
**RTL variant:** `after` &nbsp; **Seed:** `1`

## Legend

✅ pass / closed / target met &middot; ⚠️ partial / below target / known limitation &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Buckets

<!-- click a bucket row to open its ordered-merge trace and linked per-case pages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) |
|:---:|---|---:|---:|---|
| ⚠️ | [`BASIC`](buckets/BASIC.md) | 130 | 130 | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.73 |
| ⚠️ | [`EDGE`](buckets/EDGE.md) | 130 | 130 | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 |
| ⚠️ | [`PROF`](buckets/PROF.md) | 130 | 130 | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=62.03 |
| ⚠️ | [`ERROR`](buckets/ERROR.md) | 130 | 130 | stmt=96.11, branch=90.13, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=55.07 |

## Cross / continuous-frame runs

| status | run_id | kind | build | bucket | seq | txns | cross_pct |
|:---:|---|---|---|---|---|---:|---:|
| ❌ | [`all_buckets_frame_cfg_a`](cross/all_buckets_frame_cfg_a.md) | all_buckets_frame | CFG_A | - | ALL_CFG_A | 800 | 65.62 |
| ⚠️ | [`all_buckets_frame_cfg_b`](cross/all_buckets_frame_cfg_b.md) | all_buckets_frame | CFG_B | - | ALL_CFG_B | 76 | 40.62 |
| ⚠️ | [`all_buckets_frame_cfg_c`](cross/all_buckets_frame_cfg_c.md) | all_buckets_frame | CFG_C | - | ALL_CFG_C | 36 | 40.62 |
| ✅ | [`all_buckets_frame_cfg_d`](cross/all_buckets_frame_cfg_d.md) | all_buckets_frame | CFG_D | - | ALL_CFG_D | 27 | 59.38 |
| ⚠️ | [`all_buckets_frame_cfg_e`](cross/all_buckets_frame_cfg_e.md) | all_buckets_frame | CFG_E | - | ALL_CFG_E | 66 | 40.62 |
| ⚠️ | [`all_buckets_frame_cfg_f`](cross/all_buckets_frame_cfg_f.md) | all_buckets_frame | CFG_F | - | ALL_CFG_F | 3 | 37.5 |
| ⚠️ | [`all_buckets_frame_cfg_g`](cross/all_buckets_frame_cfg_g.md) | all_buckets_frame | CFG_G | - | ALL_CFG_G | 17 | 37.5 |
| ❌ | [`bucket_frame_basic_cfg_a`](cross/bucket_frame_basic_cfg_a.md) | bucket_frame | CFG_A | BASIC | BASIC_CFG_A | 87 | 59.38 |
| ⚠️ | [`bucket_frame_basic_cfg_b`](cross/bucket_frame_basic_cfg_b.md) | bucket_frame | CFG_B | BASIC | BASIC_CFG_B | 1 | 34.38 |
| ❌ | [`bucket_frame_edge_cfg_a`](cross/bucket_frame_edge_cfg_a.md) | bucket_frame | CFG_A | EDGE | EDGE_CFG_A | 61 | 40.62 |
| ⚠️ | [`bucket_frame_edge_cfg_b`](cross/bucket_frame_edge_cfg_b.md) | bucket_frame | CFG_B | EDGE | EDGE_CFG_B | 5 | 37.5 |
| ⚠️ | [`bucket_frame_edge_cfg_c`](cross/bucket_frame_edge_cfg_c.md) | bucket_frame | CFG_C | EDGE | EDGE_CFG_C | 3 | 37.5 |
| ⚠️ | [`bucket_frame_edge_cfg_d`](cross/bucket_frame_edge_cfg_d.md) | bucket_frame | CFG_D | EDGE | EDGE_CFG_D | 1 | 34.38 |
| ⚠️ | [`bucket_frame_edge_cfg_e`](cross/bucket_frame_edge_cfg_e.md) | bucket_frame | CFG_E | EDGE | EDGE_CFG_E | 1 | 34.38 |
| ⚠️ | [`bucket_frame_edge_cfg_f`](cross/bucket_frame_edge_cfg_f.md) | bucket_frame | CFG_F | EDGE | EDGE_CFG_F | 1 | 34.38 |
| ❌ | [`bucket_frame_error_cfg_a`](cross/bucket_frame_error_cfg_a.md) | bucket_frame | CFG_A | ERROR | ERROR_CFG_A | 101 | 62.5 |
| ⚠️ | [`bucket_frame_error_cfg_b`](cross/bucket_frame_error_cfg_b.md) | bucket_frame | CFG_B | ERROR | ERROR_CFG_B | 9 | 46.88 |
| ⚠️ | [`bucket_frame_error_cfg_c`](cross/bucket_frame_error_cfg_c.md) | bucket_frame | CFG_C | ERROR | ERROR_CFG_C | 1 | 34.38 |
| ⚠️ | [`bucket_frame_error_cfg_d`](cross/bucket_frame_error_cfg_d.md) | bucket_frame | CFG_D | ERROR | ERROR_CFG_D | 8 | 37.5 |
| ⚠️ | [`bucket_frame_error_cfg_e`](cross/bucket_frame_error_cfg_e.md) | bucket_frame | CFG_E | ERROR | ERROR_CFG_E | 1 | 34.38 |
| ⚠️ | [`bucket_frame_error_cfg_f`](cross/bucket_frame_error_cfg_f.md) | bucket_frame | CFG_F | ERROR | ERROR_CFG_F | 1 | 34.38 |
| ⚠️ | [`bucket_frame_error_cfg_g`](cross/bucket_frame_error_cfg_g.md) | bucket_frame | CFG_G | ERROR | ERROR_CFG_G | 1 | 34.38 |
| ✅ | [`bucket_frame_prof_cfg_a`](cross/bucket_frame_prof_cfg_a.md) | bucket_frame | CFG_A | PROF | PROF_CFG_A | 569 | 65.62 |
| ⚠️ | [`bucket_frame_prof_cfg_b`](cross/bucket_frame_prof_cfg_b.md) | bucket_frame | CFG_B | PROF | PROF_CFG_B | 61 | 40.62 |
| ⚠️ | [`bucket_frame_prof_cfg_c`](cross/bucket_frame_prof_cfg_c.md) | bucket_frame | CFG_C | PROF | PROF_CFG_C | 32 | 37.5 |
| ✅ | [`bucket_frame_prof_cfg_d`](cross/bucket_frame_prof_cfg_d.md) | bucket_frame | CFG_D | PROF | PROF_CFG_D | 18 | 59.38 |
| ⚠️ | [`bucket_frame_prof_cfg_e`](cross/bucket_frame_prof_cfg_e.md) | bucket_frame | CFG_E | PROF | PROF_CFG_E | 64 | 37.5 |
| ⚠️ | [`bucket_frame_prof_cfg_f`](cross/bucket_frame_prof_cfg_f.md) | bucket_frame | CFG_F | PROF | PROF_CFG_F | 1 | 34.38 |
| ⚠️ | [`bucket_frame_prof_cfg_g`](cross/bucket_frame_prof_cfg_g.md) | bucket_frame | CFG_G | PROF | PROF_CFG_G | 16 | 37.5 |
| ✅ | [`cross_good_error_good_cfg_a`](cross/cross_good_error_good_cfg_a.md) | cross | CFG_A | - | GOOD_ERROR_GOOD | 15 | 53.12 |
| ✅ | [`cross_interleave_mix_cfg_a`](cross/cross_interleave_mix_cfg_a.md) | cross | CFG_A | - | INTERLEAVE_MIX | 8 | 50.0 |
| ✅ | [`cross_super_long_counter_soak_cfg_a`](cross/cross_super_long_counter_soak_cfg_a.md) | cross | CFG_A | - | SUPER_LONG_COUNTER_SOAK | 12306 | 56.25 |

## Random long-run cases

<!-- each random case has a txn_growth page; pages are pending until checkpoint UCDBs exist. -->

| status | case_id | bucket | observed_txn | growth_page |
|:---:|---|---|---:|---|
| ❓ | [`P069_mode0_comma_random_positions`](cases/P069_mode0_comma_random_positions.md) | PROF | 50000 | [growth](txn_growth/P069_mode0_comma_random_positions.md) |
| ❓ | [`P074_mode1_comma_random_positions`](cases/P074_mode1_comma_random_positions.md) | PROF | 25000 | [growth](txn_growth/P074_mode1_comma_random_positions.md) |
| ❓ | [`P086_cfgF_addr8_unused_space_sweep`](cases/P086_cfgF_addr8_unused_space_sweep.md) | PROF | 1 | [growth](txn_growth/P086_cfgF_addr8_unused_space_sweep.md) |
| ❓ | [`P118_seed01_clean_mix_100k_cycles`](cases/P118_seed01_clean_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P118_seed01_clean_mix_100k_cycles.md) |
| ❓ | [`P119_seed02_clean_mix_100k_cycles`](cases/P119_seed02_clean_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P119_seed02_clean_mix_100k_cycles.md) |
| ❓ | [`P120_seed03_error_mix_100k_cycles`](cases/P120_seed03_error_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P120_seed03_error_mix_100k_cycles.md) |
| ❓ | [`P121_seed04_stop_mix_100k_cycles`](cases/P121_seed04_stop_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P121_seed04_stop_mix_100k_cycles.md) |
| ❓ | [`P122_seed05_mask_mix_100k_cycles`](cases/P122_seed05_mask_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P122_seed05_mask_mix_100k_cycles.md) |
| ❓ | [`P123_seed06_mode_mix_100k_cycles`](cases/P123_seed06_mode_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P123_seed06_mode_mix_100k_cycles.md) |
| ❓ | [`P124_seed07_channel_mix_100k_cycles`](cases/P124_seed07_channel_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P124_seed07_channel_mix_100k_cycles.md) |
| ❓ | [`P125_seed08_badcrc_mix_100k_cycles`](cases/P125_seed08_badcrc_mix_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P125_seed08_badcrc_mix_100k_cycles.md) |
| ❓ | [`P126_seed09_comma_mix_mode0_100k_cycles`](cases/P126_seed09_comma_mix_mode0_100k_cycles.md) | PROF | 128 | [growth](txn_growth/P126_seed09_comma_mix_mode0_100k_cycles.md) |
| ❓ | [`P127_seed10_comma_mix_mode1_100k_cycles`](cases/P127_seed10_comma_mix_mode1_100k_cycles.md) | PROF | 64 | [growth](txn_growth/P127_seed10_comma_mix_mode1_100k_cycles.md) |
| ❓ | [`P130_seed13_upgrade_gap_observation_bundle`](cases/P130_seed13_upgrade_gap_observation_bundle.md) | PROF | 64 | [growth](txn_growth/P130_seed13_upgrade_gap_observation_bundle.md) |
| ❓ | [`X018_ctrl_random_illegal_word_seed1`](cases/X018_ctrl_random_illegal_word_seed1.md) | ERROR | 64 | [growth](txn_growth/X018_ctrl_random_illegal_word_seed1.md) |
| ❓ | [`X019_ctrl_random_illegal_word_seed2`](cases/X019_ctrl_random_illegal_word_seed2.md) | ERROR | 64 | [growth](txn_growth/X019_ctrl_random_illegal_word_seed2.md) |
| ❓ | [`X025_ctrl_valid_low_payload_noise`](cases/X025_ctrl_valid_low_payload_noise.md) | ERROR | 1 | [growth](txn_growth/X025_ctrl_valid_low_payload_noise.md) |
| ❓ | [`X083_mode0_abort_then_postabort_garbage`](cases/X083_mode0_abort_then_postabort_garbage.md) | ERROR | 1 | [growth](txn_growth/X083_mode0_abort_then_postabort_garbage.md) |

## Totals

<!-- merged_total_code_coverage is the merge across all evidenced cases in all buckets. -->

- planned_cases = `520`
- evidenced_cases = `520`
- excluded_cases = `0`
- merged total code coverage: `stmt=97.67, branch=92.76, cond=93.10, expr=98.77, fsm_state=100.00, fsm_trans=93.33, toggle=63.56`
- functional coverage: `100.0% (520/520)`

---
_[Dashboard](../DV_REPORT.md) &middot; [Coverage](../DV_COV.md)_
