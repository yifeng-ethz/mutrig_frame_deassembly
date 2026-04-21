# DV Coverage Summary — mutrig_frame_deassembly

This page is the coverage summary only. Per-case incremental coverage lives under
[`REPORT/cases/`](REPORT/cases/); per-bucket ordered-merge traces live under
[`REPORT/buckets/`](REPORT/buckets/).

## Targets vs merged totals

<!-- merged_pct = merge across all evidenced isolated-mode UCDBs across all buckets. -->

| status | metric | merged_pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.94 | 95.0 |
| ✅ | branch | 94.00 | 90.0 |
| ℹ️ | cond | 85.00 | - |
| ℹ️ | expr | 98.77 | - |
| ❓ | fsm_state | n/a | 95.0 |
| ❓ | fsm_trans | n/a | 90.0 |
| ⚠️ | toggle | 57.02 | 80.0 |

## Per-bucket merged totals

| status | bucket | stmt | branch | cond | expr | fsm_state | fsm_trans | toggle |
|:---:|---|---|---|---|---|---|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 97.94 | 94.00 | 80.00 | 98.77 | n/a | n/a | 48.60 |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 95.19 | 89.33 | 70.00 | 98.77 | n/a | n/a | 48.37 |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 92.78 | 86.67 | 65.00 | 98.77 | n/a | n/a | 52.29 |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 95.19 | 89.33 | 67.50 | 98.77 | n/a | n/a | 50.48 |

## Continuous-frame baselines by build

<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->

| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns | ref |
|:---:|---|---|---|---|---:|---|---|---|---:|---:|---|
| ✅ | [`all_buckets_frame_cfg_a_fix2`](REPORT/cross/all_buckets_frame_cfg_a_fix2.md) | all_buckets_frame | CFG_A | - | 474 | 96.54 | 91.33 | 59.53 | 68.75 | 775 | - |
| ⚠️ | [`all_buckets_frame_cfg_b`](REPORT/cross/all_buckets_frame_cfg_b.md) | all_buckets_frame | CFG_B | - | 25 | 85.16 | 73.51 | 51.51 | 40.62 | 76 | - |
| ⚠️ | [`all_buckets_frame_cfg_c`](REPORT/cross/all_buckets_frame_cfg_c.md) | all_buckets_frame | CFG_C | - | 6 | 80.16 | 66.23 | 50.58 | 40.62 | 36 | - |
| ✅ | [`all_buckets_frame_cfg_d`](REPORT/cross/all_buckets_frame_cfg_d.md) | all_buckets_frame | CFG_D | - | 5 | 80.16 | 66.23 | 47.48 | 59.38 | 27 | - |
| ⚠️ | [`all_buckets_frame_cfg_e`](REPORT/cross/all_buckets_frame_cfg_e.md) | all_buckets_frame | CFG_E | - | 5 | 80.93 | 67.55 | 51.25 | 40.62 | 66 | - |
| ⚠️ | [`all_buckets_frame_cfg_f`](REPORT/cross/all_buckets_frame_cfg_f.md) | all_buckets_frame | CFG_F | - | 3 | 80.54 | 65.56 | 20.36 | 37.5 | 3 | - |
| ⚠️ | [`all_buckets_frame_cfg_g`](REPORT/cross/all_buckets_frame_cfg_g.md) | all_buckets_frame | CFG_G | - | 2 | 80.16 | 64.24 | 44.79 | 37.5 | 17 | - |
| ⚠️ | [`bucket_frame_basic_cfg_b`](REPORT/cross/bucket_frame_basic_cfg_b.md) | bucket_frame | CFG_B | BASIC | 1 | 77.34 | 62.25 | 16.79 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_edge_cfg_a`](REPORT/cross/bucket_frame_edge_cfg_a.md) | bucket_frame | CFG_A | EDGE | 119 | 91.35 | 81.33 | 48.01 | 40.62 | 24 | - |
| ⚠️ | [`bucket_frame_edge_cfg_b`](REPORT/cross/bucket_frame_edge_cfg_b.md) | bucket_frame | CFG_B | EDGE | 5 | 80.86 | 65.56 | 22.73 | 37.5 | 5 | - |
| ⚠️ | [`bucket_frame_edge_cfg_c`](REPORT/cross/bucket_frame_edge_cfg_c.md) | bucket_frame | CFG_C | EDGE | 3 | 80.16 | 66.23 | 46.21 | 37.5 | 3 | - |
| ⚠️ | [`bucket_frame_edge_cfg_d`](REPORT/cross/bucket_frame_edge_cfg_d.md) | bucket_frame | CFG_D | EDGE | 1 | 79.77 | 63.58 | 16.32 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_edge_cfg_e`](REPORT/cross/bucket_frame_edge_cfg_e.md) | bucket_frame | CFG_E | EDGE | 1 | 79.77 | 63.58 | 16.54 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_edge_cfg_f`](REPORT/cross/bucket_frame_edge_cfg_f.md) | bucket_frame | CFG_F | EDGE | 1 | 80.16 | 64.90 | 17.41 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_error_cfg_b`](REPORT/cross/bucket_frame_error_cfg_b.md) | bucket_frame | CFG_B | ERROR | 9 | 85.16 | 71.52 | 33.32 | 46.88 | 9 | - |
| ⚠️ | [`bucket_frame_error_cfg_c`](REPORT/cross/bucket_frame_error_cfg_c.md) | bucket_frame | CFG_C | ERROR | 1 | 79.77 | 63.58 | 16.16 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_error_cfg_d`](REPORT/cross/bucket_frame_error_cfg_d.md) | bucket_frame | CFG_D | ERROR | 1 | 80.16 | 64.24 | 39.80 | 37.5 | 8 | - |
| ⚠️ | [`bucket_frame_error_cfg_e`](REPORT/cross/bucket_frame_error_cfg_e.md) | bucket_frame | CFG_E | ERROR | 1 | 80.54 | 64.90 | 16.39 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_error_cfg_f`](REPORT/cross/bucket_frame_error_cfg_f.md) | bucket_frame | CFG_F | ERROR | 1 | 80.16 | 64.90 | 17.67 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_error_cfg_g`](REPORT/cross/bucket_frame_error_cfg_g.md) | bucket_frame | CFG_G | ERROR | 1 | 79.77 | 63.58 | 16.79 | 34.38 | 1 | - |
| ✅ | [`bucket_frame_prof_cfg_a`](REPORT/cross/bucket_frame_prof_cfg_a.md) | bucket_frame | CFG_A | PROF | 110 | 93.77 | 86.00 | 57.52 | 65.62 | 568 | - |
| ⚠️ | [`bucket_frame_prof_cfg_b`](REPORT/cross/bucket_frame_prof_cfg_b.md) | bucket_frame | CFG_B | PROF | 10 | 80.86 | 67.55 | 50.68 | 40.62 | 61 | - |
| ⚠️ | [`bucket_frame_prof_cfg_c`](REPORT/cross/bucket_frame_prof_cfg_c.md) | bucket_frame | CFG_C | PROF | 2 | 80.16 | 64.24 | 43.95 | 37.5 | 32 | - |
| ✅ | [`bucket_frame_prof_cfg_d`](REPORT/cross/bucket_frame_prof_cfg_d.md) | bucket_frame | CFG_D | PROF | 3 | 80.16 | 66.23 | 45.98 | 59.38 | 18 | - |
| ⚠️ | [`bucket_frame_prof_cfg_e`](REPORT/cross/bucket_frame_prof_cfg_e.md) | bucket_frame | CFG_E | PROF | 3 | 80.93 | 67.55 | 50.94 | 37.5 | 64 | - |
| ⚠️ | [`bucket_frame_prof_cfg_f`](REPORT/cross/bucket_frame_prof_cfg_f.md) | bucket_frame | CFG_F | PROF | 1 | 80.16 | 64.90 | 16.99 | 34.38 | 1 | - |
| ⚠️ | [`bucket_frame_prof_cfg_g`](REPORT/cross/bucket_frame_prof_cfg_g.md) | bucket_frame | CFG_G | PROF | 1 | 80.16 | 64.24 | 42.39 | 37.5 | 16 | - |
| ✅ | [`cross_good_error_good_cfg_a`](REPORT/cross/cross_good_error_good_cfg_a.md) | cross | CFG_A | - | 0 | 84.82 | 70.86 | 45.46 | 53.12 | 15 | - |
| ✅ | [`cross_interleave_mix_cfg_a`](REPORT/cross/cross_interleave_mix_cfg_a.md) | cross | CFG_A | - | 0 | 89.88 | 82.12 | 47.24 | 50.0 | 8 | - |
| ✅ | [`cross_super_long_counter_soak_cfg_a`](REPORT/cross/cross_super_long_counter_soak_cfg_a.md) | cross | CFG_A | - | 0 | 87.16 | 75.50 | 52.19 | 56.25 | 12306 | - |
_Regenerate with `python3 ~/.codex/skills/dv-workflow/scripts/dv_report_gen.py --tb <tb>`._
