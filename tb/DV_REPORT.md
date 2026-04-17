# ✅ DV Report — mutrig_frame_deassembly

**DUT:** `frame_rcv_ip` &nbsp; **Date:** `2026-04-18` &nbsp;
**RTL variant:** `after` &nbsp; **Seed:** `1`

This page is the chief-architect dashboard. All per-case evidence lives under [`REPORT/`](REPORT/README.md).

## Legend

✅ pass / closed &middot; ⚠️ partial / below target &middot; ❌ failed / missing evidence &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | failed_cases | `0` |
| ✅ | signoff_runs_with_failures | `0` |
| ✅ | unimplemented_cases | `0` |
| ✅ | stale_artifacts | `0` |

## Bucket summary

<!-- status: overall per-bucket health; merged columns: bucket-local ordered-merge percentages. -->

| status | bucket | planned | evidenced | merged (stmt/branch/cond/expr/fsm_state/fsm_trans/toggle) | functional |
|:---:|---|---:|---:|---|---|
| ⚠️ | [`BASIC`](REPORT/buckets/BASIC.md) | 130 | 130 | stmt=97.67, branch=92.76, cond=79.31, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=39.73 | 100.0% (130/130) |
| ⚠️ | [`EDGE`](REPORT/buckets/EDGE.md) | 130 | 130 | stmt=96.50, branch=90.79, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=66.67, toggle=53.89 | 100.0% (130/130) |
| ⚠️ | [`PROF`](REPORT/buckets/PROF.md) | 130 | 130 | stmt=95.33, branch=89.47, cond=86.21, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=62.03 | 100.0% (130/130) |
| ⚠️ | [`ERROR`](REPORT/buckets/ERROR.md) | 130 | 130 | stmt=96.11, branch=90.13, cond=89.66, expr=98.77, fsm_state=100.00, fsm_trans=80.00, toggle=55.07 | 100.0% (130/130) |

## Totals

| status | metric | pct | target |
|:---:|---|---|---|
| ✅ | stmt | 97.67 | 95.0 |
| ✅ | branch | 92.76 | 90.0 |
| ℹ️ | cond | 93.10 | - |
| ℹ️ | expr | 98.77 | - |
| ✅ | fsm_state | 100.00 | 95.0 |
| ✅ | fsm_trans | 93.33 | 90.0 |
| ⚠️ | toggle | 63.56 | 80.0 |

- functional coverage: `100.0% (520/520)`

## Cross / continuous-frame signoff

<!-- one row per run; follow the run_id link for the full transaction-growth curve. -->

| status | run_id | kind | build | seq | txns | cross_pct | ref |
|:---:|---|---|---|---|---:|---:|---|
| ✅ | [`all_buckets_frame_cfg_a`](REPORT/cross/all_buckets_frame_cfg_a.md) | all_buckets_frame | CFG_A | ALL_CFG_A | 738 | 68.75 | [FRCV-2026-04-17-003](BUG_HISTORY.md) |
| ⚠️ | [`all_buckets_frame_cfg_b`](REPORT/cross/all_buckets_frame_cfg_b.md) | all_buckets_frame | CFG_B | ALL_CFG_B | 76 | 40.62 | - |
| ⚠️ | [`all_buckets_frame_cfg_c`](REPORT/cross/all_buckets_frame_cfg_c.md) | all_buckets_frame | CFG_C | ALL_CFG_C | 36 | 40.62 | - |
| ✅ | [`all_buckets_frame_cfg_d`](REPORT/cross/all_buckets_frame_cfg_d.md) | all_buckets_frame | CFG_D | ALL_CFG_D | 27 | 59.38 | - |
| ⚠️ | [`all_buckets_frame_cfg_e`](REPORT/cross/all_buckets_frame_cfg_e.md) | all_buckets_frame | CFG_E | ALL_CFG_E | 66 | 40.62 | - |
| ⚠️ | [`all_buckets_frame_cfg_f`](REPORT/cross/all_buckets_frame_cfg_f.md) | all_buckets_frame | CFG_F | ALL_CFG_F | 3 | 37.5 | - |
| ⚠️ | [`all_buckets_frame_cfg_g`](REPORT/cross/all_buckets_frame_cfg_g.md) | all_buckets_frame | CFG_G | ALL_CFG_G | 17 | 37.5 | - |
| ✅ | [`bucket_frame_basic_cfg_a`](REPORT/cross/bucket_frame_basic_cfg_a.md) | bucket_frame | CFG_A | BASIC_CFG_A | 84 | 59.38 | [FRCV-2026-04-17-003](BUG_HISTORY.md) |
| ⚠️ | [`bucket_frame_basic_cfg_b`](REPORT/cross/bucket_frame_basic_cfg_b.md) | bucket_frame | CFG_B | BASIC_CFG_B | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_edge_cfg_a`](REPORT/cross/bucket_frame_edge_cfg_a.md) | bucket_frame | CFG_A | EDGE_CFG_A | 24 | 40.62 | [FRCV-2026-04-17-003](BUG_HISTORY.md) |
| ⚠️ | [`bucket_frame_edge_cfg_b`](REPORT/cross/bucket_frame_edge_cfg_b.md) | bucket_frame | CFG_B | EDGE_CFG_B | 5 | 37.5 | - |
| ⚠️ | [`bucket_frame_edge_cfg_c`](REPORT/cross/bucket_frame_edge_cfg_c.md) | bucket_frame | CFG_C | EDGE_CFG_C | 3 | 37.5 | - |
| ⚠️ | [`bucket_frame_edge_cfg_d`](REPORT/cross/bucket_frame_edge_cfg_d.md) | bucket_frame | CFG_D | EDGE_CFG_D | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_edge_cfg_e`](REPORT/cross/bucket_frame_edge_cfg_e.md) | bucket_frame | CFG_E | EDGE_CFG_E | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_edge_cfg_f`](REPORT/cross/bucket_frame_edge_cfg_f.md) | bucket_frame | CFG_F | EDGE_CFG_F | 1 | 34.38 | - |
| ✅ | [`bucket_frame_error_cfg_a`](REPORT/cross/bucket_frame_error_cfg_a.md) | bucket_frame | CFG_A | ERROR_CFG_A | 76 | 78.12 | [FRCV-2026-04-17-003](BUG_HISTORY.md) |
| ⚠️ | [`bucket_frame_error_cfg_b`](REPORT/cross/bucket_frame_error_cfg_b.md) | bucket_frame | CFG_B | ERROR_CFG_B | 9 | 46.88 | - |
| ⚠️ | [`bucket_frame_error_cfg_c`](REPORT/cross/bucket_frame_error_cfg_c.md) | bucket_frame | CFG_C | ERROR_CFG_C | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_error_cfg_d`](REPORT/cross/bucket_frame_error_cfg_d.md) | bucket_frame | CFG_D | ERROR_CFG_D | 8 | 37.5 | - |
| ⚠️ | [`bucket_frame_error_cfg_e`](REPORT/cross/bucket_frame_error_cfg_e.md) | bucket_frame | CFG_E | ERROR_CFG_E | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_error_cfg_f`](REPORT/cross/bucket_frame_error_cfg_f.md) | bucket_frame | CFG_F | ERROR_CFG_F | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_error_cfg_g`](REPORT/cross/bucket_frame_error_cfg_g.md) | bucket_frame | CFG_G | ERROR_CFG_G | 1 | 34.38 | - |
| ✅ | [`bucket_frame_prof_cfg_a`](REPORT/cross/bucket_frame_prof_cfg_a.md) | bucket_frame | CFG_A | PROF_CFG_A | 569 | 65.62 | - |
| ⚠️ | [`bucket_frame_prof_cfg_b`](REPORT/cross/bucket_frame_prof_cfg_b.md) | bucket_frame | CFG_B | PROF_CFG_B | 61 | 40.62 | - |
| ⚠️ | [`bucket_frame_prof_cfg_c`](REPORT/cross/bucket_frame_prof_cfg_c.md) | bucket_frame | CFG_C | PROF_CFG_C | 32 | 37.5 | - |
| ✅ | [`bucket_frame_prof_cfg_d`](REPORT/cross/bucket_frame_prof_cfg_d.md) | bucket_frame | CFG_D | PROF_CFG_D | 18 | 59.38 | - |
| ⚠️ | [`bucket_frame_prof_cfg_e`](REPORT/cross/bucket_frame_prof_cfg_e.md) | bucket_frame | CFG_E | PROF_CFG_E | 64 | 37.5 | - |
| ⚠️ | [`bucket_frame_prof_cfg_f`](REPORT/cross/bucket_frame_prof_cfg_f.md) | bucket_frame | CFG_F | PROF_CFG_F | 1 | 34.38 | - |
| ⚠️ | [`bucket_frame_prof_cfg_g`](REPORT/cross/bucket_frame_prof_cfg_g.md) | bucket_frame | CFG_G | PROF_CFG_G | 16 | 37.5 | - |
| ✅ | [`cross_good_error_good_cfg_a`](REPORT/cross/cross_good_error_good_cfg_a.md) | cross | CFG_A | GOOD_ERROR_GOOD | 15 | 53.12 | - |
| ✅ | [`cross_interleave_mix_cfg_a`](REPORT/cross/cross_interleave_mix_cfg_a.md) | cross | CFG_A | INTERLEAVE_MIX | 8 | 50.0 | - |
| ✅ | [`cross_super_long_counter_soak_cfg_a`](REPORT/cross/cross_super_long_counter_soak_cfg_a.md) | cross | CFG_A | SUPER_LONG_COUNTER_SOAK | 12306 | 56.25 | - |
## Index

- [`REPORT/README.md`](REPORT/README.md) — reviewer entry point
- [`REPORT/buckets/`](REPORT/buckets/) — ordered-merge trace per bucket
- [`REPORT/cases/`](REPORT/cases/) — one page per case
- [`REPORT/cross/`](REPORT/cross/) — one page per continuous-frame run
- [`REPORT/txn_growth/`](REPORT/txn_growth/) — checkpoint UCDB curves for random cases
- [`DV_COV.md`](DV_COV.md) — coverage targets vs. merged totals (summary)
- [`DV_REPORT.json`](DV_REPORT.json) — machine-readable source of truth

_This dashboard is generated by `~/.codex/skills/dv-workflow/scripts/dv_report_gen.py`. Edits are overwritten; fix the JSON or the generator instead._
