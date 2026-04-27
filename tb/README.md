# mutrig_frame_deassembly DV

## Entry Points

- dashboard: [`DV_REPORT.md`](DV_REPORT.md)
- coverage summary: [`DV_COV.md`](DV_COV.md)
- evidence tree: [`REPORT/README.md`](REPORT/README.md)
- bug ledger: [`BUG_HISTORY.md`](BUG_HISTORY.md)
- formal plan: [`DV_FORMAL.md`](DV_FORMAL.md)

## Common Commands

```bash
make -C tb/uvm run_case CASE_ID=B041
python3 tb/scripts/run_isolated_cases.py --skip-existing
python3 tb/scripts/run_isolated_cases.py --case-list B039_long_zero_hit_frame_headerinfo_pulse,X119_cfgD_channel_width8_random_wide_values
python3 tb/scripts/run_full_random_parallel.py --parallel 4 --duration-min 20
python3 tb/scripts/generate_dv_report.py
make -C tb/formal compile
```

## Notes

- `run_full_random_parallel.py` is the supplemental long-run stress driver for randomized all-bucket continuous-frame sequences.
- `run_isolated_cases.py` now auto-derives `BUILD_TAG` from `*_cfgB_*` ... `*_cfgG_*` case IDs, so cfg-specific isolated reruns no longer need a manual `--build-tag` override unless you want to force a different package build.
- Practical isolated documented PROF reruns intentionally cap the iteration count to a representative sample and insert a one-cycle inter-frame settle. This keeps isolated harness timing aligned with the documented contract after the QuestaOne migration; use the continuous-frame drivers for the long soak baselines.
- Parallel workers use unique `WORK_SUFFIX` values so the UVM compile/work directories do not collide.
- The generated report is derived from `DV_REPORT.json`; regenerate the tree instead of hand-editing `REPORT/`.
