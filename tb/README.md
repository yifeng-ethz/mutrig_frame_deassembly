# mutrig_frame_deassembly DV

## Entry Points

- dashboard: [`DV_REPORT.md`](DV_REPORT.md)
- coverage summary: [`DV_COV.md`](DV_COV.md)
- evidence tree: [`REPORT/README.md`](REPORT/README.md)
- bug ledger: [`BUG_HISTORY.md`](BUG_HISTORY.md)

## Common Commands

```bash
make -C tb/uvm run_case CASE_ID=B041
python3 tb/scripts/run_isolated_cases.py --skip-existing
python3 tb/scripts/run_full_random_parallel.py --parallel 4 --duration-min 20
python3 tb/scripts/generate_dv_report.py
```

## Notes

- `run_full_random_parallel.py` is the supplemental long-run stress driver for randomized all-bucket continuous-frame sequences.
- Parallel workers use unique `WORK_SUFFIX` values so the UVM compile/work directories do not collide.
- The generated report is derived from `DV_REPORT.json`; regenerate the tree instead of hand-editing `REPORT/`.
