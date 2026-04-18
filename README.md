# MuTRiG Frame Deassembly Mu3E IP

MuTRiG frame receiver / deassembly IP that converts the `rx8b1k` byte stream into
`hit_type0` packets, `headerinfo` metadata pulses, and CSR-visible frame / CRC counters.

**Version:** `26.0.6.0418`
**Module name:** `mutrig_frame_deassembly`
**Platform Designer group:** `Mu3e Data Plane / Modules`

## Layout

- [`rtl/`](rtl/) contains the live VHDL sources.
- [`script/`](script/) contains the Platform Designer packaging and CMSIS-SVD artifacts.
- [`tb/`](tb/README.md) contains the direct TB, UVM harness, DV plans, generated report, and helper scripts.
- [`syn/`](syn/SYN_REPORT.md) contains the standalone Quartus signoff scaffold.
- [`doc/`](doc/SIGNOFF.md) contains the signoff dashboard and the timing/DV closure notes.

Root-level legacy filenames are kept as compatibility symlinks. New edits should target the
canonical `rtl/` and `script/` trees.

## Current Signoff State

- DV dashboard: [`tb/DV_REPORT.md`](tb/DV_REPORT.md)
- DV evidence tree: [`tb/REPORT/README.md`](tb/REPORT/README.md)
- Open harness issue: continuous-frame `FRCV_SCB_CASE_END` warning / pending-queue cleanup (`FRCV-2026-04-18-005`)
- Standalone synthesis scaffold: [`syn/quartus/`](syn/quartus/)

## Long-Run Workflow

The deterministic seed anchors remain documented in [`tb/DV_PROF.md`](tb/DV_PROF.md), but
long-run closure also requires randomized all-bucket continuous-frame sequences:

- report regeneration: `python3 tb/scripts/generate_dv_report.py`
- isolated documented cases: `python3 tb/scripts/run_isolated_cases.py`
- full-random all-bucket soak: `python3 tb/scripts/run_full_random_parallel.py --parallel 4 --duration-min 20`
