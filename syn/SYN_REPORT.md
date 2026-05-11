# SYN Report - mutrig_frame_deassembly

**Revision:** `mutrig_frame_deassembly_syn_cfg_a`
**Date:** `2026-05-11`
**Device:** `5AGXBA7D4F31C5`
**Build basis:** `26.1.0.0506`, validated source commit `d61762a`

This file records the measured standalone Quartus signoff result for the current `rtl/` and `script/` layout.

## Build

- project: `mutrig_frame_deassembly_syn`
- revision: `mutrig_frame_deassembly_syn_cfg_a`
- nominal datapath clock assumption: `125 MHz`
- tightened standalone signoff clock: `137.5 MHz` (`7.273 ns`)
- fit mode: Quartus Standard Fit, no seed scan
- compile command: `quartus_sh --flow compile mutrig_frame_deassembly_syn -c mutrig_frame_deassembly_syn_cfg_a`
- compile timestamp: `Mon May 11 10:51:58 2026`
- report-timing command: `quartus_sta --do_report_timing --multicorner=on mutrig_frame_deassembly_syn -c mutrig_frame_deassembly_syn_cfg_a`
- report-timing timestamp: `Mon May 11 10:55:09 2026`
- harness constraint notes:
  - `reset_n` is a false path into the standalone harness
  - `probe_out[31:0]` is a false-path debug signature output and is intentionally excluded from timing closure

## Timing Result

- setup: PASS, worst selected slack `+2.530 ns` on `clk125`
- hold: PASS, worst selected slack `+0.174 ns` on `clk125`
- pulse width: PASS, worst selected slack `+2.766 ns`
- constraint status: fully constrained for setup and hold

Corner highlights from TimeQuest `--multicorner=on`:

| corner | setup slack | hold slack |
|---|---:|---:|
| Slow 1100mV 85C | `+2.530 ns` | `+0.326 ns` |
| Slow 1100mV 0C | `+2.616 ns` | `+0.312 ns` |
| Fast 1100mV 85C | `+4.435 ns` | `+0.190 ns` |
| Fast 1100mV 0C | `+4.687 ns` | `+0.174 ns` |

The requested FEB-style `900mV` / `100C` labels are not the available operating model names for this Arria V standalone project in Quartus 18.1. The pass/fail result uses all four TimeQuest operating conditions reported by the project.

Worst setup path at the limiting Slow 1100mV 85C model:

- from: `mutrig_frame_deassembly_syn_harness:u_harness|asi_rx8b1k_data[2]`
- to: `mutrig_frame_deassembly_syn_harness:u_harness|frame_rcv_ip:u_dut|aso_hit_type0_error[1]`
- data arrival: `8.483 ns`
- data required: `11.013 ns`
- slack: `+2.530 ns`

## Resource Result

- Logic (ALMs): `226 / 91,680` (`< 1%`)
- Registers: `290`
- Block memory bits: `0 / 13,987,840` (`0%`)
- RAM blocks: `0 / 1,366` (`0%`)
- DSP blocks: `0 / 800` (`0%`)
- PLLs: `0 / 21` (`0%`)
- Pins: `34 / 426` (`8%`)

## Static Screen

- canonical standalone RTL: PASS via `tb/static/mutrig_frame_deassembly_static.f`
- mixed SV shell plus VHDL DUT: PASS via `tb/static/mutrig_frame_deassembly_sv_shell_static.f`
- lint: `Error (0)`
- CDC: `Violations (0)`
- RDC: `Violation (0)`

## Warnings

- Quartus reports 15 non-fatal warnings in the full compile.
- The timing run itself is clean after the harness constraint model; the remaining warnings are expected standalone harness and unconstrained-board-pin warnings.
- Questa static runs report warnings for inferred clock/reset/domain setup, but the hard signoff counts are zero: lint `Error (0)`, CDC `Violations (0)`, RDC `Violation (0)`.

## Artifacts

- [`quartus/mutrig_frame_deassembly_syn.qpf`](quartus/mutrig_frame_deassembly_syn.qpf)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a.qsf`](quartus/mutrig_frame_deassembly_syn_cfg_a.qsf)
- [`quartus/mutrig_frame_deassembly_syn.sdc`](quartus/mutrig_frame_deassembly_syn.sdc)
- [`quartus/mutrig_frame_deassembly_syn_harness.vhd`](quartus/mutrig_frame_deassembly_syn_harness.vhd)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd`](quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd)
- [`quartus/run_signoff.sh`](quartus/run_signoff.sh)
- [`quartus/signoff_20260511T085158Z.log`](quartus/signoff_20260511T085158Z.log)
- [`quartus/sta_report_timing_20260511T085509Z.log`](quartus/sta_report_timing_20260511T085509Z.log)
- [`quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.rpt`](quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.rpt)
- [`quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.summary`](quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.summary)
- [`quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.fit.summary`](quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.fit.summary)
- [`../tb/static/mutrig_frame_deassembly_static.f`](../tb/static/mutrig_frame_deassembly_static.f)
- [`../tb/static/mutrig_frame_deassembly_sv_shell_static.f`](../tb/static/mutrig_frame_deassembly_sv_shell_static.f)
