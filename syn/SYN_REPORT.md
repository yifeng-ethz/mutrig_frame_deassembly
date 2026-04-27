# ✅ SYN Report — mutrig_frame_deassembly

**Revision:** `mutrig_frame_deassembly_syn_cfg_a` &nbsp; **Date:** `2026-04-21` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Build basis:** `26.0.6.0418 QuestaOne harness-refresh closure`

This file records the measured standalone Quartus signoff result for the current `rtl/` and
`script/` layout.

## Build

- project: `mutrig_frame_deassembly_syn`
- revision: `mutrig_frame_deassembly_syn_cfg_a`
- nominal datapath clock assumption: `125 MHz`
- tightened standalone signoff clock: `137.5 MHz` (`7.273 ns`)
- fit mode: Quartus Standard Fit, no seed scan
- harness constraint notes:
  - `reset_n` is a false path into the standalone harness
  - `probe_out[31:0]` is a false-path debug signature output and is intentionally excluded from timing closure

## Timing Result

- setup: PASS, worst selected slack `2.717 ns` on `clk125`
- hold: PASS, worst selected slack `0.159 ns` on `clk125`
- pulse width: PASS, worst selected slack `2.766 ns`
- constraint status: fully constrained for setup and hold

Corner highlights from TimeQuest:

- Slow `85C`: setup `2.717 ns`, hold `0.287 ns`
- Slow `0C`: setup `2.796 ns`, hold `0.267 ns`
- Fast `85C`: setup `4.722 ns`, hold `0.174 ns`
- Fast `0C`: setup `4.915 ns`, hold `0.159 ns`

## Resource Result

- Logic (ALMs): `225 / 91,680` (`< 1%`)
- Registers: `296`
- Block memory bits: `0 / 13,987,840` (`0%`)
- RAM blocks: `0 / 1,366` (`0%`)
- DSP blocks: `0 / 800` (`0%`)
- PLLs: `0 / 21` (`0%`)
- Pins: `34 / 426` (`8%`)

## Warnings

- Quartus reports 13 non-fatal warnings in the full compile.
- The timing run itself is clean after the harness constraint fix; the remaining warnings are the expected unused-signal warnings in the standalone harness and internal debug/helper signals.

## Artifacts

- [`quartus/mutrig_frame_deassembly_syn.qpf`](quartus/mutrig_frame_deassembly_syn.qpf)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a.qsf`](quartus/mutrig_frame_deassembly_syn_cfg_a.qsf)
- [`quartus/mutrig_frame_deassembly_syn.sdc`](quartus/mutrig_frame_deassembly_syn.sdc)
- [`quartus/mutrig_frame_deassembly_syn_harness.vhd`](quartus/mutrig_frame_deassembly_syn_harness.vhd)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd`](quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd)
- [`quartus/run_signoff.sh`](quartus/run_signoff.sh)
- [`quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.rpt`](quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.rpt)
- [`quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.fit.summary`](quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.fit.summary)
