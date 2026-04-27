# RTL Note

## Scope

This note tracks the standalone signoff story for `mutrig_frame_deassembly` after the repo
cleanup that moved live sources into `rtl/` and packaging into `script/`, plus the
QuestaOne harness-refresh closure on 2026-04-21.

## Pre-Fit Model

- expected critical logic: parser control, byte-unpack path, CRC-check handoff, and run-control gating
- expected storage mapping: small register-heavy control, plus the CRC helper logic from `crc16_calc.vhd`
- expected signoff clock: `125 MHz` nominal datapath clock, tightened to `137.5 MHz` for standalone signoff

## Constraint Model

- `clk125` is constrained at `7.273 ns`
- `reset_n` is marked false-path into the harness
- `probe_out[31:0]` is a debug-only signature output from the harness and is marked false-path so TimeQuest closes the DUT logic rather than an artificial board-level requirement on the signature pin

## Measured Result

- standalone compile: PASS
- timing status: PASS at the tightened standalone target, with the design fully constrained for setup and hold
- worst selected setup slack: `2.717 ns`
- worst selected hold slack: `0.159 ns`
- resource footprint: `225` ALMs, `296` registers, `0` RAM blocks, `0` DSP blocks

## Pre-Fit vs Post-Fit

- The result matches the pre-fit expectation that this IP is register/control heavy rather than RAM heavy.
- No unexpected RAM or DSP inference appeared in the standalone build.
- The measured resource split stayed in simple ALM/FF logic around the parser/run-control/CRC structure, which is consistent with the RTL.

## DV Correlation

- Current DV evidence lives under [`../tb/`](../tb/).
- The latest QuestaOne failures were explained by harness expectation / retirement mismatches in the documented-case engine, not by a new RTL defect.
- Canonical clean reruns:
  - [`../tb/uvm/logs/bucket_frame_prof_cfg_a_after_s1.log`](../tb/uvm/logs/bucket_frame_prof_cfg_a_after_s1.log)
  - [`../tb/uvm/logs/all_buckets_frame_cfg_a_fix2_after_s1.log`](../tb/uvm/logs/all_buckets_frame_cfg_a_fix2_after_s1.log)

## Reproduction

- synthesis: `cd syn/quartus && ./run_signoff.sh`
- DV dashboard refresh from the repo root: `python3 scripts/dv_case_report.py --ip-root mutrig_frame_deassembly --dut-name frame_rcv_ip --report-title mutrig_frame_deassembly --seed 1 --rtl-variant after --vcover /data1/questaone_sim-2026.1_1/questasim/bin/vcover && python3 mutrig_frame_deassembly/tb/scripts/generate_dv_report.py`
