# RTL Note

## Scope

This note tracks the standalone signoff story for `mutrig_frame_deassembly` after the repo
cleanup that moved live sources into `rtl/` and packaging into `script/`.

## Pre-Fit Model

- expected critical logic: parser control, byte-unpack path, CRC-check handoff, and run-control gating
- expected storage mapping: small register-heavy control, plus the CRC helper logic from `crc16_calc.vhd`
- expected signoff clock: `125 MHz` nominal datapath clock, tightened to `137.5 MHz` for standalone signoff

## Current State

- DV evidence exists under [`../tb/`](../tb/)
- the standalone Quartus scaffold exists under [`../syn/quartus/`](../syn/quartus/)
- no standalone compile has been executed yet after the layout refresh

Update this note with the actual fitter/TimeQuest evidence once the first compile is available.
