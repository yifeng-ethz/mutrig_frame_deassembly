# ⏳ SYN Report — mutrig_frame_deassembly

**Revision:** `mutrig_frame_deassembly_syn_cfg_a` &nbsp; **Date:** `2026-04-17` &nbsp;
**Device:** `5AGXBA7D4F31C5` &nbsp; **Build basis:** `26.0.6.0417 layout refresh`

This file is the standalone synthesis signoff placeholder for the active layout/metadata
refresh. The Quartus project scaffold exists under [`quartus/`](quartus/), but the compile
has not been executed yet after moving the live sources into `rtl/` and the packaging into
`script/`.

## Intended Build

- project: `mutrig_frame_deassembly_syn`
- revision: `mutrig_frame_deassembly_syn_cfg_a`
- nominal datapath clock assumption: `125 MHz`
- tightened standalone signoff clock: `137.5 MHz` (`7.273 ns`)
- fit mode: Quartus Standard Fit, no seed scan

## Artifacts

- [`quartus/mutrig_frame_deassembly_syn.qpf`](quartus/mutrig_frame_deassembly_syn.qpf)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a.qsf`](quartus/mutrig_frame_deassembly_syn_cfg_a.qsf)
- [`quartus/mutrig_frame_deassembly_syn.sdc`](quartus/mutrig_frame_deassembly_syn.sdc)
- [`quartus/mutrig_frame_deassembly_syn_harness.vhd`](quartus/mutrig_frame_deassembly_syn_harness.vhd)
- [`quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd`](quartus/mutrig_frame_deassembly_syn_cfg_a_top.vhd)
- [`quartus/run_signoff.sh`](quartus/run_signoff.sh)

## Pending

- run the standalone compile
- capture WNS / hold slack / Fmax
- capture logic / register / memory usage
- update [`../doc/rtl_note.md`](../doc/rtl_note.md) with actual results
