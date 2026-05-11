# Signoff - mutrig_frame_deassembly

**DUT:** `frame_rcv_ip`
**IP version:** `26.1.0.0506`
**Refresh date:** `2026-05-11`
**Validated source commit before this documentation refresh:** `d61762a`

## Health

| status | field | value |
|---|---|---|
| PASS | overall_signoff | Latest committed IP passes standalone Quartus STA in isolation at the 1.1x target and has clean static lint/CDC/RDC screens. |
| PASS | standalone_sta | All four TimeQuest multicorner setup slacks are non-negative at `137.5 MHz` / `7.273 ns`. |
| PASS | static_screen | Questa Static screen reports lint `Error (0)`, CDC `Violations (0)`, and RDC `Violation (0)`. |
| PASS | dv_closure | Current DV dashboard remains the active functional evidence baseline: [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md). |

## Standalone Target

| item | value |
|---|---|
| project | `syn/quartus/mutrig_frame_deassembly_syn.qpf` |
| revision | `mutrig_frame_deassembly_syn_cfg_a` |
| top | `mutrig_frame_deassembly_syn_cfg_a_top` |
| device | `5AGXBA7D4F31C5` |
| nominal target | `125 MHz` |
| standalone signoff target | `137.5 MHz` (`1.1 x 125 MHz`) |
| SDC period | `7.273 ns` on `clk125` |
| fitter mode | Quartus Standard Fit, no seed scan |

## Quartus Evidence

| artifact | value |
|---|---|
| compile command | `quartus_sh --flow compile mutrig_frame_deassembly_syn -c mutrig_frame_deassembly_syn_cfg_a` |
| compile log | `syn/quartus/signoff_20260511T085158Z.log` |
| compile timestamp | `Mon May 11 10:51:58 2026` |
| flow report | `syn/quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.flow.rpt` |
| fitter summary | `syn/quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.fit.summary` |
| STA summary | `syn/quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.summary` |
| STA report | `syn/quartus/output_files/mutrig_frame_deassembly_syn_cfg_a/mutrig_frame_deassembly_syn_cfg_a.sta.rpt` |
| report-timing command | `quartus_sta --do_report_timing --multicorner=on mutrig_frame_deassembly_syn -c mutrig_frame_deassembly_syn_cfg_a` |
| report-timing log | `syn/quartus/sta_report_timing_20260511T085509Z.log` |
| report-timing timestamp | `Mon May 11 10:55:09 2026` |
| compile result | Quartus full compilation successful, `0 errors`, `15 warnings` |
| STA result | Timing Analyzer successful, `0 errors`, `0 warnings`; setup and hold fully constrained |

## Four-Corner Setup Slack

Quartus 18.1 for `5AGXBA7D4F31C5` exposes the following four multicorner operating models in this standalone project. The requested FEB-style `900mV` / `100C` labels are not the available Arria V model names here; the signoff judgment below uses every TimeQuest operating condition reported by `--multicorner=on`.

| corner | setup slack | endpoint TNS | worst setup path | data arrival | data required |
|---|---:|---:|---|---:|---:|
| Slow 1100mV 85C | `+2.530 ns` | `0.000 ns` | `asi_rx8b1k_data[2]` to `frame_rcv_ip:u_dut|aso_hit_type0_error[1]` | `8.483 ns` | `11.013 ns` |
| Slow 1100mV 0C | `+2.616 ns` | `0.000 ns` | `asi_rx8b1k_data[2]` to `frame_rcv_ip:u_dut|aso_hit_type0_error[1]` | `8.369 ns` | `10.985 ns` |
| Fast 1100mV 85C | `+4.435 ns` | `0.000 ns` | `frame_rcv_ip:u_dut|p_state_wait_cnt[1]` to `frame_rcv_ip:u_dut|s_o_word[10]` | `4.888 ns` | `9.323 ns` |
| Fast 1100mV 0C | `+4.687 ns` | `0.000 ns` | `frame_rcv_ip:u_dut|p_state_wait_cnt[1]` to `frame_rcv_ip:u_dut|s_o_word[10]` | `4.569 ns` | `9.256 ns` |

Worst-overall setup slack is `+2.530 ns` on `clk125` at the Slow 1100mV 85C model. Worst selected hold slack is `+0.174 ns` at the Fast 1100mV 0C model.

## Resource Evidence

| resource | value |
|---|---:|
| ALMs | `226 / 91,680` (`< 1%`) |
| registers | `290` |
| pins | `34 / 426` (`8%`) |
| block memory bits | `0 / 13,987,840` (`0%`) |
| RAM blocks | `0 / 1,366` (`0%`) |
| DSP blocks | `0 / 800` (`0%`) |
| PLLs | `0 / 21` (`0%`) |

## Static Screen

| context | command | log | result |
|---|---|---|---|
| canonical standalone RTL | `python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py --top frame_rcv_ip --filelist tb/static/mutrig_frame_deassembly_static.f --modes lint,cdc,rdc --work-dir tb/static/questa_static_20260511_frcv_static rtl/frame_rcv_ip.vhd rtl/crc16_calc.vhd` | `tb/static/questa_static_20260511_frcv_static/questa_static_screen.log` | lint `Error (0)`, CDC `Violations (0)`, RDC `Violation (0)` |
| SV shell plus VHDL DUT | `python3 ~/.codex/skills/rtl-linter-and-checker/scripts/questa_static_screen.py --top frame_rcv_ip_dut_sv --filelist tb/static/mutrig_frame_deassembly_sv_shell_static.f --modes lint,cdc,rdc --work-dir tb/static/questa_static_20260511_sv_shell rtl/crc16_calc.vhd rtl/frame_rcv_ip.vhd rtl/sv_ver/frame_rcv_ip/frame_rcv_ip_dut_sv.sv` | `tb/static/questa_static_20260511_sv_shell/questa_static_screen.log` | lint `Error (0)`, CDC `Violations (0)`, RDC `Violation (0)` |

`rtl/ordered_packet_scheduler.vhd` contains only a legacy design note/comment block and no HDL design unit, so it is not part of the standalone Quartus or qverify elaboration context.

## Verdict

PASS. The latest committed `mutrig_frame_deassembly` IP meets standalone timing in isolation at the `137.5 MHz` signoff target. No older VERSION was used, no VERSION downgrade was made, and no standalone timing regression entry is required in `tb/BUG_HISTORY.md`.
