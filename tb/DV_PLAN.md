# DV Plan: mutrig_frame_deassembly

**DUT:** `frame_rcv_ip`  
**IP packaging file:** `script/mutrig_frame_deassembly_hw.tcl`  
**Primary RTL:** `rtl/frame_rcv_ip.vhd`  
**Date:** 2026-04-15  
**Methodology:** Phase-0 DV planning per the local Codex `dv-workflow` and the Claude `dv-workflow` guidance  
**Status:** Planning package for harness implementation signoff

## 1. Scope

This plan covers the standalone MuTRiG frame receiver IP that converts the `rx8b1k` byte stream into:

- `hit_type0` Avalon-ST output packets
- `headerinfo` Avalon-ST metadata pulses
- CSR-visible status and frame/CRC counters
- run-control-managed parser enable behavior on the `ctrl` Avalon-ST sink

In scope:

- actual parser behavior in `proc_frame_rcv_comb`
- actual run-control decode and gating in `proc_run_control_mgmt_agent`
- actual enable masking in `proc_enable_ctrl`
- actual counter/readback behavior in `proc_avmm_slave_csr`
- actual output formatting in `proc_output_pkt_hits*` and `proc_output_header_info*`
- the termination mismatch called out in `RUN_SEQ_UPGRADE_PLAN.md`
- Qsys-visible contract from `script/mutrig_frame_deassembly_hw.tcl`

Out of scope:

- downstream `mts_processor` behavior beyond the receiver-facing contract
- board-level LVDS analog behavior
- changes to the RTL itself
- packaging refactors beyond using the current `_hw.tcl` as the verification contract source

## 2. DUT Summary

### 2.1 Interface contract

| Interface | Type | Direction | Notes |
|---|---|---|---|
| `rx8b1k` | Avalon-ST | sink | `data[8:0]`, error bits `loss_sync_pattern parity_error decode_error`, channel width from `CHANNEL_WIDTH` |
| `hit_type0` | Avalon-ST | source | 45-bit packed hit payload with `sop/eop/error/valid/channel` |
| `headerinfo` | Avalon-ST | source | 42-bit frame metadata pulse |
| `csr` | Avalon-MM | slave | 32-bit data, address width from `CSR_ADDR_WIDTH`, `readLatency=1` in `_hw.tcl` |
| `ctrl` | Avalon-ST | sink | 9-bit one-hot run-control command |
| `clock_sink` / `reset_sink` | clock/reset | sink | single parser clock domain |

### 2.2 Key architectural facts from RTL

1. `FS_IDLE` only opens on `enable='1' and i_byteisk='1' and i_data=c_header`.
2. `enable` is registered and depends on:
   - `receiver_force_go='1'` in `IDLE`
   - `receiver_go='1'` plus `csr.control(0)='1'` in `RUNNING`
3. `RUN_PREPARE` synchronously resets parser state and the frame/CRC counters.
4. `TERMINATING` immediately drops `receiver_go` to zero, so no fresh header can open once the parser returns to `FS_IDLE`.
5. `TERMINATING` does not abort an already-open frame because `enable` is only consumed in `FS_IDLE`.
6. `asi_ctrl_ready` is driven high unconditionally in the current RTL.
7. `asi_rx8b1k_valid` is not used by the parser state machine; verification must reflect that real contract.
8. `MODE_HALT` changes the response to an in-frame `K28.5` comma (`x"BC"` with `i_byteisk='1'`):
   - `MODE_HALT=0`: return to `FS_IDLE`
   - `MODE_HALT=1`: hold state and continue
9. `headerinfo_valid` is a one-cycle pulse derived from the rising edge of `n_frame_info_ready`.

### 2.3 Run-sequence contract from `RUN_SEQ_UPGRADE_PLAN.md`

The current receiver is one leg of the termination mismatch:

- the emulator fix prevents fresh post-terminate frame starts
- `frame_rcv_ip` still closes fresh header detection immediately in `TERMINATING`
- an already-open frame may still drain through to `hit_type0`
- the current RTL does **not** synthesize an explicit terminal marker
- the current RTL does **not** delay `asi_ctrl_ready` until drain completion

The DV plan therefore has two obligations:

1. prove the present RTL contract accurately
2. expose the exact gaps against the upgrade plan so later RTL work has a precise baseline

## 3. Verification Targets

| ID | Target | RTL / Packaging anchor | Main observability |
|---|---|---|---|
| F01 | Run-control decode | `proc_run_control_mgmt_agent` | `run_state_cmd`, `receiver_go`, `receiver_force_go`, `asi_ctrl_ready` |
| F02 | Parser enable gating | `proc_enable_ctrl` | `enable`, `FS_IDLE` start acceptance |
| F03 | Long frame parsing | `FS_FRAME_COUNTER`..`FS_CRC_CHECK` | `hit_type0`, `headerinfo`, counters |
| F04 | Short frame parsing | `FS_UNPACK` / `FS_UNPACK_EXTRA` | `hit_type0`, `headerinfo`, counters |
| F05 | Output packing | `proc_output_pkt_hits*` | payload field mapping and `error[2:0]` |
| F06 | Headerinfo pulse formatting | `proc_output_header_info*` | `headerinfo_data/valid/channel` |
| F07 | CRC check behavior | `FS_CRC_CHECK` | `aso_hit_type0_error(1)`, `crc_err_counter` |
| F08 | Hit-error behavior | `proc_output_pkt_hits` | `aso_hit_type0_error(0)` |
| F09 | Frame counters | `proc_avmm_slave_csr` | head, tail, and `frame_counter` readback |
| F10 | CSR protocol | `proc_avmm_slave_csr`, `_hw.tcl` | `waitrequest`, read/write semantics |
| F11 | Channel-width generic | `_hw.tcl` `myelaborate` | `channel/maxChannel` behavior across builds |
| F12 | CSR address-width generic | `_hw.tcl` `myelaborate` | address decode and unused-word behavior |
| F13 | `MODE_HALT` corner behavior | parser comma handling | abort-vs-hold contract |
| F14 | IDLE monitoring path | `receiver_force_go` path | header acceptance while run state is `IDLE` |
| F15 | `RUN_PREPARE` reset semantics | parser + CSR logic | state/counter reset timing |
| F16 | `TERMINATING` boundary | `RUN_SEQ_UPGRADE_PLAN.md` + RTL | no fresh header from `FS_IDLE`, open-frame drain |
| F17 | Current ready-handshake gap | `asi_ctrl_ready <= '1'` | negative verification evidence |
| F18 | Packaging/interface conformance | `_hw.tcl` | symbol widths, read latency, interface naming |

## 4. Plan Files And Counts

| File | Range / content | Planned count |
|---|---|---|
| [DV_PLAN.md](DV_PLAN.md) | Scope, contract, targets | n/a |
| [DV_HARNESS.md](DV_HARNESS.md) | UVM architecture and SVA plan | n/a |
| [DV_FORMAL.md](DV_FORMAL.md) | Packet-shape formal-readiness and proof targets | n/a |
| [DV_BASIC.md](DV_BASIC.md) | `B001..B130` | 130 |
| [DV_EDGE.md](DV_EDGE.md) | `E001..E130` | 130 |
| [DV_PROF.md](DV_PROF.md) | `P001..P130` | 130 |
| [DV_ERROR.md](DV_ERROR.md) | `X001..X130` | 130 |
| [DV_CROSS.md](DV_CROSS.md) | Functional coverage contract | n/a |
| [DV_COV.md](DV_COV.md) | Mandatory per-bucket coverage tables and execution-mode baselines | n/a |

## 5. Harness Direction

The harness contract is defined in [DV_HARNESS.md](DV_HARNESS.md). The environment must model the real sink/source protocol rather than silently converting the stream into abstract transactions at the DUT boundary.

Required structure:

- `rx8b1k` source agent that can inject real `K28.0`, `K28.4`, `K28.5`, CRC bytes, and per-byte error bits
- `ctrl` source agent that drives the real one-hot run-state words in `feb_system_v2` order
- `csr` agent for read/write timing and `waitrequest` checking
- monitors for `hit_type0` and `headerinfo`
- parser-based scoreboard that reconstructs frame expectations from the actual byte stream
- SVA for AVMM timing, control one-hot assumptions, parser-state invariants, and termination behavior
- mixed-language formal shell and packet-shape proof plan in [DV_FORMAL.md](DV_FORMAL.md)

## 6. Execution Modes

The maintained DV execution modes are:

1. `isolated`: default per-test timeframe with fresh DUT start
2. `bucket_frame`: continuous no-restart execution for every verification bucket in case-id order
3. `all_buckets_frame`: continuous no-restart execution across all sign-off buckets in bucket order, then case-id order

`bucket_frame` and `all_buckets_frame` are mandatory baselines for [DV_CROSS.md](DV_CROSS.md) and [DV_COV.md](DV_COV.md).

Continuous-frame rules:
- directed cases execute one transaction per case
- random cases execute several transactions per case
- the DUT is not restarted between cases inside `bucket_frame` or `all_buckets_frame`

## 7. Coverage Goals

- statement coverage: `>95%`
- branch/FSM transition coverage: `>90%`
- toggle coverage: `>80%`
- functional coverage: `>95%` against [DV_CROSS.md](DV_CROSS.md)
- `DV_COV.md` complete for isolated, `bucket_frame`, and `all_buckets_frame`
- zero unexpected SVA failures in clean regressions

Special closure requirements for this IP:

- explicit evidence for the current `TERMINATING` behavior
- explicit evidence for the unconditional-`ready` gap
- explicit evidence that `asi_rx8b1k_valid` is ignored by the present RTL
- compile/elaboration coverage for `_hw.tcl` generic-driven width changes

## 8. Pass/Fail Criteria

1. All BASIC cases pass on the default packaged build.
2. EDGE cases close the parser, CSR, and run-control boundaries without unexplained holes.
3. PROF cases demonstrate no deadlock, no stuck state, and coherent counter behavior under long runs.
4. ERROR cases cleanly distinguish real design behavior from expected future upgrade behavior.
5. Every open mismatch against `RUN_SEQ_UPGRADE_PLAN.md` is documented as a known RTL gap, not hidden inside the testbench.

## 9. Assumptions And Open Questions

1. The current RTL is the golden contract for this Phase-0 plan, even where it conflicts with the desired upgraded contract.
2. `asi_rx8b1k_valid` is intentionally or accidentally ignored; the testbench must model this accurately and flag it as a risk.
3. The current receiver does not create a synthetic terminal EOP marker; downstream closure belongs to later RTL work.
4. `MODE_HALT` comments and implementation wording are historically inconsistent, so the test plan anchors to the **implemented** branch behavior in the source code.
5. `DEBUG_LV` is generation-time only; DV uses compile/elaboration matrix checks rather than run-time behavioral checks.

## 10. Signoff Gate

Per the `dv-workflow`, implementation of the UVM harness should start only after this markdown set is reviewed as the authoritative Phase-0 verification intent for `mutrig_frame_deassembly`, including the `DV_COV.md` execution-mode and coverage-table contract.
