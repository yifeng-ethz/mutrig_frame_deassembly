# DV Cross Coverage: mutrig_frame_deassembly

This file defines the functional coverage contract that the UVM environment must implement for `frame_rcv_ip`. The bins and crosses are derived from:

- `rtl/frame_rcv_ip.vhd`
- `script/mutrig_frame_deassembly_hw.tcl`
- `RUN_SEQ_UPGRADE_PLAN.md`

The goal is not raw bin count inflation. Every coverpoint below is intended to prove a real parser, control, or packaging contract path.

## 1. Core Coverpoints

| Coverpoint | Bins | Source / reason |
|---|---|---|
| `cp_run_state_cmd` | `IDLE`, `RUN_PREPARE`, `SYNC`, `RUNNING`, `TERMINATING`, `LINK_TEST`, `SYNC_TEST`, `RESET`, `OUT_OF_DAQ`, `ERROR` | full decode space of `run_state_t` |
| `cp_enable_source` | `force_go`, `receiver_go_and_csr1`, `blocked_by_csr0`, `blocked_by_state` | `proc_enable_ctrl` |
| `cp_parser_state` | `FS_IDLE`, `FS_FRAME_COUNTER`, `FS_EVENT_COUNTER`, `FS_UNPACK`, `FS_UNPACK_EXTRA`, `FS_CRC_CALC`, `FS_CRC_CHECK` | parser FSM closure |
| `cp_frame_mode` | `long`, `short`, `cec_other` | `p_frame_flags(4 downto 2)` decode |
| `cp_frame_len_bucket` | `0`, `1`, `2`, `3_4`, `5_15`, `16_63`, `64_255`, `256_1023` | long/short frame size closure |
| `cp_hit_count_bucket` | `0`, `1`, `2`, `odd_gt1`, `even_gt2`, `max1023` | output packet count closure |
| `cp_crc_result` | `good`, `bad` | `FS_CRC_CHECK` |
| `cp_error0_source` | `clean`, `parity`, `decode`, `t_badhit`, `e_badhit` | `aso_hit_type0_error(0)` provenance |
| `cp_error1` | `clear`, `crc_bad` | CRC error contract |
| `cp_error2` | `clear`, `loss_sync_seen` | lane-corrupt contract |
| `cp_mode_halt` | `0_abort`, `1_hold` | `MODE_HALT` generic |
| `cp_rx_valid_mode` | `valid_high`, `valid_low` | current RTL ignores `asi_rx8b1k_valid`; coverage must expose that |
| `cp_channel_width_cfg` | `1`, `4`, `8` | `_hw.tcl` generic-driven widths |
| `cp_csr_addr_width_cfg` | `1`, `2`, `8` | `_hw.tcl` generic-driven address widths |
| `cp_ctrl_word_shape` | `legal_onehot`, `all_zero`, `two_hot`, `all_one` | control-negative closure |
| `cp_header_accept_window` | `idle_monitoring`, `running_enabled`, `running_masked`, `terminating_blocked` | start-condition closure |
| `cp_terminating_boundary` | `before_header`, `mid_header`, `mid_payload`, `crc_stage`, `after_eop` | upgrade-plan anchor |
| `cp_headerinfo_pulse` | `zero_hit`, `nonzero_hit`, `long`, `short` | `headerinfo_valid` generation |
| `cp_counter_snapshot` | `pre_sop`, `open_frame`, `post_eop`, `post_run_prepare` | CSR counter semantics |
| `cp_comma_position` | `idle`, `frame_counter`, `event_counter`, `unpack`, `crc_calc` | comma recovery closure |

## 2. Required Crosses

| Cross | Intent |
|---|---|
| `cp_run_state_cmd x cp_enable_source` | prove each state drives the expected enable path |
| `cp_run_state_cmd x cp_header_accept_window` | distinguish `IDLE` monitoring, `RUNNING` masked/unmasked, and `TERMINATING` block |
| `cp_run_state_cmd x cp_terminating_boundary` | prove coverage for all stop points |
| `cp_frame_mode x cp_frame_len_bucket` | long/short size-space closure |
| `cp_frame_mode x cp_hit_count_bucket` | output packetization closure |
| `cp_frame_mode x cp_crc_result` | CRC handling in both long and short modes |
| `cp_frame_mode x cp_error0_source` | hit-error sources in both modes |
| `cp_frame_mode x cp_error2` | loss-sync propagation irrespective of frame mode |
| `cp_mode_halt x cp_comma_position` | comma handling across all injected positions |
| `cp_mode_halt x cp_parser_state` | verify state outcome differs only where intended |
| `cp_channel_width_cfg x cp_frame_mode` | output formatting survives width changes |
| `cp_csr_addr_width_cfg x cp_counter_snapshot` | counter/CSR behavior across address-width builds |
| `cp_ctrl_word_shape x cp_run_state_cmd` | invalid control words land in `ERROR` and legal words do not |
| `cp_rx_valid_mode x cp_header_accept_window` | capture the current “valid ignored” behavior explicitly |
| `cp_headerinfo_pulse x cp_frame_len_bucket` | metadata pulse closure for zero, small, and large frames |
| `cp_counter_snapshot x cp_crc_result` | counter-read timing around good/bad frame boundaries |
| `cp_terminating_boundary x cp_frame_mode` | termination coverage in both long and short parsing |
| `cp_terminating_boundary x cp_hit_count_bucket` | stop-point coverage for empty, small, and large frames |

## 3. Illegal / Excluded Bins

| Item | Treatment | Reason |
|---|---|---|
| `cp_parser_state=FS_UNPACK_EXTRA` with `cp_frame_mode=long` | illegal bin | short-mode-only state |
| `cp_header_accept_window=terminating_blocked` with parser not in `FS_IDLE` | excluded from acceptance coverage | open-frame drain is a different contract |
| `cp_error1=crc_bad` on zero-hit frame without forced bad CRC bytes | excluded unless explicitly injected | zero-hit good frames are legal and common |
| mid-frame channel change as a normal pass case | illegal stimulus | source comment says dynamic changing the channel is unsupported |

## 4. Traceability To Test Buckets

| Coverage area | Main testcase sources |
|---|---|
| run-state and enable closure | `DV_BASIC` B001-B038, B105-B130; `DV_EDGE` E001-E030, E111-E130 |
| long parser and output packing | `DV_BASIC` B039-B064; `DV_EDGE` E031-E058 |
| short parser and nibble pairing | `DV_BASIC` B065-B081; `DV_EDGE` E038-E070 |
| headerinfo and counters | `DV_BASIC` B082-B128; `DV_EDGE` E046-E050, E102-E105 |
| CRC, hit errors, loss-sync | `DV_BASIC` B053-B060, B076-B080, B112-B120; `DV_EDGE` E071-E090 |
| comma / `MODE_HALT` | `DV_BASIC` B093-B099; `DV_EDGE` E081-E088; `DV_PROF` P066-P078 |
| system-cadence / termination | `DV_BASIC` B105-B130; `DV_EDGE` E111-E130; `DV_PROF` P027-P039, P105-P117 |
| CSR/generic matrix | `DV_BASIC` B023-B034; `DV_EDGE` E091-E110; `DV_ERROR` X105-X130 |

## 5. Closure Notes

1. Coverage is only closed when both the present RTL contract and the known upgrade gaps have evidence.
2. A covered bin that contradicts `RUN_SEQ_UPGRADE_PLAN.md` is still valuable, but it must be tagged as “current RTL behavior” rather than “target upgraded behavior”.
3. The harness should track not only whether a bin fired, but whether the scoreboard and SVA agreed on the same interpretation.
4. `bucket_frame` and `all_buckets_frame` are mandatory `DV_CROSS` baselines:
   - each bucket-frame run executes cases in case-id order without restarting the DUT between cases
   - the all-buckets-frame run executes buckets in bucket order and cases in case-id order without restarting the DUT between cases
   - directed cases contribute one transaction per case in these continuous frames
   - random cases contribute several transactions per case in these continuous frames
5. The in-bench `SUPER_LONG_COUNTER_SOAK` run is only a seed anchor, not full closure:
   - it is allowed to remain as a deterministic regression probe
   - long-run closure must also include `tb/scripts/run_full_random_parallel.py`
   - that runner draws documented cases from all buckets with replacement, executes `frcv_doc_case_list_test` in `all_buckets_frame` mode, and is intended to run for tens of minutes per worker in parallel
