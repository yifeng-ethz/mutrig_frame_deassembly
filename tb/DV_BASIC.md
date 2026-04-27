# DV Basic Cases — mutrig_frame_deassembly

**Companion docs:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md), [DV_CROSS.md](DV_CROSS.md)

**ID Range:** `B001-B130`  
**Total:** 130 cases  
**Purpose:** deterministic feature-completion coverage for the present `frame_rcv_ip` contract

All cases in this file are grounded in the current RTL behavior, including the known termination limitations captured in `RUN_SEQ_UPGRADE_PLAN.md`.

## 1. Run-Control And Enable Basics (`B001-B038`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| B001_reset_idle_outputs_quiet | D | Assert reset, hold clock, release into default state | `hit_type0_valid=0`, `headerinfo_valid=0`, CSR counters clear | reset branch of parser, outputs, CSR |
| B002_idle_force_go_monitor_path_open | D | Stay in `IDLE`, drive a valid header sequence | parser remains blocked because current RTL keeps `receiver_force_go=0` in `IDLE` | `proc_run_control_mgmt_agent`, `proc_enable_ctrl` |
| B003_idle_force_go_ignores_csr_mask | D | Write `csr.control(0)=0`, remain in `IDLE`, drive header | frame stays blocked; there is no `IDLE` monitor-mode bypass in current RTL | `receiver_force_go` path is inactive in `IDLE` |
| B004_running_needs_csr_enable_high | D | Enter `RUNNING` with `csr.control(0)=1` and drive header | fresh header opens only when running and unmasked | `receiver_go` plus CSR enable |
| B005_running_mask_low_blocks_header_start | D | Enter `RUNNING`, write `csr.control(0)=0`, drive header | `FS_IDLE` does not leave idle | start condition in `proc_frame_rcv_comb` |
| B006_run_prepare_clears_parser_state | D | Open a frame, then issue `RUN_PREPARE` | parser returns to `FS_IDLE`, partial frame is discarded without false outputs | reset path on `run_state_cmd = RUN_PREPARE` |
| B007_sync_keeps_parser_closed | D | Hold `SYNC` and drive header | no new frame start | `run_state_cmd = SYNC` with `receiver_go=0` |
| B008_running_opens_header_detection | D | `RUN_PREPARE -> SYNC -> RUNNING`, then header | first valid frame opens after `RUNNING` | run-control happy path |
| B009_terminating_blocks_new_header_from_idle | D | Reach `TERMINATING` while parser is idle, then drive header | no fresh frame start | upgrade-plan current receiver behavior |
| B010_terminating_allows_open_frame_to_finish | D | Issue `TERMINATING` after header already started | current frame drains to final `eop` | `enable` only consumed in `FS_IDLE` |
| B011_ctrl_unknown_word_enters_error | D | Drive non-one-hot control word | `run_state_cmd=ERROR`, parser stays blocked | default branch of control decode |
| B012_ctrl_ready_high_with_valid | D | Pulse any legal control word | `asi_ctrl_ready` stays high during valid | unconditional ready assignment |
| B013_ctrl_ready_high_without_valid | D | Leave control bus idle | `asi_ctrl_ready` remains high | unconditional ready assignment |
| B014_ctrl_decode_idle_force_go | D | Send `IDLE` word | `receiver_force_go=0`, `receiver_go=0` | control decode table |
| B015_ctrl_decode_run_prepare_disable | D | Send `RUN_PREPARE` word | both `receiver_go` and `receiver_force_go` low | control decode table |
| B016_ctrl_decode_sync_disable | D | Send `SYNC` word | parser blocked for new headers | control decode table |
| B017_ctrl_decode_running_go | D | Send `RUNNING` word | `receiver_go=1`, `receiver_force_go=0` | control decode table |
| B018_ctrl_decode_terminating_stop_new_headers | D | Send `TERMINATING` word | `receiver_go=0`, no fresh header from idle | control decode table |
| B019_ctrl_decode_link_test_disable | D | Send `LINK_TEST` word | parser stays blocked | non-running state decode |
| B020_ctrl_decode_sync_test_disable | D | Send `SYNC_TEST` word | parser stays blocked | non-running state decode |
| B021_ctrl_decode_reset_disable | D | Send `RESET` word | parser stays blocked | non-running state decode |
| B022_ctrl_decode_out_of_daq_disable | D | Send `OUT_OF_DAQ` word | parser stays blocked | non-running state decode |
| B023_csr_control_default_enable | D | Read CSR word 0 after reset | `csr.control(0)=1` by default | `proc_avmm_slave_csr` reset load |
| B024_csr_control_write_zero_masks_running | D | In `RUNNING`, write control bit low | next header is blocked | CSR mask path |
| B025_csr_control_write_one_unmasks_running | D | In `RUNNING`, write control bit high | next header accepted again | CSR mask path |
| B026_csr_read_addr0_control_status | D | Read CSR address 0 | readdata contains `status` and `control` fields | CSR word 0 mux |
| B027_csr_read_addr1_crc_counter | D | Read CSR address 1 after bad frame | readback matches internal CRC error count | CSR word 1 mux |
| B028_csr_read_addr2_frame_counter | D | Read CSR address 2 around open/closed frame | readback matches `head-tail` contract | CSR word 2 mux |
| B029_csr_read_unused_word_zero | D | Read address above implemented map | default zero data | CSR read default path |
| B030_csr_write_unused_word_no_side_effect | D | Write unused address then rerun clean frame | no parser or counter behavior changes | CSR write decode |
| B031_csr_waitrequest_low_on_read | D | Perform a read transaction | `waitrequest` drops during service | `_hw.tcl` AVMM contract |
| B032_csr_waitrequest_low_on_write | D | Perform a write transaction | `waitrequest` drops during service | `_hw.tcl` AVMM contract |
| B033_csr_waitrequest_high_when_idle | D | Leave AVMM idle | `waitrequest=1` when no access is active | CSR idle branch |
| B034_csr_reserved_control_bits_roundtrip | D | Write upper control bits alongside bit0 | readback preserves written value, bit0 still controls enable | CSR storage behavior |
| B035_fs_idle_requires_k28_0_and_kchar | D | Drive `K28.0` with enable high | `FS_IDLE -> FS_FRAME_COUNTER` | start condition in `FS_IDLE` |
| B036_data_byte_without_kchar_no_start | D | Drive `0x1C` as data with `i_byteisk=0` | no frame start | start condition requires `i_byteisk=1` |
| B037_header_byte_with_enable_low_no_start | D | Force enable low, then drive `K28.0` | parser stays idle | start condition requires `enable=1` |
| B038_header_byte_with_enable_high_starts_frame | D | Enable high plus `K28.0` | `n_new_frame` pulses and header parsing begins | start condition happy path |

## 2. Long-Frame Parse Basics (`B039-B064`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| B039_long_zero_hit_frame_headerinfo_pulse | D | Long frame with event count 0 | one `headerinfo_valid` pulse, no hit output | `FS_EVENT_COUNTER -> FS_CRC_CALC` zero-hit path |
| B040_long_zero_hit_frame_no_hit_valid | D | Long zero-hit frame | `aso_hit_type0_valid` never asserts | zero-hit long-frame behavior |
| B041_long_one_hit_sop_and_eop_same_transfer | D | Long frame length 1 | single hit has both `sop` and `eop` | `n_word_cnt` comparisons |
| B042_long_two_hit_sop_first_only | D | Long frame length 2 | `sop` only on first hit | output SOP generation |
| B043_long_two_hit_eop_last_only | D | Long frame length 2 | `eop` only on second hit | output EOP generation |
| B044_long_frame_number_lsb_capture | D | Use distinctive low byte of frame number | `headerinfo[33:26]` low byte matches stream | frame-number capture |
| B045_long_frame_number_msb_capture | D | Use distinctive high byte of frame number | `headerinfo[41:34]` high byte matches stream | frame-number capture |
| B046_long_frame_flags_capture | D | Long frame with custom flags | `headerinfo[5:0]` equals frame flags | headerinfo pack |
| B047_long_frame_len_capture | D | Long frame with known length | `headerinfo[15:6]` equals frame length | headerinfo pack |
| B048_long_hit_channel_unpack | D | Long hit with channel extremes | output channel field matches encoded hit | long unpack data map |
| B049_long_hit_tcc_unpack | D | Long hit with known coarse timestamp | output `T_CC` matches input bytes | long unpack data map |
| B050_long_hit_tfine_unpack | D | Long hit with known fine timestamp | output `T_Fine` matches input bytes | long unpack data map |
| B051_long_hit_ecc_unpack | D | Long hit with known energy coarse timestamp | output `E_CC` matches input bytes | long unpack data map |
| B052_long_hit_eflag_unpack | D | Long hit with `E_Flag=1` | output `E_Flag` at bit 0 is correct | long compaction logic |
| B053_long_t_badhit_raises_error0 | D | Long hit with `T_BadHit=1` | `aso_hit_type0_error(0)=1` on that hit | hit-error generation |
| B054_long_e_badhit_raises_error0 | D | Long hit with `E_BadHit=1` | `aso_hit_type0_error(0)=1` on that hit | hit-error generation |
| B055_long_parity_error_raises_error0 | D | Assert parity error on a long-hit byte | error bit 0 rises on emitted hit | hit-error latch path |
| B056_long_decode_error_raises_error0 | D | Assert decode error on a long-hit byte | error bit 0 rises on emitted hit | hit-error latch path |
| B057_long_loss_sync_propagates_error2 | D | Assert loss-sync during a long frame | every emitted hit shows `error(2)=1` while active | lane-corrupt propagation |
| B058_long_crc_good_keeps_error1_low | D | Long frame with correct CRC | no post-frame `error(1)` sideband pulse is emitted | `FS_CRC_CHECK` good path |
| B059_long_crc_bad_raises_error1_on_eop | D | Long frame with incorrect CRC | one-cycle post-frame `error(1)` sideband pulse is emitted off the `valid/eop` beat | `FS_CRC_CHECK` bad path |
| B060_long_crc_bad_increments_crc_counter | D | Send one bad long frame | `crc_err_counter` increments by one | CSR update from `p_crc_err_count` |
| B061_sop_increments_frame_counter_head | D | One clean long frame | `frame_counter_head` increments on `sop` | CSR head counter logic |
| B062_eop_increments_frame_counter_tail | D | One clean long frame | `frame_counter_tail` increments on `eop` | CSR tail counter logic |
| B063_frame_counter_register_is_head_minus_tail | D | Sample during and after long frame | word2 reflects `head-tail` semantics | CSR frame-counter logic |
| B064_run_prepare_resets_head_tail_counters | D | Accumulate frames, then issue `RUN_PREPARE` | head/tail counters return to zero | run-prepare reset of counters |

## 3. Short-Frame Parse Basics (`B065-B081`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| B065_short_zero_hit_frame_headerinfo_pulse | D | Short frame with event count 0 | metadata pulse occurs, no hit output | zero-hit short path |
| B066_short_one_hit_sop_eop_same_transfer | D | Short frame length 1 | single hit carries both `sop` and `eop` | short output timing |
| B067_short_two_hit_even_pairing | D | Short frame length 2 | two hits unpack correctly over `UNPACK/UNPACK_EXTRA` | short packer even case |
| B068_short_three_hit_odd_pairing | D | Short frame length 3 | odd-hit residual nibble is paired correctly | `FS_UNPACK_EXTRA` odd path |
| B069_short_four_hit_even_followup | D | Short frame length 4 | repeated even/odd alternation stays aligned | short packer sequencing |
| B070_short_mode_selected_by_flags_100 | D | Use frame flag pattern `100xxx` | DUT takes short path, not long path | `p_txflag_isShort` |
| B071_short_path_zeroes_ecc_field | D | Emit short hit with nonzero compact bits elsewhere | output `E_CC` field remains zero | short output compaction |
| B072_short_path_uses_bit0_as_eflag | D | Toggle bit0 in short payload | output `E_Flag` follows bit0 | short output compaction |
| B073_short_hit_channel_unpack | D | Short hit with known channel code | output hit channel matches encoded value | short unpack data map |
| B074_short_hit_tcc_unpack | D | Short hit with known coarse timestamp | output `T_CC` matches encoded value | short unpack data map |
| B075_short_hit_tfine_unpack | D | Short hit with known fine timestamp | output `T_Fine` matches encoded value | short unpack data map |
| B076_short_parity_error_raises_error0 | D | Assert parity error during short unpack | `error(0)=1` on affected hit | hit-error latch path |
| B077_short_decode_error_raises_error0 | D | Assert decode error during short unpack | `error(0)=1` on affected hit | hit-error latch path |
| B078_short_loss_sync_propagates_error2 | D | Assert loss-sync during short frame | emitted hits carry `error(2)=1` | lane-corrupt propagation |
| B079_short_crc_bad_raises_error1_on_last_hit | D | Short frame with bad CRC | one-cycle post-frame `error(1)` sideband pulse is emitted off the `valid/eop` beat | short CRC bad path |
| B080_short_crc_counter_accumulates | D | Send two bad short frames | `crc_err_counter` increments twice | shared CRC counter logic |
| B081_short_frame_counter_updates | D | Clean short frame | head/tail/frame-counter semantics match long mode | mode-independent counters |

## 4. Headerinfo, Output, And Recovery Basics (`B082-B104`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| B082_headerinfo_one_pulse_per_frame | D | Clean frame with multiple hits | `headerinfo_valid` pulses once only | `proc_output_header_info_comb` |
| B083_headerinfo_channel_matches_input | D | Drive known ingress channel | metadata channel equals ingress channel | headerinfo channel assignment |
| B084_headerinfo_frame_number_field_map | D | Distinct frame number pattern | metadata field map matches spec | `headerinfo_data[41:26]` |
| B085_headerinfo_word_count_field_map | D | Frame with known hit count | metadata word-count field matches emitted hits | `headerinfo_data[25:16]` |
| B086_headerinfo_frame_len_field_map | D | Frame with known frame length | metadata length field matches event count | `headerinfo_data[15:6]` |
| B087_headerinfo_flags_field_map | D | Flag-rich frame | metadata flags field matches input flags | `headerinfo_data[5:0]` |
| B088_output_channel_matches_input_channel | D | Change ASIC channel between frames | `hit_type0_channel` tracks ingress channel | output channel assignment |
| B089_hit_valid_only_on_complete_word | D | Observe every cycle around one long hit | `valid` asserts only when full word is assembled | `n_new_word` contract |
| B090_no_hit_valid_in_crc_states | D | Watch zero-hit and nonzero-hit frames through CRC | no stray `valid` in `FS_CRC_CALC/CHECK` | output timing discipline |
| B091_no_sop_without_valid | D | Sweep long and short frames | `sop` never rises alone | SOP generation rules |
| B092_no_eop_without_valid | D | Sweep long and short frames | `eop` never rises alone | EOP generation rules |
| B093_idle_comma_does_not_create_frame | D | Drive `K28.5` while idle | parser stays idle, no metadata, no hit output | idle comma handling |
| B094_mode_halt0_comma_inside_frame_drops_to_idle | D | `MODE_HALT=0`, inject comma mid-frame | parser aborts to `FS_IDLE` | comma handling branch |
| B095_mode_halt1_comma_inside_frame_holds_state | D | `MODE_HALT=1`, inject comma mid-frame | parser does not drop to `FS_IDLE` immediately | comma handling branch |
| B096_running_mask_toggle_between_frames | D | Toggle `csr.control(0)` between clean frames | next frame start obeys new mask value | registered enable control |
| B097_mask_toggle_midframe_does_not_abort_current_frame | D | Drop `csr.control(0)` after a frame opened | current frame still completes | `enable` only used in `FS_IDLE` |
| B098_idle_monitoring_accepts_header_without_running | D | Leave run state in `IDLE`, do not send `RUNNING` | header stays blocked until `RUNNING` reopens parsing | `IDLE` is quiescent in current RTL |
| B099_idle_monitoring_still_ignores_csr_zero | D | In `IDLE`, write `csr.control(0)=0`, drive frame | frame remains blocked; CSR mask does not matter while `IDLE` is already closed | no monitor-mode bypass of CSR |
| B100_link_test_keeps_outputs_quiet | D | Hold `LINK_TEST` and drive bytes | no new header start | blocked state behavior |
| B101_sync_test_keeps_outputs_quiet | D | Hold `SYNC_TEST` and drive bytes | no new header start | blocked state behavior |
| B102_reset_state_keeps_outputs_quiet | D | Hold `RESET` and drive bytes | no new header start | blocked state behavior |
| B103_out_of_daq_keeps_outputs_quiet | D | Hold `OUT_OF_DAQ` and drive bytes | no new header start | blocked state behavior |
| B104_error_state_keeps_outputs_quiet | D | Enter `ERROR` with illegal control word and drive bytes | no new header start | control decode error state |

## 5. System Cadence And Counter Basics (`B105-B130`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| B105_run_prepare_sync_running_spine | D | Drive `RUN_PREPARE -> SYNC -> RUNNING` then one frame | bring-up sequence parses first legal frame cleanly | planned system cadence |
| B106_running_to_terminating_during_header_bytes | D | Issue `TERMINATING` while header bytes are still being consumed | frame already opened continues through payload | open-frame drain behavior |
| B107_running_to_terminating_during_unpack_finishes_frame | D | Stop during hit unpack | all declared hits emerge once, then parser returns idle | open-frame drain behavior |
| B108_terminating_to_idle_returns_quiescent | D | Finish frame, then issue `IDLE` | outputs return quiet and stay blocked until a later `RUNNING` command | run-state quiescence |
| B109_terminating_blocks_second_header_after_first_eop | D | Two back-to-back candidate frames, stop after first opens | first frame completes, second never starts | upgrade-plan current contract |
| B110_idle_after_terminating_restores_monitoring | D | `TERMINATING -> IDLE`, then drive new frame | `IDLE` remains quiescent; parsing does not reopen without `RUNNING` | current `IDLE` contract |
| B111_system_run_sequence_spine_matches_upgrade_plan | D | Replay full system cadence with open-frame stop | observed behavior matches current description in upgrade plan | cross-check with plan |
| B112_long_good_frames_leave_crc_counter_stable | D | Many clean long frames | CRC counter remains unchanged | CRC counter baseline |
| B113_bad_frames_accumulate_crc_counter | D | Mix multiple bad long/short frames | CRC counter equals number of bad frames | shared CRC accounting |
| B114_mixed_good_bad_crc_count_matches_bad_frames | D | Alternate clean and bad frames | only bad frames increment counter | CRC accounting |
| B115_frame_counter_zero_after_complete_frame | D | Read frame counter after clean frame completes | `frame_counter=0` once head and tail both advanced | head-tail difference |
| B116_frame_counter_nonzero_during_open_frame | D | Read frame counter after `sop` but before `eop` | `frame_counter` reflects open-frame imbalance | head-tail difference |
| B117_status_updates_on_new_frame_flags | D | Send frame with new flag pattern | CSR status field updates to latest flags | CSR status update on `n_frame_info_ready` |
| B118_status_holds_last_flags_between_frames | D | Read CSR status during idle gap | last frame flags persist until next frame | CSR status storage |
| B119_clean_short_hit_clears_all_error_bits | D | Clean short frame | `error[2:0]=000` on all hits | output error defaults |
| B120_clean_long_hit_clears_all_error_bits | D | Clean long frame | `error[2:0]=000` on all hits | output error defaults |
| B121_control_write_does_not_clear_crc_counter | D | Increment CRC counter, then write CSR control | CRC counter unchanged | CSR side-effect boundary |
| B122_control_write_does_not_clear_frame_counter | D | Hold open frame, write CSR control | frame counter semantics unaffected | CSR side-effect boundary |
| B123_channel_change_between_frames_is_observed | D | Change ingress channel only between frames | next frame and metadata show new channel | channel passthrough |
| B124_channel_change_midframe_is_prohibited_in_harness | D | Deliberately toggle channel inside a frame | scoreboard flags illegal stimulus instead of accepting new semantics | source comment on unsupported dynamic channel |
| B125_headerinfo_and_hit_channel_track_same_asic | D | Multi-frame run with changing channels | metadata and hit output agree on channel per frame | packaging + output consistency |
| B126_zero_hit_long_then_zero_hit_short_sequence | D | Send zero-hit long then zero-hit short frame | two metadata pulses, zero hits total | zero-hit mode transition |
| B127_good_long_then_good_short_sequence | D | Clean long frame followed by clean short frame | mode switch does not corrupt counters or outputs | frame-mode transition |
| B128_run_prepare_after_crc_error_resets_crc_counter | D | Create bad frame, then issue `RUN_PREPARE` | CRC counter clears to zero | run-prepare reset contract |
| B129_upgrade_contract_no_fresh_header_after_terminating | D | Frame open, stop, wait for idle, present new header | second frame is blocked in current RTL | `RUN_SEQ_UPGRADE_PLAN.md` gap anchor |
| B130_upgrade_contract_terminal_frame_finishes_once | D | Stop mid-frame with pending hits | exactly one terminal frame drains, no duplicate `eop` | current drain behavior |

**Row count:** 130 cases (`B001-B130`).
