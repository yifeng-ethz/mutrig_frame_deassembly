# DV Edge Cases — mutrig_frame_deassembly

**ID Range:** `E001-E130`  
**Total:** 130 cases  
**Purpose:** boundary, corner, and contract-edge coverage for parser control, framing, counters, and termination

## 1. Control-Timing And Header-Start Edges (`E001-E030`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| E001_ctrl_state_update_is_one_cycle_late_from_valid | D | Apply a legal control word on one cycle and sample outputs immediately | `run_state_cmd` updates after the clock edge, but decoded enable effect appears on the following cycle | sequential control process ordering |
| E002_running_command_same_cycle_header_not_accepted | D | Present `RUNNING` command and header on the same edge from a blocked state | header is not accepted until the next cycle | one-cycle decode/effect lag |
| E003_idle_command_same_cycle_header_still_sees_previous_running | D | Drive `IDLE` command and header on same cycle while previously `RUNNING` | acceptance follows prior-state decode, not new decode | sequential control process ordering |
| E004_terminating_command_same_cycle_open_frame_continues | D | Apply `TERMINATING` exactly as a frame is already open | current frame is not aborted | open-frame drain path |
| E005_back_to_back_running_and_terminating_words | D | Issue `RUNNING` then `TERMINATING` on adjacent cycles | header-open window exists for exactly the registered running window | control edge sequencing |
| E006_duplicate_running_words_idempotent | D | Hold `RUNNING` word for multiple cycles | no duplicate side effects beyond staying in running | control decode stability |
| E007_duplicate_idle_words_idempotent | D | Hold `IDLE` word for multiple cycles | monitor mode remains stable, no extra outputs | control decode stability |
| E008_ctrl_valid_pulse_one_cycle_only_still_latches_state | D | Pulse legal control word for one cycle only | state is retained after valid drops | latched `run_state_cmd` behavior |
| E009_ctrl_valid_held_high_repeated_word_stable | D | Hold valid high with same legal word | state/output decode remains stable | latched control behavior |
| E010_ctrl_illegal_zero_word_maps_error | D | Drive `000000000` | state becomes `ERROR` | control default decode |
| E011_ctrl_twohot_word_maps_error | D | Drive any two-hot control word | state becomes `ERROR` | control default decode |
| E012_ctrl_allones_word_maps_error | D | Drive `111111111` | state becomes `ERROR` | control default decode |
| E013_ctrl_valid_low_keeps_previous_state | D | Change `asi_ctrl_data` while `valid=0` | prior run state is held | `else run_state_cmd <= run_state_cmd` |
| E014_ctrl_ready_does_not_backpressure_illegal_word | D | Drive illegal word with valid high | `ready` stays high despite illegal command | unconditional ready gap |
| E015_ctrl_ready_high_during_reset_release | D | Release reset and sample `ready` immediately after | `ready` comes back high in the running clocked logic | control reset behavior |
| E016_fs_idle_ignores_header_when_byteisk0_data1c | D | Drive `0x1C` with `i_byteisk=0` | no start despite header byte value | `FS_IDLE` start condition |
| E017_fs_idle_ignores_kchar_non_header_9c | D | Drive `K28.4` in `FS_IDLE` | no start | start condition value check |
| E018_fs_idle_ignores_kchar_nonheader_other | D | Drive another K-character in `FS_IDLE` | no start | start condition value check |
| E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low | D | Hold `asi_rx8b1k_valid=0` while presenting a legal header | no frame start or metadata emission; regression guard for the old valid-ignore bug | `proc_input_wrapper_comb` valid gate |
| E020_midframe_valid_low_does_not_pause_parser | D | Drop `asi_rx8b1k_valid` during frame bytes | parser halts to idle and discards the partial frame without outputs | invalid input must stop parsing |
| E021_midframe_channel_change_not_tracked_internally | D | Toggle ingress channel inside a frame | output channel is direct passthrough, not pipelined per byte | source-code comment on unsupported dynamic channel |
| E022_header_after_comma_recovery_mode0_restarts_cleanly | D | `MODE_HALT=0`, inject comma, then a fresh header | parser restarts from clean `FS_IDLE` on next header | mode0 recovery edge |
| E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header | D | `MODE_HALT=1`, inject comma and then header-like byte | parser continues old frame semantics, not new frame start | mode1 hold edge |
| E024_header_immediately_after_run_prepare_blocked | D | Assert `RUN_PREPARE` and immediately present header | no start until state leaves prepare/reset window | run-prepare reset edge |
| E025_header_immediately_after_sync_blocked | D | Assert `SYNC` then present header same cycle | no start | sync blocked edge |
| E026_header_immediately_after_running_opened_next_cycle | D | Assert `RUNNING`, then header on next cycle | first post-running cycle accepts header | decode-lag boundary |
| E027_header_immediately_after_terminating_blocked | D | Assert `TERMINATING`, then header on next cycle from idle | no start | stop boundary |
| E028_header_with_csr_enable_toggle_same_cycle_follows_registered_enable | D | Toggle CSR enable on same cycle as header | acceptance follows prior registered enable value | registered `enable` behavior |
| E029_header_following_write_zero_then_one_requires_clocked_enable | D | Write `0`, then `1`, and probe adjacent headers | parser opens only after enable register updates | `proc_enable_ctrl` register timing |
| E030_back_to_back_headers_without_crc_gap_second_header_dropped_as_corrupt | D | Inject a new header before current frame reaches CRC check | parser treats bytes as in-frame corruption, not a clean second start | parser not reentrant mid-frame |

## 2. Frame-Length, Number, And Packing Boundaries (`E031-E070`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| E031_frame_number_0000_propagates | D | Frame number `0x0000` | metadata reports exactly `0x0000` | frame-number capture |
| E032_frame_number_ffff_propagates | D | Frame number `0xFFFF` | metadata reports exactly `0xFFFF` | frame-number capture |
| E033_frame_number_wrap_ffff_to_0000 | D | Consecutive frames `0xFFFF`, `0x0000` | second metadata pulse is independent and clean | frame-number sequencing |
| E034_frame_len_zero_long_goes_crc_path_directly | D | Long-mode frame length 0 | parser skips unpack and moves to CRC path | `FS_EVENT_COUNTER` zero-length branch |
| E035_frame_len_one_long_single_word | D | Long-mode frame length 1 | exactly one output word, single `sop/eop` | long min nonzero length |
| E036_frame_len_two_long_two_words | D | Long-mode frame length 2 | two words only | long small-count edge |
| E037_frame_len_max_long_1023_words | S | Long-mode frame length `1023` | parser completes without counter overflow | 10-bit frame length boundary |
| E038_frame_len_zero_short_goes_crc_path_directly | D | Short-mode frame length 0 | zero-hit short frame behaves like clean metadata-only frame | short zero-length edge |
| E039_frame_len_one_short_single_word | D | Short-mode frame length 1 | one output word only | short min nonzero length |
| E040_frame_len_two_short_even_pair | D | Short-mode frame length 2 | even pairing completes without stray nibble | short even-count edge |
| E041_frame_len_three_short_odd_pair_plus_residual | D | Short-mode frame length 3 | odd residual path through `FS_UNPACK_EXTRA` is clean | short odd-count edge |
| E042_frame_len_max_short_1023_words | S | Short-mode frame length `1023` | parser completes without word-count corruption | 10-bit frame length boundary |
| E043_frame_flags_short_bit_exact_100 | D | Flags `[4:2]="100"` | short-mode path selected | `p_txflag_isShort` exact match |
| E044_frame_flags_nonshort_000_long | D | Flags `[4:2]="000"` | long-mode path selected | long-mode boundary |
| E045_frame_flags_cec_101_uses_long_output_path | D | Flags `[4:2]="101"` | parser does not enter short path | `p_txflag_isCEC` present but unused |
| E046_word_count_headerinfo_for_zero_length_frame | D | Zero-hit frame | headerinfo word-count field is `0` | metadata word-count pack |
| E047_word_count_headerinfo_for_single_hit_frame | D | One-hit frame | headerinfo word-count field is `1` | metadata word-count pack |
| E048_word_count_headerinfo_for_max_length_frame | S | Max-length frame | headerinfo word-count matches full emission count | metadata word-count pack |
| E049_startofpacket_asserts_when_n_word_cnt_becomes1 | D | One- and two-hit frames | `sop` is aligned to first completed word only | SOP comparator boundary |
| E050_endofpacket_asserts_when_n_word_cnt_equals_frame_len | D | Sweep lengths 1 and 2 | `eop` aligns to last completed word only | EOP comparator boundary |
| E051_long_hit_channel_00000_boundary | D | Long hit with channel `0` | output field is `0` | long field lower bound |
| E052_long_hit_channel_11111_boundary | D | Long hit with channel `31` | output field is `31` | long field upper bound |
| E053_long_tcc_zero_boundary | D | Long hit with `T_CC=0` | output field is `0` | long coarse lower bound |
| E054_long_tcc_allones_boundary | D | Long hit with `T_CC=0x7FFF` | output field is all ones | long coarse upper bound |
| E055_long_tfine_zero_boundary | D | Long hit with `T_Fine=0` | output field is `0` | long fine lower bound |
| E056_long_tfine_allones_boundary | D | Long hit with `T_Fine=31` | output field is all ones | long fine upper bound |
| E057_long_ecc_zero_boundary | D | Long hit with `E_CC=0` | output field is `0` | energy coarse lower bound |
| E058_long_ecc_allones_boundary | D | Long hit with `E_CC=0x7FFF` | output field is all ones | energy coarse upper bound |
| E059_short_hit_channel_00000_boundary | D | Short hit with channel `0` | output field is `0` | short field lower bound |
| E060_short_hit_channel_11111_boundary | D | Short hit with channel `31` | output field is `31` | short field upper bound |
| E061_short_tcc_zero_boundary | D | Short hit with `T_CC=0` | output field is `0` | short coarse lower bound |
| E062_short_tcc_allones_boundary | D | Short hit with `T_CC=0x7FFF` | output field is all ones | short coarse upper bound |
| E063_short_tfine_zero_boundary | D | Short hit with `T_Fine=0` | output field is `0` | short fine lower bound |
| E064_short_tfine_allones_boundary | D | Short hit with `T_Fine=31` | output field is all ones | short fine upper bound |
| E065_short_even_hit_extra_nibble_boundary_0 | D | Even short hit where shared nibble is `0x0` | no nibble contamination | `FS_UNPACK_EXTRA` low nibble edge |
| E066_short_even_hit_extra_nibble_boundary_f | D | Even short hit where shared nibble is `0xF` | no nibble contamination | `FS_UNPACK_EXTRA` high nibble edge |
| E067_short_odd_hit_saved_nibble_boundary_0 | D | Odd short hit with saved nibble `0x0` | saved nibble is inserted correctly | odd-hit carry edge |
| E068_short_odd_hit_saved_nibble_boundary_f | D | Odd short hit with saved nibble `0xF` | saved nibble is inserted correctly | odd-hit carry edge |
| E069_short_last_odd_hit_goes_direct_to_crc | D | Odd-length short frame ending on odd hit | parser leaves unpack path cleanly for CRC | odd tail edge |
| E070_short_last_even_hit_goes_from_unpack_extra_to_crc | D | Even-length short frame ending in `UNPACK_EXTRA` | parser transitions cleanly into CRC stage | even tail edge |

## 3. CRC, Error, And Comma-Recovery Boundaries (`E071-E090`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| E071_crc_magic_match_7ff2_only_case | D | Drive a frame whose CRC engine ends at `0x7FF2` | no CRC error is flagged | `FS_CRC_CHECK` magic compare |
| E072_crc_magic_onebit_off_is_error | D | Drive a frame whose CRC engine ends at `0x7FF3` | CRC error is flagged | exact magic compare |
| E073_crc_error_only_asserts_on_eop_cycle | D | Bad-CRC frame with multiple hits | `error(1)` appears only on terminal hit | EOP-only CRC flagging |
| E074_hit_error_latches_across_multi_byte_long_word | D | Assert byte error early in a long hit | `error(0)` survives until word commit | hit-error latch lifetime |
| E075_hit_error_clears_after_word_commit | D | Inject error on hit N, keep hit N+1 clean | only hit N reports `error(0)` | hit-error clear behavior |
| E076_hit_error_latches_across_multi_byte_short_word | D | Assert byte error during short-word reconstruction | committed short hit reports `error(0)` | short hit-error latch |
| E077_loss_sync_on_header_byte_sets_error2_entire_frame | D | Assert loss-sync at frame start | all emitted hits show `error(2)=1` | lane error spans frame |
| E078_loss_sync_only_on_last_byte_still_sets_error2 | D | Assert loss-sync only at end of frame | terminal hit still reports `error(2)=1` | lane error observation edge |
| E079_parity_error_on_nonhit_state_does_not_set_error0 | D | Assert parity error during header bytes only | no hit is falsely marked unless unpacked hit saw the error | hit-error state gating |
| E080_decode_error_on_nonhit_state_does_not_set_error0 | D | Assert decode error during CRC bytes only | no earlier hit is falsely marked | hit-error state gating |
| E081_mode_halt0_comma_on_frame_counter_byte_aborts | D | `MODE_HALT=0`, comma in frame-number field | parser returns to idle | abort timing edge |
| E082_mode_halt0_comma_on_event_counter_byte_aborts | D | `MODE_HALT=0`, comma in frame-length field | parser returns to idle | abort timing edge |
| E083_mode_halt0_comma_on_unpack_byte_aborts | D | `MODE_HALT=0`, comma in payload byte | parser returns to idle | abort timing edge |
| E084_mode_halt1_comma_on_frame_counter_byte_holds_state | D | `MODE_HALT=1`, comma in frame-number field | parser does not drop to idle | hold timing edge |
| E085_mode_halt1_comma_on_event_counter_byte_holds_state | D | `MODE_HALT=1`, comma in frame-length field | parser does not drop to idle | hold timing edge |
| E086_mode_halt1_comma_on_unpack_byte_holds_state | D | `MODE_HALT=1`, comma in payload byte | parser does not drop to idle | hold timing edge |
| E087_recovery_after_mode0_abort_starts_next_clean_frame | D | Abort one frame in mode0, then inject a clean frame | next frame parses cleanly | mode0 recovery closure |
| E088_mode1_resume_with_error_bits_preserved | D | Hold through comma in mode1 and complete frame | emitted hits reflect the current in-frame error history only | mode1 recovery semantics |
| E089_clean_frame_after_bad_crc_clears_error1 | D | Bad frame followed by clean frame | clean frame has `error(1)=0` | CRC error nonstickiness |
| E090_clean_frame_after_hit_error_clears_error0 | D | Hit-error frame followed by clean frame | clean frame has `error(0)=0` | hit-error nonstickiness |

## 4. CSR And Generic Boundaries (`E091-E110`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| E091_csr_addr_width1_aliases_words0_1_only | D | Build with `CSR_ADDR_WIDTH=1` and sweep addresses | only low address bit is significant | `_hw.tcl` generic width |
| E092_csr_addr_width2_supports_words0_2_exactly | D | Default build address sweep | words 0-2 decode, others default | default address width |
| E093_csr_addr_width8_high_unused_reads_zero | D | Build with `CSR_ADDR_WIDTH=8`, read high addresses | unused space reads default zero | `_hw.tcl` generic width |
| E094_channel_width1_headerinfo_maxchannel1 | D | Build with `CHANNEL_WIDTH=1` | metadata channel range is 0..1 | `_hw.tcl myelaborate` |
| E095_channel_width1_output_channel_singlebit | D | Build with `CHANNEL_WIDTH=1`, toggle channels 0/1 | hit output channel follows single-bit width | `_hw.tcl myelaborate` |
| E096_channel_width4_default_maxchannel15 | D | Default build, sweep 0..15 | all packaged default channels propagate | `_hw.tcl` default |
| E097_channel_width8_large_channel_passthrough | D | Build with `CHANNEL_WIDTH=8`, drive `0xA5` | metadata and hit channel keep full width | `_hw.tcl myelaborate` |
| E098_channel_width_change_between_builds_no_scoreboard_rewrite | D | Rebuild with widths 1,4,8 | same logical checks scale by config | packaging/generic closure |
| E099_csr_read_and_write_same_cycle_prioritizes_read | D | Assert `read` and `write` in same cycle | read path wins, write does not commit that cycle | read-before-write `if/elsif` order |
| E100_write_control_zero_midframe_affects_next_fs_idle_only | D | Write mask low during payload | current frame continues; next fresh header blocked | registered enable edge |
| E101_write_control_one_midframe_does_not_create_spurious_start | D | Write mask high while already parsing | no duplicate header or new frame is created | registered enable edge |
| E102_control_status_read_during_frame_updates_after_headerinfo | D | Read word0 before and after `headerinfo_valid` | status updates only after new frame info is ready | CSR status update timing |
| E103_crc_counter_read_on_bad_eop_cycle_returns_preincremented_value | D | Read word1 exactly on bad-frame `eop` cycle | read observes pre-update or prior registered value; next read observes increment | clocked CSR timing edge |
| E104_frame_counter_read_on_sop_cycle_returns_preincremented_value | D | Read word2 on first-hit cycle | count reflects previous registered value | clocked CSR timing edge |
| E105_frame_counter_read_on_eop_cycle_returns_preupdated_difference | D | Read word2 on last-hit cycle | value updates cleanly only after register commit | clocked CSR timing edge |
| E106_waitrequest_release_during_reset | D | Sample AVMM during reset assertion and release | interface does not hang the testbench | CSR reset branch |
| E107_headerinfo_channel_width_tracks_generic | D | Rebuild across width set and sample metadata | metadata channel port width matches generic | `_hw.tcl myelaborate` |
| E108_hit_type0_channel_width_tracks_generic | D | Rebuild across width set and sample hit output | hit output channel width matches generic | `_hw.tcl myelaborate` |
| E109_rx_channel_width_tracks_generic | D | Rebuild across width set and drive ingress channels | sink channel width matches generic | `_hw.tcl myelaborate` |
| E110_debug_lv0_and_debug_lv2_behave_identically_functionally | D | Compare `DEBUG_LV=0` and `DEBUG_LV=2` on same stimuli | parser-visible behavior is unchanged | generation parameter closure |

## 5. Termination, Truncation, And Upgrade-Plan Edges (`E111-E130`)

| ID | Method | Scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| E111_terminating_before_header_byte_blocks_frame | D | Move to `TERMINATING` while idle, then drive full frame | no header acceptance | current terminate contract |
| E112_terminating_on_header_byte_same_cycle_depends_on_old_state | D | Present `TERMINATING` and header on same edge from running | outcome follows old-state decode and is captured in waveforms | one-cycle control lag |
| E113_terminating_after_header_before_len_allows_completion | D | Stop after header start but before frame length is fully received | frame still completes if bytes continue | open-frame drain boundary |
| E114_terminating_after_first_long_hit_before_second_hit | D | Two-hit long frame, stop after first hit | second hit still emerges, no third frame opens | drain-on-open-frame |
| E115_terminating_after_first_short_hit_before_second_hit | D | Two-hit short frame, stop after first hit | remaining in-frame hit still emerges | drain-on-open-frame |
| E116_terminating_during_crc_calc | D | Stop when frame is already in CRC byte phase | final `eop` still occurs once | drain-on-open-frame |
| E117_terminating_during_crc_check | D | Stop one cycle before CRC compare completes | final hit and CRC flag behavior stay coherent | drain-on-open-frame |
| E118_idle_after_terminating_then_header_next_cycle_accepted_in_monitor_mode | D | `TERMINATING -> IDLE`, then new header | monitor mode reopens parser on next header | `IDLE` force-go recovery |
| E119_running_to_idle_same_cycle_header_follows_old_state_running | D | Apply `IDLE` and header together from running | acceptance follows prior decoded running window | one-cycle control lag |
| E120_running_to_run_prepare_same_cycle_resets_next_cycle | D | Apply `RUN_PREPARE` during running | reset effect happens on next registered cycle | control/reset edge |
| E121_repeated_terminating_words_do_not_duplicate_eop | D | Hold `TERMINATING` high-equivalent word across multiple cycles | no duplicate `eop` or duplicated frame completion | drain idempotence |
| E122_repeated_idle_words_do_not_force_extra_output | D | Hold `IDLE` word while parser idle | no spurious metadata or hits | control idempotence |
| E123_header_after_error_state_needs_new_running_or_idle_monitoring | D | Enter `ERROR`, then go `IDLE` or `RUNNING` and retry | parser only recovers after legal state change | error-state recovery |
| E124_truncated_long_frame_never_asserts_eop | D | Declare long length N but stop byte stream early | no false `eop` is emitted | truncation edge |
| E125_truncated_short_frame_never_asserts_eop | D | Declare short length N but stop byte stream early | no false `eop` is emitted | truncation edge |
| E126_excess_bytes_after_declared_long_frame_ignored_until_next_header | D | Send extra payload-like bytes after long frame completion | no extra hit output until next real header | post-frame byte handling |
| E127_excess_bytes_after_declared_short_frame_ignored_until_next_header | D | Send extra payload-like bytes after short frame completion | no extra hit output until next real header | post-frame byte handling |
| E128_upgrade_plan_gap_new_frame_dropped_if_first_header_arrives_post_terminating | D | Emulate fresh post-stop frame arrival | current DUT drops that frame at `FS_IDLE` | exact failing hole in upgrade plan |
| E129_upgrade_plan_option_a_terminal_marker_not_synthesized_by_current_dut | D | Stop after accepted traffic and watch downstream outputs | no synthetic terminal `eop` is created beyond real frame end | current-vs-target contract |
| E130_upgrade_plan_current_ready_does_not_wait_for_drain | D | Stop mid-frame and sample `asi_ctrl_ready` | `ready` stays high even while draining work remains | ready-handshake gap |

**Row count:** 130 cases (`E001-E130`).
