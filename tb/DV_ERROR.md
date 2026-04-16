# DV Error / Negative Cases — mutrig_frame_deassembly

**ID Range:** `X001-X130`  
**Total:** 130 cases  
**Purpose:** reset, malformed-input, illegal-control, and contract-violation coverage for the current `frame_rcv_ip` implementation

## 1. Reset-Fault Injection (`X001-X013`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X001_reset_in_fs_idle | F | Assert reset while parser idle | outputs and counters clear immediately, no hang | reset behavior |
| X002_reset_in_frame_counter | F | Assert reset during frame-number byte capture | parser returns to `FS_IDLE`, partial frame is discarded | parser reset |
| X003_reset_in_event_counter | F | Assert reset during frame-length capture | parser returns to `FS_IDLE`, no stale metadata | parser reset |
| X004_reset_in_long_unpack | F | Assert reset during long-hit unpack | partial hit is dropped, no false valid | parser reset |
| X005_reset_in_short_unpack | F | Assert reset during short-hit unpack | partial hit is dropped, no false valid | parser reset |
| X006_reset_in_unpack_extra | F | Assert reset in `FS_UNPACK_EXTRA` | saved nibble state is cleared | parser reset |
| X007_reset_in_crc_calc | F | Assert reset during CRC-byte phase | no false `eop` or CRC update | parser reset |
| X008_reset_in_crc_check | F | Assert reset at compare cycle | no stale error flag leaks after reset | parser reset |
| X009_reset_during_bad_crc_frame | F | Reset while a bad frame is finishing | CRC counter does not increment spuriously after reset | reset vs CRC |
| X010_reset_during_hit_error_frame | F | Reset while hit error is latched | `error(0)` does not leak into post-reset frame | reset vs hit-error latch |
| X011_reset_during_losssync_frame | F | Reset while loss-sync is asserted | `error(2)` does not leak into post-reset frame | reset vs lane error |
| X012_reset_while_csr_read_active | F | Assert reset during a CSR read | interface recovers without permanent waitrequest stall | AVMM reset robustness |
| X013_reset_while_ctrl_valid_high | F | Assert reset during control command | post-reset state returns to `IDLE`, not stale command | control reset robustness |

## 2. Control-Word And Control-Protocol Negatives (`X014-X026`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X014_ctrl_allzero_word | F | Drive `000000000` with valid high | state enters `ERROR`, parser stays blocked | control decode default |
| X015_ctrl_twohot_idle_running | F | Drive `IDLE|RUNNING` two-hot word | state enters `ERROR` | control decode default |
| X016_ctrl_twohot_running_terminating | F | Drive `RUNNING|TERMINATING` two-hot word | state enters `ERROR` | control decode default |
| X017_ctrl_allones_word | F | Drive `111111111` | state enters `ERROR` | control decode default |
| X018_ctrl_random_illegal_word_seed1 | F | Random non-one-hot word set #1 | no crash, `ERROR` reached | illegal control stability |
| X019_ctrl_random_illegal_word_seed2 | F | Random non-one-hot word set #2 | no crash, `ERROR` reached | illegal control stability |
| X020_ctrl_valid_glitch_no_word_change | F | Toggle valid for one cycle with illegal payload | state change is deterministic and captured | control input robustness |
| X021_ctrl_payload_changes_while_valid_high | F | Change payload across consecutive valid cycles | only sampled words on clocks matter, no combinational glitching | clocked control decode |
| X022_ctrl_illegal_then_legal_recovery_idle | F | Illegal word then `IDLE` | parser recovers only after legal state update | error recovery |
| X023_ctrl_illegal_then_legal_recovery_running | F | Illegal word then `RUNNING` | parser recovers only after legal state update | error recovery |
| X024_ctrl_ready_not_indicating_recovery | F | Illegal word followed by drain work | `ready` stays high and does not indicate recovery progress | known ready gap |
| X025_ctrl_valid_low_payload_noise | F | Randomize `asi_ctrl_data` while `valid=0` | no unintended state transitions | control hold behavior |
| X026_ctrl_fast_oscillation_legal_illegal_mix | F | Rapid legal/illegal command stream | no testbench or DUT deadlock | control robustness |

## 3. Malformed Long-Frame Negatives (`X027-X039`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X027_long_header_only_no_counters | F | Drive `K28.0` then stop | no false metadata or hit emission | incomplete frame handling |
| X028_long_missing_frame_number_byte1 | F | Header plus one frame-number byte only | parser never emits hit or metadata as complete frame | incomplete frame handling |
| X029_long_missing_event_counter_msb | F | Omit event-counter MSB byte | no false hit count interpretation | incomplete frame handling |
| X030_long_missing_event_counter_lsb | F | Omit event-counter LSB byte | no false unpack start | incomplete frame handling |
| X031_long_declared_len1_missing_payload | F | Declare one-hit frame, omit payload bytes | no `valid`, no false `eop` | truncation |
| X032_long_declared_len2_only_one_hit_present | F | Declare two-hit frame, provide one hit only | no false second hit or `eop` | truncation |
| X033_long_declared_len_large_cut_midpayload | F | Declare large frame, stop mid-payload | parser does not self-complete | truncation |
| X034_long_bad_trailer_symbol_replaced_with_data | F | Corrupt the byte where trailer would have been implied | CRC/error behavior follows bytes seen, no hidden trailer check | no explicit trailer check in RTL |
| X035_long_extra_payload_after_declared_len | F | Provide more payload bytes than frame length | extra bytes do not create extra hits for same frame | frame-length boundary |
| X036_long_new_header_inside_payload | F | Inject `K28.0` while still unpacking payload | parser treats it as data/corruption, not a nested frame | nonreentrant parser |
| X037_long_comma_inside_payload_mode0 | F | `MODE_HALT=0`, comma during payload | parser aborts frame | comma abort |
| X038_long_comma_inside_payload_mode1 | F | `MODE_HALT=1`, comma during payload | parser holds state rather than clean abort | comma hold |
| X039_long_valid_low_entire_malformed_frame | F | Keep `asi_rx8b1k_valid=0` for malformed long frame | current RTL still reacts to bytes; behavior is documented as risk | valid-ignored gap |

## 4. Malformed Short-Frame Negatives (`X040-X052`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X040_short_header_only_no_counters | F | Drive `K28.0` with short flags then stop | no false metadata or hits | incomplete frame handling |
| X041_short_missing_frame_number_byte1 | F | Header plus one frame-number byte only | no false metadata | incomplete frame handling |
| X042_short_missing_event_counter_msb | F | Omit event-counter MSB byte in short frame | no false short-mode start | incomplete frame handling |
| X043_short_missing_event_counter_lsb | F | Omit event-counter LSB byte in short frame | no false unpack start | incomplete frame handling |
| X044_short_declared_len1_missing_payload | F | Declare one short hit, omit payload bytes | no false `valid` or `eop` | truncation |
| X045_short_declared_len2_halfword_only | F | Declare two short hits, stop after incomplete even/odd pair | no false extra hit | truncation |
| X046_short_declared_len3_cut_in_unpack_extra | F | Stop in `FS_UNPACK_EXTRA` | saved nibble does not leak into next frame | truncation |
| X047_short_declared_len_large_cut_midpayload | F | Large short frame cut mid-stream | parser does not self-complete | truncation |
| X048_short_new_header_inside_payload | F | Inject `K28.0` while short payload is active | parser does not start nested frame | nonreentrant parser |
| X049_short_extra_payload_after_declared_len | F | Provide excess payload bytes beyond short length | no extra hits are created for same frame | frame-length boundary |
| X050_short_comma_inside_payload_mode0 | F | `MODE_HALT=0`, comma in short payload | frame aborts | comma abort |
| X051_short_comma_inside_payload_mode1 | F | `MODE_HALT=1`, comma in short payload | state holds rather than clean abort | comma hold |
| X052_short_valid_low_entire_malformed_frame | F | Keep `asi_rx8b1k_valid=0` for malformed short frame | current RTL still reacts to bytes; behavior is documented as risk | valid-ignored gap |

## 5. CRC Fault Injection Negatives (`X053-X065`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X053_long_bad_crc_onebit_flip | F | Flip one CRC-related byte in long frame | `error(1)` on final hit, counter increments once | CRC bad path |
| X054_long_bad_crc_twobit_flip | F | Flip two bytes in long frame | same single-frame error behavior | CRC bad path |
| X055_long_bad_crc_allzero_tail | F | Force CRC bytes to zero | single CRC error only | CRC bad path |
| X056_long_bad_crc_allones_tail | F | Force CRC bytes to all ones | single CRC error only | CRC bad path |
| X057_short_bad_crc_onebit_flip | F | Flip one CRC-related byte in short frame | `error(1)` on final hit, counter increments once | CRC bad path |
| X058_short_bad_crc_twobit_flip | F | Flip two bytes in short frame | same single-frame error behavior | CRC bad path |
| X059_short_bad_crc_allzero_tail | F | Force CRC bytes to zero in short frame | single CRC error only | CRC bad path |
| X060_short_bad_crc_allones_tail | F | Force CRC bytes to all ones in short frame | single CRC error only | CRC bad path |
| X061_clean_after_bad_crc_long | F | Bad long frame followed by clean long frame | clean frame clears `error(1)` | CRC nonstickiness |
| X062_clean_after_bad_crc_short | F | Bad short frame followed by clean short frame | clean frame clears `error(1)` | CRC nonstickiness |
| X063_multiple_bad_crc_reads_midupdate | F | Poll word1 while many bad frames occur | final count equals total bad frames despite read interference | CRC counter robustness |
| X064_bad_crc_during_terminating | F | Stop mid-frame and corrupt CRC | final drain frame still reports one CRC error | termination + CRC |
| X065_bad_crc_during_runprepare_reset | F | Assert `RUN_PREPARE` before bad frame completes | CRC counter resets instead of counting abandoned work | run-prepare reset precedence |

## 6. Byte-Error And Lane-Error Negatives (`X066-X078`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X066_long_parity_error_first_payload_byte | F | Assert parity error on first byte of long hit | emitted hit has `error(0)=1` | hit-error latch |
| X067_long_parity_error_last_payload_byte | F | Assert parity error on last byte of long hit | emitted hit has `error(0)=1` | hit-error latch |
| X068_long_decode_error_middle_payload_byte | F | Assert decode error mid long hit | emitted hit has `error(0)=1` | hit-error latch |
| X069_short_parity_error_even_hit | F | Assert parity error on even short hit | emitted hit has `error(0)=1` | hit-error latch |
| X070_short_parity_error_odd_hit | F | Assert parity error on odd short hit | emitted hit has `error(0)=1` | hit-error latch |
| X071_short_decode_error_unpack_extra | F | Assert decode error in `UNPACK_EXTRA` | affected committed hit has `error(0)=1` | hit-error latch |
| X072_hiterror_then_clean_hit_same_frame | F | Error on one hit, next hit clean in same frame | only erroneous hit is tagged | hit-error clear timing |
| X073_losssync_on_header_only | F | Assert loss-sync at frame start only | all hits in that frame observe `error(2)=1` | lane error span |
| X074_losssync_midpayload_only | F | Assert loss-sync during payload only | emitted hits while signal high show `error(2)=1` | lane error span |
| X075_losssync_on_crc_only | F | Assert loss-sync only during CRC phase | terminal hit still shows `error(2)` if sampled high | lane error span |
| X076_parity_and_decode_same_hit | F | Assert parity and decode on same hit | single `error(0)` is still set, no double-count artifact | error-bit collapse |
| X077_hiterror_plus_losssync_same_frame | F | Combine per-hit error and lane error | `error(0)` and `error(2)` are simultaneously meaningful | orthogonal error bits |
| X078_clean_frame_after_combined_errors | F | Combined-error frame followed by clean frame | all error bits clear on clean frame | error nonstickiness |

## 7. Comma / `MODE_HALT` Negative Space (`X079-X091`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X079_mode0_comma_in_frame_counter | F | `MODE_HALT=0`, comma in frame-number field | parser aborts current frame | mode0 abort |
| X080_mode0_comma_in_event_counter | F | `MODE_HALT=0`, comma in event-counter field | parser aborts current frame | mode0 abort |
| X081_mode0_comma_in_long_unpack | F | `MODE_HALT=0`, comma in long payload | parser aborts current frame | mode0 abort |
| X082_mode0_comma_in_short_unpack | F | `MODE_HALT=0`, comma in short payload | parser aborts current frame | mode0 abort |
| X083_mode0_abort_then_postabort_garbage | F | Abort via comma then drive random bytes | no spurious metadata/hits until next real header | abort recovery |
| X084_mode0_abort_then_immediate_header | F | Abort and present next header one cycle later | clean new frame can start from idle | abort recovery |
| X085_mode1_comma_in_frame_counter | F | `MODE_HALT=1`, comma in frame-number field | parser does not drop to idle | mode1 hold |
| X086_mode1_comma_in_event_counter | F | `MODE_HALT=1`, comma in event-counter field | parser does not drop to idle | mode1 hold |
| X087_mode1_comma_in_long_unpack | F | `MODE_HALT=1`, comma in long payload | parser does not drop to idle | mode1 hold |
| X088_mode1_comma_in_short_unpack | F | `MODE_HALT=1`, comma in short payload | parser does not drop to idle | mode1 hold |
| X089_mode1_hold_then_bad_crc | F | Hold through comma and also corrupt CRC | both behaviors remain observable without hang | mode1 mixed fault |
| X090_mode1_hold_then_runprepare | F | Hold through comma then issue `RUN_PREPARE` | reset wins cleanly | reset precedence |
| X091_mode_compare_same_corrupt_stream | D | Replay identical corrupt stream on mode0 and mode1 builds | only abort-vs-hold differs | generic contract split |

## 8. CSR / AVMM Misuse Negatives (`X092-X104`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X092_csr_read_unused_address_flood | F | Flood high unused addresses with reads | all return default zero, no state change | CSR default read path |
| X093_csr_write_unused_address_flood | F | Flood high unused addresses with writes | no parser state, counter, or control mutation | CSR default write path |
| X094_csr_read_write_same_cycle_overlap | F | Assert read and write together repeatedly | read-priority behavior remains deterministic | `if read elsif write` order |
| X095_csr_control_write_during_header_accept | F | Write control as header arrives | acceptance follows registered timing, no metastable behavior | registered enable |
| X096_csr_control_write_during_payload | F | Write control during unpack | current frame not corrupted | registered enable |
| X097_csr_poll_word2_every_cycle_open_frame | F | Read frame counter every cycle while frame open | no AVMM-induced parser deadlock | CSR robustness |
| X098_csr_poll_word1_every_cycle_badcrc | F | Read CRC counter every cycle during bad frames | counter converges to expected value | CSR robustness |
| X099_csr_mask_low_then_illegal_ctrl | F | Mask off parser and then send illegal control words | illegal control handling still works | independent control planes |
| X100_csr_mask_high_after_error_state | F | Recover from `ERROR` and rewrite control bit | parser only restarts after legal state decode | independent control planes |
| X101_waitrequest_expected_high_idle_negative | F | Assert testbench assumption that idle waitrequest is low | negative check proves current contract is high when idle | AVMM current behavior |
| X102_waitrequest_stuck_low_detection | F | Hold repeated accesses and monitor for stuck-low bug | interface returns high when idle again | AVMM robustness |
| X103_word0_status_read_before_headerinfo | F | Read word0 before metadata pulse completes | old status persists, no speculative update | status timing |
| X104_word0_status_read_after_runprepare | F | Read word0 immediately after run-prepare | status/counters reflect reset state | run-prepare reset timing |

## 9. Termination / Upgrade-Plan Negatives (`X105-X117`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X105_postterminate_fresh_frame_attempt_from_idle | F | Stop to `TERMINATING`, wait idle, then send new header | current DUT drops the fresh frame | known upgrade hole |
| X106_postterminate_header_same_cycle_as_ctrl_edge | F | Present terminate command and header together | result follows old-state decode and is waveform-documented | one-cycle lag |
| X107_stop_midframe_without_terminal_marker | F | Stop mid-frame and watch for extra terminal indication | no synthetic marker is generated | current-vs-target gap |
| X108_stop_midframe_ready_never_drops | F | Stop with work outstanding and sample `ready` | `ready` stays high despite drain in progress | current-vs-target gap |
| X109_stop_during_long_frame_then_second_header | F | Stop after first long hit, then offer second frame header | second frame is blocked once parser returns to idle | termination gap |
| X110_stop_during_short_frame_then_second_header | F | Stop after first short hit, then offer second frame header | second frame is blocked once parser returns to idle | termination gap |
| X111_stop_then_illegal_ctrl_then_idle | F | Stop, send illegal control, then recover to idle | current frame outcome is deterministic, recovery remains possible | termination + control error |
| X112_stop_then_runprepare_before_eop | F | Stop mid-frame and then assert `RUN_PREPARE` | reset flushes outstanding partial work | reset precedence |
| X113_stop_then_sync_without_idle | F | `TERMINATING` followed by `SYNC` with candidate headers | no fresh frame starts | blocked-state sequencing |
| X114_stop_then_running_reopen_window | F | `TERMINATING` followed later by `RUNNING` | new header starts only after legal reopen window | run-control recovery |
| X115_emulator_mismatch_repro_openframe_only | F | Drive bytes representing a frame committed before stop | current receiver drains only if header was already accepted | upgrade-plan mismatch |
| X116_emulator_mismatch_repro_postedge_header | F | Drive header only after stop edge | current receiver drops frame completely | upgrade-plan mismatch |
| X117_termination_gap_signature_regression | F | Canonical bug-repro bundle from upgrade plan | preserves a repeatable waveform/signature testcase for later RTL fixes | upgrade-plan evidence |

## 10. Generic / Build / Packaging Negatives (`X118-X130`)

| ID | Method | Negative scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| X118_cfgC_channel_width1_invalid_channel_drive | F | Build `CHANNEL_WIDTH=1`, attempt to drive wider symbolic channel set | harness constrains/flags illegal stimulus, DUT remains stable | `_hw.tcl` width contract |
| X119_cfgD_channel_width8_random_wide_values | F | Build `CHANNEL_WIDTH=8`, spray wide channel values during errors | no truncation beyond configured width | `_hw.tcl` width contract |
| X120_cfgE_addrwidth1_highaddr_alias_misuse | F | Build `CSR_ADDR_WIDTH=1`, issue software accesses above range | aliasing behavior is documented and deterministic | `_hw.tcl` width contract |
| X121_cfgF_addrwidth8_highaddr_abuse | F | Build `CSR_ADDR_WIDTH=8`, hammer high addresses during traffic | unused space remains harmless | `_hw.tcl` width contract |
| X122_cfgB_modehalt1_abort_assumption_negative | F | Reuse mode0 abort checker on mode1 build | negative check proves build-specific contract difference | `MODE_HALT` split |
| X123_cfgA_modehalt0_hold_assumption_negative | F | Reuse mode1 hold checker on mode0 build | negative check proves build-specific contract difference | `MODE_HALT` split |
| X124_cfgG_debuglv2_functional_delta_negative | F | Compare cfg A vs cfg G on identical traffic | no unintended functional delta appears | `DEBUG_LV` generation-only intent |
| X125_hw_tcl_readlatency_mismatch_negative | F | Force a zero-latency AVMM assumption in checker | negative check demonstrates `_hw.tcl readLatency=1` must be honored | packaging contract |
| X126_hw_tcl_maxchannel_mismatch_negative | F | Use stale maxChannel assumption after width rebuild | negative check demonstrates `myelaborate` owns maxChannel | packaging contract |
| X127_hw_tcl_ctrl_readylatency_assumption_negative | F | Assume control ready can backpressure | negative check demonstrates `readyLatency=0` and always-ready current RTL | packaging contract |
| X128_hw_tcl_headerinfo_channelwidth_mismatch | F | Build narrow/wide channel variants with stale monitor width | negative check demonstrates monitor must track elaborated width | packaging contract |
| X129_build_matrix_compile_smoke_failfast | F | Compile all planned config builds back-to-back | any generic/elaboration incompatibility fails fast before UVM work | packaging/generic closure |
| X130_known_gap_bundle_current_contract | F | Aggregate current-gap negatives: valid ignored, always-ready, no synthetic terminal marker | locks down current design limitations as explicit expected failures or expected-current behaviors | upgrade baseline |

**Row count:** 130 cases (`X001-X130`).
