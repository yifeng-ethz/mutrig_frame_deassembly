# DV Performance / Stress Cases — mutrig_frame_deassembly

**Companion docs:** [DV_PLAN.md](DV_PLAN.md), [DV_HARNESS.md](DV_HARNESS.md), [DV_CROSS.md](DV_CROSS.md)

**ID Range:** `P001-P130`  
**Total:** 130 cases  
**Purpose:** sustained traffic, soak, sweep, and long-run stability coverage for the current `frame_rcv_ip` contract

## 1. Long-Frame Length Sweep (`P001-P013`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P001_long_len0_10k_frames | S | 10k zero-hit long frames | no hit output, one metadata pulse per frame, no deadlock | long zero-hit steady state |
| P002_long_len1_10k_frames | S | 10k one-hit long frames | stable `sop/eop` coincidence, counters remain coherent | long min nonzero steady state |
| P003_long_len2_10k_frames | S | 10k two-hit long frames | first/last hit boundaries never slip | long two-word steady state |
| P004_long_len3_10k_frames | S | 10k three-hit long frames | no off-by-one in `n_word_cnt` | long small-count stress |
| P005_long_len7_5k_frames | S | 5k seven-hit long frames | sustained unpack throughput is stable | long unpack cadence |
| P006_long_len15_2k_frames | S | 2k fifteen-hit long frames | counters and metadata remain aligned | long medium size |
| P007_long_len31_1k_frames | S | 1k thirty-one-hit long frames | no parser-state drift | long medium size |
| P008_long_len63_512_frames | S | 512 sixty-three-hit long frames | CRC and EOP remain aligned at scale | long medium size |
| P009_long_len127_256_frames | S | 256 127-hit long frames | stable hit emission over long payloads | long large size |
| P010_long_len255_128_frames | S | 128 255-hit long frames | no counter rollover or timeout | long large size |
| P011_long_len511_64_frames | S | 64 511-hit long frames | sustained large-frame drain remains clean | long large size |
| P012_long_len767_32_frames | S | 32 767-hit long frames | no late CRC or word-count corruption | long near-max size |
| P013_long_len1023_16_frames | S | 16 max-length long frames | parser survives full 10-bit frame-length range | long max size boundary |

## 2. Short-Frame Length Sweep (`P014-P026`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P014_short_len0_10k_frames | S | 10k zero-hit short frames | metadata-only path stays stable | short zero-hit steady state |
| P015_short_len1_10k_frames | S | 10k one-hit short frames | one-hit `sop/eop` remains stable | short min nonzero steady state |
| P016_short_len2_10k_frames | S | 10k two-hit short frames | `UNPACK/UNPACK_EXTRA` alternation stays aligned | short even-count stress |
| P017_short_len3_10k_frames | S | 10k three-hit short frames | odd residual nibble path remains stable | short odd-count stress |
| P018_short_len7_5k_frames | S | 5k seven-hit short frames | no nibble carry corruption | short medium size |
| P019_short_len15_2k_frames | S | 2k fifteen-hit short frames | stable short-word pacing | short medium size |
| P020_short_len31_1k_frames | S | 1k thirty-one-hit short frames | metadata and hit count remain aligned | short medium size |
| P021_short_len63_512_frames | S | 512 sixty-three-hit short frames | no odd/even phase drift | short large size |
| P022_short_len127_256_frames | S | 256 127-hit short frames | no short-packer deadlock | short large size |
| P023_short_len255_128_frames | S | 128 255-hit short frames | no CRC/EOP slip at high occupancy | short large size |
| P024_short_len511_64_frames | S | 64 511-hit short frames | stable short unpack over long runs | short large size |
| P025_short_len767_32_frames | S | 32 767-hit short frames | no nibble corruption or counter drift | short near-max size |
| P026_short_len1023_16_frames | S | 16 max-length short frames | parser survives full 10-bit short length range | short max size boundary |

## 3. System-Cadence Dwell Sweep (`P027-P039`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P027_cadence_1_1_16_1 | S | `RUN_PREPARE=1`, `SYNC=1`, `RUNNING=16`, `TERMINATING=1`, repeat 1k times | no deadlock, first frame always starts after running window | `feb_system_v2` style spine |
| P028_cadence_1_1_64_1 | S | same cadence with 64-cycle running window | parser opens/closes deterministically | system cadence |
| P029_cadence_2_2_64_2 | S | 2-cycle prepare/sync and 2-cycle terminate dwell | control lag remains stable | control timing |
| P030_cadence_4_4_128_4 | S | longer control dwells | no stuck state across transitions | control timing |
| P031_cadence_8_8_256_8 | S | medium soak cadence | counters and outputs stay coherent | control timing |
| P032_cadence_16_16_512_16 | K | long dwell cadence | no missed first frame after long prepare/sync | control timing |
| P033_cadence_running_only_1024 | S | stay in `RUNNING` for 1024 cycles bursts before stop | sustained continuous parsing remains stable | running-only stress |
| P034_cadence_idle_monitoring_only | S | remain in `IDLE` for an extended cadence run without reopening `RUNNING` | parser stays blocked and quiescent for the whole run | quiescent `IDLE` contract |
| P035_cadence_stop_after_every_frame | S | stop after each accepted frame | exactly one frame drains per cycle | stop/start churn |
| P036_cadence_stop_every_other_frame | S | alternate one running frame and one blocked interval | no stale header acceptance leaks across states | state transitions |
| P037_cadence_legal_and_illegal_ctrl_mix | S | interleave legal commands with illegal words | recovery to legal states remains deterministic | control decode robustness |
| P038_cadence_runprepare_burst_reset | S | frequent `RUN_PREPARE` pulses during long regressions | parser and counters always reinitialize cleanly | run-prepare reset |
| P039_cadence_terminating_hold_open_frame | S | repeatedly stop mid-frame during varying payload phases | open frame always drains once, no fresh post-stop header | termination baseline |

## 4. CSR-Mask And Counter Stress (`P040-P052`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P040_mask_toggle_every_frame | S | flip `csr.control(0)` between consecutive frames | acceptance follows registered mask with no stray starts | `proc_enable_ctrl` |
| P041_mask_toggle_every_2_frames | S | flip mask every 2 frames | frame grouping matches mask windows | CSR gating |
| P042_mask_toggle_every_4_frames | S | flip mask every 4 frames | no counter drift across windows | CSR gating |
| P043_mask_toggle_every_8_frames | S | flip mask every 8 frames | no stale enable value after many cycles | CSR gating |
| P044_mask_toggle_every_16_frames | S | flip mask every 16 frames | long-running mask control stable | CSR gating |
| P045_mask_low_long_idle_soak | K | hold mask low in `RUNNING` for 100k cycles | no fresh headers accepted for whole soak | masked running contract |
| P046_mask_high_long_running_soak | K | hold mask high in `RUNNING` for 100k cycles with clean frames | no lost headers due to stale enable | masked running contract |
| P047_midframe_mask_drop_repeated | S | drop mask mid-frame for 1k frames | current open frame always finishes | `enable` only consumed in idle |
| P048_midframe_mask_raise_repeated | S | raise mask mid-frame after previous blocked window | no mid-frame spurious header | registered enable |
| P049_counter_poll_during_open_frame | S | poll word2 during every frame at fixed offsets | `head-tail` semantics remain monotonic | frame-counter read timing |
| P050_counter_poll_during_bad_crc_mix | S | poll word1 while injecting periodic bad CRC frames | eventual count equals observed bad-frame total | CRC counter behavior |
| P051_status_poll_after_every_headerinfo | S | read word0 after every metadata pulse for 10k frames | status always matches most recent flags | CSR status timing |
| P052_control_write_soak_no_counter_side_effect | K | repeatedly write control register while parsing clean frames | writes do not corrupt counters or metadata | CSR side-effect boundary |

## 5. CRC Mix And Error-Rate Stress (`P053-P065`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P053_all_clean_long_crc | K | 50k clean long frames | `crc_err_counter` remains zero | good CRC baseline |
| P054_all_clean_short_crc | K | 50k clean short frames | `crc_err_counter` remains zero | good CRC baseline |
| P055_bad_every_32nd_frame | S | inject bad CRC every 32nd frame | counter increments exactly on schedule | bad-frame sparsity |
| P056_bad_every_16th_frame | S | inject bad CRC every 16th frame | counter increments exactly on schedule | bad-frame sparsity |
| P057_bad_every_8th_frame | S | inject bad CRC every 8th frame | counter increments exactly on schedule | bad-frame sparsity |
| P058_bad_every_4th_frame | S | inject bad CRC every 4th frame | counter increments exactly on schedule | bad-frame density |
| P059_bad_every_other_frame | S | alternate good and bad frames | no CRC state leakage across frames | bad-frame density |
| P060_all_bad_long_crc | S | 10k bad long frames | every terminal hit reports `error(1)` | persistent bad CRC |
| P061_all_bad_short_crc | S | 10k bad short frames | every terminal hit reports `error(1)` | persistent bad CRC |
| P062_mixed_long_short_bad_ratio10 | S | 10% bad frames across mixed modes | counter matches expected bad total | shared CRC counter |
| P063_mixed_long_short_bad_ratio50 | S | 50% bad frames across mixed modes | no counter miscount at high density | shared CRC counter |
| P064_bad_crc_plus_loss_sync_mix | S | combine bad CRC with lane loss-sync on some frames | `error(1)` and `error(2)` remain independently correct | error-bit orthogonality |
| P065_bad_crc_plus_hit_error_mix | S | combine bad CRC with parity/decode hit errors | terminal hit and per-hit error sources stay separable | error-bit orthogonality |

## 6. `MODE_HALT` Comma Stress (`P066-P078`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P066_mode0_comma_every_64th_frame_counter | S | `MODE_HALT=0`, comma at frame-counter byte every 64th frame | parser aborts exactly on injected frames | mode0 abort |
| P067_mode0_comma_every_64th_event_counter | S | `MODE_HALT=0`, comma at event-counter byte every 64th frame | parser aborts exactly on injected frames | mode0 abort |
| P068_mode0_comma_every_64th_unpack | S | `MODE_HALT=0`, comma at payload byte every 64th frame | parser aborts exactly on injected frames | mode0 abort |
| P069_mode0_comma_random_positions | R | `MODE_HALT=0`, random comma positions in 10k frames | parser never wedges after abort | mode0 recovery |
| P070_mode0_abort_then_clean_restart | S | alternating corrupt and clean frames | clean restarts always work | mode0 recovery |
| P071_mode1_comma_every_64th_frame_counter | S | `MODE_HALT=1`, comma at frame-counter byte every 64th frame | parser holds state and continues current frame contract | mode1 hold |
| P072_mode1_comma_every_64th_event_counter | S | `MODE_HALT=1`, comma at event-counter byte every 64th frame | no unintended reset to idle | mode1 hold |
| P073_mode1_comma_every_64th_unpack | S | `MODE_HALT=1`, comma at payload byte every 64th frame | no unintended reset to idle | mode1 hold |
| P074_mode1_comma_random_positions | R | `MODE_HALT=1`, random comma positions in 10k frames | parser never deadlocks | mode1 hold |
| P075_mode1_hold_with_loss_sync_mix | S | combine commas and loss-sync in mode1 | error2 behavior remains coherent | mode1 + error mix |
| P076_mode0_abort_with_loss_sync_mix | S | combine commas and loss-sync in mode0 | abort semantics still win | mode0 + error mix |
| P077_mode0_to_mode1_build_comparison | D | same corrupted stream on cfg A vs cfg B | only abort-vs-hold behavior changes | generic-only behavior delta |
| P078_mode1_to_mode0_build_comparison | D | same corrupted stream on cfg B vs cfg A | no other functional deltas appear | generic-only behavior delta |

## 7. Generic And Packaging Regression Matrix (`P079-P091`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P079_cfgA_default_long_soak | K | `CFG_A`, 20k clean long frames | packaged default stays stable | `_hw.tcl` default config |
| P080_cfgA_default_short_soak | K | `CFG_A`, 20k clean short frames | packaged default stays stable | `_hw.tcl` default config |
| P081_cfgB_modehalt1_clean_soak | K | `CFG_B`, clean mixed traffic | `MODE_HALT=1` build matches default on clean traffic | generic build matrix |
| P082_cfgB_modehalt1_comma_soak | K | `CFG_B`, periodic commas | hold behavior remains stable over long run | generic build matrix |
| P083_cfgC_channel1_alternating_soak | K | `CFG_C`, alternate channel `0/1` | width-1 build stays coherent | `CHANNEL_WIDTH=1` |
| P084_cfgD_channel8_wide_sweep | K | `CFG_D`, sweep wide channel values | width-8 build keeps full channel information | `CHANNEL_WIDTH=8` |
| P085_cfgE_addr1_poll_soak | K | `CFG_E`, heavy CSR polling | narrow address build remains stable | `CSR_ADDR_WIDTH=1` |
| P086_cfgF_addr8_unused_space_sweep | K | `CFG_F`, random high-address reads | wide address build keeps unused space harmless | `CSR_ADDR_WIDTH=8` |
| P087_cfgG_debuglv2_functional_soak | K | `CFG_G`, default traffic | `DEBUG_LV=2` functional behavior matches cfg A | generation parameter |
| P088_cfgC_termination_soak | K | `CFG_C`, repeated stop mid-frame | min-width channel build still honors stop contract | channel generic + termination |
| P089_cfgD_zerohit_onehit_mix | K | `CFG_D`, zero/one-hit alternation | wide-channel build preserves metadata/hit alignment | channel generic + parser |
| P090_cfgE_control_churn_soak | K | `CFG_E`, rapid run-control cycling | narrow-address build survives heavy control churn | addr generic + run-control |
| P091_crossbuild_reference_trace_equivalence | D | replay same clean trace on cfg A/B/G | outputs identical where generics should not affect function | packaging equivalence |

## 8. Mixed Mode / Channel / Error Alternation Stress (`P092-P104`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P092_long_short_alternate_every_frame | S | alternate long and short clean frames for 20k frames | no stale mode state | mode switching |
| P093_long_long_short_short_repeat | S | repeat LLSS pattern | no mode decode drift | mode switching |
| P094_zerohit_to_fullhit_alternate | S | alternate zero-hit and high-hit-count frames | metadata and counters remain aligned | hit-count swing |
| P095_channel_rotate_0_to_15 | S | rotate ingress channel every frame | metadata and hit channel track channel changes | channel passthrough |
| P096_channel_rotate_sparse | S | use nonconsecutive channels only | no hidden dependence on contiguous channels | channel passthrough |
| P097_long_clean_short_bad_repeat | S | alternate clean long and bad short frames | error1 isolated to bad short frames | mixed-mode error isolation |
| P098_short_clean_long_bad_repeat | S | alternate clean short and bad long frames | error1 isolated to bad long frames | mixed-mode error isolation |
| P099_long_hiterror_short_clean_repeat | S | alternate hit-error long and clean short frames | error0 does not leak across frames | mixed-mode error isolation |
| P100_short_hiterror_long_clean_repeat | S | alternate hit-error short and clean long frames | error0 does not leak across frames | mixed-mode error isolation |
| P101_losssync_burst_every_100_frames | S | inject loss-sync burst every 100th frame | error2 appears only on affected frames | loss-sync burst stress |
| P102_parity_burst_every_100_frames | S | inject parity error burst every 100th frame | error0 appears only on affected hits | parity burst stress |
| P103_decode_burst_every_100_frames | S | inject decode error burst every 100th frame | error0 appears only on affected hits | decode burst stress |
| P104_metadata_poll_under_mode_mix | S | read status/word counts while alternating modes | CSR metadata remains coherent under mixed traffic | status and headerinfo stability |

## 9. Termination Stress Sweep (`P105-P117`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P105_stop_before_header_every_run | S | in each run, stop before any header arrives | zero frames emitted | stop-before-start contract |
| P106_stop_on_header_start_every_run | S | stop just after header accept | opened frame completes if bytes continue | open-frame drain |
| P107_stop_after_event_counter_every_run | S | stop after frame length captured | frame completes, no next frame starts | open-frame drain |
| P108_stop_after_first_long_hit_every_run | S | stop after first long hit in large frame | remaining in-frame hits complete | long drain |
| P109_stop_after_first_short_hit_every_run | S | stop after first short hit in large frame | remaining in-frame hits complete | short drain |
| P110_stop_during_crc_calc_every_run | S | stop during CRC-byte phase | exactly one `eop` still occurs | drain at CRC stage |
| P111_stop_during_crc_check_every_run | S | stop during compare cycle | completion remains single-shot | drain at CRC stage |
| P112_stop_then_immediate_idle_every_run | S | `TERMINATING` immediately followed by `IDLE` | no fresh frame starts until back in monitoring window | stop-to-idle boundary |
| P113_stop_every_other_frame_mixed_modes | S | alternating stop points across long/short modes | contract holds in both modes | termination x mode |
| P114_stop_under_bad_crc_mix | S | stop frames that will also fail CRC | drain behavior remains coherent with error1 | termination + CRC |
| P115_stop_under_hiterror_mix | S | stop frames carrying hit errors | drain behavior remains coherent with error0 | termination + hit error |
| P116_stop_under_losssync_mix | S | stop frames carrying loss-sync | drain behavior remains coherent with error2 | termination + lane error |
| P117_stop_soak_10k_cycles_no_fresh_poststop_header | K | thousands of stop events with candidate post-stop headers | no fresh header after terminate from idle | upgrade-plan key proof |

## 10. Long Soak / Seeded Regression Anchors (`P118-P130`)

| ID | Method | Stress scenario | Primary checks | Contract anchor |
|---|---|---|---|---|
| P118_seed01_clean_mix_100k_cycles | K | seed 1 randomized clean long/short mix for 100k cycles | no deadlock, counters and outputs coherent | long soak anchor |
| P119_seed02_clean_mix_100k_cycles | K | seed 2 randomized clean mix | same closure goals | long soak anchor |
| P120_seed03_error_mix_100k_cycles | K | seed 3 random parity/decode/loss-sync injections | no assertion or scoreboard drift | long soak anchor |
| P121_seed04_stop_mix_100k_cycles | K | seed 4 random stop points | no fresh post-stop header | long soak anchor |
| P122_seed05_mask_mix_100k_cycles | K | seed 5 random CSR mask toggles | no stale-enable bug | long soak anchor |
| P123_seed06_mode_mix_100k_cycles | K | seed 6 random mode changes between frames | no stale mode bug | long soak anchor |
| P124_seed07_channel_mix_100k_cycles | K | seed 7 random channel changes between frames | metadata/hit channel coherence | long soak anchor |
| P125_seed08_badcrc_mix_100k_cycles | K | seed 8 random CRC corruption schedule | CRC count exactness | long soak anchor |
| P126_seed09_comma_mix_mode0_100k_cycles | K | seed 9 random comma injections in `MODE_HALT=0` | no wedge after aborts | long soak anchor |
| P127_seed10_comma_mix_mode1_100k_cycles | K | seed 10 random comma injections in `MODE_HALT=1` | no wedge after holds | long soak anchor |
| P128_seed11_cfgD_wide_channel_100k_cycles | K | seed 11 on wide-channel build | wide-channel functional stability | generic soak anchor |
| P129_seed12_cfgE_narrow_addr_100k_cycles | K | seed 12 on narrow-address build | CSR functional stability | generic soak anchor |
| P130_seed13_upgrade_gap_observation_bundle | K | seed 13 with randomized stop plus candidate post-stop headers | reproduces and freezes current termination limitation | upgrade-plan evidence anchor |

**Row count:** 130 cases (`P001-P130`).

## 11. Supplemental Long-Run Requirement

- `P118-P130` remain the deterministic seed anchors used to reproduce known long-run patterns.
- Long-run closure must also include randomized all-bucket sequences launched through `tb/scripts/run_full_random_parallel.py`.
- That runner samples documented cases from BASIC, EDGE, PROF, and ERROR with replacement, executes them in continuous `all_buckets_frame` mode, and is intended to run for tens of minutes per worker in parallel.
- Any new long-run scoreboarding drift or unexpected-output failures found there must update `BUG_HISTORY.md`, `DV_REPORT.json`, and the generated `REPORT/cross/` evidence.
