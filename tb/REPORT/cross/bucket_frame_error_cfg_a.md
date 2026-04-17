# ❌ bucket_frame_error_cfg_a

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `ERROR` &nbsp; **Sequence:** `ERROR_CFG_A`

## Summary

<!-- field legend:
  case_count              = number of plan cases composed into this run
  effort                  = practical (capped per case) or extensive (full planned stress)
  iter_cap, payload_cap   = practical-mode budget caps
  txns                    = total transactions driven through the DUT in this run
  functional_cross_pct    = functional coverage against DV_CROSS.md (percent)
  queued_overlap          = transactions enqueued before the previous drained
  counter_checks_failed   = scoreboard counter mismatches observed (0 is required for pass)
  unexpected_outputs      = outputs the scoreboard did not predict
-->

| status | field | value |
|:---:|---|---|
| ℹ️ | case_count | `116` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `101` |
| ❌ | functional_cross_pct | `62.5` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ❌ | unexpected_outputs | `26` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 96.11 |
| branch | 90.07 |
| cond | 86.21 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 80.00 |
| toggle | 59.44 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `X014_ctrl_allzero_word` | `` | 34.38 | 11 | hit_eop |
| 2 | `X015_ctrl_twohot_idle_running` | `` | 34.38 | 0 | hit_eop |
| 3 | `X016_ctrl_twohot_running_terminating` | `` | 37.50 | 1 | hit_eop |
| 4 | `X017_ctrl_allones_word` | `` | 37.50 | 0 | hit_eop |
| 5 | `X018_ctrl_random_illegal_word_seed1` | `` | 40.62 | 1 | hit_eop |
| 6 | `X018_ctrl_random_illegal_word_seed1` | `` | 53.12 | 4 | hit_eop |
| 7 | `X018_ctrl_random_illegal_word_seed1` | `` | 56.25 | 1 | hit_eop |
| 8 | `X018_ctrl_random_illegal_word_seed1` | `` | 56.25 | 0 | hit_eop |
| 9 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 1 | hit_eop |
| 10 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 11 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 12 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 13 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 14 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 15 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 16 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 17 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 18 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 19 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 20 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 21 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 22 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 23 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 24 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 25 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 26 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 27 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 28 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 29 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 30 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 31 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 32 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 33 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 34 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 35 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 36 | `X018_ctrl_random_illegal_word_seed1` | `` | 59.38 | 0 | hit_eop |
| 37 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 38 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 39 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 40 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 41 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 42 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 43 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 44 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 45 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 46 | `X019_ctrl_random_illegal_word_seed2` | `` | 59.38 | 0 | hit_eop |
| 47 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 1 | hit_eop |
| 48 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 49 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 50 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 51 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 52 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 53 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 54 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 55 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 56 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 57 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 58 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 59 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 60 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 61 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 62 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 63 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 64 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 65 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 66 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 67 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 68 | `X019_ctrl_random_illegal_word_seed2` | `` | 62.50 | 0 | hit_eop |
| 69 | `X020_ctrl_valid_glitch_no_word_change` | `` | 62.50 | 0 | hit_eop |
| 70 | `X021_ctrl_payload_changes_while_valid_high` | `` | 62.50 | 0 | hit_eop |
| 71 | `X022_ctrl_illegal_then_legal_recovery_idle` | `` | 62.50 | 0 | hit_eop |
| 72 | `X023_ctrl_illegal_then_legal_recovery_running` | `` | 62.50 | 0 | hit_eop |
| 73 | `X024_ctrl_ready_not_indicating_recovery` | `` | 62.50 | 0 | hit_eop |
| 74 | `X025_ctrl_valid_low_payload_noise` | `` | 62.50 | 0 | hit_eop |
| 75 | `X026_ctrl_fast_oscillation_legal_illegal_mix` | `` | 62.50 | 0 | hit_eop |
| 76 | `X032_long_declared_len2_only_one_hit_present` | `` | 62.50 | 0 | hit_eop |
| 77 | `X034_long_bad_trailer_symbol_replaced_with_data` | `` | 62.50 | 0 | hit_eop |
| 78 | `X035_long_extra_payload_after_declared_len` | `` | 62.50 | 0 | hit_eop |
| 79 | `X036_long_new_header_inside_payload` | `` | 62.50 | 0 | hit_eop |
| 80 | `X039_long_valid_low_entire_malformed_frame` | `` | 62.50 | 0 | hit_eop |
| 81 | `X045_short_declared_len2_halfword_only` | `` | 62.50 | 0 | hit_eop |
| 82 | `X046_short_declared_len3_cut_in_unpack_extra` | `` | 62.50 | 0 | hit_eop |
| 83 | `X048_short_new_header_inside_payload` | `` | 62.50 | 0 | hit_eop |
| 84 | `X049_short_extra_payload_after_declared_len` | `` | 62.50 | 0 | hit_eop |
| 85 | `X052_short_valid_low_entire_malformed_frame` | `` | 62.50 | 0 | hit_eop |
| 86 | `X053_long_bad_crc_onebit_flip` | `` | 62.50 | 0 | hit_eop |
| 87 | `X054_long_bad_crc_twobit_flip` | `` | 62.50 | 0 | hit_eop |
| 88 | `X055_long_bad_crc_allzero_tail` | `` | 62.50 | 0 | hit_eop |
| 89 | `X056_long_bad_crc_allones_tail` | `` | 62.50 | 0 | hit_eop |
| 90 | `X057_short_bad_crc_onebit_flip` | `` | 62.50 | 0 | hit_eop |
| 91 | `X058_short_bad_crc_twobit_flip` | `` | 62.50 | 0 | hit_eop |
| 92 | `X059_short_bad_crc_allzero_tail` | `` | 62.50 | 0 | hit_eop |
| 93 | `X060_short_bad_crc_allones_tail` | `` | 62.50 | 0 | hit_eop |
| 94 | `X061_clean_after_bad_crc_long` | `` | 62.50 | 0 | hit_eop |
| 95 | `X062_clean_after_bad_crc_short` | `` | 62.50 | 0 | hit_eop |
| 96 | `X063_multiple_bad_crc_reads_midupdate` | `` | 62.50 | 0 | hit_eop |
| 97 | `X064_bad_crc_during_terminating` | `` | 62.50 | 0 | hit_eop |
| 98 | `X066_long_parity_error_first_payload_byte` | `` | 62.50 | 0 | hit_eop |
| 99 | `X067_long_parity_error_last_payload_byte` | `` | 62.50 | 0 | hit_eop |
| 100 | `X068_long_decode_error_middle_payload_byte` | `` | 62.50 | 0 | hit_eop |
| 101 | `X069_short_parity_error_even_hit` | `` | 62.50 | 0 | hit_eop |

## Issue reference

| status | field | value |
|:---:|---|---|
| ℹ️ | bug_ref | [FRCV-2026-04-17-003](../../BUG_HISTORY.md) |
| ❌ | rerun_date | `2026-04-17` |
| ❌ | rerun_verdict | Rerun reproduced unexpected_outputs=26 with counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. |
| ℹ️ | rerun_log | [uvm/logs/bucket_frame_error_cfg_a_after_s1.log](../../uvm/logs/bucket_frame_error_cfg_a_after_s1.log) |

---
_Back to [dashboard](../../DV_REPORT.md)_
