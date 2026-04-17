# ✅ bucket_frame_error_cfg_a

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
| ℹ️ | txns | `76` |
| ✅ | functional_cross_pct | `78.12` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

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
| 1 | `X004_reset_in_long_unpack` | `` | 34.38 | 11 | header_only_or_zero_hit |
| 2 | `X005_reset_in_short_unpack` | `` | 43.75 | 3 | header_only_or_zero_hit |
| 3 | `X006_reset_in_unpack_extra` | `` | 46.88 | 1 | header_only_or_zero_hit |
| 4 | `X007_reset_in_crc_calc` | `` | 53.12 | 2 | hit_eop |
| 5 | `X008_reset_in_crc_check` | `` | 56.25 | 1 | hit_eop |
| 6 | `X009_reset_during_bad_crc_frame` | `` | 56.25 | 0 | hit_eop |
| 7 | `X010_reset_during_hit_error_frame` | `` | 59.38 | 1 | header_only_or_zero_hit |
| 8 | `X010_reset_during_hit_error_frame` | `` | 62.50 | 1 | hit_eop |
| 9 | `X011_reset_during_losssync_frame` | `` | 62.50 | 0 | header_only_or_zero_hit |
| 10 | `X011_reset_during_losssync_frame` | `` | 62.50 | 0 | hit_eop |
| 11 | `X021_ctrl_payload_changes_while_valid_high` | `` | 65.62 | 1 | hit_eop |
| 12 | `X022_ctrl_illegal_then_legal_recovery_idle` | `` | 65.62 | 0 | hit_eop |
| 13 | `X023_ctrl_illegal_then_legal_recovery_running` | `` | 65.62 | 0 | hit_eop |
| 14 | `X025_ctrl_valid_low_payload_noise` | `` | 65.62 | 0 | hit_eop |
| 15 | `X026_ctrl_fast_oscillation_legal_illegal_mix` | `` | 65.62 | 0 | hit_eop |
| 16 | `X031_long_declared_len1_missing_payload` | `` | 68.75 | 1 | header_only_or_zero_hit |
| 18 | `X033_long_declared_len_large_cut_midpayload` | `` | 71.88 | 1 | header_only_or_zero_hit |
| 22 | `X037_long_comma_inside_payload_mode0` | `` | 75.00 | 1 | header_only_or_zero_hit |
| 32 | `X056_long_bad_crc_allones_tail` | `` | 75.00 | 0 | hit_eop |
| 40 | `X064_bad_crc_during_terminating` | `` | 78.12 | 1 | hit_eop |
| 64 | `X100_csr_mask_high_after_error_state` | `` | 78.12 | 0 | hit_eop |

## Issue reference

| status | field | value |
|:---:|---|---|
| ℹ️ | bug_ref | [FRCV-2026-04-17-003](../../BUG_HISTORY.md) |
| ✅ | rerun_date | `2026-04-18` |
| ✅ | rerun_verdict | Before fix on 2026-04-17 this run reproduced unexpected_outputs=26 with counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. After fix on 2026-04-18 the rerun is clean: unexpected_outputs=0, counter_checks_failed=0, UVM_ERROR=0, and UVM_FATAL=0. |
| ℹ️ | rerun_log | [uvm/logs/bucket_frame_error_cfg_a_after_s1.log](../../uvm/logs/bucket_frame_error_cfg_a_after_s1.log) |

---
_Back to [dashboard](../../DV_REPORT.md)_
