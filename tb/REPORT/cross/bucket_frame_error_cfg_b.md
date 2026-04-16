# ⚠️ bucket_frame_error_cfg_b

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_B` &nbsp; **Bucket:** `ERROR` &nbsp; **Sequence:** `ERROR_CFG_B`

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
| ℹ️ | case_count | `9` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `9` |
| ⚠️ | functional_cross_pct | `46.88` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 85.16 |
| branch | 71.52 |
| cond | 62.07 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 60.00 |
| toggle | 33.32 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `X038_long_comma_inside_payload_mode1` | `` | 34.38 | 11 | hit_eop |
| 2 | `X051_short_comma_inside_payload_mode1` | `` | 46.88 | 4 | hit_eop |
| 3 | `X085_mode1_comma_in_frame_counter` | `` | 46.88 | 0 | hit_eop |
| 4 | `X086_mode1_comma_in_event_counter` | `` | 46.88 | 0 | hit_eop |
| 5 | `X087_mode1_comma_in_long_unpack` | `` | 46.88 | 0 | hit_eop |
| 6 | `X088_mode1_comma_in_short_unpack` | `` | 46.88 | 0 | hit_eop |
| 7 | `X089_mode1_hold_then_bad_crc` | `` | 46.88 | 0 | hit_eop |
| 8 | `X090_mode1_hold_then_runprepare` | `` | 46.88 | 0 | hit_eop |
| 9 | `X122_cfgB_modehalt1_abort_assumption_negative` | `` | 46.88 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
