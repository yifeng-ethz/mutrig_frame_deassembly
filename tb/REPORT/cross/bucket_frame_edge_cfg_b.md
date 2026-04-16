# ⚠️ bucket_frame_edge_cfg_b

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_B` &nbsp; **Bucket:** `EDGE` &nbsp; **Sequence:** `EDGE_CFG_B`

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
| ℹ️ | case_count | `5` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `5` |
| ⚠️ | functional_cross_pct | `37.5` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.86 |
| branch | 65.56 |
| cond | 55.17 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 40.00 |
| toggle | 22.73 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header` | `` | 34.38 | 11 | hit_eop |
| 2 | `E084_mode_halt1_comma_on_frame_counter_byte_holds_state` | `` | 37.50 | 1 | hit_eop |
| 3 | `E085_mode_halt1_comma_on_event_counter_byte_holds_state` | `` | 37.50 | 0 | hit_eop |
| 4 | `E086_mode_halt1_comma_on_unpack_byte_holds_state` | `` | 37.50 | 0 | hit_eop |
| 5 | `E088_mode1_resume_with_error_bits_preserved` | `` | 37.50 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
