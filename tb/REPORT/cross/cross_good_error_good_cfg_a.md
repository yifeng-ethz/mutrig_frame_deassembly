# ✅ cross_good_error_good_cfg_a

**Kind:** `cross` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `GOOD_ERROR_GOOD`

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
| ℹ️ | case_count | `0` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `None` |
| ℹ️ | payload_cap | `None` |
| ℹ️ | txns | `15` |
| ✅ | functional_cross_pct | `53.12` |
| ℹ️ | queued_overlap | `14` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 84.82 |
| branch | 70.86 |
| cond | 65.52 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 53.33 |
| toggle | 45.46 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `B041_long_one_hit_sop_and_eop_same_transfer` | `GOOD_ERROR_GOOD` | 37.50 | 12 | hit_eop |
| 2 | `X066_long_parity_error_first_payload_byte` | `GOOD_ERROR_GOOD` | 43.75 | 2 | hit_eop |
| 3 | `B127_good_long_then_good_short_sequence` | `GOOD_ERROR_GOOD` | 43.75 | 0 | hit_eop |
| 4 | `B041_long_one_hit_sop_and_eop_same_transfer` | `GOOD_ERROR_GOOD` | 43.75 | 0 | hit_eop |
| 5 | `X066_long_parity_error_first_payload_byte` | `GOOD_ERROR_GOOD` | 43.75 | 0 | hit_eop |
| 6 | `B127_good_long_then_good_short_sequence` | `GOOD_ERROR_GOOD` | 53.12 | 3 | hit_eop |
| 7 | `B041_long_one_hit_sop_and_eop_same_transfer` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 8 | `X066_long_parity_error_first_payload_byte` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 9 | `B127_good_long_then_good_short_sequence` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 10 | `B041_long_one_hit_sop_and_eop_same_transfer` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 11 | `X066_long_parity_error_first_payload_byte` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 12 | `B127_good_long_then_good_short_sequence` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 13 | `B041_long_one_hit_sop_and_eop_same_transfer` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 14 | `X066_long_parity_error_first_payload_byte` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |
| 15 | `B127_good_long_then_good_short_sequence` | `GOOD_ERROR_GOOD` | 53.12 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
