# ✅ cross_interleave_mix_cfg_a

**Kind:** `cross` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `INTERLEAVE_MIX`

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
| ℹ️ | txns | `8` |
| ✅ | functional_cross_pct | `50.0` |
| ℹ️ | queued_overlap | `8` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 89.88 |
| branch | 82.12 |
| cond | 86.21 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 66.67 |
| toggle | 47.24 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P097_long_clean_short_bad_repeat` | `INTERLEAVE_MIX` | 37.50 | 12 | hit_eop |
| 2 | `P116_stop_under_losssync_mix` | `INTERLEAVE_MIX` | 40.62 | 1 | hit_eop |
| 3 | `P098_short_clean_long_bad_repeat` | `INTERLEAVE_MIX` | 50.00 | 3 | hit_eop |
| 4 | `X077_hiterror_plus_losssync_same_frame` | `INTERLEAVE_MIX` | 50.00 | 0 | hit_eop |
| 5 | `P097_long_clean_short_bad_repeat` | `INTERLEAVE_MIX` | 50.00 | 0 | hit_eop |
| 6 | `P116_stop_under_losssync_mix` | `INTERLEAVE_MIX` | 50.00 | 0 | hit_eop |
| 7 | `P098_short_clean_long_bad_repeat` | `INTERLEAVE_MIX` | 50.00 | 0 | hit_eop |
| 8 | `X077_hiterror_plus_losssync_same_frame` | `INTERLEAVE_MIX` | 50.00 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
