# ⚠️ bucket_frame_basic_cfg_b

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_B` &nbsp; **Bucket:** `BASIC` &nbsp; **Sequence:** `BASIC_CFG_B`

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
| ℹ️ | case_count | `1` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `1` |
| ⚠️ | functional_cross_pct | `34.38` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 77.34 |
| branch | 62.25 |
| cond | 48.28 |
| expr | 98.77 |
| fsm_state | 71.43 |
| fsm_trans | 26.67 |
| toggle | 16.79 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `B095_mode_halt1_comma_inside_frame_holds_state` | `` | 34.38 | 11 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
