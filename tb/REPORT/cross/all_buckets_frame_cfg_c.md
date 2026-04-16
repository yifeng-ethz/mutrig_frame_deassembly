# ⚠️ all_buckets_frame_cfg_c

**Kind:** `all_buckets_frame` &nbsp; **Build:** `CFG_C` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `ALL_CFG_C`

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
| ℹ️ | case_count | `6` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `36` |
| ⚠️ | functional_cross_pct | `40.62` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.16 |
| branch | 66.23 |
| cond | 62.07 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 40.00 |
| toggle | 50.58 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `E094_channel_width1_headerinfo_maxchannel1` | `` | 34.38 | 11 | hit_eop |
| 2 | `E095_channel_width1_output_channel_singlebit` | `` | 37.50 | 1 | hit_eop |
| 3 | `E096_channel_width4_default_maxchannel15` | `` | 37.50 | 0 | hit_eop |
| 4 | `P083_cfgC_channel1_alternating_soak` | `` | 37.50 | 0 | hit_eop |
| 5 | `P083_cfgC_channel1_alternating_soak` | `` | 37.50 | 0 | hit_eop |
| 6 | `P083_cfgC_channel1_alternating_soak` | `` | 37.50 | 0 | hit_eop |
| 7 | `P083_cfgC_channel1_alternating_soak` | `` | 37.50 | 0 | hit_eop |
| 8 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 1 | hit_eop |
| 9 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 10 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 11 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 12 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 13 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 14 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 15 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 16 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 17 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 18 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 19 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 20 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 21 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 22 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 23 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 24 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 25 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 26 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 27 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 28 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 29 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 30 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 31 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 32 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 33 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 34 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 35 | `P083_cfgC_channel1_alternating_soak` | `` | 40.62 | 0 | hit_eop |
| 36 | `P088_cfgC_termination_soak` | `` | 40.62 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
