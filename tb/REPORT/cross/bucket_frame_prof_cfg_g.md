# ⚠️ bucket_frame_prof_cfg_g

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_G` &nbsp; **Bucket:** `PROF` &nbsp; **Sequence:** `PROF_CFG_G`

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
| ℹ️ | txns | `16` |
| ⚠️ | functional_cross_pct | `37.5` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.16 |
| branch | 64.24 |
| cond | 58.62 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 40.00 |
| toggle | 42.39 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P087_cfgG_debuglv2_functional_soak` | `` | 34.38 | 11 | hit_eop |
| 2 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 1 | hit_eop |
| 3 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 4 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 5 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 6 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 7 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 8 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 9 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 10 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 11 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 12 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 13 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 14 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 15 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |
| 16 | `P087_cfgG_debuglv2_functional_soak` | `` | 37.50 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
