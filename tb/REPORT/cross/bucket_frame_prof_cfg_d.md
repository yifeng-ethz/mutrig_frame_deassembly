# ✅ bucket_frame_prof_cfg_d

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_D` &nbsp; **Bucket:** `PROF` &nbsp; **Sequence:** `PROF_CFG_D`

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
| ℹ️ | case_count | `3` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `18` |
| ✅ | functional_cross_pct | `59.38` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.16 |
| branch | 66.23 |
| cond | 58.62 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 40.00 |
| toggle | 45.98 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P084_cfgD_channel8_wide_sweep` | `` | 34.38 | 11 | hit_eop |
| 2 | `P089_cfgD_zerohit_onehit_mix` | `` | 37.50 | 1 | hit_eop |
| 3 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 40.62 | 1 | hit_eop |
| 4 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 53.12 | 4 | hit_eop |
| 5 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 56.25 | 1 | hit_eop |
| 6 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 56.25 | 0 | hit_eop |
| 7 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 1 | hit_eop |
| 8 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 9 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 10 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 11 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 12 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 13 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 14 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 15 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 16 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 17 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |
| 18 | `P128_seed11_cfgD_wide_channel_100k_cycles` | `` | 59.38 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
