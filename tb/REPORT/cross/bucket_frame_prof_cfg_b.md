# ⚠️ bucket_frame_prof_cfg_b

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_B` &nbsp; **Bucket:** `PROF` &nbsp; **Sequence:** `PROF_CFG_B`

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
| ℹ️ | case_count | `10` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `61` |
| ⚠️ | functional_cross_pct | `40.62` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.86 |
| branch | 67.55 |
| cond | 62.07 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 53.33 |
| toggle | 50.68 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P071_mode1_comma_every_64th_frame_counter` | `` | 34.38 | 11 | hit_eop |
| 2 | `P072_mode1_comma_every_64th_event_counter` | `` | 37.50 | 1 | hit_eop |
| 3 | `P073_mode1_comma_every_64th_unpack` | `` | 37.50 | 0 | hit_eop |
| 4 | `P074_mode1_comma_random_positions` | `` | 40.62 | 1 | hit_eop |
| 5 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 6 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 7 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 8 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 9 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 10 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 11 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 12 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 13 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 14 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 15 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 16 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 17 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 18 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 19 | `P074_mode1_comma_random_positions` | `` | 40.62 | 0 | hit_eop |
| 20 | `P075_mode1_hold_with_loss_sync_mix` | `` | 40.62 | 0 | hit_eop |
| 21 | `P077_mode0_to_mode1_build_comparison` | `` | 40.62 | 0 | hit_eop |
| 22 | `P078_mode1_to_mode0_build_comparison` | `` | 40.62 | 0 | hit_eop |
| 23 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 24 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 25 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 26 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 27 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 28 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 29 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 30 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 31 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 32 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 33 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 34 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 35 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 36 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 37 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 38 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 39 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 40 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 41 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 42 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 43 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 44 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 45 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 46 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 47 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 48 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 49 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 50 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 51 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 52 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 53 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 54 | `P081_cfgB_modehalt1_clean_soak` | `` | 40.62 | 0 | hit_eop |
| 55 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 56 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 57 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 58 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 59 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 60 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |
| 61 | `P082_cfgB_modehalt1_comma_soak` | `` | 40.62 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
