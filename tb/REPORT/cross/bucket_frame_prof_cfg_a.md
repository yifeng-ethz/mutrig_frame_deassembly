# ✅ bucket_frame_prof_cfg_a

**Kind:** `bucket_frame` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `PROF` &nbsp; **Sequence:** `PROF_CFG_A`

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
| ℹ️ | case_count | `110` |
| ℹ️ | effort | `practical` |
| ℹ️ | iter_cap | `32` |
| ℹ️ | payload_cap | `2048` |
| ℹ️ | txns | `1157` |
| ✅ | functional_cross_pct | `71.88` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 94.16 |
| branch | 87.33 |
| cond | 72.50 |
| expr | 98.77 |
| fsm_state | n/a |
| fsm_trans | n/a |
| toggle | 56.52 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P001_long_len0_10k_frames` | `` | 34.38 | 11 | header_only_or_zero_hit |
| 2 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 3 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 4 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 5 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 6 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 7 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 8 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 9 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 10 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 11 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 12 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 13 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 14 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 15 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 16 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 32 | `P001_long_len0_10k_frames` | `` | 34.38 | 0 | header_only_or_zero_hit |
| 33 | `P002_long_len1_10k_frames` | `` | 37.50 | 1 | hit_eop |
| 34 | `P002_long_len1_10k_frames` | `` | 40.62 | 1 | hit_eop |
| 64 | `P002_long_len1_10k_frames` | `` | 40.62 | 0 | hit_eop |
| 65 | `P003_long_len2_10k_frames` | `` | 43.75 | 1 | hit_eop |
| 97 | `P004_long_len3_10k_frames` | `` | 46.88 | 1 | hit_eop |
| 128 | `P004_long_len3_10k_frames` | `` | 46.88 | 0 | hit_eop |
| 129 | `P005_long_len7_5k_frames` | `` | 50.00 | 1 | hit_eop |
| 182 | `P007_long_len31_1k_frames` | `` | 53.12 | 1 | hit_eop |
| 203 | `P014_short_len0_10k_frames` | `` | 62.50 | 3 | header_only_or_zero_hit |
| 256 | `P015_short_len1_10k_frames` | `` | 62.50 | 0 | hit_eop |
| 439 | `P039_cadence_terminating_hold_open_frame` | `` | 65.62 | 1 | hit_eop |
| 512 | `P048_midframe_mask_raise_repeated` | `` | 65.62 | 0 | hit_eop |
| 567 | `P066_mode0_comma_every_64th_frame_counter` | `` | 68.75 | 1 | header_only_or_zero_hit |
| 570 | `P069_mode0_comma_random_positions` | `` | 71.88 | 1 | header_only_or_zero_hit |
| 1024 | `P123_seed06_mode_mix_100k_cycles` | `` | 71.88 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
