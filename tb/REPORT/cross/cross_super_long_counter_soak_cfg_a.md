# ✅ cross_super_long_counter_soak_cfg_a

**Kind:** `cross` &nbsp; **Build:** `CFG_A` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `SUPER_LONG_COUNTER_SOAK`

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
| ℹ️ | effort | `extensive` |
| ℹ️ | iter_cap | `None` |
| ℹ️ | payload_cap | `None` |
| ℹ️ | txns | `12306` |
| ✅ | functional_cross_pct | `56.25` |
| ℹ️ | queued_overlap | `9248` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 87.16 |
| branch | 75.50 |
| cond | 79.31 |
| expr | 98.77 |
| fsm_state | 100.00 |
| fsm_trans | 60.00 |
| toggle | 52.19 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `P118_seed01_clean_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 37.50 | 12 | hit_eop |
| 2 | `P120_seed03_error_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 53.12 | 5 | hit_eop |
| 3 | `P121_seed04_stop_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 1 | hit_eop |
| 4 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 5 | `P118_seed01_clean_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 6 | `P120_seed03_error_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 7 | `P121_seed04_stop_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 8 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 9 | `P118_seed01_clean_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 10 | `P120_seed03_error_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 11 | `P121_seed04_stop_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 12 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 13 | `P118_seed01_clean_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 14 | `P120_seed03_error_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 15 | `P121_seed04_stop_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 16 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 32 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 64 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 128 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 256 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 512 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 1024 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 2048 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 3072 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 4096 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 5120 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 6144 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 7168 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 8192 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 9216 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 10240 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 11264 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |
| 12288 | `P125_seed08_badcrc_mix_100k_cycles` | `SUPER_LONG_COUNTER_SOAK` | 56.25 | 0 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
