# ⚠️ all_buckets_frame_cfg_e

**Kind:** `all_buckets_frame` &nbsp; **Build:** `CFG_E` &nbsp; **Bucket:** `-` &nbsp; **Sequence:** `ALL_CFG_E`

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
| ℹ️ | txns | `66` |
| ⚠️ | functional_cross_pct | `40.62` |
| ℹ️ | queued_overlap | `0` |
| ✅ | counter_checks_failed | `0` |
| ✅ | unexpected_outputs | `0` |

## Code coverage

<!-- merged code coverage produced by this single run (not ordered-merged into any bucket). -->

| metric | pct |
|---|---|
| stmt | 80.93 |
| branch | 67.55 |
| cond | 58.62 |
| expr | 98.77 |
| fsm_state | 85.71 |
| fsm_trans | 40.00 |
| toggle | 51.25 |

## Transaction growth curve

<!-- each row is one transaction step: which planned case fired, current functional-cross percent, -->
<!-- delta_bins = number of new cross bins hit at this step; reason = scoreboard checkpoint trigger. -->

| txn | case | seq | pct | delta_bins | reason |
|---:|---|---|---|---:|---|
| 1 | `E091_csr_addr_width1_aliases_words0_1_only` | `` | 34.38 | 11 | hit_eop |
| 2 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 1 | hit_eop |
| 3 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 4 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 5 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 6 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 7 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 8 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 9 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 10 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 11 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 12 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 13 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 14 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 15 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 16 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 17 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 18 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 19 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 20 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 21 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 22 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 23 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 24 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 25 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 26 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 27 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 28 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 29 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 30 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 31 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 32 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 33 | `P085_cfgE_addr1_poll_soak` | `` | 37.50 | 0 | hit_eop |
| 34 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 35 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 36 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 37 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 38 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 39 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 40 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 41 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 42 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 43 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 44 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 45 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 46 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 47 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 48 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 49 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 50 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 51 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 52 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 53 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 54 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 55 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 56 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 57 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 58 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 59 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 60 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 61 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 62 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 63 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 64 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 65 | `P090_cfgE_control_churn_soak` | `` | 37.50 | 0 | hit_eop |
| 66 | `P129_seed12_cfgE_narrow_addr_100k_cycles` | `` | 40.62 | 1 | hit_eop |

---
_Back to [dashboard](../../DV_REPORT.md)_
