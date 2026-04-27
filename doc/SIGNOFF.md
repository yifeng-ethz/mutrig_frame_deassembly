# ✅ Signoff — mutrig_frame_deassembly

**DUT:** `frame_rcv_ip` &nbsp; **Date:** `2026-04-21`

## Legend

✅ pass / closed &middot; ⚠️ partial / caveat &middot; ❌ failed / blocked &middot; ❓ pending &middot; ℹ️ informational

## Health

| status | field | value |
|:---:|---|---|
| ✅ | overall_signoff | `pass for the current QuestaOne DV refresh and standalone synthesis refresh` |
| ✅ | dv_closure | `failed_cases=[]`, `unimplemented_count=0`, `stale_artifact_without_engine_marker_count=0` in `tb/DV_REPORT.json` |
| ✅ | continuous_frame_signoff | `bucket_frame_prof_cfg_a_after_s1` and `all_buckets_frame_cfg_a_fix2_after_s1` are clean on hard-fail metrics |
| ✅ | standalone_syn | mini-project is fully constrained and closes the tightened `137.5 MHz` / `7.273 ns` target |
| ✅ | latest_issue_classification | harness-only; no newly reproduced RTL bug was required to explain the QuestaOne failures |

## Verification

| status | area | result | source |
|:---:|---|---|---|
| ✅ | isolated documented cases | all documented cases have fresh `DOC_CASE_ENGINE_V2` evidence and pass in the current `after` / seed `1` dashboard refresh | [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md) |
| ✅ | continuous-frame PROF baseline | `23/32` cross bins, `1157` txns, `unexpected_outputs=0`, `counter_checks_failed=0`, `UVM_WARNING=0` | [`../tb/uvm/logs/bucket_frame_prof_cfg_a_after_s1.log`](../tb/uvm/logs/bucket_frame_prof_cfg_a_after_s1.log) |
| ✅ | continuous-frame all-buckets baseline | `31/32` cross bins, `1462` txns, `counter_checks_passed=40`, `unexpected_outputs=0`, `UVM_WARNING=0` | [`../tb/uvm/logs/all_buckets_frame_cfg_a_fix2_after_s1.log`](../tb/uvm/logs/all_buckets_frame_cfg_a_fix2_after_s1.log) |
| ✅ | bug ledger | harness-only QuestaOne migration fixes are recorded and the old pending queue-retirement warning is closed in the live ledger | [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md) |

## Synthesis

| status | item | value |
|:---:|---|---|
| ✅ | revision | `mutrig_frame_deassembly_syn_cfg_a` |
| ✅ | device | `5AGXBA7D4F31C5` |
| ✅ | signoff constraint | `137.5 MHz` / `7.273 ns` on `clk125` |
| ✅ | setup slack | `+2.717 ns` |
| ✅ | worst selected hold slack | `+0.159 ns` |
| ✅ | constraint state | `Design is fully constrained for setup and hold requirements` |
| ✅ | detail report | [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md) |

## Notes

- The latest newly failing cases were closed in harness code, mainly `tb/uvm/frcv_cases.svh` and the report-refresh flow. No new RTL bug was reproduced during this cleanup.
- `FRCV-2026-04-18-005` is now closed: mode-0 comma-abort cases retire cleanly with correct pre-abort expectations and no real-EOP expectation.
- The generated DV dashboard can still show warning icons for non-blocking coverage-plan items such as toggle targets or sparse auxiliary-build continuous-frame crosses. Those warnings do not indicate a current regression failure.
