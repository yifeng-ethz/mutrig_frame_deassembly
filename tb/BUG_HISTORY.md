# BUG_HISTORY

This ledger records real RTL and DV-harness bugs found during verification.
Each fixed bug must point at the testcase or regression that exposed it and the git commit that carried the fix.

| bug_id | date | scope | ip_or_tb | first_seen_case | first_seen_runtime_s | coverage_or_signoff_context | symptom | root_cause | fix_summary | fix_commit | status |
| --- | --- | --- | --- | --- | ---: | --- | --- | --- | --- | --- | --- |
| FRCV-2026-04-16-001 | 2026-04-16 | rtl | `frame_rcv_ip` | `tb_int_longrun_sanity_test +TB_INT_LONGRUN_CASE_ID=1` | 41 | FEB long-run sanity, `require_lossless_feb`, stage `H0->H1` | Lane 1 dropped 3 terminating-tail hits because downstream closed before the last delayed frame arrived. | `aso_hit_type0_endofrun` pulsed as soon as `TERMINATING` saw local parser idle, even though the upstream 8b1k lane could still deliver a delayed final MuTRiG frame. | Add a guarded quiet window before asserting `hit_type0_endofrun`, and cover the delayed-tail terminate path in the direct and UVM IP benches. | ad61b7f | fixed |
