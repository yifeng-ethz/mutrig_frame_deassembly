# TODO

- Resolve `FRCV-2026-04-17-003`: root-cause the continuous-frame `unexpected_outputs` scoreboard gap and refresh `DV_REPORT.json` plus `REPORT/cross/`.
- Resolve `FRCV-2026-04-17-004`: implement distinct malformed-packet injections for `bad_trailer`, `extra_payload_after_declared_len`, and `new_header_inside_payload`, then rerun the affected ERROR cases and decide which ones should increment `crc_err_counter`.
- Run the new randomized all-bucket long soak for tens of minutes per worker and keep the generated report in sync with the latest evidence.
- Execute the standalone Quartus scaffold under `syn/quartus/` and replace the placeholder synthesis report with real timing/resource data.
- Keep `VERSION`, `script/mutrig_frame_deassembly_hw.tcl`, `script/mutrig_frame_deassembly.svd`, `tb/BUG_HISTORY.md`, and the generated DV report aligned on every verified fix batch.
