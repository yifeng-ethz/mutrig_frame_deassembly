# TODO

- Resolve `FRCV-2026-04-18-005`: classify or suppress benign `FRCV_SCB_CASE_END` warnings in continuous-frame mode, then rerun the affected `bucket_frame` / `all_buckets_frame` signoff sets with the final warning policy.
- Run the randomized all-bucket long soak for tens of minutes per worker and keep `tb/DV_REPORT.json`, `tb/DV_REPORT.md`, and `tb/REPORT/` in sync with the latest evidence.
- Execute the standalone Quartus scaffold under `syn/quartus/` and replace the placeholder synthesis report with real timing/resource data.
- Extend the `/data1` OSS formal flow beyond the current CRC proofs to packet-boundary targets `B1`/`B9` and to malformed-packet reach/drop properties.
- Keep `VERSION`, packaging metadata, `tb/BUG_HISTORY.md`, and the generated DV report aligned on every verified fix batch.
