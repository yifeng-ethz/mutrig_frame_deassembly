# mutrig_frame_deassembly Signoff

## Status

- DV status: partial
- Standalone synthesis status: scaffolded, compile pending
- Open blocking issue: `FRCV-2026-04-17-003` continuous-frame scoreboard `unexpected_outputs`

## Evidence

- DV dashboard: [`../tb/DV_REPORT.md`](../tb/DV_REPORT.md)
- DV coverage summary: [`../tb/DV_COV.md`](../tb/DV_COV.md)
- DV bug ledger: [`../tb/BUG_HISTORY.md`](../tb/BUG_HISTORY.md)
- Standalone synthesis report: [`../syn/SYN_REPORT.md`](../syn/SYN_REPORT.md)
- Timing/resource note: [`rtl_note.md`](rtl_note.md)

## Next Gate

1. Run the randomized all-bucket long soak in parallel and determine whether the red cross runs are a scoreboard bug or a real RTL mismatch.
2. Refresh the generated DV report after the new long-run evidence lands.
3. Compile the `syn/quartus/` project and replace the placeholder timing/resource report with measured results.
