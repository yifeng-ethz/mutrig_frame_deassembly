# Static Screen Contexts

This directory keeps deterministic filelists for the Questa Static lint/CDC/RDC screen.

- `mutrig_frame_deassembly_static.f`: canonical standalone VHDL DUT context, matching the Quartus signoff RTL dependency order.
- `mutrig_frame_deassembly_sv_shell_static.f`: mixed-language context for the SystemVerilog wrapper plus active VHDL DUT.

Generated qverify work directories are intentionally ignored by git. The signoff documentation cites the exact local transcript paths from the current run.
