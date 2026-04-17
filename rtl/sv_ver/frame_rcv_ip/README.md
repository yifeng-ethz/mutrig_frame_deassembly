## frame_rcv_ip mixed-language shell

This directory holds the SystemVerilog-facing shell for the active VHDL
`frame_rcv_ip` implementation.

Intent:

- keep the packaged VHDL DUT as the executable golden implementation
- expose stable debug/contract taps for simulation SVA and future formal work
- avoid a full source rewrite until there is a real need for source-level SV ownership

The wrapper is deliberately thin: it instantiates the VHDL entity unchanged and
re-exports internal parser/output signals that matter for packet-shape contract
checks.
