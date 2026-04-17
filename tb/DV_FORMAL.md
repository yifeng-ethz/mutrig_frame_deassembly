# DV_FORMAL: mutrig_frame_deassembly — packet-shape formal plan

**Companion to:** `DV_PLAN.md`, `DV_HARNESS.md`, `DV_REPORT.md`  
**DUT:** `frame_rcv_ip`  
**Primary RTL:** `rtl/frame_rcv_ip.vhd`  
**Mixed-language shell:** `rtl/sv_ver/frame_rcv_ip/frame_rcv_ip_dut_sv.sv`  
**Formal harness:** `tb/formal/frcv_formal_top.sv`  
**Date:** 2026-04-17  
**Status:** Active formal-readiness and assertion plan for the current VHDL DUT.

This plan applies the packet-structure methodology from Doug Smith,
"Doing the Impossible: Using Formal Verification on Packet Based Data Paths"
(DVCon US 2023; video: <https://youtu.be/SbdgOf4Zf1o>; paper:
<https://dvcon-proceedings.org/wp-content/uploads/1032-Doing-the-Impossible-Using-Formal-Verification-on-Packet-Based-Data-Paths.pdf>)
to `frame_rcv_ip`.

The key decision is the same one the video/paper recommends for legacy packet
parsers:

- keep the golden parser implementation intact
- wrap it with a small SV shell that exposes packet-boundary signals
- constrain the legal byte grammar first
- prove the local parser/output contracts against that grammar

For this IP, that means **do not rewrite the VHDL parser yet**. The right first
step is mixed-language SVA around the current `frame_rcv_ip`.

## 1. Active implementation in tree

Added in the current tree:

- `rtl/sv_ver/frame_rcv_ip/frame_rcv_ip_dut_sv.sv`
  - thin SV-facing wrapper around the VHDL DUT
  - exports `n_new_frame`, `n_new_word`, `n_frame_len`, `n_frame_flags`,
    `n_frame_number`, `n_crc_error`, `p_crc_err_count`, and CSR/frame counters
- `tb/uvm/sva/frcv_parser_boundary_sva.sv`
  - checks legal parser-open and payload-word boundary conditions
- `tb/uvm/sva/frcv_output_contract_sva.sv`
  - checks `proc_frame_rcv_comb` to `proc_output_pkt_hits*` and
    `proc_output_header_info*` packet structure
- `tb/uvm/sva/frcv_counter_contract_sva.sv`
  - checks CRC and frame-counter accounting
- `tb/formal/frcv_formal_pkt_assumptions.sv`
  - constrains a legal ingress byte grammar for future formal runs
- `tb/formal/frcv_formal_top.sv`
  - mixed-language formal harness around the wrapper plus the active SVA

Current environment note:

- `qverify` / PropCheck is not on `PATH` in this workspace as of 2026-04-17
- the harness is therefore compile-validated today and laid out so a formal
  tool invocation can be added without changing the structure again

## 2. Seven-step packet method mapped to this DUT

The DVCon method maps onto `frame_rcv_ip` like this:

1. Model the control logic.
   Constrain legal `rx8b1k` framing and keep the DUT enabled while a legal
   frame is being explored.
2. Define the packet structure.
   Break the ingress stream into:
   - header `K28.0`
   - 2 frame-counter bytes
   - 2 event-counter bytes
   - payload bytes
   - 2 CRC bytes
3. Define packet constraints.
   Use small local constraints instead of unconstrained raw byte streams:
   - body bytes are data bytes, not K-codes
   - error bits are clear in the good-packet grammar
   - channel stays stable across a frame
4. Apply the packet constraints.
   Walk a small harness FSM through the byte phases instead of trying to solve
   the full parser state space at once.
5. Model the packet driving logic.
   The formal harness leaves the payload bytes symbolic but deterministically
   programs the CSR enable bit and sends a `RUNNING` control command.
6. Generate the packet.
   The formal harness lets the tool pick the actual payload bytes while the
   phase machine constrains the legal structure.
7. Check the packet.
   Assert the parser/output/counter contracts on the real DUT boundary.

## 3. Packet grammar for `frame_rcv_ip`

### 3.1 Good-frame grammar

A legal frame on `asi_rx8b1k_*` is:

1. `K28.0` header byte (`i_byteisk=1`, `i_data=8'h1C`)
2. two frame-counter bytes
3. event-counter high byte
   - `frame_flags = byte[7:2]`
   - `frame_len[9:8] = byte[1:0]`
4. event-counter low byte
   - `frame_len[7:0] = byte`
5. payload bytes
   - long mode: `6 * frame_len`
   - short mode: `3 * frame_len + ceil(frame_len / 2)`
6. two CRC bytes

### 3.2 Mode split

`p_txflag_isShort` is derived from `p_frame_flags(4 downto 2) = "100"`.
That means the formal grammar must branch at the event-counter high byte:

- long mode proofs use the `FS_UNPACK` 6-byte hit cadence
- short mode proofs use the `FS_UNPACK` / `FS_UNPACK_EXTRA` alternating cadence

### 3.3 Zero-length special case

`frame_len = 0` skips payload parsing entirely:

- `FS_EVENT_COUNTER -> FS_CRC_CALC`
- no `hit_type0_valid`
- one `headerinfo_valid` pulse still occurs

This is a mandatory separate proof target, not just a random corner.

## 4. Boundary positions inside the IP

These are the explicit packet boundaries to prove. This is the main checklist
for the IP.

| ID | Boundary | RTL anchor | Why it matters | Current collateral |
|---|---|---|---|---|
| B0 | raw `rx8b1k` byte lane -> parser start pulse | `FS_IDLE` header detect, `n_new_frame` | proves frames only open on enabled `K28.0` | `frcv_parser_boundary_sva` |
| B1 | run-control decode -> parser enable | `proc_run_control_mgmt_agent`, `proc_enable_ctrl` | controls whether a header can legally start parsing | wrapper exports `enable`, `receiver_go`, `terminating_pending` |
| B2 | frame-counter bytes -> captured `n_frame_number` | `FS_FRAME_COUNTER` | packet serial must be packed into `headerinfo_data[41:26]` correctly | `frcv_output_contract_sva` |
| B3 | event-counter bytes -> `n_frame_len`, `n_frame_flags`, `n_frame_info_ready` | `FS_EVENT_COUNTER`, `proc_output_header_info*` | packet metadata boundary for every frame, including zero-length frames | `frcv_output_contract_sva` |
| B4 | long-hit byte collection -> emitted hit word | `FS_UNPACK` long path | catches 6-byte packing and `sop/eop` off-by-one errors | `frcv_output_contract_sva` |
| B5 | short-hit byte/nibble collection -> emitted hit word | `FS_UNPACK` + `FS_UNPACK_EXTRA` short path | catches nibble-sharing bugs and odd/even hit boundary mistakes | `frcv_output_contract_sva` |
| B6 | parser next-state hit pulse -> registered `hit_type0` outputs | `n_new_word`, `proc_output_pkt_hits*` | exact "between processes" packet contract | `frcv_output_contract_sva` |
| B7 | CRC check result -> hit error1 + CRC counter | `FS_CRC_CHECK`, `n_crc_error`, `p_crc_err_count` | proves CRC indication and counting are coherent | `frcv_counter_contract_sva` |
| B8 | SOP/EOP outputs -> frame counters | `proc_avmm_slave_csr` | proves scoreboard/accounting path matches markers | `frcv_counter_contract_sva` |
| B9 | TERMINATING delayed-tail admit -> end-of-run pulse | `terminating_pending`, `terminating_frame_start_seen`, `terminating_empty_frame_done` | known high-risk closure path from recent bug fixes | wrapper ready, formal property backlog |

## 5. Properties to prove per boundary

### 5.1 B0: frame start legality

Mandatory property:

- `n_new_frame` implies:
  - `enable=1`
  - `asi_rx8b1k_valid=1`
  - `i_byteisk=1`
  - `i_data=K28.0`

Implemented today in `frcv_parser_boundary_sva`.

### 5.2 B1: enable/run-control boundary

Proof targets:

- `RUNNING` plus `csr.control(0)=1` is sufficient for `enable=1`
- `receiver_force_go=1` is sufficient for `enable=1`
- `TERMINATING` plus `terminating_pending=1` still allows delayed final-frame admission
- once `terminating_pending` clears, fresh header admission stops

Status:

- wrapper visibility is in tree
- formal properties are planned but not yet coded

### 5.3 B2/B3: header metadata packing

Mandatory properties:

- one `headerinfo_valid` pulse per frame
- pulse width is exactly one cycle
- payload equals:
  - `[5:0]   = n_frame_flags`
  - `[15:6]  = n_frame_len`
  - `[25:16] = n_word_cnt`
  - `[41:26] = n_frame_number`
- channel equals the sampled frame channel

Implemented today in `frcv_output_contract_sva`.

### 5.4 B4: long-hit structural boundary

Mandatory properties:

- every long-mode `n_new_word` produces exactly one `hit_type0_valid`
- packed hit data matches the sampled 48-bit parser word
- `sop` fires iff sampled `n_word_cnt == 1`
- `eop` fires iff sampled `n_word_cnt == n_frame_len`

Implemented today in `frcv_output_contract_sva`.

### 5.5 B5: short-hit structural boundary

Mandatory properties:

- the short-mode output pack uses `{asic, channel, T_CC, T_Fine, 0, E_Flag}`
- long-mode-only `E_CC` bits stay zero in short mode
- odd/even short-hit alternation does not create missing or duplicate hit pulses

Implemented today for the output packet shape. The deeper odd/even internal
alternation proof remains a dedicated formal backlog item.

### 5.6 B6: bad-hit flag propagation

Mandatory property:

- if sampled hit data carries `T_BadHit` or long-mode `E_BadHit`, then the
  corresponding emitted hit asserts `error[0]`

Status:

- kept as an explicit proof target
- not yet promoted into the active SVA set because the packed raw-word-to-bad-hit
  oracle still needs a dedicated local decode helper

### 5.7 B7: CRC boundary

Mandatory properties:

- `error[1]` only appears on `eop`
- `n_crc_error` increments `p_crc_err_count` by exactly one
- no decrement or multi-step jump is allowed

Implemented today in `frcv_counter_contract_sva`.

### 5.8 B8: frame counter boundary

Mandatory properties:

- `sop` increments `frame_counter_head`
- `eop` increments `frame_counter_tail`
- `frame_counter_head >= frame_counter_tail`

Implemented today in `frcv_counter_contract_sva`.

### 5.9 B9: terminate/drain boundary

Mandatory future properties:

- entering `TERMINATING` arms `terminating_pending`
- a delayed final header seen while pending is still admissible
- first empty frame in `TERMINATING` can close the drain and pulse end-of-run
- after the drain closes, no new frame start is accepted

Status:

- visibility exists in the wrapper
- properties still need to be added

## 6. Malformed-packet proof backlog

The formal plan needs two classes of malformed proofs:

1. malformed frames that should still reach `FS_CRC_CHECK` and increment the
   CRC counter
2. malformed frames that should be dropped or aborted before CRC check

Known current harness gap from `BUG_HISTORY.md`:

- `FRCV-2026-04-17-004`
- `X034` bad trailer replacement
- `X035` extra payload after declared length
- `X036` new header inside payload
- `X048` short new header inside payload
- `X049` short extra payload after declared length

Those cases are explicitly part of the formal backlog because the current
doc-case engine does not yet inject them distinctly enough for signoff.

## 7. Simulation vs formal split

The intended workflow is:

1. keep the packet-boundary SVA active in UVM simulation
2. use the same SVA modules in the formal harness
3. add formal-only input assumptions to bound the state space
4. move one boundary at a time from "simulation observed" to "formally proved"

That avoids the common failure mode where simulation and formal grow two
different contract definitions.

## 8. What is explicitly not done

Not done in this step:

- no full SV rewrite of `frame_rcv_ip`
- no mutation of the packaged VHDL RTL
- no claim of formal proof closure yet
- no attempt to solve malformed-packet proofs before the missing stimulus hooks
  are modeled distinctly

## 9. Immediate next proof candidates

The highest-value next additions are:

1. B9 terminate/drain properties around `terminating_pending`
2. deeper short-mode internal alternation properties at the `UNPACK` /
   `UNPACK_EXTRA` boundary
3. malformed-packet properties for the five currently under-modeled cases
4. CRC-good packet assumptions that tie payload bytes to the two CRC bytes for
   non-error acceptance proofs
