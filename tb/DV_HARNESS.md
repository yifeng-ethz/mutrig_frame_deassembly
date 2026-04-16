# DV Harness: mutrig_frame_deassembly

**DUT:** `frame_rcv_ip`  
**Harness prefix:** `frcv_`  
**Methodology:** UVM 1.2 environment plus bound SVA  
**Purpose:** define the reusable verification architecture before any large testcase implementation

## 1. Harness Goals

The harness must prove the real byte-stream parser contract, not a cleaned-up transaction model. In practice that means:

- drive true `rx8b1k` byte traffic including `K28.0`, `K28.4`, `K28.5`, CRC bytes, and injected per-byte error bits
- drive true one-hot run-control words on `ctrl`
- preserve clock-accurate ordering between control transitions and byte-stream arrival
- reconstruct expected `hit_type0` packets from the ingress bytes with a reference parser model
- expose current RTL gaps from `RUN_SEQ_UPGRADE_PLAN.md` rather than masking them

## 2. Planned Directory Structure

```text
tb/
  DV_PLAN.md
  DV_HARNESS.md
  DV_BASIC.md
  DV_EDGE.md
  DV_PROF.md
  DV_ERROR.md
  DV_CROSS.md
  uvm/
    Makefile
    tb_top.sv
    frcv_env_pkg.sv
    frcv_env.sv
    frcv_scoreboard.sv
    frcv_coverage.sv
    frcv_base_test.sv
    agents/
      frcv_rx_agent/
      frcv_ctrl_agent/
      frcv_csr_agent/
      frcv_hit0_mon/
      frcv_hdr_mon/
    sequences/
    tests/
    sva/
      frcv_avmm_sva.sv
      frcv_ctrl_sva.sv
      frcv_rx8b1k_sva.sv
      frcv_internal_sva.sv
```

## 3. Top-Level Connections

`tb_top.sv` owns:

- DUT instantiation with generic overrides for `CHANNEL_WIDTH`, `CSR_ADDR_WIDTH`, `MODE_HALT`, and `DEBUG_LV`
- single clock/reset generation
- interface hookups for `rx8b1k`, `ctrl`, and `csr`
- monitor taps for `hit_type0` and `headerinfo`
- internal debug probes:
  - `run_state_cmd`
  - `receiver_go`
  - `receiver_force_go`
  - `enable`
  - `p_state`, `n_state`
  - `p_word_cnt`, `p_frame_len`, `p_frame_flags`
  - `p_crc_err_count`, `n_crc_error`
  - `n_frame_info_ready`, `aso_headerinfo_valid`

These debug signals are required because several contracts are not visible from the pins alone:

- `IDLE` monitoring via `receiver_force_go`
- one-cycle control decode latency
- `TERMINATING` blocking only fresh headers from `FS_IDLE`
- `MODE_HALT` internal abort-vs-hold behavior

## 4. Agents And Monitors

### 4.1 `frcv_rx_agent`

Purpose:

- drive the byte stream on `asi_rx8b1k_data`, `asi_rx8b1k_error`, `asi_rx8b1k_channel`
- generate precise byte sequences for:
  - long and short frames
  - good and bad CRC endings
  - in-frame `K28.5` commas
  - parity/decode/loss-sync error bursts
  - truncated and overlong payloads

Sequence item fields:

- `byte_value[7:0]`
- `is_k`
- `err_loss_sync`
- `err_parity`
- `err_decode`
- `channel`
- `rx_valid`
- `tag` and `frame_id` for debug correlation

Important note:

- `rx_valid` must still be modeled even though the current RTL ignores it, because that is a first-class contract risk and a planned upgrade topic.

### 4.2 `frcv_ctrl_agent`

Purpose:

- drive one-hot run-control commands:
  - `IDLE`
  - `RUN_PREPARE`
  - `SYNC`
  - `RUNNING`
  - `TERMINATING`
  - `LINK_TEST`
  - `SYNC_TEST`
  - `RESET`
  - `OUT_OF_DAQ`
  - invalid or multi-hot words for negative tests

Required sequence support:

- `feb_system_v2` cadence: `RUN_PREPARE -> SYNC -> RUNNING -> TERMINATING -> IDLE`
- one-cycle pulses
- held-valid commands
- back-to-back command transitions

The agent must timestamp each accepted command so the scoreboard can measure the one-cycle decode lag in the current RTL.

### 4.3 `frcv_csr_agent`

Purpose:

- perform AVMM reads and writes
- check `waitrequest` timing against the `_hw.tcl` `readLatency=1` declaration
- model address widths from `CSR_ADDR_WIDTH`

Required transaction types:

- write control register `csr.control`
- read words 0, 1, 2 and unused addresses
- same-cycle read+write negative access
- mid-frame reads of counters/status

### 4.4 Output monitors

`frcv_hit0_mon` captures:

- `aso_hit_type0_valid`
- `aso_hit_type0_startofpacket`
- `aso_hit_type0_endofpacket`
- `aso_hit_type0_error`
- `aso_hit_type0_data`
- `aso_hit_type0_channel`

`frcv_hdr_mon` captures:

- `aso_headerinfo_valid`
- `aso_headerinfo_data`
- `aso_headerinfo_channel`

## 5. Reference Model / Scoreboard

The scoreboard is byte-accurate and contract-driven.

### 5.1 Ingress-side reference parser

The reference model must reconstruct the intended frame semantics from the driven `rx8b1k` stream, with the same framing assumptions as the DUT:

- start only on `K28.0` with `enable` high
- count header bytes, event counter bytes, payload bytes, and CRC bytes
- decode short-vs-long mode from `frame_flags`
- unpack long hits as 6-byte records
- unpack short hits using the alternating `UNPACK` / `UNPACK_EXTRA` nibble-sharing rule

This is intentionally not a copy of the DUT FSM. It is a spec model derived from the written contract so that:

- off-by-one errors in `n_word_cnt`
- bad short-mode nibble pairing
- wrong `sop/eop`
- incorrect `headerinfo` packing

all show up as scoreboard mismatches.

### 5.2 Run-control model

The scoreboard must also track:

- current commanded run state
- the current RTL’s one-cycle command decode lag
- whether fresh header start should be legal in the present contract
- whether an already-open frame is allowed to finish after `TERMINATING`

### 5.3 Counter model

Expected counter behavior:

- `crc_err_counter`: increment once per frame whose final CRC comparison is bad
- `frame_counter_head`: increment on each `sop`
- `frame_counter_tail`: increment on each `eop`
- `frame_counter`: sampled as `head - tail` on `eop`

The scoreboard must compare both pin outputs and CSR readback against these rules.

## 6. Coverage Model

The implementation must map directly to [DV_CROSS.md](DV_CROSS.md). Minimum covergroup families:

- `cov_run_state`
- `cov_enable_source`
- `cov_frame_mode`
- `cov_frame_len_bucket`
- `cov_hit_count_bucket`
- `cov_crc_result`
- `cov_error_bits`
- `cov_mode_halt`
- `cov_channel_width_cfg`
- `cov_csr_addr_width_cfg`
- `cov_terminating_boundary`
- `cov_headerinfo_fields`

## 7. Assertions

### 7.1 `frcv_avmm_sva`

Checks:

- `waitrequest` low only during active read/write service
- registered CSR response timing consistent with `readLatency=1`
- unused addresses do not mutate writable state
- read+write overlap follows the implemented read-priority behavior

### 7.2 `frcv_ctrl_sva`

Checks:

- `asi_ctrl_ready` never deasserts in the current RTL build
- legal one-hot commands decode to the expected run state on the following cycle
- illegal words enter `ERROR`
- `TERMINATING` removes `receiver_go` but does not assert `receiver_force_go`

### 7.3 `frcv_rx8b1k_sva`

Checks:

- `FS_IDLE` opens only on header and enable
- `TERMINATING` never allows a fresh `FS_IDLE -> FS_FRAME_COUNTER` transition
- `RUN_PREPARE` returns parser state to `FS_IDLE`
- `MODE_HALT=0` comma injection forces `FS_IDLE`
- `MODE_HALT=1` comma injection does not force `FS_IDLE`

### 7.4 `frcv_internal_sva`

Checks:

- `sop` only when `valid`
- `eop` only when `valid`
- zero-length frames never emit `hit_type0_valid`
- `headerinfo_valid` is a one-cycle pulse
- no duplicate `eop` for a single frame
- no fresh header start after the `TERMINATING` edge once the parser is back in `FS_IDLE`

## 8. Test Classes

Planned base classes:

- `frcv_base_test`: clock/reset, env build, default config, bounded timeout
- `frcv_basic_test`: directed cases from `DV_BASIC.md`
- `frcv_edge_test`: protocol and boundary cases from `DV_EDGE.md`
- `frcv_prof_test`: soak and sweep cases from `DV_PROF.md`
- `frcv_error_test`: negative/fault cases from `DV_ERROR.md`

Shared sequence families:

- frame builders: long, short, zero-hit, truncated, bad-crc, comma-corrupt
- control sequences: single word, held word, back-to-back word, system cadence
- CSR sequences: write/readback, mid-frame reads, address sweep

## 9. Configuration Matrix

Minimum build matrix:

| Build ID | CHANNEL_WIDTH | CSR_ADDR_WIDTH | MODE_HALT | DEBUG_LV | Purpose |
|---|---:|---:|---:|---:|---|
| CFG_A | 4 | 2 | 0 | 0 | packaged default |
| CFG_B | 4 | 2 | 1 | 0 | comma-hold behavior |
| CFG_C | 1 | 2 | 0 | 0 | min channel width |
| CFG_D | 8 | 2 | 0 | 0 | wide channel passthrough |
| CFG_E | 4 | 1 | 0 | 0 | narrow CSR address |
| CFG_F | 4 | 8 | 0 | 0 | wide CSR address |
| CFG_G | 4 | 2 | 0 | 2 | sim-only debug build |

## 10. Waveform And Debug Requirements

For every signature failure or contract proof, keep a focused waveform recipe with:

- run-control word and `ready`
- `receiver_go`, `receiver_force_go`, `enable`
- parser state and byte counters
- ingress byte stream
- `hit_type0` and `headerinfo`
- CSR readback when counters or status are checked

Signature waveforms required:

- `IDLE` monitoring start
- `RUN_PREPARE -> SYNC -> RUNNING` bring-up
- `RUNNING -> TERMINATING` open-frame drain
- post-terminate blocked fresh header
- `MODE_HALT` comma abort vs hold
- bad CRC frame and counter increment

## 11. Known Risks The Harness Must Not Hide

1. `asi_rx8b1k_valid` is ignored by current RTL.
2. `asi_ctrl_ready` is permanently high in current RTL.
3. There is no synthetic terminal marker in current RTL.
4. The present control process applies new `run_state_cmd` and old decoded outputs in the same cycle, which creates a one-cycle control-effect lag.
5. Channel changes inside a frame are unsupported by design comment and must be treated as illegal stimulus in the mainline scoreboard.
