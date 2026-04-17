//------------------------------------------------------------------------------
// frcv_formal_pkt_assumptions
// Description:
//   Packet-shape assumptions for `frame_rcv_ip`, following the DVCon packet
//   formal method: constrain the legal byte grammar first, then prove the DUT's
//   local parser/output contracts against that grammar.
//------------------------------------------------------------------------------
module frcv_formal_pkt_assumptions #(
  parameter int CHANNEL_WIDTH = 4
) (
  input logic                     clk,
  input logic                     rst,
  input logic [8:0]               rx_data,
  input logic                     rx_valid,
  input logic [2:0]               rx_error,
  input logic [CHANNEL_WIDTH-1:0] rx_channel,
  input logic                     dbg_enable
);

  typedef enum logic [2:0] {
    PH_IDLE,
    PH_FRAME_HI,
    PH_FRAME_LO,
    PH_EVENT_HI,
    PH_EVENT_LO,
    PH_PAYLOAD,
    PH_CRC_HI,
    PH_CRC_LO
  } formal_phase_t;

  formal_phase_t              phase_q;
  logic [CHANNEL_WIDTH-1:0]   frame_channel_q;
  logic [9:0]                 frame_len_q;
  logic [12:0]                payload_bytes_left_q;
  logic                       short_mode_q;

  function automatic logic [12:0] short_payload_bytes(
    input logic [9:0] frame_len_v
  );
    logic [12:0] len_v;
    begin
      len_v = {3'b000, frame_len_v};
      return (len_v * 13'd3) + ((len_v + 13'd1) >> 1);
    end
  endfunction

  function automatic logic [12:0] payload_bytes_for_mode(
    input logic       short_mode_v,
    input logic [9:0] frame_len_v
  );
    logic [12:0] len_v;
    begin
      len_v = {3'b000, frame_len_v};
      if (short_mode_v) begin
        return short_payload_bytes(frame_len_v);
      end
      return len_v * 13'd6;
    end
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      phase_q              <= PH_IDLE;
      frame_channel_q      <= '0;
      frame_len_q          <= '0;
      payload_bytes_left_q <= '0;
      short_mode_q         <= 1'b0;
    end else begin
      case (phase_q)
        PH_IDLE: begin
          if (dbg_enable && rx_valid && rx_data[8] && (rx_data[7:0] == 8'h1C)) begin
            phase_q         <= PH_FRAME_HI;
            frame_channel_q <= rx_channel;
          end
        end

        PH_FRAME_HI: phase_q <= PH_FRAME_LO;
        PH_FRAME_LO: phase_q <= PH_EVENT_HI;

        PH_EVENT_HI: begin
          short_mode_q    <= (rx_data[6:4] == 3'b100);
          frame_len_q[9:8] <= rx_data[1:0];
          phase_q         <= PH_EVENT_LO;
        end

        PH_EVENT_LO: begin
          logic [9:0] frame_len_v;
          logic [12:0] payload_bytes_v;

          frame_len_v     = {frame_len_q[9:8], rx_data[7:0]};
          payload_bytes_v = payload_bytes_for_mode(short_mode_q, frame_len_v);

          frame_len_q          <= frame_len_v;
          payload_bytes_left_q <= payload_bytes_v;
          phase_q              <= (payload_bytes_v == 13'd0) ? PH_CRC_HI : PH_PAYLOAD;
        end

        PH_PAYLOAD: begin
          if (payload_bytes_left_q == 13'd1) begin
            payload_bytes_left_q <= '0;
            phase_q              <= PH_CRC_HI;
          end else begin
            payload_bytes_left_q <= payload_bytes_left_q - 13'd1;
          end
        end

        PH_CRC_HI: phase_q <= PH_CRC_LO;
        PH_CRC_LO: phase_q <= PH_IDLE;
        default:   phase_q <= PH_IDLE;
      endcase
    end
  end

  property p_body_bytes_are_data_not_k;
    @(posedge clk) disable iff (rst)
      (phase_q inside {PH_FRAME_HI, PH_FRAME_LO, PH_EVENT_HI, PH_EVENT_LO, PH_PAYLOAD, PH_CRC_HI, PH_CRC_LO})
      |-> (rx_valid && !rx_data[8]);
  endproperty

  property p_body_channel_is_stable;
    @(posedge clk) disable iff (rst)
      (phase_q inside {PH_FRAME_HI, PH_FRAME_LO, PH_EVENT_HI, PH_EVENT_LO, PH_PAYLOAD, PH_CRC_HI, PH_CRC_LO})
      |-> (rx_channel == frame_channel_q);
  endproperty

  property p_body_error_bits_clear;
    @(posedge clk) disable iff (rst)
      (phase_q inside {PH_FRAME_HI, PH_FRAME_LO, PH_EVENT_HI, PH_EVENT_LO, PH_PAYLOAD, PH_CRC_HI, PH_CRC_LO})
      |-> (rx_error == 3'b000);
  endproperty

  property p_frame_starts_only_when_enabled;
    @(posedge clk) disable iff (rst)
      (rx_valid && rx_data[8] && (rx_data[7:0] == 8'h1C)) |-> dbg_enable;
  endproperty

  assume property (p_body_bytes_are_data_not_k);
  assume property (p_body_channel_is_stable);
  assume property (p_body_error_bits_clear);
  assume property (p_frame_starts_only_when_enabled);

  cover property (
    @(posedge clk) disable iff (rst)
      phase_q == PH_IDLE ##1 phase_q == PH_FRAME_HI ##1 phase_q == PH_FRAME_LO ##1
      phase_q == PH_EVENT_HI ##1 phase_q == PH_EVENT_LO ##[1:$] phase_q == PH_CRC_LO
  );

endmodule
