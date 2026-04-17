//------------------------------------------------------------------------------
// frcv_output_contract_sva
// Description:
//   Checks the packet-shape boundary from the parser next-state signals into the
//   registered `hit_type0` / `headerinfo` outputs. This is the main reusable
//   contract block for both simulation and future formal harnesses.
//------------------------------------------------------------------------------
module frcv_output_contract_sva #(
  parameter int CHANNEL_WIDTH = 4
) (
  input logic                         clk,
  input logic                         rst,
  input logic [CHANNEL_WIDTH-1:0]     rx_channel,
  input logic [CHANNEL_WIDTH-1:0]     hit_channel,
  input logic [44:0]                  hit_data,
  input logic                         hit_valid,
  input logic                         hit_sop,
  input logic                         hit_eop,
  input logic [2:0]                   hit_error,
  input logic [41:0]                  headerinfo_data,
  input logic                         headerinfo_valid,
  input logic [CHANNEL_WIDTH-1:0]     headerinfo_channel,
  input logic                         dbg_p_new_word,
  input logic [47:0]                  dbg_s_o_word,
  input logic [9:0]                   dbg_p_word_cnt,
  input logic [9:0]                   dbg_p_frame_len,
  input logic [5:0]                   dbg_p_frame_flags,
  input logic                         dbg_n_new_word,
  input logic [47:0]                  dbg_n_word,
  input logic [9:0]                   dbg_n_word_cnt,
  input logic [9:0]                   dbg_n_frame_len,
  input logic [15:0]                  dbg_n_frame_number,
  input logic [5:0]                   dbg_n_frame_flags,
  input logic                         dbg_n_frame_info_ready
);

  logic                         exp_hdr_pending;
  logic [41:0]                  exp_hdr_data;
  logic [CHANNEL_WIDTH-1:0]     exp_hdr_channel;

  function automatic logic [3:0] resize_channel_to_asic(
    input logic [CHANNEL_WIDTH-1:0] channel_v
  );
    logic [3:0] resized_v;
    begin
      resized_v = '0;
      if (CHANNEL_WIDTH >= 4) begin
        resized_v = channel_v[3:0];
      end else begin
        resized_v[CHANNEL_WIDTH-1:0] = channel_v;
      end
      return resized_v;
    end
  endfunction

  function automatic logic [44:0] pack_hit_data(
    input logic [47:0]              raw_word,
    input logic [5:0]               frame_flags,
    input logic [CHANNEL_WIDTH-1:0] channel_v
  );
    logic [44:0] packed_v;
    logic short_mode_v;
    begin
      packed_v     = '0;
      short_mode_v = (frame_flags[4:2] == 3'b100);
      packed_v[44:41] = resize_channel_to_asic(channel_v);
      packed_v[40:36] = raw_word[47:43];
      packed_v[35:21] = raw_word[41:27];
      packed_v[20:16] = raw_word[26:22];
      if (short_mode_v) begin
        packed_v[15:1] = '0;
        packed_v[0]    = raw_word[0];
      end else begin
        packed_v[15:1] = raw_word[19:5];
        packed_v[0]    = raw_word[20];
      end
      return packed_v;
    end
  endfunction

  function automatic logic [41:0] pack_headerinfo_data(
    input logic [15:0] frame_number_v,
    input logic [9:0]  frame_len_v,
    input logic [9:0]  word_cnt_v,
    input logic [5:0]  frame_flags_v
  );
    logic [41:0] packed_v;
    begin
      packed_v           = '0;
      packed_v[5:0]      = frame_flags_v;
      packed_v[15:6]     = frame_len_v;
      packed_v[25:16]    = word_cnt_v;
      packed_v[41:26]    = frame_number_v;
      return packed_v;
    end
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
      exp_hdr_pending <= 1'b0;
      exp_hdr_data    <= '0;
      exp_hdr_channel <= '0;
    end else begin
      exp_hdr_pending <= dbg_n_frame_info_ready;
      exp_hdr_data    <= pack_headerinfo_data(
        dbg_n_frame_number, dbg_n_frame_len, dbg_n_word_cnt, dbg_n_frame_flags);
      exp_hdr_channel <= rx_channel;
    end
  end

  property p_hit_pending_matches_output;
    @(posedge clk) disable iff (rst)
      dbg_p_new_word |-> (
        hit_valid &&
        (hit_data == pack_hit_data(dbg_s_o_word, dbg_p_frame_flags, rx_channel)) &&
        (hit_channel == rx_channel) &&
        (hit_sop == (dbg_p_word_cnt == 10'd1)) &&
        (hit_eop == (dbg_p_word_cnt == dbg_p_frame_len))
      );
  endproperty

  property p_no_hit_without_pending;
    @(posedge clk) disable iff (rst)
      !dbg_p_new_word |-> (!hit_valid && !hit_sop && !hit_eop);
  endproperty

  property p_markers_require_valid;
    @(posedge clk) disable iff (rst)
      (hit_sop || hit_eop) |-> hit_valid;
  endproperty

  property p_error1_only_on_eop;
    @(posedge clk) disable iff (rst)
      (hit_valid && hit_error[1]) |-> hit_eop;
  endproperty

  property p_header_pending_matches_output;
    @(posedge clk) disable iff (rst)
      exp_hdr_pending |-> (
        headerinfo_valid &&
        (headerinfo_data == exp_hdr_data) &&
        (headerinfo_channel == exp_hdr_channel)
      );
  endproperty

  property p_no_header_without_pending;
    @(posedge clk) disable iff (rst)
      !exp_hdr_pending |-> !headerinfo_valid;
  endproperty

  property p_headerinfo_single_cycle;
    @(posedge clk) disable iff (rst)
      headerinfo_valid |=> !headerinfo_valid;
  endproperty

  assert property (p_hit_pending_matches_output)
    else $error("FRCV_OUTPUT_SVA hit output mismatched the sampled parser word");

  assert property (p_no_hit_without_pending)
    else $error("FRCV_OUTPUT_SVA hit output fired without a sampled parser word");

  assert property (p_markers_require_valid)
    else $error("FRCV_OUTPUT_SVA SOP/EOP asserted without hit_valid");

  assert property (p_error1_only_on_eop)
    else $error("FRCV_OUTPUT_SVA CRC error bit asserted off-EOP");

  assert property (p_header_pending_matches_output)
    else $error("FRCV_OUTPUT_SVA headerinfo output mismatched the sampled frame header");

  assert property (p_no_header_without_pending)
    else $error("FRCV_OUTPUT_SVA headerinfo_valid fired without frame-info pulse");

  assert property (p_headerinfo_single_cycle)
    else $error("FRCV_OUTPUT_SVA headerinfo_valid must stay a single-cycle pulse");

endmodule
