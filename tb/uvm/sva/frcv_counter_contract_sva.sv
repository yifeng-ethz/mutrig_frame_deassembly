//------------------------------------------------------------------------------
// frcv_counter_contract_sva
// Description:
//   Counter-side contract checks for CRC and frame accounting.
//------------------------------------------------------------------------------
module frcv_counter_contract_sva (
  input logic        clk,
  input logic        rst,
  input logic        hit_sop,
  input logic        hit_eop,
  input logic        dbg_n_crc_error,
  input logic [31:0] dbg_p_crc_err_count,
  input logic [31:0] dbg_frame_counter_head,
  input logic [31:0] dbg_frame_counter_tail
);

  logic        exp_crc_pending;
  logic [31:0] exp_crc_count;
  logic        exp_sop_pending;
  logic [31:0] exp_head_count;
  logic        exp_eop_pending;
  logic [31:0] exp_tail_count;

  always_ff @(posedge clk) begin
    if (rst) begin
      exp_crc_pending <= 1'b0;
      exp_crc_count   <= '0;
      exp_sop_pending <= 1'b0;
      exp_head_count  <= '0;
      exp_eop_pending <= 1'b0;
      exp_tail_count  <= '0;
    end else begin
      exp_crc_pending <= 1'b1;
      exp_crc_count   <= dbg_p_crc_err_count + {{31{1'b0}}, dbg_n_crc_error};
      exp_sop_pending <= hit_sop;
      exp_head_count  <= dbg_frame_counter_head + 32'd1;
      exp_eop_pending <= hit_eop;
      exp_tail_count  <= dbg_frame_counter_tail + 32'd1;
    end
  end

  property p_crc_count_matches_next_state;
    @(posedge clk) disable iff (rst)
      exp_crc_pending |-> (dbg_p_crc_err_count == exp_crc_count);
  endproperty

  property p_head_counter_follows_sop;
    @(posedge clk) disable iff (rst)
      exp_sop_pending |-> (dbg_frame_counter_head == exp_head_count);
  endproperty

  property p_tail_counter_follows_eop;
    @(posedge clk) disable iff (rst)
      exp_eop_pending |-> (dbg_frame_counter_tail == exp_tail_count);
  endproperty

  property p_head_not_below_tail;
    @(posedge clk) disable iff (rst)
      (dbg_frame_counter_head >= dbg_frame_counter_tail);
  endproperty

  assert property (p_crc_count_matches_next_state)
    else $error("FRCV_COUNTER_SVA CRC error counter diverged from n_crc_error");

  assert property (p_head_counter_follows_sop)
    else $error("FRCV_COUNTER_SVA frame_counter_head missed a SOP");

  assert property (p_tail_counter_follows_eop)
    else $error("FRCV_COUNTER_SVA frame_counter_tail missed an EOP");

  assert property (p_head_not_below_tail)
    else $error("FRCV_COUNTER_SVA frame_counter_head dropped below frame_counter_tail");

endmodule
