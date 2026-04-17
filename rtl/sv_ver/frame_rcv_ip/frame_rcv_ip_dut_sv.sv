//------------------------------------------------------------------------------
// frame_rcv_ip_dut_sv
// Description:
//   Thin SystemVerilog-facing shell around the active VHDL `frame_rcv_ip`.
//   This keeps the packaged RTL intact while exporting the internal next-state
//   and output-contract taps needed by the UVM SVA layer and future formal work.
//------------------------------------------------------------------------------
module frame_rcv_ip_dut_sv #(
  parameter int CHANNEL_WIDTH  = 4,
  parameter int CSR_ADDR_WIDTH = 2,
  parameter int MODE_HALT      = 0,
  parameter int DEBUG_LV       = 0
) (
  input  logic [8:0]                     asi_rx8b1k_data,
  input  logic                           asi_rx8b1k_valid,
  input  logic [2:0]                     asi_rx8b1k_error,
  input  logic [CHANNEL_WIDTH-1:0]       asi_rx8b1k_channel,
  output logic [CHANNEL_WIDTH-1:0]       aso_hit_type0_channel,
  output logic                           aso_hit_type0_startofpacket,
  output logic                           aso_hit_type0_endofpacket,
  output logic                           aso_hit_type0_endofrun,
  output logic [2:0]                     aso_hit_type0_error,
  output logic [44:0]                    aso_hit_type0_data,
  output logic                           aso_hit_type0_valid,
  output logic [41:0]                    aso_headerinfo_data,
  output logic                           aso_headerinfo_valid,
  output logic [CHANNEL_WIDTH-1:0]       aso_headerinfo_channel,
  output logic [31:0]                    avs_csr_readdata,
  input  logic                           avs_csr_read,
  input  logic [CSR_ADDR_WIDTH-1:0]      avs_csr_address,
  output logic                           avs_csr_waitrequest,
  input  logic                           avs_csr_write,
  input  logic [31:0]                    avs_csr_writedata,
  input  logic [8:0]                     asi_ctrl_data,
  input  logic                           asi_ctrl_valid,
  output logic                           asi_ctrl_ready,
  input  logic                           i_rst,
  input  logic                           i_clk,
  output logic                           dbg_enable,
  output logic                           dbg_receiver_go,
  output logic                           dbg_receiver_force_go,
  output logic                           dbg_terminating_pending,
  output logic [31:0]                    dbg_crc_err_counter,
  output logic [31:0]                    dbg_frame_counter,
  output logic [31:0]                    dbg_frame_counter_head,
  output logic [31:0]                    dbg_frame_counter_tail,
  output logic                           dbg_n_new_frame,
  output logic                           dbg_n_new_word,
  output logic                           dbg_p_new_word,
  output logic [47:0]                    dbg_n_word,
  output logic [47:0]                    dbg_s_o_word,
  output logic [9:0]                     dbg_n_word_cnt,
  output logic [9:0]                     dbg_p_word_cnt,
  output logic [9:0]                     dbg_n_frame_len,
  output logic [9:0]                     dbg_p_frame_len,
  output logic [15:0]                    dbg_n_frame_number,
  output logic [5:0]                     dbg_n_frame_flags,
  output logic [5:0]                     dbg_p_frame_flags,
  output logic                           dbg_n_frame_info_ready,
  output logic                           dbg_n_frame_info_ready_d1,
  output logic                           dbg_headerinfo_valid_comb,
  output logic                           dbg_n_crc_error,
  output logic [31:0]                    dbg_p_crc_err_count
);

  frame_rcv_ip #(
    .CHANNEL_WIDTH (CHANNEL_WIDTH),
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH),
    .MODE_HALT     (MODE_HALT),
    .DEBUG_LV      (DEBUG_LV)
  ) dut (
    .asi_rx8b1k_data             (asi_rx8b1k_data),
    .asi_rx8b1k_valid            (asi_rx8b1k_valid),
    .asi_rx8b1k_error            (asi_rx8b1k_error),
    .asi_rx8b1k_channel          (asi_rx8b1k_channel),
    .aso_hit_type0_channel       (aso_hit_type0_channel),
    .aso_hit_type0_startofpacket (aso_hit_type0_startofpacket),
    .aso_hit_type0_endofpacket   (aso_hit_type0_endofpacket),
    .aso_hit_type0_endofrun      (aso_hit_type0_endofrun),
    .aso_hit_type0_error         (aso_hit_type0_error),
    .aso_hit_type0_data          (aso_hit_type0_data),
    .aso_hit_type0_valid         (aso_hit_type0_valid),
    .aso_headerinfo_data         (aso_headerinfo_data),
    .aso_headerinfo_valid        (aso_headerinfo_valid),
    .aso_headerinfo_channel      (aso_headerinfo_channel),
    .avs_csr_readdata            (avs_csr_readdata),
    .avs_csr_read                (avs_csr_read),
    .avs_csr_address             (avs_csr_address),
    .avs_csr_waitrequest         (avs_csr_waitrequest),
    .avs_csr_write               (avs_csr_write),
    .avs_csr_writedata           (avs_csr_writedata),
    .asi_ctrl_data               (asi_ctrl_data),
    .asi_ctrl_valid              (asi_ctrl_valid),
    .asi_ctrl_ready              (asi_ctrl_ready),
    .i_rst                       (i_rst),
    .i_clk                       (i_clk)
  );

  assign dbg_enable               = dut.enable;
  assign dbg_receiver_go          = dut.receiver_go;
  assign dbg_receiver_force_go    = dut.receiver_force_go;
  assign dbg_terminating_pending  = dut.terminating_pending;
  assign dbg_crc_err_counter      = dut.csr.crc_err_counter;
  assign dbg_frame_counter        = dut.csr.frame_counter;
  assign dbg_frame_counter_head   = dut.csr.frame_counter_head;
  assign dbg_frame_counter_tail   = dut.csr.frame_counter_tail;
  assign dbg_n_new_frame          = dut.n_new_frame;
  assign dbg_n_new_word           = dut.n_new_word;
  assign dbg_p_new_word           = dut.p_new_word;
  assign dbg_n_word               = dut.n_word;
  assign dbg_s_o_word             = dut.s_o_word;
  assign dbg_n_word_cnt           = dut.n_word_cnt;
  assign dbg_p_word_cnt           = dut.p_word_cnt;
  assign dbg_n_frame_len          = dut.n_frame_len;
  assign dbg_p_frame_len          = dut.p_frame_len;
  assign dbg_n_frame_number       = dut.n_frame_number;
  assign dbg_n_frame_flags        = dut.n_frame_flags;
  assign dbg_p_frame_flags        = dut.p_frame_flags;
  assign dbg_n_frame_info_ready   = dut.n_frame_info_ready;
  assign dbg_n_frame_info_ready_d1 = dut.n_frame_info_ready_d1;
  assign dbg_headerinfo_valid_comb = dut.aso_headerinfo_valid_comb;
  assign dbg_n_crc_error          = dut.n_crc_error;
  assign dbg_p_crc_err_count      = dut.p_crc_err_count;

endmodule
