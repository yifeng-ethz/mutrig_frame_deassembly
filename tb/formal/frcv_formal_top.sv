//------------------------------------------------------------------------------
// frcv_formal_top
// Description:
//   Mixed-language formal-facing harness for `frame_rcv_ip`.
//   The active VHDL DUT stays intact; this shell adds packet-shape assumptions
//   and output/counter assertions around it.
//------------------------------------------------------------------------------
module frcv_formal_top #(
  parameter int CHANNEL_WIDTH  = 4,
  parameter int CSR_ADDR_WIDTH = 2,
  parameter int MODE_HALT      = 0,
  parameter int DEBUG_LV       = 0
) (
  input logic                     i_clk,
  input logic                     i_rst,
  input logic [8:0]               sym_rx_data,
  input logic                     sym_rx_valid,
  input logic [2:0]               sym_rx_error,
  input logic [CHANNEL_WIDTH-1:0] sym_rx_channel
);

  logic [CHANNEL_WIDTH-1:0] aso_hit_type0_channel;
  logic                     aso_hit_type0_startofpacket;
  logic                     aso_hit_type0_endofpacket;
  logic                     aso_hit_type0_endofrun;
  logic [2:0]               aso_hit_type0_error;
  logic [44:0]              aso_hit_type0_data;
  logic                     aso_hit_type0_valid;
  logic [41:0]              aso_headerinfo_data;
  logic                     aso_headerinfo_valid;
  logic [CHANNEL_WIDTH-1:0] aso_headerinfo_channel;
  logic [31:0]              avs_csr_readdata;
  logic                     avs_csr_waitrequest;
  logic                     asi_ctrl_ready;

  logic                     dbg_enable;
  logic                     dbg_receiver_go;
  logic                     dbg_receiver_force_go;
  logic                     dbg_terminating_pending;
  logic [31:0]              dbg_crc_err_counter;
  logic [31:0]              dbg_frame_counter;
  logic [31:0]              dbg_frame_counter_head;
  logic [31:0]              dbg_frame_counter_tail;
  logic                     dbg_n_new_frame;
  logic                     dbg_n_new_word;
  logic                     dbg_p_new_word;
  logic [47:0]              dbg_n_word;
  logic [47:0]              dbg_s_o_word;
  logic [9:0]               dbg_n_word_cnt;
  logic [9:0]               dbg_p_word_cnt;
  logic [9:0]               dbg_n_frame_len;
  logic [9:0]               dbg_p_frame_len;
  logic [15:0]              dbg_n_frame_number;
  logic [5:0]               dbg_n_frame_flags;
  logic [5:0]               dbg_p_frame_flags;
  logic                     dbg_n_frame_info_ready;
  logic                     dbg_n_frame_info_ready_d1;
  logic                     dbg_headerinfo_valid_comb;
  logic                     dbg_n_crc_error;
  logic [31:0]              dbg_p_crc_err_count;

  logic                     cfg_written_q;
  logic                     run_sent_q;
  logic                     avs_csr_read;
  logic [CSR_ADDR_WIDTH-1:0] avs_csr_address;
  logic                     avs_csr_write;
  logic [31:0]              avs_csr_writedata;
  logic [8:0]               asi_ctrl_data;
  logic                     asi_ctrl_valid;

  always_ff @(posedge i_clk) begin
    if (i_rst) begin
      cfg_written_q    <= 1'b0;
      run_sent_q       <= 1'b0;
      avs_csr_read     <= 1'b0;
      avs_csr_address  <= '0;
      avs_csr_write    <= 1'b0;
      avs_csr_writedata <= 32'h0000_0001;
      asi_ctrl_data    <= 9'b000001000;
      asi_ctrl_valid   <= 1'b0;
    end else begin
      avs_csr_read     <= 1'b0;
      avs_csr_address  <= '0;
      avs_csr_writedata <= 32'h0000_0001;

      if (!cfg_written_q) begin
        avs_csr_write <= 1'b1;
        cfg_written_q <= 1'b1;
      end else begin
        avs_csr_write <= 1'b0;
      end

      if (!run_sent_q) begin
        asi_ctrl_valid <= 1'b1;
        run_sent_q     <= 1'b1;
      end else begin
        asi_ctrl_valid <= 1'b0;
      end

      asi_ctrl_data <= 9'b000001000;
    end
  end

  frame_rcv_ip_dut_sv #(
    .CHANNEL_WIDTH (CHANNEL_WIDTH),
    .CSR_ADDR_WIDTH(CSR_ADDR_WIDTH),
    .MODE_HALT     (MODE_HALT),
    .DEBUG_LV      (DEBUG_LV)
  ) dut (
    .asi_rx8b1k_data             (sym_rx_data),
    .asi_rx8b1k_valid            (sym_rx_valid),
    .asi_rx8b1k_error            (sym_rx_error),
    .asi_rx8b1k_channel          (sym_rx_channel),
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
    .i_clk                       (i_clk),
    .dbg_enable                  (dbg_enable),
    .dbg_receiver_go             (dbg_receiver_go),
    .dbg_receiver_force_go       (dbg_receiver_force_go),
    .dbg_terminating_pending     (dbg_terminating_pending),
    .dbg_crc_err_counter         (dbg_crc_err_counter),
    .dbg_frame_counter           (dbg_frame_counter),
    .dbg_frame_counter_head      (dbg_frame_counter_head),
    .dbg_frame_counter_tail      (dbg_frame_counter_tail),
    .dbg_n_new_frame             (dbg_n_new_frame),
    .dbg_n_new_word              (dbg_n_new_word),
    .dbg_p_new_word              (dbg_p_new_word),
    .dbg_n_word                  (dbg_n_word),
    .dbg_s_o_word                (dbg_s_o_word),
    .dbg_n_word_cnt              (dbg_n_word_cnt),
    .dbg_p_word_cnt              (dbg_p_word_cnt),
    .dbg_n_frame_len             (dbg_n_frame_len),
    .dbg_p_frame_len             (dbg_p_frame_len),
    .dbg_n_frame_number          (dbg_n_frame_number),
    .dbg_n_frame_flags           (dbg_n_frame_flags),
    .dbg_p_frame_flags           (dbg_p_frame_flags),
    .dbg_n_frame_info_ready      (dbg_n_frame_info_ready),
    .dbg_n_frame_info_ready_d1   (dbg_n_frame_info_ready_d1),
    .dbg_headerinfo_valid_comb   (dbg_headerinfo_valid_comb),
    .dbg_n_crc_error             (dbg_n_crc_error),
    .dbg_p_crc_err_count         (dbg_p_crc_err_count)
  );

  frcv_formal_pkt_assumptions #(
    .CHANNEL_WIDTH(CHANNEL_WIDTH)
  ) u_pkt_assumptions (
    .clk       (i_clk),
    .rst       (i_rst),
    .rx_data   (sym_rx_data),
    .rx_valid  (sym_rx_valid),
    .rx_error  (sym_rx_error),
    .rx_channel(sym_rx_channel),
    .dbg_enable(dbg_enable)
  );

  frcv_parser_boundary_sva u_parser_contract (
    .clk           (i_clk),
    .rst           (i_rst),
    .rx_data       (sym_rx_data),
    .rx_valid      (sym_rx_valid),
    .dbg_enable    (dbg_enable),
    .dbg_n_new_frame(dbg_n_new_frame),
    .dbg_n_new_word(dbg_n_new_word),
    .dbg_n_crc_error(dbg_n_crc_error)
  );

  frcv_output_contract_sva #(
    .CHANNEL_WIDTH(CHANNEL_WIDTH)
  ) u_output_contract (
    .clk                 (i_clk),
    .rst                 (i_rst),
    .rx_channel          (sym_rx_channel),
    .hit_channel         (aso_hit_type0_channel),
    .hit_data            (aso_hit_type0_data),
    .hit_valid           (aso_hit_type0_valid),
    .hit_sop             (aso_hit_type0_startofpacket),
    .hit_eop             (aso_hit_type0_endofpacket),
    .hit_error           (aso_hit_type0_error),
    .headerinfo_data     (aso_headerinfo_data),
    .headerinfo_valid    (aso_headerinfo_valid),
    .headerinfo_channel  (aso_headerinfo_channel),
    .dbg_p_new_word      (dbg_p_new_word),
    .dbg_s_o_word        (dbg_s_o_word),
    .dbg_p_word_cnt      (dbg_p_word_cnt),
    .dbg_p_frame_len     (dbg_p_frame_len),
    .dbg_p_frame_flags   (dbg_p_frame_flags),
    .dbg_n_new_word      (dbg_n_new_word),
    .dbg_n_word          (dbg_n_word),
    .dbg_n_word_cnt      (dbg_n_word_cnt),
    .dbg_n_frame_len     (dbg_n_frame_len),
    .dbg_n_frame_number  (dbg_n_frame_number),
    .dbg_n_frame_flags   (dbg_n_frame_flags),
    .dbg_n_frame_info_ready(dbg_n_frame_info_ready)
  );

  frcv_counter_contract_sva u_counter_contract (
    .clk                 (i_clk),
    .rst                 (i_rst),
    .hit_sop             (aso_hit_type0_startofpacket),
    .hit_eop             (aso_hit_type0_endofpacket),
    .dbg_n_crc_error     (dbg_n_crc_error),
    .dbg_p_crc_err_count (dbg_p_crc_err_count),
    .dbg_frame_counter_head(dbg_frame_counter_head),
    .dbg_frame_counter_tail(dbg_frame_counter_tail)
  );

endmodule
