`timescale 1ps/1ps

`ifndef TB_CHANNEL_WIDTH
`define TB_CHANNEL_WIDTH 4
`endif

`ifndef TB_CSR_ADDR_WIDTH
`define TB_CSR_ADDR_WIDTH 2
`endif

`ifndef TB_MODE_HALT
`define TB_MODE_HALT 0
`endif

`ifndef TB_DEBUG_LV
`define TB_DEBUG_LV 0
`endif

module tb_top;
  import uvm_pkg::*;
  import frcv_env_pkg::*;
  `include "uvm_macros.svh"

  localparam int TB_CHANNEL_WIDTH  = `TB_CHANNEL_WIDTH;
  localparam int TB_CSR_ADDR_WIDTH = `TB_CSR_ADDR_WIDTH;
  localparam int TB_MODE_HALT      = `TB_MODE_HALT;
  localparam int TB_DEBUG_LV       = `TB_DEBUG_LV;
  time global_timeout = 2_000_000ns;
  int unsigned global_timeout_ns;

  logic clk = 1'b0;
  logic rst_init = 1'b1;
  wire  rst;

  logic [TB_CHANNEL_WIDTH-1:0]  dut_rx_channel;
  logic [TB_CHANNEL_WIDTH-1:0]  dut_hit_channel;
  logic                         dut_hit_endofrun;
  logic [TB_CHANNEL_WIDTH-1:0]  dut_headerinfo_channel;
  logic [TB_CSR_ADDR_WIDTH-1:0] dut_csr_address;
  logic                         dbg_enable;
  logic                         dbg_receiver_go;
  logic                         dbg_receiver_force_go;
  logic                         dbg_terminating_pending;
  logic [31:0]                  dbg_crc_err_counter;
  logic [31:0]                  dbg_frame_counter;
  logic [31:0]                  dbg_frame_counter_head;
  logic [31:0]                  dbg_frame_counter_tail;

  always #(CLK_PERIOD_PS/2) clk = ~clk;

  initial begin
    rst_init = 1'b1;
    #(10 * CLK_PERIOD_PS);
    rst_init = 1'b0;
  end

  frcv_rx_if   rx_if(.clk(clk), .rst(rst));
  frcv_ctrl_if ctrl_if(.clk(clk), .rst(rst));
  frcv_csr_if  csr_if(.clk(clk), .rst(rst));
  frcv_out_if  out_if(.clk(clk), .rst(rst));
  frcv_reset_if reset_if(.clk(clk));
  frcv_dbg_if  dbg_if(.clk(clk), .rst(rst));

  assign rst             = rst_init | reset_if.force_reset;
  assign dut_rx_channel  = rx_if.channel[TB_CHANNEL_WIDTH-1:0];
  assign dut_csr_address = csr_if.address[TB_CSR_ADDR_WIDTH-1:0];
  assign out_if.hit_channel        = {{(8-TB_CHANNEL_WIDTH){1'b0}}, dut_hit_channel};
  assign out_if.hit_endofrun       = dut_hit_endofrun;
  assign out_if.headerinfo_channel = {{(8-TB_CHANNEL_WIDTH){1'b0}}, dut_headerinfo_channel};
  assign dbg_if.enable             = dbg_enable;
  assign dbg_if.receiver_go        = dbg_receiver_go;
  assign dbg_if.receiver_force_go  = dbg_receiver_force_go;
  assign dbg_if.terminating_pending = dbg_terminating_pending;
  assign dbg_if.crc_err_counter    = dbg_crc_err_counter;
  assign dbg_if.frame_counter      = dbg_frame_counter;
  assign dbg_if.frame_counter_head = dbg_frame_counter_head;
  assign dbg_if.frame_counter_tail = dbg_frame_counter_tail;

  frame_rcv_ip #(
    .CHANNEL_WIDTH (TB_CHANNEL_WIDTH),
    .CSR_ADDR_WIDTH(TB_CSR_ADDR_WIDTH),
    .MODE_HALT     (TB_MODE_HALT),
    .DEBUG_LV      (TB_DEBUG_LV)
  ) dut (
    .asi_rx8b1k_data             (rx_if.data),
    .asi_rx8b1k_valid            (rx_if.valid),
    .asi_rx8b1k_error            (rx_if.error),
    .asi_rx8b1k_channel          (dut_rx_channel),
    .aso_hit_type0_channel       (dut_hit_channel),
    .aso_hit_type0_startofpacket (out_if.hit_sop),
    .aso_hit_type0_endofpacket   (out_if.hit_eop),
    .aso_hit_type0_endofrun      (dut_hit_endofrun),
    .aso_hit_type0_error         (out_if.hit_error),
    .aso_hit_type0_data          (out_if.hit_data),
    .aso_hit_type0_valid         (out_if.hit_valid),
    .aso_headerinfo_data         (out_if.headerinfo_data),
    .aso_headerinfo_valid        (out_if.headerinfo_valid),
    .aso_headerinfo_channel      (dut_headerinfo_channel),
    .avs_csr_readdata            (csr_if.readdata),
    .avs_csr_read                (csr_if.read),
    .avs_csr_address             (dut_csr_address),
    .avs_csr_waitrequest         (csr_if.waitrequest),
    .avs_csr_write               (csr_if.write),
    .avs_csr_writedata           (csr_if.writedata),
    .asi_ctrl_data               (ctrl_if.data),
    .asi_ctrl_valid              (ctrl_if.valid),
    .asi_ctrl_ready              (ctrl_if.ready),
    .i_rst                       (rst),
    .i_clk                       (clk)
  );

  assign dbg_enable             = dut.enable;
  assign dbg_receiver_go        = dut.receiver_go;
  assign dbg_receiver_force_go  = dut.receiver_force_go;
  assign dbg_terminating_pending = dut.terminating_pending;
  assign dbg_crc_err_counter    = dut.csr.crc_err_counter;
  assign dbg_frame_counter      = dut.csr.frame_counter;
  assign dbg_frame_counter_head = dut.csr.frame_counter_head;
  assign dbg_frame_counter_tail = dut.csr.frame_counter_tail;

  property p_hit_sop_requires_valid;
    @(posedge clk) disable iff (rst)
      out_if.hit_sop |-> out_if.hit_valid;
  endproperty

  property p_headerinfo_not_unknown;
    @(posedge clk) disable iff (rst)
      out_if.headerinfo_valid |-> !$isunknown(out_if.headerinfo_data);
  endproperty

  assert property (p_hit_sop_requires_valid)
    else $error("hit_sop asserted without hit_valid");

  assert property (p_headerinfo_not_unknown)
    else $error("headerinfo_data contains X/Z while valid");

  initial begin
    reset_if.force_reset = 1'b0;

    uvm_config_db#(virtual frcv_rx_if.drv)::set(
      null, "uvm_test_top.m_env.m_rx_drv", "vif", rx_if);
    uvm_config_db#(virtual frcv_ctrl_if.drv)::set(
      null, "uvm_test_top.m_env.m_ctrl_drv", "vif", ctrl_if);
    uvm_config_db#(virtual frcv_csr_if.drv)::set(
      null, "uvm_test_top.m_env.m_csr_drv", "vif", csr_if);
    uvm_config_db#(virtual frcv_out_if.mon)::set(
      null, "uvm_test_top.m_env.m_out_mon", "vif", out_if);
    uvm_config_db#(virtual frcv_ctrl_if.mon)::set(
      null, "uvm_test_top", "ctrl_vif", ctrl_if);
    uvm_config_db#(virtual frcv_csr_if.mon)::set(
      null, "uvm_test_top", "csr_vif", csr_if);
    uvm_config_db#(virtual frcv_out_if.mon)::set(
      null, "uvm_test_top", "out_vif", out_if);
    uvm_config_db#(virtual frcv_reset_if.drv)::set(
      null, "uvm_test_top", "reset_vif", reset_if);
    uvm_config_db#(virtual frcv_rx_if.mon)::set(
      null, "uvm_test_top.m_env.m_rx_mon", "vif", rx_if);
    uvm_config_db#(virtual frcv_ctrl_if.mon)::set(
      null, "uvm_test_top.m_env.m_ctrl_mon", "vif", ctrl_if);
    uvm_config_db#(virtual frcv_csr_if.mon)::set(
      null, "uvm_test_top.m_env.m_csr_mon", "vif", csr_if);
    uvm_config_db#(virtual frcv_dbg_if.mon)::set(
      null, "uvm_test_top.m_env.m_dbg_mon", "vif", dbg_if);
    uvm_config_db#(virtual frcv_dbg_if.mon)::set(
      null, "uvm_test_top.m_env.m_scb", "dbg_vif", dbg_if);

    run_test();
  end

  initial begin
    global_timeout_ns = 2_000_000;
    void'($value$plusargs("FRCV_TIMEOUT_NS=%d", global_timeout_ns));
    global_timeout = global_timeout_ns * 1ns;
    #(global_timeout);
    `uvm_fatal("TB_TOP", "Global timeout reached")
  end
endmodule
