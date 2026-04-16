`timescale 1ps/1ps

module tb_top;
  import uvm_pkg::*;
  import frcv_env_pkg::*;
  `include "uvm_macros.svh"

  logic clk = 1'b0;
  logic rst = 1'b1;

  always #(CLK_PERIOD_PS/2) clk = ~clk;

  initial begin
    rst = 1'b1;
    #(10 * CLK_PERIOD_PS);
    rst = 1'b0;
  end

  frcv_rx_if   rx_if(.clk(clk), .rst(rst));
  frcv_ctrl_if ctrl_if(.clk(clk), .rst(rst));
  frcv_csr_if  csr_if(.clk(clk), .rst(rst));
  frcv_out_if  out_if(.clk(clk), .rst(rst));

  frame_rcv_ip #(
    .CHANNEL_WIDTH (4),
    .CSR_ADDR_WIDTH(2),
    .MODE_HALT     (0),
    .DEBUG_LV      (0)
  ) dut (
    .asi_rx8b1k_data             (rx_if.data),
    .asi_rx8b1k_valid            (rx_if.valid),
    .asi_rx8b1k_error            (rx_if.error),
    .asi_rx8b1k_channel          (rx_if.channel),
    .aso_hit_type0_channel       (out_if.hit_channel),
    .aso_hit_type0_startofpacket (out_if.hit_sop),
    .aso_hit_type0_endofpacket   (out_if.hit_eop),
    .aso_hit_type0_error         (out_if.hit_error),
    .aso_hit_type0_data          (out_if.hit_data),
    .aso_hit_type0_valid         (out_if.hit_valid),
    .aso_headerinfo_data         (out_if.headerinfo_data),
    .aso_headerinfo_valid        (out_if.headerinfo_valid),
    .aso_headerinfo_channel      (out_if.headerinfo_channel),
    .avs_csr_readdata            (csr_if.readdata),
    .avs_csr_read                (csr_if.read),
    .avs_csr_address             (csr_if.address),
    .avs_csr_waitrequest         (csr_if.waitrequest),
    .avs_csr_write               (csr_if.write),
    .avs_csr_writedata           (csr_if.writedata),
    .asi_ctrl_data               (ctrl_if.data),
    .asi_ctrl_valid              (ctrl_if.valid),
    .asi_ctrl_ready              (ctrl_if.ready),
    .i_rst                       (rst),
    .i_clk                       (clk)
  );

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

    run_test();
  end

  initial begin
    #(2_000_000ns);
    `uvm_fatal("TB_TOP", "Global timeout reached")
  end
endmodule
