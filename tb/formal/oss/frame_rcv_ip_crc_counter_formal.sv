module frame_rcv_ip_crc_counter_formal;
  localparam logic [8:0] CTRL_RUNNING = 9'b000001000;

  (* gclk *) reg clk;

  reg started = 1'b0;
  reg [5:0] step = '0;
  reg saw_eop = 1'b0;

  logic [8:0]               asi_rx8b1k_data;
  logic                     asi_rx8b1k_valid;
  logic [2:0]               asi_rx8b1k_error;
  logic [3:0]               asi_rx8b1k_channel;
  logic [3:0]               aso_hit_type0_channel;
  logic                     aso_hit_type0_startofpacket;
  logic                     aso_hit_type0_endofpacket;
  logic                     aso_hit_type0_endofrun;
  logic [2:0]               aso_hit_type0_error;
  logic [44:0]              aso_hit_type0_data;
  logic                     aso_hit_type0_valid;
  logic [41:0]              aso_headerinfo_data;
  logic                     aso_headerinfo_valid;
  logic [3:0]               aso_headerinfo_channel;
  logic [31:0]              avs_csr_readdata;
  logic                     avs_csr_read;
  logic [1:0]               avs_csr_address;
  logic                     avs_csr_waitrequest;
  logic                     avs_csr_write;
  logic [31:0]              avs_csr_writedata;
  logic [8:0]               asi_ctrl_data;
  logic                     asi_ctrl_valid;
  logic                     asi_ctrl_ready;
  logic                     i_rst;

  frame_rcv_ip dut (
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
    .i_clk                       (clk)
  );

  always @* begin
    i_rst              = !started;
    asi_rx8b1k_data    = 9'h1bc;
    asi_rx8b1k_valid   = 1'b1;
    asi_rx8b1k_error   = 3'b000;
    asi_rx8b1k_channel = '0;

    avs_csr_read       = 1'b0;
    avs_csr_address    = '0;
    avs_csr_write      = 1'b0;
    avs_csr_writedata  = 32'h0000_0001;

    asi_ctrl_data      = CTRL_RUNNING;
    asi_ctrl_valid     = 1'b0;

    case (step)
      6'd0: avs_csr_write = 1'b1;
      6'd1: asi_ctrl_valid = 1'b1;
      6'd2: asi_rx8b1k_data = 9'h11c;
      6'd3: asi_rx8b1k_data = 9'h012;
      6'd4: asi_rx8b1k_data = 9'h034;
      6'd5: asi_rx8b1k_data = 9'h000;
      6'd6: asi_rx8b1k_data = 9'h001;
      6'd7: asi_rx8b1k_data = 9'h011;
      6'd8: asi_rx8b1k_data = 9'h022;
      6'd9: asi_rx8b1k_data = 9'h033;
      6'd10: asi_rx8b1k_data = 9'h044;
      6'd11: asi_rx8b1k_data = 9'h055;
      6'd12: asi_rx8b1k_data = 9'h066;
      6'd13: asi_rx8b1k_data = 9'h073;
      6'd14: asi_rx8b1k_data = 9'h018;
      6'd18: begin
        avs_csr_read    = 1'b1;
        avs_csr_address = 2'd1;
      end
      default: asi_rx8b1k_data = 9'h1bc;
    endcase
  end

  always @(posedge clk) begin
    if (!started) begin
      started <= 1'b1;
      step    <= '0;
      saw_eop <= 1'b0;
    end else if (step != 6'd20) begin
      step <= step + 6'd1;
      if (aso_hit_type0_valid && aso_hit_type0_endofpacket)
        saw_eop <= 1'b1;
    end

    if (started && step == 6'd19) begin
      assert(saw_eop);
      assert(avs_csr_readdata == 32'd1);
    end
  end
endmodule
