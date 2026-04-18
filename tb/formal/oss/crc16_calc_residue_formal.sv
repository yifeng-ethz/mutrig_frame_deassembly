module crc16_calc_residue_formal;
  localparam int unsigned MAX_PREFIX_BYTES = 4;

  (* gclk *) reg clk;
  (* anyconst *) reg [2:0] prefix_len;
  (* anyseq *) reg [7:0] prefix_byte;

  reg        started = 1'b0;
  reg [2:0]  step_count = '0;
  reg [15:0] prefix_crc = 16'hffff;

  reg        rst = 1'b1;
  reg        d_valid = 1'b0;
  reg [7:0]  din = 8'h00;

  wire [15:0] dut_crc_reg;
  wire [7:0]  dut_crc_8;

  crc16_calc dut (
    .i_clk(clk),
    .i_rst(rst),
    .i_d_valid(d_valid),
    .i_din(din),
    .o_crc_reg(dut_crc_reg),
    .o_crc_8(dut_crc_8)
  );

  function automatic [15:0] crc16_next(input [15:0] crc_reg, input [7:0] data_byte);
    reg [15:0] next_crc;
    begin
      next_crc[0]  = crc_reg[15] ^ data_byte[7] ^ crc_reg[14] ^ data_byte[6] ^
                     crc_reg[13] ^ data_byte[5] ^ crc_reg[12] ^ data_byte[4] ^
                     crc_reg[11] ^ data_byte[3] ^ crc_reg[10] ^ data_byte[2] ^
                     crc_reg[9]  ^ data_byte[1] ^ crc_reg[8]  ^ data_byte[0];
      next_crc[1]  = crc_reg[15] ^ data_byte[7] ^ crc_reg[14] ^ data_byte[6] ^
                     crc_reg[13] ^ data_byte[5] ^ crc_reg[12] ^ data_byte[4] ^
                     crc_reg[11] ^ data_byte[3] ^ crc_reg[10] ^ data_byte[2] ^
                     crc_reg[9]  ^ data_byte[1];
      next_crc[2]  = crc_reg[9]  ^ data_byte[1] ^ crc_reg[8]  ^ data_byte[0];
      next_crc[3]  = crc_reg[10] ^ data_byte[2] ^ crc_reg[9]  ^ data_byte[1];
      next_crc[4]  = crc_reg[11] ^ data_byte[3] ^ crc_reg[10] ^ data_byte[2];
      next_crc[5]  = crc_reg[12] ^ data_byte[4] ^ crc_reg[11] ^ data_byte[3];
      next_crc[6]  = crc_reg[13] ^ data_byte[5] ^ crc_reg[12] ^ data_byte[4];
      next_crc[7]  = crc_reg[14] ^ data_byte[6] ^ crc_reg[13] ^ data_byte[5];
      next_crc[8]  = crc_reg[15] ^ data_byte[7] ^ crc_reg[14] ^ data_byte[6] ^ crc_reg[0];
      next_crc[9]  = crc_reg[15] ^ data_byte[7] ^ crc_reg[1];
      next_crc[10] = crc_reg[2];
      next_crc[11] = crc_reg[3];
      next_crc[12] = crc_reg[4];
      next_crc[13] = crc_reg[5];
      next_crc[14] = crc_reg[6];
      next_crc[15] = crc_reg[15] ^ data_byte[7] ^ crc_reg[14] ^ data_byte[6] ^
                     crc_reg[13] ^ data_byte[5] ^ crc_reg[12] ^ data_byte[4] ^
                     crc_reg[11] ^ data_byte[3] ^ crc_reg[10] ^ data_byte[2] ^
                     crc_reg[9]  ^ data_byte[1] ^ crc_reg[8]  ^ data_byte[0] ^
                     crc_reg[7];
      crc16_next = next_crc;
    end
  endfunction

  always @* begin
    rst = !started;
    d_valid = 1'b0;
    din = 8'h00;
    if (started) begin
      if (step_count < prefix_len) begin
        d_valid = 1'b1;
        din = prefix_byte;
      end else if (step_count == prefix_len) begin
        d_valid = 1'b1;
        din = ~prefix_crc[15:8];
      end else if (step_count == (prefix_len + 1'b1)) begin
        d_valid = 1'b1;
        din = ~prefix_crc[7:0];
      end
    end
  end

  always @(posedge clk) begin
    assume(prefix_len <= MAX_PREFIX_BYTES);
    if (!started) begin
      started <= 1'b1;
      step_count <= '0;
      prefix_crc <= 16'hffff;
    end else begin
      if (d_valid) begin
        if (step_count < prefix_len)
          prefix_crc <= crc16_next(prefix_crc, din);
        step_count <= step_count + 1'b1;
      end

      if (!d_valid && step_count == (prefix_len + 3'd2))
        assert(dut_crc_reg == 16'h7ff2);
    end
  end
endmodule
