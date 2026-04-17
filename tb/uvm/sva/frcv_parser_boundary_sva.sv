//------------------------------------------------------------------------------
// frcv_parser_boundary_sva
// Description:
//   Lightweight parser-boundary assertions that connect the byte-lane contract
//   to the internal next-state packet markers.
//------------------------------------------------------------------------------
module frcv_parser_boundary_sva (
  input logic       clk,
  input logic       rst,
  input logic [8:0] rx_data,
  input logic       rx_valid,
  input logic       dbg_enable,
  input logic       dbg_n_new_frame,
  input logic       dbg_n_new_word,
  input logic       dbg_n_crc_error
);

  property p_new_frame_requires_enabled_header;
    @(posedge clk) disable iff (rst)
      dbg_n_new_frame |-> (dbg_enable && rx_valid && rx_data[8] && (rx_data[7:0] == 8'h1C));
  endproperty

  property p_new_word_is_payload_byte;
    @(posedge clk) disable iff (rst)
      dbg_n_new_word |-> !rx_data[8];
  endproperty

  property p_crc_error_not_concurrent_with_word_accept;
    @(posedge clk) disable iff (rst)
      dbg_n_crc_error |-> (!dbg_n_new_word && !dbg_n_new_frame);
  endproperty

  assert property (p_new_frame_requires_enabled_header)
    else $error("FRCV_PARSER_SVA new frame fired without enabled K28.0 header");

  assert property (p_new_word_is_payload_byte)
    else $error("FRCV_PARSER_SVA new word fired while current byte was marked as a K-symbol");

  assert property (p_crc_error_not_concurrent_with_word_accept)
    else $error("FRCV_PARSER_SVA CRC error overlapped a new frame/word pulse");

endmodule
