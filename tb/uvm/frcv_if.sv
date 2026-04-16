`timescale 1ps/1ps

interface frcv_rx_if(input logic clk, input logic rst);
  logic [8:0] data;
  logic       valid;
  logic [2:0] error;
  logic [7:0] channel;

  modport drv (
    output data, valid, error, channel,
    input  clk, rst
  );

  modport mon (
    input data, valid, error, channel, clk, rst
  );
endinterface

interface frcv_ctrl_if(input logic clk, input logic rst);
  logic [8:0] data;
  logic       valid;
  logic       ready;

  modport drv (
    output data, valid,
    input  ready, clk, rst
  );

  modport mon (
    input data, valid, ready, clk, rst
  );
endinterface

interface frcv_csr_if(input logic clk, input logic rst);
  logic [7:0]  address;
  logic        read;
  logic        write;
  logic [31:0] writedata;
  logic [31:0] readdata;
  logic        waitrequest;

  modport drv (
    output address, read, write, writedata,
    input  readdata, waitrequest, clk, rst
  );

  modport mon (
    input address, read, write, writedata, readdata, waitrequest, clk, rst
  );
endinterface

interface frcv_out_if(input logic clk, input logic rst);
  logic [7:0]  hit_channel;
  logic        hit_sop;
  logic        hit_eop;
  logic [2:0]  hit_error;
  logic [44:0] hit_data;
  logic        hit_valid;

  logic [41:0] headerinfo_data;
  logic        headerinfo_valid;
  logic [7:0]  headerinfo_channel;

  modport mon (
    input hit_channel, hit_sop, hit_eop, hit_error, hit_data, hit_valid,
          headerinfo_data, headerinfo_valid, headerinfo_channel, clk, rst
  );
endinterface

interface frcv_reset_if(input logic clk);
  logic force_reset;

  modport drv (
    output force_reset,
    input  clk
  );

  modport mon (
    input force_reset, clk
  );
endinterface

interface frcv_dbg_if(input logic clk, input logic rst);
  logic        enable;
  logic        receiver_go;
  logic        receiver_force_go;
  logic        terminating_pending;
  logic [31:0] crc_err_counter;
  logic [31:0] frame_counter;
  logic [31:0] frame_counter_head;
  logic [31:0] frame_counter_tail;

  modport mon (
    input enable, receiver_go, receiver_force_go, terminating_pending,
          crc_err_counter, frame_counter, frame_counter_head, frame_counter_tail,
          clk, rst
  );
endinterface
