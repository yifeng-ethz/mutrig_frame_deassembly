`timescale 1ps/1ps

package frcv_env_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  `uvm_analysis_imp_decl(_obs)
  `uvm_analysis_imp_decl(_rx)
  `uvm_analysis_imp_decl(_ctrl)
  `uvm_analysis_imp_decl(_csr)
  `uvm_analysis_imp_decl(_dbg)

  localparam logic [8:0] CTRL_IDLE        = 9'b000000001;
  localparam logic [8:0] CTRL_RUN_PREPARE = 9'b000000010;
  localparam logic [8:0] CTRL_SYNC        = 9'b000000100;
  localparam logic [8:0] CTRL_RUNNING     = 9'b000001000;
  localparam logic [8:0] CTRL_TERMINATING = 9'b000010000;
  localparam logic [8:0] CTRL_LINK_TEST   = 9'b000100000;
  localparam logic [8:0] CTRL_SYNC_TEST   = 9'b001000000;
  localparam logic [8:0] CTRL_RESET       = 9'b010000000;
  localparam logic [8:0] CTRL_OUT_OF_DAQ  = 9'b100000000;
  localparam byte unsigned HEADER_BYTE    = 8'h1c;
  localparam time CLK_PERIOD_PS           = 8000ps;

  typedef enum int unsigned {
    FRCV_OBS_HEADER,
    FRCV_OBS_HIT,
    FRCV_OBS_ENDOFRUN
  } frcv_obs_kind_e;

  typedef enum int unsigned {
    FRCV_COUNTER_CHECK_CRC,
    FRCV_COUNTER_CHECK_FRAME
  } frcv_counter_check_kind_e;

  class frcv_symbol_item extends uvm_sequence_item;
    `uvm_object_utils(frcv_symbol_item)

    bit [8:0] data;
    bit       valid;
    bit [2:0] error;
    bit [7:0] channel;

    function new(string name = "frcv_symbol_item");
      super.new(name);
      valid   = 1'b1;
      error   = '0;
      channel = 8'h02;
    endfunction
  endclass

  class frcv_ctrl_item extends uvm_sequence_item;
    `uvm_object_utils(frcv_ctrl_item)

    logic [8:0] cmd;
    int unsigned post_accept_delay_cycles;
    int unsigned timeout_cycles;
    bit          wait_for_ready;
    string       state_name;
    time         accept_time_ps;

    function new(string name = "frcv_ctrl_item");
      super.new(name);
      post_accept_delay_cycles = 0;
      timeout_cycles           = 10000;
      wait_for_ready           = 1'b1;
      state_name               = "";
      accept_time_ps           = 0;
    endfunction
  endclass

  class frcv_csr_item extends uvm_sequence_item;
    `uvm_object_utils(frcv_csr_item)

    bit        is_write;
    bit [7:0]  address;
    bit [31:0] writedata;
    bit [31:0] readdata;
    int unsigned timeout_cycles;
    time       complete_time_ps;

    function new(string name = "frcv_csr_item");
      super.new(name);
      timeout_cycles   = 1000;
      complete_time_ps = 0;
    endfunction
  endclass

  class frcv_obs_item extends uvm_object;
    `uvm_object_utils(frcv_obs_item)

    frcv_obs_kind_e kind;
    time            time_ps;

    bit [7:0]       hit_channel;
    bit             hit_sop;
    bit             hit_eop;
    bit [2:0]       hit_error;
    bit [44:0]      hit_data;

    bit [41:0]      headerinfo_data;
    bit [7:0]       headerinfo_channel;

    function new(string name = "frcv_obs_item");
      super.new(name);
    endfunction
  endclass

  class frcv_rx_obs_item extends uvm_object;
    `uvm_object_utils(frcv_rx_obs_item)

    time       time_ps;
    bit [8:0]  data;
    bit        valid;
    bit [2:0]  error;
    bit [7:0]  channel;

    function new(string name = "frcv_rx_obs_item");
      super.new(name);
    endfunction
  endclass

  class frcv_ctrl_obs_item extends uvm_object;
    `uvm_object_utils(frcv_ctrl_obs_item)

    time        time_ps;
    logic [8:0] data;
    bit         valid;
    bit         ready;

    function new(string name = "frcv_ctrl_obs_item");
      super.new(name);
    endfunction
  endclass

  class frcv_csr_obs_item extends uvm_object;
    `uvm_object_utils(frcv_csr_obs_item)

    time       time_ps;
    bit        is_write;
    bit [7:0]  address;
    bit [31:0] writedata;
    bit [31:0] readdata;
    bit        waitrequest;

    function new(string name = "frcv_csr_obs_item");
      super.new(name);
    endfunction
  endclass

  class frcv_dbg_obs_item extends uvm_object;
    `uvm_object_utils(frcv_dbg_obs_item)

    time       time_ps;
    bit        enable;
    bit        receiver_go;
    bit        receiver_force_go;
    bit        terminating_pending;
    bit [31:0] crc_err_counter;
    bit [31:0] frame_counter;
    bit [31:0] frame_counter_head;
    bit [31:0] frame_counter_tail;

    function new(string name = "frcv_dbg_obs_item");
      super.new(name);
    endfunction
  endclass

  class frcv_expected_txn extends uvm_object;
    `uvm_object_utils(frcv_expected_txn)

    string case_id;
    string execution_mode;
    string source_bucket;
    string sequence_name;
    string stop_boundary;
    bit    strict_checks;
    bit    random_case;
    bit    short_mode;
    bit    allow_outputs;
    bit    expect_headerinfo;
    bit    expect_real_eop;
    bit    expect_error0;
    bit    expect_error1;
    bit    expect_error2;
    bit    queued_overlap;
    int unsigned expected_hits;
    int unsigned expected_headers;
    int unsigned expected_real_eops;
    int unsigned expected_crc_delta;
    int unsigned counter_delay_cycles;
    int unsigned frame_len;
    int unsigned order_idx;
    bit [31:0]   base_crc_err_counter;
    bit [31:0]   base_frame_counter;
    bit [31:0]   base_frame_counter_head;
    bit [31:0]   base_frame_counter_tail;

    int unsigned observed_hits;
    int unsigned observed_headers;
    int unsigned observed_real_eops;
    bit          saw_error0;
    bit          saw_error1;
    bit          saw_error2;
    time         first_obs_time_ps;
    time         last_obs_time_ps;

    function new(string name = "frcv_expected_txn");
      super.new(name);
      execution_mode      = "isolated";
      source_bucket       = "";
      sequence_name       = "";
      stop_boundary       = "none";
      allow_outputs       = 1'b1;
      expect_headerinfo   = 1'b1;
      expect_real_eop     = 1'b1;
      expected_hits       = 1;
      expected_headers    = 1;
      expected_real_eops  = 1;
      counter_delay_cycles = 4;
      frame_len           = 1;
      order_idx           = 0;
    endfunction

    function string frame_mode_string();
      if (!allow_outputs)
        return "quiet";
      return short_mode ? "short" : "long";
    endfunction
  endclass

  class frcv_counter_check extends uvm_object;
    `uvm_object_utils(frcv_counter_check)

    frcv_counter_check_kind_e kind;
    string       case_id;
    string       sequence_name;
    int unsigned due_cycle;
    bit [31:0]   exp_crc_err_counter;
    bit [31:0]   exp_frame_counter;
    bit [31:0]   exp_frame_counter_head;
    bit [31:0]   exp_frame_counter_tail;

    function new(string name = "frcv_counter_check");
      super.new(name);
      case_id = "";
      sequence_name = "";
      due_cycle = 0;
      exp_crc_err_counter = '0;
      exp_frame_counter = '0;
      exp_frame_counter_head = '0;
      exp_frame_counter_tail = '0;
    endfunction
  endclass

  class frcv_rx_driver extends uvm_driver #(frcv_symbol_item);
    `uvm_component_utils(frcv_rx_driver)

    virtual frcv_rx_if.drv vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual frcv_rx_if.drv)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_RX_DRV", "Missing frcv_rx_if.drv")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_symbol_item item;

      vif.data    <= '0;
      vif.valid   <= 1'b0;
      vif.error   <= '0;
      vif.channel <= 8'h02;

      forever begin
        seq_item_port.get_next_item(item);
        vif.data    <= item.data;
        vif.valid   <= item.valid;
        vif.error   <= item.error;
        vif.channel <= item.channel;
        @(posedge vif.clk);
        seq_item_port.item_done();
        vif.data    <= '0;
        vif.valid   <= 1'b0;
        vif.error   <= '0;
        vif.channel <= item.channel;
      end
    endtask
  endclass

  class frcv_ctrl_driver extends uvm_driver #(frcv_ctrl_item);
    `uvm_component_utils(frcv_ctrl_driver)

    virtual frcv_ctrl_if.drv vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual frcv_ctrl_if.drv)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_CTRL_DRV", "Missing frcv_ctrl_if.drv")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_ctrl_item item;
      int unsigned   wait_cycles;

      vif.data  <= CTRL_IDLE;
      vif.valid <= 1'b0;

      forever begin
        seq_item_port.get_next_item(item);
        vif.data  <= item.cmd;
        vif.valid <= 1'b1;

        if (item.wait_for_ready) begin
          wait_cycles = 0;
          do begin
            @(posedge vif.clk);
            wait_cycles++;
            if (wait_cycles > item.timeout_cycles)
              `uvm_fatal("FRCV_CTRL_TIMEOUT",
                $sformatf("Timed out waiting for %s ready after %0d cycles",
                  item.state_name, item.timeout_cycles))
          end while (vif.ready !== 1'b1);

          item.accept_time_ps = $time;
          vif.valid <= 1'b0;
          vif.data  <= CTRL_IDLE;
        end else begin
          @(posedge vif.clk);
          item.accept_time_ps = $time;
          // Keep pulse-mode commands stable for one extra sampled cycle so
          // the synchronous run-control agent cannot miss a same-edge UVM
          // drive/update race during active datapath traffic.
          @(posedge vif.clk);
          vif.valid <= 1'b0;
          vif.data  <= CTRL_IDLE;
        end

        repeat (item.post_accept_delay_cycles)
          @(posedge vif.clk);

        seq_item_port.item_done();
      end
    endtask
  endclass

  class frcv_csr_driver extends uvm_driver #(frcv_csr_item);
    `uvm_component_utils(frcv_csr_driver)

    virtual frcv_csr_if.drv vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual frcv_csr_if.drv)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_CSR_DRV", "Missing frcv_csr_if.drv")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_csr_item item;
      int unsigned  wait_cycles;

      vif.address   <= '0;
      vif.read      <= 1'b0;
      vif.write     <= 1'b0;
      vif.writedata <= '0;

      forever begin
        seq_item_port.get_next_item(item);

        @(posedge vif.clk);
        vif.address   <= item.address;
        vif.writedata <= item.writedata;
        vif.write     <= item.is_write;
        vif.read      <= !item.is_write;

        wait_cycles = 0;
        while (vif.waitrequest === 1'b1) begin
          @(posedge vif.clk);
          wait_cycles++;
          if (wait_cycles > item.timeout_cycles)
            `uvm_fatal("FRCV_CSR_TIMEOUT",
              $sformatf("Timed out waiting for CSR completion at address 0x%0h",
                item.address))
        end

        if (!item.is_write)
          item.readdata = vif.readdata;
        item.complete_time_ps = $time;

        @(posedge vif.clk);
        vif.address   <= '0;
        vif.read      <= 1'b0;
        vif.write     <= 1'b0;
        vif.writedata <= '0;
        seq_item_port.item_done();
      end
    endtask
  endclass

  class frcv_rx_monitor extends uvm_monitor;
    `uvm_component_utils(frcv_rx_monitor)

    virtual frcv_rx_if.mon vif;
    uvm_analysis_port #(frcv_rx_obs_item) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual frcv_rx_if.mon)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_RX_MON", "Missing frcv_rx_if.mon")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_rx_obs_item obs;

      forever begin
        @(posedge vif.clk);
        if (vif.rst === 1'b1)
          continue;
        if (vif.valid !== 1'b1 && vif.data == '0 && vif.error == '0)
          continue;
        obs         = frcv_rx_obs_item::type_id::create("rx_obs");
        obs.time_ps = $time;
        obs.data    = vif.data;
        obs.valid   = vif.valid;
        obs.error   = vif.error;
        obs.channel = vif.channel;
        ap.write(obs);
      end
    endtask
  endclass

  class frcv_ctrl_monitor extends uvm_monitor;
    `uvm_component_utils(frcv_ctrl_monitor)

    virtual frcv_ctrl_if.mon vif;
    uvm_analysis_port #(frcv_ctrl_obs_item) ap;
    bit last_ready;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      last_ready = 1'b0;
      if (!uvm_config_db#(virtual frcv_ctrl_if.mon)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_CTRL_MON", "Missing frcv_ctrl_if.mon")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_ctrl_obs_item obs;

      forever begin
        @(posedge vif.clk);
        if (vif.rst === 1'b1) begin
          last_ready = vif.ready;
          continue;
        end
        if (vif.valid !== 1'b1 && vif.ready === last_ready)
          continue;
        obs         = frcv_ctrl_obs_item::type_id::create("ctrl_obs");
        obs.time_ps = $time;
        obs.data    = vif.data;
        obs.valid   = vif.valid;
        obs.ready   = vif.ready;
        ap.write(obs);
        last_ready = vif.ready;
      end
    endtask
  endclass

  class frcv_csr_monitor extends uvm_monitor;
    `uvm_component_utils(frcv_csr_monitor)

    virtual frcv_csr_if.mon vif;
    uvm_analysis_port #(frcv_csr_obs_item) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual frcv_csr_if.mon)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_CSR_MON", "Missing frcv_csr_if.mon")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_csr_obs_item obs;

      forever begin
        @(posedge vif.clk);
        if (vif.rst === 1'b1)
          continue;
        if (vif.read !== 1'b1 && vif.write !== 1'b1)
          continue;
        obs             = frcv_csr_obs_item::type_id::create("csr_obs");
        obs.time_ps     = $time;
        obs.is_write    = vif.write;
        obs.address     = vif.address;
        obs.writedata   = vif.writedata;
        obs.readdata    = vif.readdata;
        obs.waitrequest = vif.waitrequest;
        ap.write(obs);
      end
    endtask
  endclass

  class frcv_dbg_monitor extends uvm_monitor;
    `uvm_component_utils(frcv_dbg_monitor)

    virtual frcv_dbg_if.mon vif;
    uvm_analysis_port #(frcv_dbg_obs_item) ap;

    bit        last_enable;
    bit        last_receiver_go;
    bit        last_receiver_force_go;
    bit        last_terminating_pending;
    bit [31:0] last_crc_err_counter;
    bit [31:0] last_frame_counter;
    bit [31:0] last_frame_counter_head;
    bit [31:0] last_frame_counter_tail;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      last_enable             = 1'b0;
      last_receiver_go        = 1'b0;
      last_receiver_force_go  = 1'b0;
      last_terminating_pending = 1'b0;
      last_crc_err_counter    = '0;
      last_frame_counter      = '0;
      last_frame_counter_head = '0;
      last_frame_counter_tail = '0;
      if (!uvm_config_db#(virtual frcv_dbg_if.mon)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_DBG_MON", "Missing frcv_dbg_if.mon")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_dbg_obs_item obs;

      forever begin
        @(posedge vif.clk);
        if (vif.rst === 1'b1) begin
          last_enable              = vif.enable;
          last_receiver_go         = vif.receiver_go;
          last_receiver_force_go   = vif.receiver_force_go;
          last_terminating_pending = vif.terminating_pending;
          last_crc_err_counter     = vif.crc_err_counter;
          last_frame_counter       = vif.frame_counter;
          last_frame_counter_head  = vif.frame_counter_head;
          last_frame_counter_tail  = vif.frame_counter_tail;
          continue;
        end
        if (vif.enable             === last_enable &&
            vif.receiver_go        === last_receiver_go &&
            vif.receiver_force_go  === last_receiver_force_go &&
            vif.terminating_pending === last_terminating_pending &&
            vif.crc_err_counter    == last_crc_err_counter &&
            vif.frame_counter      == last_frame_counter &&
            vif.frame_counter_head == last_frame_counter_head &&
            vif.frame_counter_tail == last_frame_counter_tail)
          continue;
        obs                     = frcv_dbg_obs_item::type_id::create("dbg_obs");
        obs.time_ps             = $time;
        obs.enable              = vif.enable;
        obs.receiver_go         = vif.receiver_go;
        obs.receiver_force_go   = vif.receiver_force_go;
        obs.terminating_pending = vif.terminating_pending;
        obs.crc_err_counter     = vif.crc_err_counter;
        obs.frame_counter       = vif.frame_counter;
        obs.frame_counter_head  = vif.frame_counter_head;
        obs.frame_counter_tail  = vif.frame_counter_tail;
        ap.write(obs);
        last_enable              = vif.enable;
        last_receiver_go         = vif.receiver_go;
        last_receiver_force_go   = vif.receiver_force_go;
        last_terminating_pending = vif.terminating_pending;
        last_crc_err_counter     = vif.crc_err_counter;
        last_frame_counter       = vif.frame_counter;
        last_frame_counter_head  = vif.frame_counter_head;
        last_frame_counter_tail  = vif.frame_counter_tail;
      end
    endtask
  endclass

  class frcv_out_monitor extends uvm_monitor;
    `uvm_component_utils(frcv_out_monitor)

    virtual frcv_out_if.mon vif;
    uvm_analysis_port #(frcv_obs_item) ap;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual frcv_out_if.mon)::get(this, "", "vif", vif))
        `uvm_fatal("FRCV_OUT_MON", "Missing frcv_out_if.mon")
    endfunction

    task run_phase(uvm_phase phase);
      frcv_obs_item obs;

      forever begin
        @(posedge vif.clk);
        if (vif.rst === 1'b1)
          continue;

        if (vif.headerinfo_valid === 1'b1) begin
          obs                    = frcv_obs_item::type_id::create("header_obs");
          obs.kind               = FRCV_OBS_HEADER;
          obs.time_ps            = $time;
          obs.headerinfo_data    = vif.headerinfo_data;
          obs.headerinfo_channel = vif.headerinfo_channel;
          ap.write(obs);
        end

        if (vif.hit_valid === 1'b1) begin
          obs              = frcv_obs_item::type_id::create("hit_obs");
          obs.kind         = FRCV_OBS_HIT;
          obs.time_ps      = $time;
          obs.hit_channel  = vif.hit_channel;
          obs.hit_sop      = vif.hit_sop;
          obs.hit_eop      = vif.hit_eop;
          obs.hit_error    = vif.hit_error;
          obs.hit_data     = vif.hit_data;
          ap.write(obs);
        end

        if (vif.hit_endofrun === 1'b1) begin
          obs         = frcv_obs_item::type_id::create("endofrun_obs");
          obs.kind    = FRCV_OBS_ENDOFRUN;
          obs.time_ps = $time;
          ap.write(obs);
        end
      end
    endtask
  endclass

  class frcv_scoreboard extends uvm_component;
    `uvm_component_utils(frcv_scoreboard)

    uvm_analysis_imp_obs #(frcv_obs_item, frcv_scoreboard) obs_imp;
    uvm_analysis_imp_rx  #(frcv_rx_obs_item, frcv_scoreboard) rx_imp;
    uvm_analysis_imp_ctrl #(frcv_ctrl_obs_item, frcv_scoreboard) ctrl_imp;
    uvm_analysis_imp_csr #(frcv_csr_obs_item, frcv_scoreboard) csr_imp;
    uvm_analysis_imp_dbg #(frcv_dbg_obs_item, frcv_scoreboard) dbg_imp;
    virtual frcv_dbg_if.mon dbg_vif;

    int unsigned header_count;
    int unsigned hit_count;
    int unsigned real_eop_count;
    int unsigned endofrun_count;
    int unsigned cycle_count;
    int unsigned completed_txn_count;
    int unsigned counter_checks_passed;
    int unsigned counter_checks_failed;
    int unsigned unexpected_output_count;
    int unsigned back_to_back_events;
    int unsigned queued_overlap_events;

    time         last_header_time_ps;
    time         last_real_eop_time_ps;
    time         last_endofrun_time_ps;

    frcv_obs_item history[$];
    frcv_rx_obs_item rx_history[$];
    frcv_ctrl_obs_item ctrl_history[$];
    frcv_csr_obs_item csr_history[$];
    frcv_dbg_obs_item dbg_history[$];
    frcv_expected_txn expected_q[$];
    frcv_counter_check pending_checks[$];
    frcv_expected_txn active_txn;
    string       current_case_id;
    string       current_execution_mode;
    bit          current_random_case;
    string       coverage_bins[string];
    string       coverage_curve[$];

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function automatic string obs_kind_name(frcv_obs_kind_e kind);
      case (kind)
        FRCV_OBS_HEADER: return "header";
        FRCV_OBS_HIT: return "hit";
        FRCV_OBS_ENDOFRUN: return "endofrun";
        default: return "unknown";
      endcase
    endfunction

    function automatic string txn_summary(frcv_expected_txn txn);
      if (txn == null)
        return "null";
      return $sformatf(
        "case=%s seq=%s order=%0d exp(h=%0d hdr=%0d eop=%0d) obs(h=%0d hdr=%0d eop=%0d) stop=%s short=%0d strict=%0d",
        txn.case_id,
        txn.sequence_name,
        txn.order_idx,
        txn.expected_hits,
        txn.expected_headers,
        txn.expected_real_eops,
        txn.observed_hits,
        txn.observed_headers,
        txn.observed_real_eops,
        txn.stop_boundary,
        txn.short_mode,
        txn.strict_checks
      );
    endfunction

    function automatic void report_unexpected_output(frcv_obs_item item);
      string front_summary;

      front_summary = (expected_q.size() > 0) ? txn_summary(expected_q[0]) : "empty";
      `uvm_warning(
        "FRCV_SCB_UNEXPECTED",
        $sformatf(
          "unexpected_%s time=%0t case=%s mode=%s unexpected_count=%0d active=%s queue_depth=%0d queue_front=%s",
          obs_kind_name(item.kind),
          item.time_ps,
          current_case_id,
          current_execution_mode,
          unexpected_output_count + 1,
          txn_summary(active_txn),
          expected_q.size(),
          front_summary
        )
      )
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      obs_imp                = new("obs_imp", this);
      rx_imp                 = new("rx_imp", this);
      ctrl_imp               = new("ctrl_imp", this);
      csr_imp                = new("csr_imp", this);
      dbg_imp                = new("dbg_imp", this);
      header_count           = 0;
      hit_count              = 0;
      real_eop_count         = 0;
      endofrun_count        = 0;
      cycle_count            = 0;
      completed_txn_count    = 0;
      counter_checks_passed  = 0;
      counter_checks_failed  = 0;
      unexpected_output_count = 0;
      back_to_back_events    = 0;
      queued_overlap_events  = 0;
      last_header_time_ps    = 0;
      last_real_eop_time_ps = 0;
      last_endofrun_time_ps = 0;
      current_case_id        = "";
      current_execution_mode = "isolated";
      current_random_case    = 1'b0;
      if (!uvm_config_db#(virtual frcv_dbg_if.mon)::get(this, "", "dbg_vif", dbg_vif))
        `uvm_fatal("FRCV_SCB", "Missing dbg_vif")
    endfunction

    function void write_obs(frcv_obs_item item);
      history.push_back(item);
      case (item.kind)
        FRCV_OBS_HEADER: begin
          if (active_txn == null && expected_q.size() > 0) begin
            active_txn = expected_q.pop_front();
            if (active_txn.queued_overlap)
              queued_overlap_events++;
          end
          header_count         ++;
          last_header_time_ps  = item.time_ps;
          if (active_txn != null) begin
            active_txn.observed_headers++;
            if (active_txn.first_obs_time_ps == 0)
              active_txn.first_obs_time_ps = item.time_ps;
            active_txn.last_obs_time_ps = item.time_ps;
            if (active_txn.expected_hits == 0 && active_txn.expected_real_eops == 0)
              finalize_active_txn("header_only_or_zero_hit");
          end else begin
            report_unexpected_output(item);
            unexpected_output_count++;
          end
        end
        FRCV_OBS_HIT: begin
          time prev_real_eop_time_ps;

          prev_real_eop_time_ps = last_real_eop_time_ps;
          hit_count++;
          if (active_txn == null && expected_q.size() > 0) begin
            active_txn = expected_q.pop_front();
            if (active_txn.queued_overlap)
              queued_overlap_events++;
          end
          if (active_txn != null) begin
            active_txn.observed_hits++;
            active_txn.saw_error0 |= item.hit_error[0];
            active_txn.saw_error1 |= item.hit_error[1];
            active_txn.saw_error2 |= item.hit_error[2];
            if (active_txn.first_obs_time_ps == 0)
              active_txn.first_obs_time_ps = item.time_ps;
            active_txn.last_obs_time_ps = item.time_ps;
          end else begin
            report_unexpected_output(item);
            unexpected_output_count++;
          end
          if (item.hit_eop) begin
            real_eop_count++;
            last_real_eop_time_ps = item.time_ps;
            if (completed_txn_count != 0 && item.hit_sop && prev_real_eop_time_ps != 0 &&
                (item.time_ps - prev_real_eop_time_ps) <= CLK_PERIOD_PS)
              back_to_back_events++;
            if (active_txn != null) begin
              active_txn.observed_real_eops++;
              finalize_active_txn("hit_eop");
            end
          end
        end
        FRCV_OBS_ENDOFRUN: begin
          endofrun_count++;
          last_endofrun_time_ps = item.time_ps;
        end
        default: begin
        end
      endcase
    endfunction

    function void write_rx(frcv_rx_obs_item item);
      rx_history.push_back(item);
    endfunction

    function void write_ctrl(frcv_ctrl_obs_item item);
      ctrl_history.push_back(item);
    endfunction

    function void write_csr(frcv_csr_obs_item item);
      csr_history.push_back(item);
    endfunction

    function void write_dbg(frcv_dbg_obs_item item);
      dbg_history.push_back(item);
    endfunction

    task automatic note_case_start(string case_id, string execution_mode, bit is_random_case);
      current_case_id        = case_id;
      current_execution_mode = execution_mode;
      current_random_case    = is_random_case;
      $display("FRCV_CASE_BEGIN case=%s mode=%s random=%0d", case_id, execution_mode, is_random_case);
    endtask

    task automatic note_case_end(string case_id);
      if (active_txn != null || expected_q.size() != 0)
        `uvm_warning(
          "FRCV_SCB_CASE_END",
          $sformatf(
            "case_end_with_pending_state case=%s mode=%s active=%s queue_depth=%0d queue_front=%s",
            case_id,
            current_execution_mode,
            txn_summary(active_txn),
            expected_q.size(),
            (expected_q.size() > 0) ? txn_summary(expected_q[0]) : "empty"
          )
        )
      $display("FRCV_CASE_END case=%s mode=%s completed_txns=%0d", case_id, current_execution_mode, completed_txn_count);
      current_case_id = "";
      current_random_case = 1'b0;
    endtask

    task automatic force_complete_case_txn(string case_id, string reason);
      if (active_txn == null && expected_q.size() > 0 && expected_q[0].case_id == case_id)
        active_txn = expected_q.pop_front();

      if (active_txn != null && active_txn.case_id == case_id)
        finalize_active_txn(reason);
    endtask

    task automatic discard_pending_case_txns(string case_id);
      while (expected_q.size() > 0 && expected_q[0].case_id == case_id)
        void'(expected_q.pop_front());
    endtask

    task automatic snapshot_counters(
      output bit [31:0] crc_err_counter,
      output bit [31:0] frame_counter,
      output bit [31:0] frame_counter_head,
      output bit [31:0] frame_counter_tail
    );
      uvm_hdl_data_t raw;

      crc_err_counter = '0;
      frame_counter = '0;
      frame_counter_head = '0;
      frame_counter_tail = '0;

      if (uvm_hdl_read("tb_top.dbg_crc_err_counter", raw))
        crc_err_counter = raw[31:0];
      if (uvm_hdl_read("tb_top.dbg_frame_counter", raw))
        frame_counter = raw[31:0];
      if (uvm_hdl_read("tb_top.dbg_frame_counter_head", raw))
        frame_counter_head = raw[31:0];
      if (uvm_hdl_read("tb_top.dbg_frame_counter_tail", raw))
        frame_counter_tail = raw[31:0];
    endtask

    task automatic expect_txn(frcv_expected_txn txn);
      txn.case_id         = (txn.case_id == "") ? current_case_id : txn.case_id;
      txn.execution_mode  = (txn.execution_mode == "") ? current_execution_mode : txn.execution_mode;
      txn.random_case     = txn.random_case | current_random_case;
      txn.order_idx       = completed_txn_count + expected_q.size() + ((active_txn == null) ? 0 : 1) + 1;
      snapshot_counters(
        txn.base_crc_err_counter,
        txn.base_frame_counter,
        txn.base_frame_counter_head,
        txn.base_frame_counter_tail
      );
      expected_q.push_back(txn);
    endtask

    function automatic string len_bucket(int unsigned frame_len);
      if (frame_len == 0)
        return "0";
      if (frame_len == 1)
        return "1";
      if (frame_len == 2)
        return "2";
      if (frame_len <= 4)
        return "3_4";
      if (frame_len <= 15)
        return "5_15";
      return "16_plus";
    endfunction

    function automatic void mark_cov(string key);
      coverage_bins[key] = key;
    endfunction

    function automatic bit keep_curve_sample(int unsigned txn_idx, int unsigned delta_bins);
      if (delta_bins != 0)
        return 1'b1;
      if (txn_idx <= 16)
        return 1'b1;
      if ((txn_idx & (txn_idx - 1)) == 0)
        return 1'b1;
      if ((txn_idx % 1024) == 0)
        return 1'b1;
      return 1'b0;
    endfunction

    function automatic real cross_cov_pct();
      int unsigned goal;
      goal = 32;
      return (100.0 * coverage_bins.num()) / goal;
    endfunction

    function void finalize_active_txn(string reason);
      frcv_counter_check check_item;
      int unsigned before_bins;
      int unsigned after_bins;
      string curve_line;

      if (active_txn == null)
        return;

      before_bins = coverage_bins.num();
      mark_cov($sformatf("exec:%s", active_txn.execution_mode));
      mark_cov($sformatf("mode:%s", active_txn.frame_mode_string()));
      mark_cov($sformatf("len:%s", len_bucket(active_txn.frame_len)));
      mark_cov($sformatf("crc:%s", active_txn.saw_error1 ? "bad" : "good"));
      mark_cov($sformatf("err0:%s", active_txn.saw_error0 ? "set" : "clear"));
      mark_cov($sformatf("err2:%s", active_txn.saw_error2 ? "set" : "clear"));
      mark_cov($sformatf("casekind:%s", active_txn.random_case ? "random" : "directed"));
      mark_cov($sformatf("stop:%s", active_txn.stop_boundary));
      if (active_txn.sequence_name != "")
        mark_cov($sformatf("seq:%s", active_txn.sequence_name));
      if (active_txn.queued_overlap)
        mark_cov("queued:overlap");
      else
        mark_cov("queued:clean");
      mark_cov($sformatf("mode_crc:%s_%s", active_txn.frame_mode_string(), active_txn.saw_error1 ? "bad" : "good"));
      mark_cov($sformatf("mode_err2:%s_%s", active_txn.frame_mode_string(), active_txn.saw_error2 ? "set" : "clear"));
      after_bins = coverage_bins.num();
      curve_line = $sformatf(
        "txn=%0d case=%s seq=%s pct=%0.2f delta_bins=%0d reason=%s",
        completed_txn_count + 1,
        active_txn.case_id,
        active_txn.sequence_name,
        cross_cov_pct(),
        after_bins - before_bins,
        reason
      );
      if (keep_curve_sample(completed_txn_count + 1, after_bins - before_bins))
        coverage_curve.push_back(curve_line);

      if (active_txn.strict_checks) begin
        if (active_txn.expect_headerinfo && active_txn.observed_headers < active_txn.expected_headers)
          `uvm_error("FRCV_SCB",
            $sformatf("%s expected at least %0d headerinfo pulses, saw %0d",
              active_txn.case_id, active_txn.expected_headers, active_txn.observed_headers))
        if (active_txn.expect_real_eop && active_txn.observed_real_eops < active_txn.expected_real_eops)
          `uvm_error("FRCV_SCB",
            $sformatf("%s expected at least %0d real eops, saw %0d",
              active_txn.case_id, active_txn.expected_real_eops, active_txn.observed_real_eops))
        if (active_txn.observed_hits < active_txn.expected_hits)
          `uvm_error("FRCV_SCB",
            $sformatf("%s expected at least %0d hits, saw %0d",
              active_txn.case_id, active_txn.expected_hits, active_txn.observed_hits))
        if (active_txn.expect_error0 && !active_txn.saw_error0)
          `uvm_error("FRCV_SCB", $sformatf("%s expected hit error0 but none observed", active_txn.case_id))
        if (active_txn.expect_error1 && !active_txn.saw_error1)
          `uvm_error("FRCV_SCB", $sformatf("%s expected crc error1 but none observed", active_txn.case_id))
        if (active_txn.expect_error2 && !active_txn.saw_error2)
          `uvm_error("FRCV_SCB", $sformatf("%s expected loss-sync error2 but none observed", active_txn.case_id))
      end

      if (active_txn.strict_checks &&
          (active_txn.expected_crc_delta != 0 ||
           active_txn.expected_headers != 0 ||
           active_txn.expected_real_eops != 0)) begin
        check_item = frcv_counter_check::type_id::create($sformatf("counter_check_%0d", completed_txn_count + 1));
        check_item.kind                  = (active_txn.expected_crc_delta != 0) ? FRCV_COUNTER_CHECK_CRC : FRCV_COUNTER_CHECK_FRAME;
        check_item.case_id               = active_txn.case_id;
        check_item.sequence_name         = active_txn.sequence_name;
        check_item.due_cycle             = cycle_count + active_txn.counter_delay_cycles;
        check_item.exp_crc_err_counter   = active_txn.base_crc_err_counter + active_txn.expected_crc_delta;
        check_item.exp_frame_counter_head = active_txn.base_frame_counter_head + active_txn.expected_headers;
        check_item.exp_frame_counter_tail = active_txn.base_frame_counter_tail + active_txn.expected_real_eops;
        check_item.exp_frame_counter     = check_item.exp_frame_counter_head - check_item.exp_frame_counter_tail;
        pending_checks.push_back(check_item);
      end

      completed_txn_count++;
      active_txn = null;
    endfunction

    task automatic perform_counter_check(int unsigned idx);
      bit [31:0] crc_err_counter;
      bit [31:0] frame_counter;
      bit [31:0] frame_counter_head;
      bit [31:0] frame_counter_tail;
      frcv_counter_check check_item;

      check_item = pending_checks[idx];
      snapshot_counters(crc_err_counter, frame_counter, frame_counter_head, frame_counter_tail);
      if (check_item.kind == FRCV_COUNTER_CHECK_CRC) begin
        if (crc_err_counter < check_item.exp_crc_err_counter) begin
          counter_checks_failed++;
          `uvm_error("FRCV_SCB",
            $sformatf("%s seq=%s expected crc_err_counter to reach at least %0d got %0d",
              check_item.case_id, check_item.sequence_name, check_item.exp_crc_err_counter, crc_err_counter))
        end else begin
          counter_checks_passed++;
        end
      end else begin
        if (frame_counter_head < check_item.exp_frame_counter_head ||
            frame_counter_tail < check_item.exp_frame_counter_tail) begin
          counter_checks_failed++;
          `uvm_error("FRCV_SCB",
            $sformatf("%s seq=%s expected frame head/tail to reach at least %0d/%0d got %0d/%0d",
              check_item.case_id, check_item.sequence_name,
              check_item.exp_frame_counter_head, check_item.exp_frame_counter_tail,
              frame_counter_head, frame_counter_tail))
        end else begin
          counter_checks_passed++;
        end
      end
      pending_checks.delete(idx);
    endtask

    task run_phase(uvm_phase phase);
      forever begin
        @(posedge dbg_vif.clk);
        if (dbg_vif.rst === 1'b1)
          continue;
        cycle_count++;
        for (int idx = pending_checks.size() - 1; idx >= 0; idx--) begin
          if (pending_checks[idx].due_cycle <= cycle_count)
            perform_counter_check(idx);
        end
      end
    endtask

    function void report_phase(uvm_phase phase);
      string curve_dump;

      curve_dump = "";
      foreach (coverage_curve[idx]) begin
        if (idx != 0)
          curve_dump = {curve_dump, "; "};
        curve_dump = {curve_dump, coverage_curve[idx]};
      end
      `uvm_info("FRCV_SCB",
        $sformatf("headers=%0d hits=%0d real_eops=%0d endofruns=%0d txns=%0d counter_checks=%0d/%0d back_to_back=%0d queued_overlap=%0d",
          header_count, hit_count, real_eop_count, endofrun_count,
          completed_txn_count, counter_checks_passed, counter_checks_failed,
          back_to_back_events, queued_overlap_events),
        UVM_LOW)
      $display(
        "FRCV_CROSS_SUMMARY hit_bins=%0d total_bins=%0d pct=%0.2f txns=%0d back_to_back=%0d queued_overlap=%0d counter_checks_passed=%0d counter_checks_failed=%0d unexpected_outputs=%0d",
        coverage_bins.num(), 32, cross_cov_pct(), completed_txn_count,
        back_to_back_events, queued_overlap_events,
        counter_checks_passed, counter_checks_failed, unexpected_output_count
      );
      $display("FRCV_CROSS_CURVE %s", curve_dump);
    endfunction
  endclass

  class frcv_env extends uvm_env;
    `uvm_component_utils(frcv_env)

    uvm_sequencer #(frcv_csr_item)    m_csr_sqr;
    uvm_sequencer #(frcv_symbol_item) m_rx_sqr;
    uvm_sequencer #(frcv_ctrl_item)   m_ctrl_sqr;
    frcv_csr_driver                   m_csr_drv;
    frcv_rx_driver                    m_rx_drv;
    frcv_ctrl_driver                  m_ctrl_drv;
    frcv_rx_monitor                   m_rx_mon;
    frcv_ctrl_monitor                 m_ctrl_mon;
    frcv_csr_monitor                  m_csr_mon;
    frcv_dbg_monitor                  m_dbg_mon;
    frcv_out_monitor                  m_out_mon;
    frcv_scoreboard                   m_scb;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      m_csr_sqr  = uvm_sequencer#(frcv_csr_item)::type_id::create("m_csr_sqr", this);
      m_rx_sqr  = uvm_sequencer#(frcv_symbol_item)::type_id::create("m_rx_sqr", this);
      m_ctrl_sqr = uvm_sequencer#(frcv_ctrl_item)::type_id::create("m_ctrl_sqr", this);
      m_csr_drv = frcv_csr_driver::type_id::create("m_csr_drv", this);
      m_rx_drv  = frcv_rx_driver::type_id::create("m_rx_drv", this);
      m_ctrl_drv = frcv_ctrl_driver::type_id::create("m_ctrl_drv", this);
      m_rx_mon  = frcv_rx_monitor::type_id::create("m_rx_mon", this);
      m_ctrl_mon = frcv_ctrl_monitor::type_id::create("m_ctrl_mon", this);
      m_csr_mon = frcv_csr_monitor::type_id::create("m_csr_mon", this);
      m_dbg_mon = frcv_dbg_monitor::type_id::create("m_dbg_mon", this);
      m_out_mon = frcv_out_monitor::type_id::create("m_out_mon", this);
      m_scb     = frcv_scoreboard::type_id::create("m_scb", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      m_csr_drv.seq_item_port.connect(m_csr_sqr.seq_item_export);
      m_rx_drv.seq_item_port.connect(m_rx_sqr.seq_item_export);
      m_ctrl_drv.seq_item_port.connect(m_ctrl_sqr.seq_item_export);
      m_rx_mon.ap.connect(m_scb.rx_imp);
      m_ctrl_mon.ap.connect(m_scb.ctrl_imp);
      m_csr_mon.ap.connect(m_scb.csr_imp);
      m_dbg_mon.ap.connect(m_scb.dbg_imp);
      m_out_mon.ap.connect(m_scb.obs_imp);
    endfunction
  endclass

  class frcv_symbol_seq extends uvm_sequence #(frcv_symbol_item);
    `uvm_object_utils(frcv_symbol_seq)

    bit [8:0] data;
    bit       valid;
    bit [2:0] error;
    bit [7:0] channel;

    function new(string name = "frcv_symbol_seq");
      super.new(name);
      valid   = 1'b1;
      error   = '0;
      channel = 8'h02;
    endfunction

    task body();
      frcv_symbol_item item;
      item = frcv_symbol_item::type_id::create("symbol_item");
      start_item(item);
      item.data    = data;
      item.valid   = valid;
      item.error   = error;
      item.channel = channel;
      finish_item(item);
    endtask
  endclass

  class frcv_ctrl_seq extends uvm_sequence #(frcv_ctrl_item);
    `uvm_object_utils(frcv_ctrl_seq)

    logic [8:0] cmd;
    int unsigned post_accept_delay_cycles;
    int unsigned timeout_cycles;
    bit          wait_for_ready;
    string       state_name;
    time         accept_time_ps;

    function new(string name = "frcv_ctrl_seq");
      super.new(name);
      post_accept_delay_cycles = 0;
      timeout_cycles           = 10000;
      wait_for_ready           = 1'b1;
      state_name               = "";
      accept_time_ps           = 0;
    endfunction

    task body();
      frcv_ctrl_item item;
      item = frcv_ctrl_item::type_id::create("ctrl_item");
      start_item(item);
      item.cmd                      = cmd;
      item.post_accept_delay_cycles = post_accept_delay_cycles;
      item.timeout_cycles           = timeout_cycles;
      item.wait_for_ready           = wait_for_ready;
      item.state_name               = state_name;
      finish_item(item);
      accept_time_ps = item.accept_time_ps;
    endtask
  endclass

  class frcv_csr_write_seq extends uvm_sequence #(frcv_csr_item);
    `uvm_object_utils(frcv_csr_write_seq)

    bit [7:0]  addr;
    bit [31:0] data;

    function new(string name = "frcv_csr_write_seq");
      super.new(name);
    endfunction

    task body();
      frcv_csr_item item;
      item = frcv_csr_item::type_id::create("csr_wr");
      start_item(item);
      item.is_write  = 1'b1;
      item.address   = addr;
      item.writedata = data;
      finish_item(item);
    endtask
  endclass

  class frcv_csr_read_seq extends uvm_sequence #(frcv_csr_item);
    `uvm_object_utils(frcv_csr_read_seq)

    bit [7:0]  addr;
    bit [31:0] data;

    function new(string name = "frcv_csr_read_seq");
      super.new(name);
      data = '0;
    endfunction

    task body();
      frcv_csr_item item;
      item = frcv_csr_item::type_id::create("csr_rd");
      start_item(item);
      item.is_write  = 1'b0;
      item.address   = addr;
      item.writedata = '0;
      finish_item(item);
      data = item.readdata;
    endtask
  endclass

  class frcv_base_test extends uvm_test;
    `uvm_component_utils(frcv_base_test)

    frcv_env                  m_env;
    virtual frcv_ctrl_if.mon  ctrl_vif;
    virtual frcv_csr_if.mon   csr_vif;
    virtual frcv_out_if.mon   out_vif;
    virtual frcv_reset_if.drv reset_vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      m_env = frcv_env::type_id::create("m_env", this);

      if (!uvm_config_db#(virtual frcv_ctrl_if.mon)::get(this, "", "ctrl_vif", ctrl_vif))
        `uvm_fatal("FRCV_TEST", "Missing ctrl_vif")
      if (!uvm_config_db#(virtual frcv_csr_if.mon)::get(this, "", "csr_vif", csr_vif))
        `uvm_fatal("FRCV_TEST", "Missing csr_vif")
      if (!uvm_config_db#(virtual frcv_out_if.mon)::get(this, "", "out_vif", out_vif))
        `uvm_fatal("FRCV_TEST", "Missing out_vif")
      if (!uvm_config_db#(virtual frcv_reset_if.drv)::get(this, "", "reset_vif", reset_vif))
        `uvm_fatal("FRCV_TEST", "Missing reset_vif")
    endfunction

    task automatic wait_cycles(int unsigned cycles);
      repeat (cycles)
        @(posedge ctrl_vif.clk);
    endtask

    task automatic wait_for_reset_release();
      while (ctrl_vif.rst !== 1'b0)
        @(posedge ctrl_vif.clk);
      wait_cycles(2);
    endtask

    task automatic drive_symbol(bit is_k, byte unsigned byte_value,
                                bit [2:0] error = '0, bit [7:0] channel = 8'h02);
      frcv_symbol_seq seq;
      seq         = frcv_symbol_seq::type_id::create($sformatf("symbol_seq_%0t", $time));
      seq.data    = {is_k, byte_value};
      seq.error   = error;
      seq.channel = channel;
      seq.start(m_env.m_rx_sqr);
    endtask

    task automatic send_ctrl_and_capture(logic [8:0] cmd, string state_name,
                                         output time accept_time_ps,
                                         input int unsigned post_accept_delay_cycles = 0);
      frcv_ctrl_seq seq;
      seq                           = frcv_ctrl_seq::type_id::create($sformatf("ctrl_seq_%s_%0t", state_name, $time));
      seq.cmd                       = cmd;
      seq.state_name                = state_name;
      seq.post_accept_delay_cycles  = post_accept_delay_cycles;
      seq.wait_for_ready            = 1'b1;
      seq.start(m_env.m_ctrl_sqr);
      accept_time_ps = seq.accept_time_ps;
    endtask

    task automatic send_ctrl(logic [8:0] cmd, string state_name,
                             int unsigned post_accept_delay_cycles = 0);
      time ignored_time;
      send_ctrl_and_capture(cmd, state_name, ignored_time, post_accept_delay_cycles);
    endtask

    task automatic csr_write(bit [7:0] addr, bit [31:0] data);
      frcv_csr_write_seq seq;
      seq      = frcv_csr_write_seq::type_id::create($sformatf("csr_wr_%0t", $time));
      seq.addr = addr;
      seq.data = data;
      seq.start(m_env.m_csr_sqr);
    endtask

    task automatic csr_read(bit [7:0] addr, output bit [31:0] data);
      frcv_csr_read_seq seq;
      seq      = frcv_csr_read_seq::type_id::create($sformatf("csr_rd_%0t", $time));
      seq.addr = addr;
      seq.start(m_env.m_csr_sqr);
      data = seq.data;
    endtask

    task automatic pulse_ctrl(logic [8:0] cmd, string state_name);
      frcv_ctrl_seq seq;
      seq                 = frcv_ctrl_seq::type_id::create($sformatf("ctrl_pulse_%s_%0t", state_name, $time));
      seq.cmd             = cmd;
      seq.state_name      = state_name;
      seq.wait_for_ready  = 1'b0;
      seq.start(m_env.m_ctrl_sqr);
    endtask

    task automatic run_start();
      send_ctrl(CTRL_RUN_PREPARE, "RUN_PREPARE");
      send_ctrl(CTRL_SYNC, "SYNC");
      send_ctrl(CTRL_RUNNING, "RUNNING");
      wait_cycles(2);
    endtask

    task automatic pulse_reset(int unsigned cycles = 2);
      reset_vif.force_reset <= 1'b1;
      repeat (cycles)
        @(posedge reset_vif.clk);
      reset_vif.force_reset <= 1'b0;
      wait_cycles(2);
    endtask

    task automatic scb_case_begin(string case_id, string execution_mode, bit is_random_case);
      m_env.m_scb.note_case_start(case_id, execution_mode, is_random_case);
    endtask

    task automatic scb_case_end(string case_id);
      m_env.m_scb.note_case_end(case_id);
    endtask

    task automatic scb_force_complete_case_txn(string case_id, string reason);
      m_env.m_scb.force_complete_case_txn(case_id, reason);
    endtask

    task automatic scb_discard_pending_case_txns(string case_id);
      m_env.m_scb.discard_pending_case_txns(case_id);
    endtask

    task automatic scb_expect_txn(
      string case_id,
      string execution_mode,
      string source_bucket,
      string sequence_name,
      bit    strict_checks,
      bit    random_case,
      bit    short_mode,
      int unsigned frame_len,
      int unsigned expected_hits,
      int unsigned expected_headers,
      int unsigned expected_real_eops,
      int unsigned expected_crc_delta,
      bit    expect_error0,
      bit    expect_error1,
      bit    expect_error2,
      bit    queued_overlap,
      string stop_boundary = "none",
      int unsigned counter_delay_cycles = 4
    );
      frcv_expected_txn txn;

      txn                     = frcv_expected_txn::type_id::create($sformatf("exp_txn_%0t", $time));
      txn.case_id             = case_id;
      txn.execution_mode      = execution_mode;
      txn.source_bucket       = source_bucket;
      txn.sequence_name       = sequence_name;
      txn.strict_checks       = strict_checks;
      txn.random_case         = random_case;
      txn.short_mode          = short_mode;
      txn.frame_len           = frame_len;
      txn.expected_hits       = expected_hits;
      txn.expected_headers    = expected_headers;
      txn.expected_real_eops  = expected_real_eops;
      txn.expect_headerinfo   = (expected_headers != 0);
      txn.expect_real_eop     = (expected_real_eops != 0);
      txn.expected_crc_delta  = expected_crc_delta;
      txn.expect_error0       = expect_error0;
      txn.expect_error1       = expect_error1;
      txn.expect_error2       = expect_error2;
      txn.queued_overlap      = queued_overlap;
      txn.stop_boundary       = stop_boundary;
      txn.counter_delay_cycles = counter_delay_cycles;
      txn.allow_outputs       = (expected_headers != 0) || (expected_hits != 0) || (expected_real_eops != 0);
      m_env.m_scb.expect_txn(txn);
    endtask

    task automatic send_long_frame(int unsigned frame_number, bit [47:0] hit_word);
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, byte'(frame_number[15:8]));
      drive_symbol(1'b0, byte'(frame_number[7:0]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h01);
      for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
        drive_symbol(1'b0, byte'(hit_word[byte_idx*8 +: 8]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
    endtask

    task automatic send_empty_frame(int unsigned frame_number);
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, byte'(frame_number[15:8]));
      drive_symbol(1'b0, byte'(frame_number[7:0]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
    endtask

    task automatic wait_for_counts(int unsigned exp_hits,
                                   int unsigned exp_real_eops,
                                   int unsigned exp_headers,
                                   int unsigned max_cycles,
                                   string ctx);
      repeat (max_cycles) begin
        if (m_env.m_scb.hit_count      >= exp_hits &&
            m_env.m_scb.real_eop_count >= exp_real_eops &&
            m_env.m_scb.header_count   >= exp_headers)
          return;
        @(posedge ctrl_vif.clk);
      end
      `uvm_fatal("FRCV_TIMEOUT",
        $sformatf("%s timed out waiting for hits=%0d real_eops=%0d headers=%0d, got hits=%0d real_eops=%0d headers=%0d",
          ctx, exp_hits, exp_real_eops, exp_headers,
          m_env.m_scb.hit_count, m_env.m_scb.real_eop_count, m_env.m_scb.header_count))
    endtask

    task automatic wait_for_endofrun_count(int unsigned exp_endofruns,
                                        int unsigned max_cycles,
                                        string ctx);
      repeat (max_cycles) begin
        if (m_env.m_scb.endofrun_count >= exp_endofruns)
          return;
        @(posedge ctrl_vif.clk);
      end
      `uvm_fatal("FRCV_TIMEOUT",
        $sformatf("%s timed out waiting for endofruns=%0d, got %0d",
          ctx, exp_endofruns, m_env.m_scb.endofrun_count))
    endtask

    task automatic wait_for_ctrl_ready_low(int unsigned max_cycles, string ctx);
      repeat (max_cycles) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.ready === 1'b0)
          return;
      end
      `uvm_fatal("FRCV_TIMEOUT",
        $sformatf("%s timed out waiting for ctrl_ready to deassert", ctx))
    endtask

    task automatic wait_for_ctrl_ready_high(int unsigned max_cycles, string ctx);
      repeat (max_cycles) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.ready === 1'b1)
          return;
      end
      `uvm_fatal("FRCV_TIMEOUT",
        $sformatf("%s timed out waiting for ctrl_ready to assert", ctx))
    endtask

    function void report_phase(uvm_phase phase);
      uvm_report_server server;
      server = uvm_report_server::get_server();
      if (server.get_severity_count(UVM_FATAL) == 0 &&
          server.get_severity_count(UVM_ERROR) == 0)
        $display("*** TEST PASSED ***");
      else
        $display("*** TEST FAILED ***");
    endfunction
  endclass

  class COMBO_FRCV_001_run_contract_test extends frcv_base_test;
    `uvm_component_utils(COMBO_FRCV_001_run_contract_test)

    localparam bit [47:0] ACTIVE_HIT_WORD          = 48'hCAFEBABE1234;
    localparam bit [47:0] DELAYED_TAIL_HIT_WORD    = 48'h0F1E2D3C4B5A;
    localparam int unsigned TERMINATE_WAIT_MAX_CYCLES = 4096;
    localparam int unsigned DELAYED_TAIL_GAP_CYCLES   = 64;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      phase.raise_objection(this);

      wait_for_reset_release();
      run_start();

      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns  = m_env.m_scb.endofrun_count;

      send_long_frame(16'h0001, 48'h112233445566);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 24,
        "RUNNING long frame");

      if (m_env.m_scb.endofrun_count != base_endofruns)
        `uvm_fatal("FRCV_TEST",
          $sformatf("Unexpected end-of-run pulse during RUNNING frame, got %0d expected %0d",
            m_env.m_scb.endofrun_count, base_endofruns))

      wait_cycles(8);
      base_endofruns = m_env.m_scb.endofrun_count;
      pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
      wait_for_endofrun_count(base_endofruns + 1, TERMINATE_WAIT_MAX_CYCLES,
        "Idle TERMINATING end-of-run marker");
      wait_for_ctrl_ready_high(TERMINATE_WAIT_MAX_CYCLES, "Idle TERMINATING ready restore");

      send_ctrl(CTRL_IDLE, "IDLE");

      base_hits       = m_env.m_scb.hit_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns  = m_env.m_scb.endofrun_count;

      send_long_frame(16'h0002, 48'hA1B2C3D4E5F6);
      wait_cycles(14);

      if (m_env.m_scb.hit_count != base_hits)
        `uvm_fatal("FRCV_TEST",
          $sformatf("IDLE must stay quiescent: hit_count moved from %0d to %0d",
            base_hits, m_env.m_scb.hit_count))
      if (m_env.m_scb.header_count != base_headers)
        `uvm_fatal("FRCV_TEST",
          $sformatf("IDLE must not emit headerinfo: header_count moved from %0d to %0d",
            base_headers, m_env.m_scb.header_count))
      if (m_env.m_scb.endofrun_count != base_endofruns)
        `uvm_fatal("FRCV_TEST",
          $sformatf("IDLE should not emit end-of-run pulses: count moved from %0d to %0d",
            base_endofruns, m_env.m_scb.endofrun_count))

      run_start();

      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns  = m_env.m_scb.endofrun_count;

      fork
        begin
          send_ctrl(CTRL_TERMINATING, "TERMINATING");
        end
        begin
          wait_cycles(DELAYED_TAIL_GAP_CYCLES);
          if (m_env.m_scb.hit_count != base_hits)
            `uvm_fatal("FRCV_TEST",
              $sformatf("Delayed terminating tail must stay quiescent before the late frame: hit_count moved from %0d to %0d",
                base_hits, m_env.m_scb.hit_count))
          if (m_env.m_scb.header_count != base_headers)
            `uvm_fatal("FRCV_TEST",
              $sformatf("Delayed terminating tail must not emit headerinfo before the late frame: header_count moved from %0d to %0d",
                base_headers, m_env.m_scb.header_count))
          if (m_env.m_scb.endofrun_count != base_endofruns)
            `uvm_fatal("FRCV_TEST",
              $sformatf("Delayed terminating tail must not close early: endofrun_count moved from %0d to %0d",
                base_endofruns, m_env.m_scb.endofrun_count))
          if (ctrl_vif.ready !== 1'b0)
            `uvm_fatal("FRCV_TEST", "Delayed terminating tail must hold ctrl_ready low before the late frame arrives")
          send_long_frame(16'h0003, DELAYED_TAIL_HIT_WORD);
          wait_cycles(16);
          send_empty_frame(16'h0004);
        end
      join

      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 2, TERMINATE_WAIT_MAX_CYCLES,
        "Delayed-tail TERMINATING drain");
      wait_for_endofrun_count(base_endofruns + 1, TERMINATE_WAIT_MAX_CYCLES,
        "Delayed-tail TERMINATING end-of-run marker");
      wait_for_ctrl_ready_high(8, "Delayed-tail TERMINATING ready restore");

      run_start();

      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_endofruns  = m_env.m_scb.endofrun_count;

      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h03);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h01);

      fork
        begin
          send_ctrl(CTRL_TERMINATING, "TERMINATING");
        end
        begin
          for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
            drive_symbol(1'b0, byte'(ACTIVE_HIT_WORD[byte_idx*8 +: 8]));
          drive_symbol(1'b0, 8'h00);
          drive_symbol(1'b0, 8'h00);
          wait_cycles(16);
          send_empty_frame(16'h0004);
        end
      join

      wait_for_counts(base_hits + 1, base_real_eops + 1, m_env.m_scb.header_count, TERMINATE_WAIT_MAX_CYCLES,
        "Active TERMINATING drain");
      wait_for_endofrun_count(base_endofruns + 1, TERMINATE_WAIT_MAX_CYCLES, "Active TERMINATING end-of-run marker");
      wait_for_ctrl_ready_high(TERMINATE_WAIT_MAX_CYCLES, "Active TERMINATING ready restore");

      send_ctrl(CTRL_IDLE, "IDLE");
      phase.drop_objection(this);
    endtask
  endclass

  `include "frcv_cases.svh"
endpackage
