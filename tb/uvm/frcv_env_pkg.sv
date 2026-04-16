`timescale 1ps/1ps

package frcv_env_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  `uvm_analysis_imp_decl(_obs)

  localparam logic [8:0] CTRL_IDLE        = 9'b000000001;
  localparam logic [8:0] CTRL_RUN_PREPARE = 9'b000000010;
  localparam logic [8:0] CTRL_SYNC        = 9'b000000100;
  localparam logic [8:0] CTRL_RUNNING     = 9'b000001000;
  localparam logic [8:0] CTRL_TERMINATING = 9'b000010000;
  localparam byte unsigned HEADER_BYTE    = 8'h1c;
  localparam time CLK_PERIOD_PS           = 8000ps;

  typedef enum int unsigned {
    FRCV_OBS_HEADER,
    FRCV_OBS_HIT,
    FRCV_OBS_SYNTH_EOP
  } frcv_obs_kind_e;

  class frcv_symbol_item extends uvm_sequence_item;
    `uvm_object_utils(frcv_symbol_item)

    bit [8:0] data;
    bit       valid;
    bit [2:0] error;
    bit [3:0] channel;

    function new(string name = "frcv_symbol_item");
      super.new(name);
      valid   = 1'b1;
      error   = '0;
      channel = 4'h2;
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
    bit [1:0]  address;
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

    bit [3:0]       hit_channel;
    bit             hit_sop;
    bit             hit_eop;
    bit [2:0]       hit_error;
    bit [44:0]      hit_data;

    bit [41:0]      headerinfo_data;
    bit [3:0]       headerinfo_channel;

    function new(string name = "frcv_obs_item");
      super.new(name);
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
      vif.channel <= 4'h2;

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

        if (vif.hit_eop === 1'b1 && vif.hit_valid !== 1'b1) begin
          obs         = frcv_obs_item::type_id::create("synth_eop_obs");
          obs.kind    = FRCV_OBS_SYNTH_EOP;
          obs.time_ps = $time;
          ap.write(obs);
        end
      end
    endtask
  endclass

  class frcv_scoreboard extends uvm_component;
    `uvm_component_utils(frcv_scoreboard)

    uvm_analysis_imp_obs #(frcv_obs_item, frcv_scoreboard) obs_imp;

    int unsigned header_count;
    int unsigned hit_count;
    int unsigned real_eop_count;
    int unsigned synth_eop_count;

    time         last_header_time_ps;
    time         last_real_eop_time_ps;
    time         last_synth_eop_time_ps;

    frcv_obs_item history[$];

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      obs_imp               = new("obs_imp", this);
      header_count          = 0;
      hit_count             = 0;
      real_eop_count        = 0;
      synth_eop_count       = 0;
      last_header_time_ps   = 0;
      last_real_eop_time_ps = 0;
      last_synth_eop_time_ps = 0;
    endfunction

    function void write_obs(frcv_obs_item item);
      history.push_back(item);
      case (item.kind)
        FRCV_OBS_HEADER: begin
          header_count        ++;
          last_header_time_ps  = item.time_ps;
        end
        FRCV_OBS_HIT: begin
          hit_count++;
          if (item.hit_eop) begin
            real_eop_count++;
            last_real_eop_time_ps = item.time_ps;
          end
        end
        FRCV_OBS_SYNTH_EOP: begin
          synth_eop_count++;
          last_synth_eop_time_ps = item.time_ps;
        end
        default: begin
        end
      endcase
    endfunction

    function void report_phase(uvm_phase phase);
      `uvm_info("FRCV_SCB",
        $sformatf("headers=%0d hits=%0d real_eops=%0d synth_eops=%0d",
          header_count, hit_count, real_eop_count, synth_eop_count),
        UVM_LOW)
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
      m_out_mon = frcv_out_monitor::type_id::create("m_out_mon", this);
      m_scb     = frcv_scoreboard::type_id::create("m_scb", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      m_csr_drv.seq_item_port.connect(m_csr_sqr.seq_item_export);
      m_rx_drv.seq_item_port.connect(m_rx_sqr.seq_item_export);
      m_ctrl_drv.seq_item_port.connect(m_ctrl_sqr.seq_item_export);
      m_out_mon.ap.connect(m_scb.obs_imp);
    endfunction
  endclass

  class frcv_symbol_seq extends uvm_sequence #(frcv_symbol_item);
    `uvm_object_utils(frcv_symbol_seq)

    bit [8:0] data;
    bit       valid;
    bit [2:0] error;
    bit [3:0] channel;

    function new(string name = "frcv_symbol_seq");
      super.new(name);
      valid   = 1'b1;
      error   = '0;
      channel = 4'h2;
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

    bit [1:0]  addr;
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

    bit [1:0]  addr;
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
                                bit [2:0] error = '0, bit [3:0] channel = 4'h2);
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

    task automatic csr_write(bit [1:0] addr, bit [31:0] data);
      frcv_csr_write_seq seq;
      seq      = frcv_csr_write_seq::type_id::create($sformatf("csr_wr_%0t", $time));
      seq.addr = addr;
      seq.data = data;
      seq.start(m_env.m_csr_sqr);
    endtask

    task automatic csr_read(bit [1:0] addr, output bit [31:0] data);
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

    task automatic wait_for_synth_count(int unsigned exp_synth_eops,
                                        int unsigned max_cycles,
                                        string ctx);
      repeat (max_cycles) begin
        if (m_env.m_scb.synth_eop_count >= exp_synth_eops)
          return;
        @(posedge ctrl_vif.clk);
      end
      `uvm_fatal("FRCV_TIMEOUT",
        $sformatf("%s timed out waiting for synth_eops=%0d, got %0d",
          ctx, exp_synth_eops, m_env.m_scb.synth_eop_count))
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

    localparam bit [47:0] ACTIVE_HIT_WORD = 48'hCAFEBABE1234;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;
      time         idle_term_accept_time_ps;
      time         active_term_accept_time_ps;

      phase.raise_objection(this);

      wait_for_reset_release();
      run_start();

      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;

      send_long_frame(16'h0001, 48'h112233445566);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 24,
        "RUNNING long frame");

      if (m_env.m_scb.synth_eop_count != base_synth_eops)
        `uvm_fatal("FRCV_TEST",
          $sformatf("Unexpected synthetic EOP during RUNNING frame, got %0d expected %0d",
            m_env.m_scb.synth_eop_count, base_synth_eops))

      wait_cycles(8);
      base_synth_eops = m_env.m_scb.synth_eop_count;
      pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
      wait_for_ctrl_ready_low(4, "Idle TERMINATING ready deassert");
      wait_for_synth_count(base_synth_eops + 1, 16,
        "Idle TERMINATING synthetic marker");
      wait_for_ctrl_ready_high(16, "Idle TERMINATING ready restore");

      send_ctrl(CTRL_IDLE, "IDLE");

      base_hits       = m_env.m_scb.hit_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;

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
      if (m_env.m_scb.synth_eop_count != base_synth_eops)
        `uvm_fatal("FRCV_TEST",
          $sformatf("IDLE should not emit synthetic EOPs: synth_count moved from %0d to %0d",
            base_synth_eops, m_env.m_scb.synth_eop_count))

      run_start();

      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;

      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h03);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h01);

      fork
        begin
          pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
        end
        begin
          wait_cycles(1);
          for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
            drive_symbol(1'b0, byte'(ACTIVE_HIT_WORD[byte_idx*8 +: 8]));
          drive_symbol(1'b0, 8'h00);
          drive_symbol(1'b0, 8'h00);
        end
      join

      wait_for_ctrl_ready_low(4, "Active TERMINATING ready deassert");
      wait_for_counts(base_hits + 1, base_real_eops + 1, m_env.m_scb.header_count, 48,
        "Active TERMINATING drain");

      if (m_env.m_scb.synth_eop_count != base_synth_eops)
        `uvm_fatal("FRCV_TEST",
          $sformatf("Active TERMINATING drain must not emit synthetic EOPs: got %0d expected %0d",
            m_env.m_scb.synth_eop_count, base_synth_eops))

      wait_for_ctrl_ready_high(48, "Active TERMINATING ready restore");

      send_ctrl(CTRL_IDLE, "IDLE");
      phase.drop_objection(this);
    endtask
  endclass

  `include "frcv_cases.svh"
endpackage
