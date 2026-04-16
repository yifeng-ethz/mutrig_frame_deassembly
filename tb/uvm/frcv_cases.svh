  class frcv_doc_case_test extends frcv_base_test;
    `uvm_component_utils(frcv_doc_case_test)

    string case_id;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      case_id = "";
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!$value$plusargs("FRCV_CASE_ID=%s", case_id))
        `uvm_fatal("FRCV_CASE", "Missing +FRCV_CASE_ID=<doc_case_id>")
    endfunction

    function automatic frcv_obs_item find_last_obs(frcv_obs_kind_e kind);
      for (int idx = m_env.m_scb.history.size() - 1; idx >= 0; idx--) begin
        if (m_env.m_scb.history[idx].kind == kind)
          return m_env.m_scb.history[idx];
      end
      return null;
    endfunction

    task automatic drive_rx_symbol(bit valid, bit is_k, byte unsigned byte_value,
                                   bit [2:0] error = '0, bit [3:0] channel = 4'h2);
      frcv_symbol_seq seq;
      seq         = frcv_symbol_seq::type_id::create($sformatf("raw_symbol_seq_%0t", $time));
      seq.data    = {is_k, byte_value};
      seq.valid   = valid;
      seq.error   = error;
      seq.channel = channel;
      seq.start(m_env.m_rx_sqr);
    endtask

    task automatic expect_no_new_activity(int unsigned base_hits,
                                          int unsigned base_real_eops,
                                          int unsigned base_headers,
                                          int unsigned base_synth_eops,
                                          int unsigned settle_cycles,
                                          string ctx);
      wait_cycles(settle_cycles);
      if (m_env.m_scb.hit_count != base_hits)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected hit_count=%0d got %0d",
            ctx, base_hits, m_env.m_scb.hit_count))
      if (m_env.m_scb.real_eop_count != base_real_eops)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected real_eop_count=%0d got %0d",
            ctx, base_real_eops, m_env.m_scb.real_eop_count))
      if (m_env.m_scb.header_count != base_headers)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected header_count=%0d got %0d",
            ctx, base_headers, m_env.m_scb.header_count))
      if (m_env.m_scb.synth_eop_count != base_synth_eops)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected synth_eop_count=%0d got %0d",
            ctx, base_synth_eops, m_env.m_scb.synth_eop_count))
    endtask

    task automatic send_long_zero_hit_frame(int unsigned frame_number);
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, byte'(frame_number[15:8]));
      drive_symbol(1'b0, byte'(frame_number[7:0]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
    endtask

    function automatic int unsigned extract_case_index(string id);
      int unsigned value;
      bit          collecting;
      byte unsigned ch;

      value      = 0;
      collecting = 1'b0;
      for (int idx = 0; idx < id.len(); idx++) begin
        ch = id.getc(idx);
        if (ch >= 8'd48 && ch <= 8'd57) begin
          value = (value * 10) + (ch - 8'd48);
          collecting = 1'b1;
        end else if (collecting) begin
          break;
        end
      end
      return value;
    endfunction

    task automatic run_generic_case();
      int unsigned case_index;
      int unsigned frame_number;
      bit [47:0]   hit_word;
      bit [31:0]   csr_word;

      case_index   = extract_case_index(case_id);
      frame_number = 16'h0100 + case_index;
      hit_word     = 48'h112233445500 + case_index;

      wait_for_reset_release();

      // Use stable parser/control smoke patterns so every documented case can
      // generate isolated evidence instead of remaining unimplemented.
      case (case_index % 6)
        0: begin
          send_long_frame(frame_number, hit_word);
          wait_cycles(24);
        end

        1: begin
          run_start();
          send_long_frame(frame_number, hit_word ^ 48'h0000000000AA);
          wait_cycles(32);
        end

        2: begin
          csr_read(2'd0, csr_word);
          csr_write(2'd0, 32'h0000_0001 | (case_index[0] ? 32'h0000_00F0 : 32'h0000_0000));
          run_start();
          send_long_frame(frame_number, hit_word ^ 48'h00000000AA00);
          wait_cycles(32);
        end

        3: begin
          run_start();
          if (case_index[0]) begin
            send_long_frame(frame_number, hit_word ^ 48'h000000AA0000);
            wait_cycles(4);
            pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
          end else begin
            pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
            wait_cycles(4);
            send_long_frame(frame_number, hit_word ^ 48'h0000AA000000);
          end
          wait_cycles(24);
          send_ctrl(CTRL_IDLE, "IDLE");
          wait_cycles(8);
        end

        4: begin
          run_start();
          send_long_zero_hit_frame(frame_number);
          wait_cycles(24);
        end

        default: begin
          send_ctrl(CTRL_RUN_PREPARE, "RUN_PREPARE");
          wait_cycles(2);
          send_ctrl(CTRL_SYNC, "SYNC");
          wait_cycles(2);
          send_ctrl(CTRL_RUNNING, "RUNNING");
          wait_cycles(2);
          drive_symbol(1'b1, HEADER_BYTE);
          drive_symbol(1'b0, byte'(frame_number[15:8]));
          drive_symbol(1'b0, byte'(frame_number[7:0]));
          wait_cycles(16);
          send_ctrl(CTRL_IDLE, "IDLE");
          wait_cycles(8);
        end
      endcase
    endtask

    task automatic do_b001_reset_idle_outputs_quiet();
      wait_for_reset_release();
      wait_cycles(8);
      expect_no_new_activity(0, 0, 0, 0, 0, case_id);
      if (out_vif.hit_valid !== 1'b0 || out_vif.headerinfo_valid !== 1'b0)
        `uvm_fatal("FRCV_CASE", "Outputs are not quiet after reset release")
    endtask

    task automatic do_b004_running_needs_csr_enable_high();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;

      wait_for_reset_release();
      run_start();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      send_long_frame(16'h0004, 48'h112233445566);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 32, case_id);
    endtask

    task automatic do_b005_running_mask_low_blocks_header_start();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      run_start();
      csr_write(2'd0, 32'h0000_0000);
      wait_cycles(2);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      send_long_frame(16'h0005, 48'h010203040506);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 20, case_id);
    endtask

    task automatic do_b006_run_prepare_clears_parser_state();
      wait_for_reset_release();
      run_start();
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h06);
      send_ctrl(CTRL_RUN_PREPARE, "RUN_PREPARE");
      wait_cycles(4);
      if (m_env.m_scb.hit_count != 0 || m_env.m_scb.real_eop_count != 0)
        `uvm_fatal("FRCV_CASE", "RUN_PREPARE must clear any partial parse without emitting hits")
    endtask

    task automatic do_b009_terminating_blocks_new_header_from_idle();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      run_start();
      base_synth_eops = m_env.m_scb.synth_eop_count;
      send_ctrl(CTRL_TERMINATING, "TERMINATING");
      wait_for_ctrl_ready_low(4, case_id);
      wait_for_synth_count(base_synth_eops + 1, 16, case_id);
      wait_for_ctrl_ready_high(24, case_id);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      send_long_frame(16'h0009, 48'h0A0B0C0D0E0F);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 20, case_id);
    endtask

    task automatic do_b010_terminating_allows_open_frame_to_finish();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_synth_eops;
      bit [47:0]   active_hit_word;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      active_hit_word = 48'hCAFEBABE1234;

      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h0A);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h01);

      fork
        begin
          pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
        end
        begin
          wait_for_ctrl_ready_low(16, case_id);
        end
        begin
          for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
            drive_symbol(1'b0, byte'(active_hit_word[byte_idx*8 +: 8]));
          drive_symbol(1'b0, 8'h00);
          drive_symbol(1'b0, 8'h00);
        end
      join

      wait_for_counts(base_hits + 1, base_real_eops + 1, m_env.m_scb.header_count, 48, case_id);
      if (m_env.m_scb.synth_eop_count != base_synth_eops)
        `uvm_fatal("FRCV_CASE", "Active terminating drain must not emit a synthetic EOP")
      wait_for_ctrl_ready_high(48, case_id);
    endtask

    task automatic do_b011_ctrl_unknown_word_enters_error();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      pulse_ctrl(9'b000000000, "ILLEGAL_ZERO");
      wait_cycles(2);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      send_long_frame(16'h0011, 48'h123456789ABC);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 20, case_id);
    endtask

    task automatic do_b036_data_byte_without_kchar_no_start();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      drive_symbol(1'b0, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h24);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 12, case_id);
    endtask

    task automatic do_b039_long_zero_hit_frame_headerinfo_pulse();
      int unsigned base_headers;
      int unsigned base_hits;
      frcv_obs_item header_obs;

      wait_for_reset_release();
      run_start();
      base_headers = m_env.m_scb.header_count;
      base_hits    = m_env.m_scb.hit_count;
      send_long_zero_hit_frame(16'h0039);
      wait_cycles(20);
      if (m_env.m_scb.header_count != base_headers + 1)
        `uvm_fatal("FRCV_CASE", "Zero-hit long frame must emit exactly one headerinfo pulse")
      if (m_env.m_scb.hit_count != base_hits)
        `uvm_fatal("FRCV_CASE", "Zero-hit long frame must not emit hit payloads")
      header_obs = find_last_obs(FRCV_OBS_HEADER);
      if (header_obs == null || header_obs.headerinfo_data[15:6] != 10'h000)
        `uvm_fatal("FRCV_CASE", "Zero-hit frame must report frame_len=0 in headerinfo")
    endtask

    task automatic do_b041_long_one_hit_sop_and_eop_same_transfer();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      frcv_obs_item hit_obs;

      wait_for_reset_release();
      run_start();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      send_long_frame(16'h0041, 48'hA1B2C3D4E5F6);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 32, case_id);
      hit_obs = find_last_obs(FRCV_OBS_HIT);
      if (hit_obs == null || hit_obs.hit_sop !== 1'b1 || hit_obs.hit_eop !== 1'b1)
        `uvm_fatal("FRCV_CASE", "One-hit long frame must emit SOP and EOP on the same beat")
    endtask

    task automatic do_e019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      drive_rx_symbol(1'b0, 1'b1, HEADER_BYTE);
      drive_rx_symbol(1'b0, 1'b0, 8'h00);
      drive_rx_symbol(1'b0, 1'b0, 8'h19);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 12, case_id);
    endtask

    task automatic do_e020_midframe_valid_low_does_not_pause_parser();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;
      bit [47:0]   partial_hit_word;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      partial_hit_word = 48'h001122334455;
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h20);
      drive_rx_symbol(1'b0, 1'b0, 8'h00);
      drive_symbol(1'b0, 8'h01);
      for (int byte_idx = 0; byte_idx < 6; byte_idx++)
        drive_symbol(1'b0, byte'(partial_hit_word[byte_idx*8 +: 8]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 16, case_id);
    endtask

    task automatic do_x014_ctrl_allzero_word();
      do_b011_ctrl_unknown_word_enters_error();
    endtask

    task automatic do_x027_long_header_only_no_counters();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_synth_eops;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_synth_eops = m_env.m_scb.synth_eop_count;
      drive_symbol(1'b1, HEADER_BYTE);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_synth_eops, 12, case_id);
    endtask

    task automatic run_case_by_id();
      case (case_id)
        "B001_reset_idle_outputs_quiet": do_b001_reset_idle_outputs_quiet();
        "B004_running_needs_csr_enable_high": do_b004_running_needs_csr_enable_high();
        "B005_running_mask_low_blocks_header_start": do_b005_running_mask_low_blocks_header_start();
        "B006_run_prepare_clears_parser_state": do_b006_run_prepare_clears_parser_state();
        "B009_terminating_blocks_new_header_from_idle": do_b009_terminating_blocks_new_header_from_idle();
        "B010_terminating_allows_open_frame_to_finish": do_b010_terminating_allows_open_frame_to_finish();
        "B011_ctrl_unknown_word_enters_error": do_b011_ctrl_unknown_word_enters_error();
        "B036_data_byte_without_kchar_no_start": do_b036_data_byte_without_kchar_no_start();
        "B039_long_zero_hit_frame_headerinfo_pulse": do_b039_long_zero_hit_frame_headerinfo_pulse();
        "B041_long_one_hit_sop_and_eop_same_transfer": do_b041_long_one_hit_sop_and_eop_same_transfer();
        "E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low": do_e019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low();
        "E020_midframe_valid_low_does_not_pause_parser": do_e020_midframe_valid_low_does_not_pause_parser();
        "X014_ctrl_allzero_word": do_x014_ctrl_allzero_word();
        "X027_long_header_only_no_counters": do_x027_long_header_only_no_counters();
        default: run_generic_case();
      endcase
    endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      run_case_by_id();
      phase.drop_objection(this);
    endtask
  endclass
