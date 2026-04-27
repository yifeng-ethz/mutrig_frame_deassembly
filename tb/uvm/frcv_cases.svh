  class frcv_doc_case_test extends frcv_base_test;
    `uvm_component_utils(frcv_doc_case_test)

    string case_id;
    string build_tag;
    string execution_mode;
    string effort_mode;
    bit    require_case_id;
    localparam int unsigned MAX_DOC_ITERATIONS_ISOLATED = 128;
    localparam int unsigned MAX_DOC_ITERATIONS_CONTINUOUS = 32;
    localparam int unsigned MAX_DOC_PAYLOAD_BYTES_ISOLATED = 8192;
    localparam int unsigned MAX_DOC_PAYLOAD_BYTES_CONTINUOUS = 2048;
    localparam int unsigned MAX_DOC_ITERATIONS_ISOLATED_EXTENSIVE = 100000;
    localparam int unsigned MAX_DOC_ITERATIONS_CONTINUOUS_EXTENSIVE = 512;
    localparam int unsigned MAX_DOC_PAYLOAD_BYTES_ISOLATED_EXTENSIVE = 10000000;
    localparam int unsigned MAX_DOC_PAYLOAD_BYTES_CONTINUOUS_EXTENSIVE = 131072;
    localparam int unsigned DOC_CASE_DRAIN_MAX_CYCLES = 4096;
    localparam int unsigned TERMINATING_CLOSE_MAX_CYCLES = 2300;

    covergroup doc_case_cg with function sample(int unsigned global_case_idx,
                                                int unsigned bucket_case_idx,
                                                bit          is_random_case);
      option.per_instance = 1;
      cp_global_case: coverpoint global_case_idx {
        bins documented_cases[] = {[1:520]};
      }
      cp_bucket_case: coverpoint bucket_case_idx {
        bins bucket_cases[] = {[1:130]};
      }
      cp_case_kind: coverpoint is_random_case {
        bins directed = {0};
        bins random   = {1};
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      case_id = "";
      build_tag = "CFG_A";
      execution_mode = "isolated";
      effort_mode = "practical";
      require_case_id = 1'b1;
      doc_case_cg = new;
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!$value$plusargs("FRCV_CASE_ID=%s", case_id) && require_case_id)
        `uvm_fatal("FRCV_CASE", "Missing +FRCV_CASE_ID=<doc_case_id>")
      void'($value$plusargs("FRCV_BUILD_TAG=%s", build_tag));
      void'($value$plusargs("FRCV_EXEC_MODE=%s", execution_mode));
      void'($value$plusargs("FRCV_EFFORT=%s", effort_mode));
      if (case_id != "")
        doc_case_cg.set_inst_name($sformatf("doc_case_cg_%s", case_id));
    endfunction

    function automatic frcv_obs_item find_last_obs(frcv_obs_kind_e kind);
      for (int idx = m_env.m_scb.history.size() - 1; idx >= 0; idx--) begin
        if (m_env.m_scb.history[idx].kind == kind)
          return m_env.m_scb.history[idx];
      end
      return null;
    endfunction

    function automatic frcv_csr_obs_item find_last_csr_obs();
      if (m_env.m_scb.csr_history.size() == 0)
        return null;
      return m_env.m_scb.csr_history[m_env.m_scb.csr_history.size() - 1];
    endfunction

    task automatic drive_rx_symbol(bit valid, bit is_k, byte unsigned byte_value,
                                   bit [2:0] error = '0, bit [7:0] channel = 8'h02);
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
                                          int unsigned base_endofruns,
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
      if (m_env.m_scb.endofrun_count != base_endofruns)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected endofrun_count=%0d got %0d",
            ctx, base_endofruns, m_env.m_scb.endofrun_count))
    endtask

    task automatic expect_activity_delta(int unsigned base_hits,
                                         int unsigned base_real_eops,
                                         int unsigned base_headers,
                                         int unsigned base_endofruns,
                                         int unsigned exp_hit_delta,
                                         int unsigned exp_real_eop_delta,
                                         int unsigned exp_header_delta,
                                         int unsigned exp_endofrun_delta,
                                         int unsigned settle_cycles,
                                         string ctx);
      wait_cycles(settle_cycles);
      if (m_env.m_scb.hit_count != (base_hits + exp_hit_delta))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected hit_count=%0d got %0d",
            ctx, base_hits + exp_hit_delta, m_env.m_scb.hit_count))
      if (m_env.m_scb.real_eop_count != (base_real_eops + exp_real_eop_delta))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected real_eop_count=%0d got %0d",
            ctx, base_real_eops + exp_real_eop_delta, m_env.m_scb.real_eop_count))
      if (m_env.m_scb.header_count != (base_headers + exp_header_delta))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected header_count=%0d got %0d",
            ctx, base_headers + exp_header_delta, m_env.m_scb.header_count))
      if (m_env.m_scb.endofrun_count != (base_endofruns + exp_endofrun_delta))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected endofrun_count=%0d got %0d",
            ctx, base_endofruns + exp_endofrun_delta, m_env.m_scb.endofrun_count))
    endtask

    task automatic send_long_zero_hit_frame(int unsigned frame_number);
      byte unsigned frame_bytes[$];
      bit [15:0]    crc_trailer;

      frame_bytes.push_back(byte'(frame_number[15:8]));
      frame_bytes.push_back(byte'(frame_number[7:0]));
      frame_bytes.push_back(8'h00);
      frame_bytes.push_back(8'h00);
      crc_trailer = frcv_crc16_trailer(frame_bytes);

      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, byte'(frame_number[15:8]));
      drive_symbol(1'b0, byte'(frame_number[7:0]));
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, byte'(crc_trailer[15:8]));
      drive_symbol(1'b0, byte'(crc_trailer[7:0]));
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

    task automatic fail_unimplemented_case();
      `uvm_fatal("FRCV_CASE_UNIMPLEMENTED",
        $sformatf("Case %s is not recognized by the documented-case engine.", case_id))
    endtask

    task automatic do_b001_reset_idle_outputs_quiet();
      wait_for_reset_release();
      wait_cycles(8);
      expect_no_new_activity(0, 0, 0, 0, 0, case_id);
      if (out_vif.hit_valid !== 1'b0 || out_vif.headerinfo_valid !== 1'b0)
        `uvm_fatal("FRCV_CASE", "Outputs are not quiet after reset release")
    endtask

    task automatic do_b004_running_needs_csr_enable_high();
      wait_for_reset_release();
      run_start();
      send_clean_recovery_frame(8'h02, 16'h0004, 48'h112233445566, "none", case_id);
    endtask

    task automatic do_b005_running_mask_low_blocks_header_start();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      csr_write(2'd0, 32'h0000_0000);
      wait_cycles(2);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      send_long_frame(16'h0005, 48'h010203040506);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 20, case_id);
    endtask

    task automatic do_b006_run_prepare_clears_parser_state();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns  = m_env.m_scb.endofrun_count;
      drive_symbol(1'b1, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h06);
      send_ctrl(CTRL_RUN_PREPARE, "RUN_PREPARE");
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 8, case_id);
    endtask

    task automatic do_b009_terminating_blocks_new_header_from_idle();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      base_endofruns = m_env.m_scb.endofrun_count;
      send_ctrl(CTRL_TERMINATING, "TERMINATING");
      wait_for_ctrl_ready_low(4, case_id);
      wait_for_endofrun_count(base_endofruns + 1, TERMINATING_CLOSE_MAX_CYCLES, case_id);
      wait_for_ctrl_ready_high(24, case_id);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      send_long_frame(16'h0009, 48'h0A0B0C0D0E0F);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 20, case_id);
    endtask

    task automatic do_b010_terminating_allows_open_frame_to_finish();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      bit [47:0]   active_hit_word;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      active_hit_word = 48'hCAFEBABE1234;
      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);

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

      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 48, case_id);
      wait_for_endofrun_count(base_endofruns + 1, TERMINATING_CLOSE_MAX_CYCLES, case_id);
      wait_for_ctrl_ready_high(48, case_id);
    endtask

    task automatic do_b011_ctrl_unknown_word_enters_error();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      pulse_ctrl(9'b000000000, "ILLEGAL_ZERO");
      wait_cycles(2);
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      send_long_frame(16'h0011, 48'h123456789ABC);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 20, case_id);
    endtask

    task automatic do_b036_data_byte_without_kchar_no_start();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      drive_symbol(1'b0, HEADER_BYTE);
      drive_symbol(1'b0, 8'h00);
      drive_symbol(1'b0, 8'h24);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
    endtask

    task automatic do_b039_long_zero_hit_frame_headerinfo_pulse();
      int unsigned base_headers;
      int unsigned base_hits;
      frcv_obs_item header_obs;

      wait_for_reset_release();
      run_start();
      base_headers = m_env.m_scb.header_count;
      base_hits    = m_env.m_scb.hit_count;
      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
        random_doc_case(), 1'b0, 0, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);
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
      frcv_obs_item hit_obs;

      wait_for_reset_release();
      run_start();
      send_clean_recovery_frame(8'h02, 16'h0041, 48'hA1B2C3D4E5F6, "none", case_id);
      hit_obs = find_last_obs(FRCV_OBS_HIT);
      if (hit_obs == null || hit_obs.hit_sop !== 1'b1 || hit_obs.hit_eop !== 1'b1)
        `uvm_fatal("FRCV_CASE", "One-hit long frame must emit SOP and EOP on the same beat")
    endtask

    task automatic do_single_bad_crc_case(bit [7:0] channel,
                                          int unsigned frame_number,
                                          bit short_mode,
                                          bit expect_crc_error_pulse = 1'b0);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      bit [31:0]   csr_data;
      int unsigned wait_cycles_left;
      bit          saw_crc_error_pulse;
      bit          pulse_valid_v;
      bit          pulse_eop_v;

      wait_for_reset_release();
      run_start_for_mode();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), short_mode, 1, 1, 1, 1, 1, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
      if (expect_crc_error_pulse) begin
        saw_crc_error_pulse = 1'b0;
        pulse_valid_v = 1'b0;
        pulse_eop_v   = 1'b0;
        wait_cycles_left = 64;
        fork
          begin
            send_doc_frame(frame_number, 1, short_mode, channel, 1'b1);
            wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 48, case_id);
          end
          begin
            while (wait_cycles_left != 0) begin
              @(posedge out_vif.clk);
              if (out_vif.hit_error[1] === 1'b1) begin
                saw_crc_error_pulse = 1'b1;
                pulse_valid_v = out_vif.hit_valid;
                pulse_eop_v   = out_vif.hit_eop;
                break;
              end
              wait_cycles_left--;
            end
          end
        join
        if (!saw_crc_error_pulse)
          `uvm_fatal("FRCV_CASE",
            $sformatf("%s expected a crc_error sideband pulse while the bad frame retired", case_id))
        if (pulse_valid_v !== 1'b0 || pulse_eop_v !== 1'b0)
          `uvm_fatal("FRCV_CASE",
            $sformatf("%s expected crc_error pulse off the valid/eop beat, saw valid=%0b eop=%0b",
              case_id, pulse_valid_v, pulse_eop_v))
        wait_cycles(8);
      end else begin
        send_doc_frame(frame_number, 1, short_mode, channel, 1'b1);
        wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 48, case_id);
        wait_cycles(24);
      end
      csr_read(8'h01, csr_data);
      if (csr_data != 32'd1)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected crc counter readback 1 got %0d", case_id, csr_data))
    endtask

    task automatic do_b080_short_crc_counter_accumulates(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      bit [31:0]   base_crc_counter;
      bit [31:0]   csr_data;

      wait_for_reset_release();
      run_start_for_mode();
      csr_read(8'h01, base_crc_counter);
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;

      for (int unsigned iter = 0; iter < 2; iter++) begin
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
          random_doc_case(), 1'b1, 1, 1, 1, 1, 1, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
        send_doc_frame(16'h0080 + iter, 1, 1'b1, channel, 1'b1);
        wait_for_counts(base_hits + iter + 1, base_real_eops + iter + 1, base_headers + iter + 1, 64, case_id);
        wait_cycles(8);
      end

      wait_cycles(24);
      csr_read(8'h01, csr_data);
      if (csr_data != (base_crc_counter + 32'd2))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected crc counter %0d got %0d",
            case_id, base_crc_counter + 32'd2, csr_data))
    endtask

    task automatic do_b112_long_good_frames_leave_crc_counter_stable(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      bit [31:0]   base_crc_counter;
      bit [31:0]   csr_data;

      wait_for_reset_release();
      run_start_for_mode();
      csr_read(8'h01, base_crc_counter);
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;

      for (int unsigned iter = 0; iter < 4; iter++) begin
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
          random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
        send_doc_frame(16'h1120 + iter, 1, 1'b0, channel, 1'b0);
        wait_for_counts(base_hits + iter + 1, base_real_eops + iter + 1, base_headers + iter + 1, 64, case_id);
        wait_cycles(8);
      end

      wait_cycles(24);
      csr_read(8'h01, csr_data);
      if (csr_data != base_crc_counter)
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected crc counter to remain %0d got %0d",
            case_id, base_crc_counter, csr_data))
    endtask

    task automatic do_b113_bad_frames_accumulate_crc_counter(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      bit [31:0]   base_crc_counter;
      bit [31:0]   csr_data;
      bit          short_mode_v;

      wait_for_reset_release();
      run_start_for_mode();
      csr_read(8'h01, base_crc_counter);
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;

      for (int unsigned iter = 0; iter < 4; iter++) begin
        short_mode_v = iter[0];
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
          random_doc_case(), short_mode_v, 1, 1, 1, 1, 1, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
        send_doc_frame(16'h1130 + iter, 1, short_mode_v, channel, 1'b1);
        wait_for_counts(base_hits + iter + 1, base_real_eops + iter + 1, base_headers + iter + 1, 64, case_id);
        wait_cycles(8);
      end

      wait_cycles(24);
      csr_read(8'h01, csr_data);
      if (csr_data != (base_crc_counter + 32'd4))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected crc counter %0d got %0d",
            case_id, base_crc_counter + 32'd4, csr_data))
    endtask

    task automatic do_b114_mixed_good_bad_crc_count_matches_bad_frames(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      bit [31:0]   base_crc_counter;
      bit [31:0]   csr_data;

      wait_for_reset_release();
      run_start_for_mode();
      csr_read(8'h01, base_crc_counter);
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;

      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
      send_doc_frame(16'h1140, 1, 1'b0, channel, 1'b0);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 64, case_id);
      wait_cycles(8);

      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b0, 1, 1, 1, 1, 1, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
      send_doc_frame(16'h1141, 1, 1'b0, channel, 1'b1);
      wait_for_counts(base_hits + 2, base_real_eops + 2, base_headers + 2, 64, case_id);
      wait_cycles(8);

      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b1, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
      send_doc_frame(16'h1142, 1, 1'b1, channel, 1'b0);
      wait_for_counts(base_hits + 3, base_real_eops + 3, base_headers + 3, 64, case_id);
      wait_cycles(8);

      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b1, 1, 1, 1, 1, 1, 1'b0, 1'b0, 1'b0, 1'b0, "none", 32);
      send_doc_frame(16'h1143, 1, 1'b1, channel, 1'b1);
      wait_for_counts(base_hits + 4, base_real_eops + 4, base_headers + 4, 64, case_id);
      wait_cycles(24);
      csr_read(8'h01, csr_data);
      if (csr_data != (base_crc_counter + 32'd2))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected crc counter %0d got %0d",
            case_id, base_crc_counter + 32'd2, csr_data))
    endtask

    task automatic do_e019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      drive_rx_symbol(1'b0, 1'b1, HEADER_BYTE);
      drive_rx_symbol(1'b0, 1'b0, 8'h00);
      drive_rx_symbol(1'b0, 1'b0, 8'h19);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
    endtask

    task automatic do_e020_midframe_valid_low_does_not_pause_parser();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      bit [47:0]   partial_hit_word;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
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
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 16, case_id);
    endtask

    task automatic do_e013_ctrl_valid_low_keeps_previous_state(bit [7:0] channel);
      wait_for_reset_release();
      run_start();
      hold_forced_ctrl(CTRL_IDLE, 1'b0, 1, case_id);
      hold_forced_ctrl(CTRL_TERMINATING, 1'b0, 1, case_id);
      hold_forced_ctrl(CTRL_RUN_PREPARE, 1'b0, 1, case_id);
      release_ctrl_lines(case_id);
      wait_cycles(2);
      send_clean_recovery_frame(channel, 16'he013, 48'h111213141516, "none", case_id);
    endtask

    task automatic do_e029_header_following_write_zero_then_one_requires_clocked_enable(bit [7:0] channel);
      wait_for_reset_release();
      run_start();
      csr_write(8'h00, 32'h0000_0000);
      wait_cycles(1);
      attempt_frame_expect_blocked(channel, 16'he029, 48'h212223242526, {case_id, "_mask_low"});

      csr_write(8'h00, 32'h0000_0001);
      wait_cycles(2);
      send_clean_recovery_frame(channel, 16'he12a, 48'h313233343536, "none", case_id);
    endtask

    task automatic do_e121_repeated_terminating_words_do_not_duplicate_eop(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;

      wait_for_reset_release();
      run_start();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;

      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);
      fork
        begin
          hold_forced_ctrl(CTRL_TERMINATING, 1'b1, 3, case_id);
          release_ctrl_lines_nowait(case_id);
        end
        begin
          send_long_frame(16'he121, 48'h123456789abc);
        end
      join
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 64, case_id);
      wait_cycles(12);
      if (m_env.m_scb.hit_count != (base_hits + 1) ||
          m_env.m_scb.real_eop_count != (base_real_eops + 1) ||
          m_env.m_scb.header_count != (base_headers + 1))
        `uvm_fatal("FRCV_CASE",
          $sformatf("%s expected exactly one completed frame under repeated TERMINATING words", case_id))
    endtask

    task automatic do_e123_header_after_error_state_needs_new_running_or_idle_monitoring(bit [7:0] channel);
      wait_for_reset_release();
      pulse_ctrl(9'h12a, "ILLEGAL_RECOVER_EDGE");
      wait_cycles(2);
      attempt_frame_expect_blocked(channel, 16'he123, 48'h414243444546, {case_id, "_error"});
      pulse_ctrl(CTRL_IDLE, "IDLE");
      wait_cycles(2);
      pulse_ctrl(CTRL_RUNNING, "RUNNING");
      wait_cycles(2);
      send_clean_recovery_frame(channel, 16'he223, 48'h515253545556, "none", case_id);
    endtask

    task automatic do_x099_csr_mask_low_then_illegal_ctrl(bit [7:0] channel);
      wait_for_reset_release();
      run_start();
      csr_write(8'h00, 32'h0000_0000);
      pulse_ctrl(9'h12a, "ILLEGAL_MASK_LOW");
      wait_cycles(2);
      attempt_frame_expect_blocked(channel, 16'h0990, 48'h616263646566, {case_id, "_blocked"});
      csr_write(8'h00, 32'h0000_0001);
      pulse_ctrl(CTRL_RUNNING, "RUNNING");
      wait_cycles(2);
      send_clean_recovery_frame(channel, 16'h1990, 48'h717273747576, "none", case_id);
    endtask

    task automatic do_p034_cadence_idle_monitoring_only(bit [7:0] channel);
      int unsigned reps;

      wait_for_reset_release();
      issue_ctrl_for_mode(CTRL_IDLE, "IDLE");
      wait_cycles(2);
      reps = derive_iterations();
      for (int unsigned iter = 0; iter < reps; iter++) begin
        attempt_frame_expect_blocked(
          channel,
          16'h3400 + iter[15:0],
          48'h010203040506 + iter,
          $sformatf("%s_iter%0d", case_id, iter)
        );
      end
    endtask

    task automatic do_p045_mask_low_long_idle_soak(bit [7:0] channel);
      int unsigned reps;

      wait_for_reset_release();
      run_start();
      csr_write(8'h00, 32'h0000_0000);
      wait_cycles(2);
      reps = derive_iterations();
      for (int unsigned iter = 0; iter < reps; iter++) begin
        attempt_frame_expect_blocked(
          channel,
          16'h4500 + iter[15:0],
          48'h111213141516 + iter,
          $sformatf("%s_iter%0d", case_id, iter)
        );
      end
    endtask

    task automatic force_ctrl_lines(logic [8:0] cmd, bit valid, string ctx);
      uvm_hdl_data_t raw;

      raw = '0;
      raw[8:0] = cmd;
      if (!uvm_hdl_force("tb_top.ctrl_if.data", raw))
        `uvm_fatal("FRCV_CASE", $sformatf("%s failed to force ctrl_if.data", ctx))
      raw = '0;
      raw[0] = valid;
      if (!uvm_hdl_force("tb_top.ctrl_if.valid", raw))
        `uvm_fatal("FRCV_CASE", $sformatf("%s failed to force ctrl_if.valid", ctx))
    endtask

    task automatic release_ctrl_lines_nowait(string ctx);
      if (!uvm_hdl_release("tb_top.ctrl_if.data"))
        `uvm_fatal("FRCV_CASE", $sformatf("%s failed to release ctrl_if.data", ctx))
      if (!uvm_hdl_release("tb_top.ctrl_if.valid"))
        `uvm_fatal("FRCV_CASE", $sformatf("%s failed to release ctrl_if.valid", ctx))
    endtask

    task automatic release_ctrl_lines(string ctx);
      release_ctrl_lines_nowait(ctx);
      wait_cycles(1);
    endtask

    task automatic hold_forced_ctrl(logic [8:0] cmd, bit valid, int unsigned cycles, string ctx);
      force_ctrl_lines(cmd, valid, ctx);
      repeat (cycles)
        @(posedge ctrl_vif.clk);
    endtask

    task automatic ensure_ctrl_ready_high(int unsigned cycles, string ctx);
      repeat (cycles) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.ready !== 1'b1)
          `uvm_fatal("FRCV_CASE",
            $sformatf("%s expected ctrl_ready to stay high, saw %b", ctx, ctrl_vif.ready))
      end
    endtask

    task automatic ensure_ctrl_ready_constant(bit expected_ready, int unsigned cycles, string ctx);
      repeat (cycles) begin
        @(posedge ctrl_vif.clk);
        if (ctrl_vif.ready !== expected_ready)
          `uvm_fatal("FRCV_CASE",
            $sformatf("%s expected ctrl_ready to stay %b, saw %b", ctx, expected_ready, ctrl_vif.ready))
      end
    endtask

    task automatic attempt_frame_expect_blocked(bit [7:0] channel,
                                                int unsigned frame_number,
                                                bit [47:0] hit_word,
                                                string ctx);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      send_long_frame(frame_number, hit_word);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 20, ctx);
    endtask

    task automatic send_clean_recovery_frame(bit [7:0] channel,
                                             int unsigned frame_number,
                                             bit [47:0] hit_word,
                                             string stop_boundary,
                                             string ctx);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;

      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
        random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, stop_boundary, 6);
      send_long_frame(frame_number, hit_word);
      wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 32, ctx);
    endtask

    task automatic read_dbg_run_decode(
      output bit receiver_go_v,
      output bit receiver_force_go_v
    );
      uvm_hdl_data_t raw;

      if (!uvm_hdl_read("tb_top.dbg_receiver_go", raw))
        `uvm_fatal("FRCV_CASE", "Failed to read tb_top.dbg_receiver_go")
      receiver_go_v = raw[0];

      if (!uvm_hdl_read("tb_top.dbg_receiver_force_go", raw))
        `uvm_fatal("FRCV_CASE", "Failed to read tb_top.dbg_receiver_force_go")
      receiver_force_go_v = raw[0];
    endtask

    task automatic read_dbg_csr_word0(
      output bit [7:0] control_v,
      output bit [7:0] status_v
    );
      uvm_hdl_data_t raw;

      if (!uvm_hdl_read("tb_top.dbg_csr_control", raw))
        `uvm_fatal("FRCV_CASE", "Failed to read tb_top.dbg_csr_control")
      control_v = raw[7:0];

      if (!uvm_hdl_read("tb_top.dbg_csr_status", raw))
        `uvm_fatal("FRCV_CASE", "Failed to read tb_top.dbg_csr_status")
      status_v = raw[7:0];
    endtask

    task automatic expect_dbg_run_decode(bit exp_go, bit exp_force, string ctx);
      bit receiver_go_v;
      bit receiver_force_go_v;

      wait_cycles(2);
      read_dbg_run_decode(receiver_go_v, receiver_force_go_v);
      if (receiver_go_v !== exp_go || receiver_force_go_v !== exp_force)
        `uvm_fatal(
          "FRCV_CASE",
          $sformatf(
            "%s expected receiver_go/force_go=%0b/%0b got %0b/%0b",
            ctx, exp_go, exp_force, receiver_go_v, receiver_force_go_v
          )
        )
    endtask

    task automatic run_basic_explicit_doc_case(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      bit [31:0]   csr_data;
      frcv_csr_obs_item csr_obs;

      wait_for_reset_release();

      unique case (case_id)
        "B001_reset_idle_outputs_quiet": begin
          do_b001_reset_idle_outputs_quiet();
        end
        "B002_idle_force_go_monitor_path_open": begin
          send_ctrl(CTRL_IDLE, "IDLE");
          attempt_frame_expect_blocked(channel, 16'h0002, 48'h020304050607, case_id);
        end
        "B003_idle_force_go_ignores_csr_mask": begin
          send_ctrl(CTRL_IDLE, "IDLE");
          csr_write(8'h00, 32'h0000_0000);
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, 16'h0003, 48'h030405060708, case_id);
        end
        "B004_running_needs_csr_enable_high": begin
          do_b004_running_needs_csr_enable_high();
        end
        "B005_running_mask_low_blocks_header_start": begin
          do_b005_running_mask_low_blocks_header_start();
        end
        "B006_run_prepare_clears_parser_state": begin
          do_b006_run_prepare_clears_parser_state();
        end
        "B007_sync_keeps_parser_closed": begin
          send_ctrl(CTRL_SYNC, "SYNC");
          attempt_frame_expect_blocked(channel, 16'h0007, 48'h0708090a0b0c, case_id);
        end
        "B008_running_opens_header_detection": begin
          run_start_for_mode();
          send_clean_recovery_frame(channel, 16'h0008, 48'h08090a0b0c0d, "none", case_id);
        end
        "B009_terminating_blocks_new_header_from_idle": begin
          do_b009_terminating_blocks_new_header_from_idle();
        end
        "B010_terminating_allows_open_frame_to_finish": begin
          do_b010_terminating_allows_open_frame_to_finish();
        end
        "B011_ctrl_unknown_word_enters_error": begin
          do_b011_ctrl_unknown_word_enters_error();
        end
        "B012_ctrl_ready_high_with_valid": begin
          pulse_ctrl(CTRL_RUNNING, "RUNNING");
          ensure_ctrl_ready_high(4, case_id);
        end
        "B013_ctrl_ready_high_without_valid": begin
          ensure_ctrl_ready_high(4, case_id);
        end
        "B014_ctrl_decode_idle_force_go": begin
          pulse_ctrl(CTRL_IDLE, "IDLE");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B015_ctrl_decode_run_prepare_disable": begin
          pulse_ctrl(CTRL_RUN_PREPARE, "RUN_PREPARE");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B016_ctrl_decode_sync_disable": begin
          pulse_ctrl(CTRL_SYNC, "SYNC");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B017_ctrl_decode_running_go": begin
          pulse_ctrl(CTRL_RUNNING, "RUNNING");
          expect_dbg_run_decode(1'b1, 1'b0, case_id);
        end
        "B018_ctrl_decode_terminating_stop_new_headers": begin
          pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B019_ctrl_decode_link_test_disable": begin
          pulse_ctrl(CTRL_LINK_TEST, "LINK_TEST");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B020_ctrl_decode_sync_test_disable": begin
          pulse_ctrl(CTRL_SYNC_TEST, "SYNC_TEST");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B021_ctrl_decode_reset_disable": begin
          pulse_ctrl(CTRL_RESET, "RESET");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B022_ctrl_decode_out_of_daq_disable": begin
          pulse_ctrl(CTRL_OUT_OF_DAQ, "OUT_OF_DAQ");
          expect_dbg_run_decode(1'b0, 1'b0, case_id);
        end
        "B023_csr_control_default_enable": begin
          csr_read(8'h00, csr_data);
          if (csr_data[7:0] != 8'h01)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected csr.control=0x01 got 0x%02h", case_id, csr_data[7:0]))
        end
        "B024_csr_control_write_zero_masks_running": begin
          run_start_for_mode();
          csr_write(8'h00, 32'h0000_0000);
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, 16'h0024, 48'h240102030405, case_id);
        end
        "B025_csr_control_write_one_unmasks_running": begin
          run_start_for_mode();
          csr_write(8'h00, 32'h0000_0000);
          csr_write(8'h00, 32'h0000_0001);
          wait_cycles(2);
          send_clean_recovery_frame(channel, 16'h0025, 48'h250102030405, "none", case_id);
        end
        "B026_csr_read_addr0_control_status": begin
          csr_read(8'h00, csr_data);
          if (csr_data[7:0] != 8'h01 || csr_data[31:24] != 8'h00)
            `uvm_fatal(
              "FRCV_CASE",
              $sformatf("%s expected csr word0 control/status = 0x01/0x00 got 0x%02h/0x%02h",
                case_id, csr_data[7:0], csr_data[31:24])
            )
        end
        "B027_csr_read_addr1_crc_counter": begin
          do_single_bad_crc_case(channel, 16'h0027, 1'b0, 1'b0);
        end
        "B029_csr_read_unused_word_zero": begin
          csr_read(8'hff, csr_data);
          if (csr_data != 32'd0)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected unused csr word to read 0 got 0x%08h", case_id, csr_data))
        end
        "B031_csr_waitrequest_low_on_read": begin
          csr_read(8'h00, csr_data);
          csr_obs = find_last_csr_obs();
          if (csr_obs == null || csr_obs.waitrequest !== 1'b0)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected serviced read with waitrequest low", case_id))
        end
        "B032_csr_waitrequest_low_on_write": begin
          csr_write(8'h00, 32'h0000_0001);
          csr_obs = find_last_csr_obs();
          if (csr_obs == null || !csr_obs.is_write || csr_obs.waitrequest !== 1'b0)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected serviced write with waitrequest low", case_id))
        end
        "B033_csr_waitrequest_high_when_idle": begin
          wait_cycles(2);
          if (csr_vif.waitrequest !== 1'b1)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected idle waitrequest high, got %b", case_id, csr_vif.waitrequest))
        end
        "B034_csr_reserved_control_bits_roundtrip": begin
          bit [7:0] dbg_control_v;
          bit [7:0] dbg_status_v;

          csr_write(8'h00, 32'h0000_00A5);
          wait_cycles(2);
          read_dbg_csr_word0(dbg_control_v, dbg_status_v);
          if (dbg_control_v != 8'hA5)
            `uvm_fatal(
              "FRCV_CASE",
              $sformatf(
                "%s expected internal control image 0xA5 got 0x%02h (status=0x%02h)",
                case_id, dbg_control_v, dbg_status_v
              )
            )
          csr_read(8'h00, csr_data);
          if (csr_data[7:0] != 8'hA5)
            `uvm_fatal(
              "FRCV_CASE",
              $sformatf(
                "%s expected control roundtrip 0xA5 got 0x%02h (dbg_control=0x%02h dbg_status=0x%02h)",
                case_id, csr_data[7:0], dbg_control_v, dbg_status_v
              )
            )
        end
        "B036_data_byte_without_kchar_no_start": begin
          do_b036_data_byte_without_kchar_no_start();
        end
        "B039_long_zero_hit_frame_headerinfo_pulse": begin
          do_b039_long_zero_hit_frame_headerinfo_pulse();
        end
        "B041_long_one_hit_sop_and_eop_same_transfer": begin
          do_b041_long_one_hit_sop_and_eop_same_transfer();
        end
        "B059_long_crc_bad_raises_error1_on_eop": begin
          do_single_bad_crc_case(channel, 16'h0059, 1'b0, 1'b1);
        end
        "B060_long_crc_bad_increments_crc_counter": begin
          do_single_bad_crc_case(channel, 16'h0060, 1'b0, 1'b0);
        end
        "B079_short_crc_bad_raises_error1_on_last_hit": begin
          do_single_bad_crc_case(channel, 16'h0079, 1'b1, 1'b1);
        end
        "B080_short_crc_counter_accumulates": begin
          do_b080_short_crc_counter_accumulates(channel);
        end
        "B112_long_good_frames_leave_crc_counter_stable": begin
          do_b112_long_good_frames_leave_crc_counter_stable(channel);
        end
        "B113_bad_frames_accumulate_crc_counter": begin
          do_b113_bad_frames_accumulate_crc_counter(channel);
        end
        "B114_mixed_good_bad_crc_count_matches_bad_frames": begin
          do_b114_mixed_good_bad_crc_count_matches_bad_frames(channel);
        end
        "B098_idle_monitoring_accepts_header_without_running": begin
          send_ctrl(CTRL_IDLE, "IDLE");
          attempt_frame_expect_blocked(channel, 16'h0098, 48'h980102030405, case_id);
        end
        "B099_idle_monitoring_still_ignores_csr_zero": begin
          send_ctrl(CTRL_IDLE, "IDLE");
          csr_write(8'h00, 32'h0000_0000);
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, 16'h0099, 48'h990102030405, case_id);
        end
        default: begin
          `uvm_fatal("FRCV_CASE_UNIMPLEMENTED",
            $sformatf("Missing explicit BASIC case handler for %s", case_id))
        end
      endcase
    endtask

    task automatic run_random_illegal_ctrl_set(bit [7:0] channel,
                                               int unsigned frame_number,
                                               int unsigned seed_idx);
      logic [8:0] illegal_words[8];

      if (seed_idx == 1) begin
        illegal_words = '{
          9'h003, 9'h00c, 9'h022, 9'h054,
          9'h0c1, 9'h12a, 9'h155, 9'h1a4
        };
      end else begin
        illegal_words = '{
          9'h006, 9'h018, 9'h044, 9'h092,
          9'h109, 9'h18c, 9'h172, 9'h0d3
        };
      end

      foreach (illegal_words[idx]) begin
        pulse_ctrl(illegal_words[idx], $sformatf("ILLEGAL_%03h", illegal_words[idx]));
        wait_cycles(1);
      end
      attempt_frame_expect_blocked(channel, frame_number, 48'h112233445566, case_id);
    endtask

    task automatic run_error_control_negative_case(bit [7:0] channel);
      int unsigned frame_number;

      wait_for_reset_release();
      frame_number = 16'h9000 | bucket_case_index();

      case (case_id)
        "X014_ctrl_allzero_word": begin
          do_x014_ctrl_allzero_word();
        end
        "X015_ctrl_twohot_idle_running": begin
          pulse_ctrl(CTRL_IDLE | CTRL_RUNNING, "ILLEGAL_TWOHOT_IDLE_RUNNING");
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, frame_number, 48'h010203040506, case_id);
        end
        "X016_ctrl_twohot_running_terminating": begin
          pulse_ctrl(CTRL_RUNNING | CTRL_TERMINATING, "ILLEGAL_TWOHOT_RUNNING_TERMINATING");
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, frame_number, 48'h0a0b0c0d0e0f, case_id);
        end
        "X017_ctrl_allones_word": begin
          pulse_ctrl(9'h1ff, "ILLEGAL_ALLONES");
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, frame_number, 48'h111213141516, case_id);
        end
        "X018_ctrl_random_illegal_word_seed1": begin
          run_random_illegal_ctrl_set(channel, frame_number, 1);
        end
        "X019_ctrl_random_illegal_word_seed2": begin
          run_random_illegal_ctrl_set(channel, frame_number, 2);
        end
        "X020_ctrl_valid_glitch_no_word_change": begin
          hold_forced_ctrl(9'h12a, 1'b0, 2, case_id);
          hold_forced_ctrl(9'h12a, 1'b1, 1, case_id);
          hold_forced_ctrl(9'h12a, 1'b0, 2, case_id);
          release_ctrl_lines(case_id);
          attempt_frame_expect_blocked(channel, frame_number, 48'h202122232425, case_id);
        end
        "X021_ctrl_payload_changes_while_valid_high": begin
          hold_forced_ctrl(9'h12a, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_RUNNING, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_IDLE, 1'b0, 1, case_id);
          release_ctrl_lines(case_id);
          wait_cycles(2);
          send_clean_recovery_frame(channel, frame_number, 48'h313233343536, "none", case_id);
        end
        "X022_ctrl_illegal_then_legal_recovery_idle": begin
          pulse_ctrl(9'h12a, "ILLEGAL_RECOVER_IDLE");
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, frame_number, 48'h414243444546, {case_id, "_pre_recovery"});
          pulse_ctrl(CTRL_IDLE, "IDLE");
          wait_cycles(2);
          pulse_ctrl(CTRL_RUNNING, "RUNNING");
          wait_cycles(2);
          send_clean_recovery_frame(channel, frame_number + 16'h0100, 48'h4748494a4b4c, "none", case_id);
        end
        "X023_ctrl_illegal_then_legal_recovery_running": begin
          pulse_ctrl(9'h12a, "ILLEGAL_RECOVER_RUNNING");
          wait_cycles(2);
          attempt_frame_expect_blocked(channel, frame_number, 48'h515253545556, {case_id, "_pre_recovery"});
          pulse_ctrl(CTRL_RUNNING, "RUNNING");
          wait_cycles(2);
          send_clean_recovery_frame(channel, frame_number + 16'h0100, 48'h5758595a5b5c, "none", case_id);
        end
        "X024_ctrl_ready_not_indicating_recovery": begin
          bit ready_level;

          pulse_ctrl(9'h12a, "ILLEGAL_READY");
          wait_cycles(2);
          ready_level = ctrl_vif.ready;
          ensure_ctrl_ready_constant(ready_level, 8, case_id);
          attempt_frame_expect_blocked(channel, frame_number, 48'h616263646566, case_id);
          pulse_ctrl(CTRL_RUNNING, "RUNNING");
          wait_for_ctrl_ready_high(8, {case_id, "_post_running"});
        end
        "X025_ctrl_valid_low_payload_noise": begin
          run_start_for_mode();
          hold_forced_ctrl(9'h12a, 1'b0, 1, case_id);
          hold_forced_ctrl(9'h0d3, 1'b0, 1, case_id);
          hold_forced_ctrl(CTRL_TERMINATING, 1'b0, 1, case_id);
          hold_forced_ctrl(CTRL_IDLE, 1'b0, 1, case_id);
          release_ctrl_lines(case_id);
          send_clean_recovery_frame(channel, frame_number, 48'h717273747576, "none", case_id);
        end
        "X026_ctrl_fast_oscillation_legal_illegal_mix": begin
          hold_forced_ctrl(CTRL_RUNNING, 1'b1, 1, case_id);
          hold_forced_ctrl(9'h12a, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_SYNC, 1'b1, 1, case_id);
          hold_forced_ctrl(9'h0d3, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_RUN_PREPARE, 1'b1, 1, case_id);
          hold_forced_ctrl(9'h155, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_RUNNING, 1'b1, 1, case_id);
          hold_forced_ctrl(CTRL_IDLE, 1'b0, 1, case_id);
          release_ctrl_lines(case_id);
          wait_cycles(2);
          send_clean_recovery_frame(channel, frame_number, 48'h818283848586, "none", case_id);
        end
        default: begin
          `uvm_fatal("FRCV_CASE_UNIMPLEMENTED",
            $sformatf("Missing explicit ERROR control handler for %s", case_id))
        end
      endcase
    endtask

    task automatic do_x014_ctrl_allzero_word();
      do_b011_ctrl_unknown_word_enters_error();
    endtask

    task automatic do_x027_long_header_only_no_counters();
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      run_start();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      drive_symbol(1'b1, HEADER_BYTE);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
    endtask

    task automatic send_doc_header_prefix(
      int unsigned frame_number,
      int unsigned frame_len,
      bit          short_mode,
      bit [7:0]    channel,
      int unsigned metadata_bytes
    );
      drive_symbol(1'b1, HEADER_BYTE, '0, channel);
      if (metadata_bytes >= 1)
        drive_symbol(1'b0, byte'(frame_number[15:8]), '0, channel);
      if (metadata_bytes >= 2)
        drive_symbol(1'b0, byte'(frame_number[7:0]), '0, channel);
      if (metadata_bytes >= 3)
        drive_symbol(1'b0, frame_len_msb_byte(short_mode, frame_len), '0, channel);
      if (metadata_bytes >= 4)
        drive_symbol(1'b0, byte'(frame_len[7:0]), '0, channel);
    endtask

    task automatic send_doc_payload_prefix(
      int unsigned frame_number,
      int unsigned frame_len,
      bit          short_mode,
      bit [7:0]    channel,
      int unsigned payload_bytes_to_send,
      bit          force_valid_low = 1'b0,
      integer      comma_at_idx    = -1
    );
      byte unsigned payload_byte;

      for (int unsigned idx = 0; idx < payload_bytes_to_send; idx++) begin
        if (comma_at_idx >= 0 && integer'(idx) == comma_at_idx) begin
          drive_symbol(1'b1, 8'hbc, '0, channel);
        end else begin
          payload_byte = payload_byte_value(frame_number, idx, short_mode);
          if (force_valid_low)
            drive_rx_symbol(1'b0, 1'b0, payload_byte, '0, channel);
          else
            drive_symbol(1'b0, payload_byte, '0, channel);
        end
      end
    endtask

    task automatic send_doc_crc_bytes(
      bit [7:0]    channel,
      bit          crc_hi_is_k,
      byte unsigned crc_hi,
      bit          crc_lo_is_k,
      byte unsigned crc_lo
    );
      drive_symbol(crc_hi_is_k, crc_hi, '0, channel);
      drive_symbol(crc_lo_is_k, crc_lo, '0, channel);
    endtask

    task automatic run_error_malformed_case(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      int unsigned frame_number;
      int unsigned frame_len;

      wait_for_reset_release();
      run_start_for_mode();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      frame_number   = 16'h8e00 | bucket_case_index();

      unique case (case_id)
        "X027_long_header_only_no_counters": begin
          do_x027_long_header_only_no_counters();
        end
        "X028_long_missing_frame_number_byte1": begin
          send_doc_header_prefix(frame_number, 1, 1'b0, channel, 1);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X029_long_missing_event_counter_msb": begin
          send_doc_header_prefix(frame_number, 1, 1'b0, channel, 2);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X030_long_missing_event_counter_lsb": begin
          send_doc_header_prefix(frame_number, 1, 1'b0, channel, 3);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X031_long_declared_len1_missing_payload": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b0, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 12, case_id);
        end
        "X032_long_declared_len2_only_one_hit_present": begin
          frame_len = 2;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b0, frame_len, 1, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, 6);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 1, 0, 1, 0, 20, case_id);
        end
        "X033_long_declared_len_large_cut_midpayload": begin
          frame_len = 8;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b0, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, 3);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X034_long_bad_trailer_symbol_replaced_with_data": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
            random_doc_case(), 1'b0, frame_len, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "bad_trailer", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, payload_byte_count(1'b0, frame_len));
          send_doc_crc_bytes(channel, 1'b1, 8'h9c, 1'b0, 8'h00);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 1, 1, 1, 0, 20, case_id);
        end
        "X035_long_extra_payload_after_declared_len": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
            random_doc_case(), 1'b0, frame_len, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "extra_payload", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, payload_byte_count(1'b0, frame_len));
          send_doc_crc_bytes(
            channel,
            1'b0,
            payload_byte_value(frame_number, payload_byte_count(1'b0, frame_len), 1'b0),
            1'b0,
            payload_byte_value(frame_number, payload_byte_count(1'b0, frame_len) + 1, 1'b0)
          );
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 1, 1, 1, 0, 20, case_id);
        end
        "X036_long_new_header_inside_payload": begin
          frame_len = 2;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
            random_doc_case(), 1'b0, frame_len, 2, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "nested_header", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          for (int unsigned idx = 0; idx < payload_byte_count(1'b0, frame_len); idx++) begin
            if (idx == 5)
              drive_symbol(1'b1, HEADER_BYTE, '0, channel);
            else
              drive_symbol(1'b0, payload_byte_value(frame_number, idx, 1'b0), '0, channel);
          end
          send_doc_crc_bytes(channel, 1'b0, 8'h00, 1'b0, 8'h00);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 2, 1, 1, 0, 24, case_id);
        end
        "X037_long_comma_inside_payload_mode0": begin
          frame_len = 2;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b0, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "mode0_abort", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, 2, 1'b0, 1);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X039_long_valid_low_entire_malformed_frame": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b0, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b0, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b0, channel, payload_byte_count(1'b0, frame_len), 1'b1);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X040_short_header_only_no_counters": begin
          drive_symbol(1'b1, HEADER_BYTE, '0, channel);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X041_short_missing_frame_number_byte1": begin
          send_doc_header_prefix(frame_number, 1, 1'b1, channel, 1);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X042_short_missing_event_counter_msb": begin
          send_doc_header_prefix(frame_number, 1, 1'b1, channel, 2);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X043_short_missing_event_counter_lsb": begin
          send_doc_header_prefix(frame_number, 1, 1'b1, channel, 3);
          expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        end
        "X044_short_declared_len1_missing_payload": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b1, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 12, case_id);
        end
        "X047_short_declared_len_large_cut_midpayload": begin
          frame_len = 8;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b1, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b1, channel, 3);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X048_short_new_header_inside_payload": begin
          frame_len = 3;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
            random_doc_case(), 1'b1, frame_len, 3, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "nested_header", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          for (int unsigned idx = 0; idx < payload_byte_count(1'b1, frame_len); idx++) begin
            if (idx == 5)
              drive_symbol(1'b1, HEADER_BYTE, '0, channel);
            else
              drive_symbol(1'b0, payload_byte_value(frame_number, idx, 1'b1), '0, channel);
          end
          send_doc_crc_bytes(channel, 1'b0, 8'h00, 1'b0, 8'h00);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 3, 1, 1, 0, 28, case_id);
        end
        "X049_short_extra_payload_after_declared_len": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
            random_doc_case(), 1'b1, frame_len, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "extra_payload", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b1, channel, payload_byte_count(1'b1, frame_len));
          send_doc_crc_bytes(
            channel,
            1'b0,
            payload_byte_value(frame_number, payload_byte_count(1'b1, frame_len), 1'b1),
            1'b0,
            payload_byte_value(frame_number, payload_byte_count(1'b1, frame_len) + 1, 1'b1)
          );
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 1, 1, 1, 0, 20, case_id);
        end
        "X050_short_comma_inside_payload_mode0": begin
          frame_len = 2;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b1, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "mode0_abort", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b1, channel, 2, 1'b0, 1);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X051_short_comma_inside_payload_mode1": begin
          frame_len = 2;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b1, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "mode0_abort", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b1, channel, 2, 1'b0, 1);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        "X052_short_valid_low_entire_malformed_frame": begin
          frame_len = 1;
          scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
            random_doc_case(), 1'b1, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
          send_doc_header_prefix(frame_number, frame_len, 1'b1, channel, 4);
          send_doc_payload_prefix(frame_number, frame_len, 1'b1, channel, payload_byte_count(1'b1, frame_len), 1'b1);
          expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 16, case_id);
        end
        default: begin
          `uvm_fatal("FRCV_CASE_UNIMPLEMENTED",
            $sformatf("Missing explicit ERROR malformed handler for %s", case_id))
        end
      endcase
    endtask

    function automatic string lower_string(string text);
      string out_s;
      byte unsigned ch;

      out_s = text;
      for (int idx = 0; idx < out_s.len(); idx++) begin
        ch = out_s.getc(idx);
        if (ch >= 8'd65 && ch <= 8'd90)
          out_s.putc(idx, ch + 8'd32);
      end
      return out_s;
    endfunction

    function automatic int find_fragment(string haystack, string needle);
      bit matched;

      if (needle.len() == 0)
        return 0;
      if (haystack.len() < needle.len())
        return -1;

      for (int idx = 0; idx <= (haystack.len() - needle.len()); idx++) begin
        matched = 1'b1;
        for (int off = 0; off < needle.len(); off++) begin
          if (haystack.getc(idx + off) != needle.getc(off)) begin
            matched = 1'b0;
            break;
          end
        end
        if (matched)
          return idx;
      end
      return -1;
    endfunction

    function automatic bit contains_ci(string fragment);
      return find_fragment(lower_string(case_id), lower_string(fragment)) >= 0;
    endfunction

    function automatic int unsigned parse_scaled_slice(
      string text,
      int    start_idx,
      int    end_idx,
      int unsigned default_value
    );
      int unsigned value;
      bit          saw_digit;
      byte unsigned ch;

      value     = 0;
      saw_digit = 1'b0;
      for (int idx = start_idx; idx <= end_idx; idx++) begin
        ch = text.getc(idx);
        if (ch >= 8'd48 && ch <= 8'd57) begin
          value = (value * 10) + (ch - 8'd48);
          saw_digit = 1'b1;
        end else if ((ch == 8'd107 || ch == 8'd75) && saw_digit) begin
          value = value * 1000;
        end
      end

      if (!saw_digit)
        return default_value;
      return value;
    endfunction

    function automatic int unsigned extract_count_before_suffix(string suffix, int unsigned default_value);
      string lower_id;
      int    suffix_pos;
      int    start_idx;
      int    end_idx;

      lower_id   = lower_string(case_id);
      suffix_pos = find_fragment(lower_id, suffix);
      if (suffix_pos < 0)
        return default_value;

      end_idx = suffix_pos - 1;
      while (end_idx >= 0 && lower_id.getc(end_idx) == 8'd95)
        end_idx--;
      if (end_idx < 0)
        return default_value;

      start_idx = end_idx;
      while (start_idx >= 0 && lower_id.getc(start_idx) != 8'd95)
        start_idx--;
      start_idx++;

      return parse_scaled_slice(lower_id, start_idx, end_idx, default_value);
    endfunction

    function automatic int unsigned extract_number_after_marker(string marker, int unsigned default_value);
      string lower_id;
      int    marker_pos;
      int    start_idx;
      int    end_idx;

      lower_id   = lower_string(case_id);
      marker_pos = find_fragment(lower_id, lower_string(marker));
      if (marker_pos < 0)
        return default_value;

      start_idx = marker_pos + marker.len();
      end_idx   = start_idx;
      while (end_idx < lower_id.len()) begin
        if (lower_id.getc(end_idx) < 8'd48 || lower_id.getc(end_idx) > 8'd57)
          break;
        end_idx++;
      end
      if (end_idx == start_idx)
        return default_value;
      return parse_scaled_slice(lower_id, start_idx, end_idx - 1, default_value);
    endfunction

    function automatic int unsigned bucket_case_index();
      return extract_case_index(case_id);
    endfunction

    function automatic int unsigned global_case_index();
      byte unsigned bucket_code;
      int unsigned  bucket_offset;

      if (case_id.len() == 0)
        return 0;

      bucket_code = case_id.getc(0);
      case (bucket_code)
        "B": bucket_offset = 0;
        "E": bucket_offset = 130;
        "P": bucket_offset = 260;
        "X": bucket_offset = 390;
        default: bucket_offset = 0;
      endcase
      return bucket_offset + bucket_case_index();
    endfunction

    function automatic string bucket_name_for_case(string id);
      if (id.len() == 0)
        return "UNKNOWN";
      case (id.getc(0))
        "B": return "BASIC";
        "E": return "EDGE";
        "P": return "PROF";
        "X": return "ERROR";
        default: return "UNKNOWN";
      endcase
    endfunction

    function automatic bit random_doc_case();
      return contains_ci("random") || contains_ci("seed");
    endfunction

    task automatic issue_ctrl_for_mode(logic [8:0] cmd, string state_name);
      if (execution_mode == "isolated") begin
        send_ctrl(cmd, state_name);
      end else begin
        pulse_ctrl(cmd, state_name);
        wait_cycles(2);
      end
    endtask

    task automatic pulse_ctrl_async(logic [8:0] cmd, string state_name);
      fork
        begin
          automatic logic [8:0] async_cmd = cmd;
          automatic string      async_state = state_name;
          pulse_ctrl(async_cmd, async_state);
        end
      join_none
    endtask

    task automatic run_start_for_mode();
      csr_write(8'h00, 32'h0000_0001);
      issue_ctrl_for_mode(CTRL_RUN_PREPARE, "RUN_PREPARE");
      issue_ctrl_for_mode(CTRL_SYNC, "SYNC");
      issue_ctrl_for_mode(CTRL_RUNNING, "RUNNING");
      wait_cycles(2);
    endtask

    function automatic bit recognized_doc_case();
      return (uvm_re_match("^[BEPX][0-9][0-9][0-9]_[A-Za-z0-9_]+$", case_id) == 0);
    endfunction

    function automatic int unsigned doc_iteration_cap();
      if (effort_mode == "extensive") begin
        if (execution_mode == "isolated")
          return MAX_DOC_ITERATIONS_ISOLATED_EXTENSIVE;
        return MAX_DOC_ITERATIONS_CONTINUOUS_EXTENSIVE;
      end
      if (execution_mode == "isolated")
        return MAX_DOC_ITERATIONS_ISOLATED;
      return MAX_DOC_ITERATIONS_CONTINUOUS;
    endfunction

    function automatic int unsigned doc_payload_cap();
      if (effort_mode == "extensive") begin
        if (execution_mode == "isolated")
          return MAX_DOC_PAYLOAD_BYTES_ISOLATED_EXTENSIVE;
        return MAX_DOC_PAYLOAD_BYTES_CONTINUOUS_EXTENSIVE;
      end
      if (execution_mode == "isolated")
        return MAX_DOC_PAYLOAD_BYTES_ISOLATED;
      return MAX_DOC_PAYLOAD_BYTES_CONTINUOUS;
    endfunction

    function automatic int unsigned derive_iterations();
      int unsigned count;
      int unsigned cap;

      cap = doc_iteration_cap();

      if (effort_mode != "extensive" &&
          execution_mode == "isolated" &&
          case_id.len() != 0 &&
          case_id.getc(0) == "P")
        cap = (cap > 8) ? 8 : cap;

      count = extract_count_before_suffix("frames", 0);
      if (count != 0)
        return (count > cap) ? cap : count;
      count = extract_count_before_suffix("cycles", 0);
      if (count != 0)
        return (count > cap) ? cap : count;
      if (contains_ci("soak"))
        return cap;
      if (contains_ci("seed"))
        return cap;
      if (contains_ci("random"))
        return cap / 2;
      if (contains_ci("repeat"))
        return cap / 4;
      return 1;
    endfunction

    function automatic int unsigned derive_frame_len();
      int unsigned value;

      value = extract_number_after_marker("len", 0);
      if (contains_ci("max"))
        return 1023;
      if (contains_ci("zero_hit") || contains_ci("len0"))
        return 0;
      if (contains_ci("one_hit") || contains_ci("single_word") || contains_ci("min_nonzero"))
        return 1;
      if (contains_ci("two_hit") || contains_ci("even_pair"))
        return 2;
      if (contains_ci("three_hit") || contains_ci("odd_pair"))
        return 3;
      if (contains_ci("four_hit"))
        return 4;
      if (contains_ci("seven_hit"))
        return 7;
      if (contains_ci("fifteen_hit"))
        return 15;
      if (contains_ci("thirty_one_hit"))
        return 31;
      if (contains_ci("sixty_three_hit"))
        return 63;
      if (contains_ci("one_hit") || contains_ci("single"))
        return 1;
      if (value != 0)
        return value;
      if (contains_ci("header_only"))
        return 0;
      return 1;
    endfunction

    function automatic bit short_mode_case();
      return contains_ci("short") ||
             contains_ci("flags_100") ||
             contains_ci("fast_mode");
    endfunction

    function automatic int unsigned payload_byte_count(bit short_mode, int unsigned frame_len);
      if (frame_len == 0)
        return 0;
      if (short_mode)
        return (frame_len * 3) + ((frame_len + 1) / 2);
      return frame_len * 6;
    endfunction

    function automatic bit [7:0] frame_len_msb_byte(bit short_mode, int unsigned frame_len);
      bit [7:0] result;

      result = 8'h00;
      if (short_mode)
        result[6:4] = 3'b100;
      result[1:0] = frame_len[9:8];
      return result;
    endfunction

    function automatic byte unsigned payload_byte_value(int unsigned frame_number, int unsigned payload_idx, bit short_mode);
      return byte'(((frame_number * 13) ^ (payload_idx * 29) ^ (short_mode ? 8'h5a : 8'h96)) & 8'hff);
    endfunction

    function automatic bit [7:0] derived_channel_value();
      int unsigned idx;

      idx = bucket_case_index();
      if (contains_ci("channel1") || contains_ci("width1"))
        return byte'(idx[0]);
      if (contains_ci("channel8") || contains_ci("width8") || contains_ci("wide_channel"))
        return byte'((idx * 7) & 8'hff);
      return byte'((idx % 16) & 8'h0f);
    endfunction

    task automatic apply_case_control_state();
      if (contains_ci("link_test")) begin
        issue_ctrl_for_mode(CTRL_LINK_TEST, "LINK_TEST");
      end else if (contains_ci("sync_test")) begin
        issue_ctrl_for_mode(CTRL_SYNC_TEST, "SYNC_TEST");
      end else if (contains_ci("out_of_daq")) begin
        issue_ctrl_for_mode(CTRL_OUT_OF_DAQ, "OUT_OF_DAQ");
      end else if (contains_ci("reset_state")) begin
        issue_ctrl_for_mode(CTRL_RESET, "RESET");
      end else if ((contains_ci("idle_force_go") || contains_ci("idle_monitoring") || contains_ci(" stay in `idle`")) &&
                   !contains_ci("running_to_idle")) begin
        issue_ctrl_for_mode(CTRL_IDLE, "IDLE");
      end else if (contains_ci("sync_keeps") || contains_ci("decode_sync_disable")) begin
        issue_ctrl_for_mode(CTRL_SYNC, "SYNC");
      end else begin
        run_start_for_mode();
      end
    endtask

    task automatic apply_case_csr_ops();
      bit [31:0] ignored_read;

      if (contains_ci("write_zero") || contains_ci("mask_low"))
        csr_write(8'h00, 32'h0000_0000);
      if (contains_ci("write_one") || contains_ci("mask_high"))
        csr_write(8'h00, 32'h0000_0001);
      if (contains_ci("read_addr0") || contains_ci("word0") || contains_ci("status"))
        csr_read(8'h00, ignored_read);
      if (contains_ci("addr1") || contains_ci("crc_counter") || contains_ci("word1"))
        csr_read(8'h01, ignored_read);
      if (contains_ci("addr2") || contains_ci("frame_counter") || contains_ci("word2"))
        csr_read(8'h02, ignored_read);
      if (contains_ci("unused") || contains_ci("highaddr") || contains_ci("unused_space"))
        csr_read(8'hff, ignored_read);
    endtask

    task automatic send_doc_frame(
      int unsigned frame_number,
      int unsigned frame_len,
      bit          short_mode,
      bit [7:0]    channel,
      bit          inject_bad_crc        = 1'b0,
      bit          inject_valid_gap      = 1'b0,
      bit          inject_loss_sync      = 1'b0,
      bit          inject_parity_error   = 1'b0,
      bit          inject_decode_error   = 1'b0,
      bit          inject_comma          = 1'b0,
      int unsigned truncate_after_bytes  = 32'hffff_ffff,
      int unsigned terminate_after_bytes = 32'hffff_ffff
    );
      int unsigned payload_bytes;
      bit [2:0]    err_bits;
      byte unsigned payload_byte;
      byte unsigned frame_bytes[$];
      bit [15:0]    crc_trailer;

      payload_bytes = payload_byte_count(short_mode, frame_len);
      frame_bytes.push_back(byte'(frame_number[15:8]));
      frame_bytes.push_back(byte'(frame_number[7:0]));
      frame_bytes.push_back(frame_len_msb_byte(short_mode, frame_len));
      frame_bytes.push_back(byte'(frame_len[7:0]));
      for (int unsigned idx = 0; idx < payload_bytes; idx++)
        frame_bytes.push_back(payload_byte_value(frame_number, idx, short_mode));
      crc_trailer = frcv_crc16_trailer(frame_bytes);
      if (inject_bad_crc)
        crc_trailer ^= 16'h0001;

      drive_symbol(1'b1, HEADER_BYTE, '0, channel);
      drive_symbol(1'b0, byte'(frame_number[15:8]), '0, channel);
      drive_symbol(1'b0, byte'(frame_number[7:0]), '0, channel);
      drive_symbol(1'b0, frame_len_msb_byte(short_mode, frame_len), '0, channel);
      drive_symbol(1'b0, byte'(frame_len[7:0]), '0, channel);

      for (int unsigned idx = 0; idx < payload_bytes; idx++) begin
        if (terminate_after_bytes != 32'hffff_ffff && idx == terminate_after_bytes)
          pulse_ctrl_async(CTRL_TERMINATING, "TERMINATING");

        if (truncate_after_bytes != 32'hffff_ffff && idx >= truncate_after_bytes) begin
          wait_cycles(2);
          return;
        end

        payload_byte = payload_byte_value(frame_number, idx, short_mode);
        if (inject_valid_gap && idx == (payload_bytes / 2)) begin
          drive_rx_symbol(1'b0, 1'b0, payload_byte, '0, channel);
          continue;
        end
        if (inject_comma && idx == (payload_bytes / 2)) begin
          drive_symbol(1'b1, 8'hbc, '0, channel);
          continue;
        end

        err_bits = '0;
        if (inject_loss_sync && idx >= (payload_bytes / 2))
          err_bits[2] = 1'b1;
        if (inject_parity_error && idx == (payload_bytes / 3))
          err_bits[1] = 1'b1;
        if (inject_decode_error && idx == ((payload_bytes * 2) / 3))
          err_bits[0] = 1'b1;
        drive_symbol(1'b0, payload_byte, err_bits, channel);
      end

      drive_symbol(1'b0, byte'(crc_trailer[15:8]), '0, channel);
      drive_symbol(1'b0, byte'(crc_trailer[7:0]), '0, channel);
    endtask

    task automatic send_doc_frame_body(
      int unsigned frame_number,
      int unsigned frame_len,
      bit          short_mode,
      bit [7:0]    channel,
      bit          inject_bad_crc        = 1'b0,
      bit          inject_valid_gap      = 1'b0,
      bit          inject_loss_sync      = 1'b0,
      bit          inject_parity_error   = 1'b0,
      bit          inject_decode_error   = 1'b0,
      bit          inject_comma          = 1'b0,
      int unsigned truncate_after_bytes  = 32'hffff_ffff,
      int unsigned terminate_after_bytes = 32'hffff_ffff
    );
      int unsigned payload_bytes;
      bit [2:0]    err_bits;
      byte unsigned payload_byte;
      byte unsigned frame_bytes[$];
      bit [15:0]    crc_trailer;

      frame_bytes.push_back(byte'(frame_number[15:8]));
      frame_bytes.push_back(byte'(frame_number[7:0]));
      frame_bytes.push_back(frame_len_msb_byte(short_mode, frame_len));
      frame_bytes.push_back(byte'(frame_len[7:0]));

      payload_bytes = payload_byte_count(short_mode, frame_len);
      for (int unsigned idx = 0; idx < payload_bytes; idx++)
        frame_bytes.push_back(payload_byte_value(frame_number, idx, short_mode));
      crc_trailer = frcv_crc16_trailer(frame_bytes);
      if (inject_bad_crc)
        crc_trailer ^= 16'h0001;

      drive_symbol(1'b0, byte'(frame_number[15:8]), '0, channel);
      drive_symbol(1'b0, byte'(frame_number[7:0]), '0, channel);
      drive_symbol(1'b0, frame_len_msb_byte(short_mode, frame_len), '0, channel);
      drive_symbol(1'b0, byte'(frame_len[7:0]), '0, channel);
      for (int unsigned idx = 0; idx < payload_bytes; idx++) begin
        if (terminate_after_bytes != 32'hffff_ffff && idx == terminate_after_bytes)
          pulse_ctrl_async(CTRL_TERMINATING, "TERMINATING");

        if (truncate_after_bytes != 32'hffff_ffff && idx >= truncate_after_bytes) begin
          wait_cycles(2);
          return;
        end

        payload_byte = payload_byte_value(frame_number, idx, short_mode);
        if (inject_valid_gap && idx == (payload_bytes / 2)) begin
          drive_rx_symbol(1'b0, 1'b0, payload_byte, '0, channel);
          continue;
        end
        if (inject_comma && idx == (payload_bytes / 2)) begin
          drive_symbol(1'b1, 8'hbc, '0, channel);
          continue;
        end

        err_bits = '0;
        if (inject_loss_sync && idx >= (payload_bytes / 2))
          err_bits[2] = 1'b1;
        if (inject_parity_error && idx == (payload_bytes / 3))
          err_bits[1] = 1'b1;
        if (inject_decode_error && idx == ((payload_bytes * 2) / 3))
          err_bits[0] = 1'b1;
        drive_symbol(1'b0, payload_byte, err_bits, channel);
      end

      drive_symbol(1'b0, byte'(crc_trailer[15:8]), '0, channel);
      drive_symbol(1'b0, byte'(crc_trailer[7:0]), '0, channel);
    endtask

    task automatic drive_symbol_and_maybe_reset(
      bit           is_k,
      byte unsigned byte_value,
      bit [2:0]     error,
      bit [7:0]     channel,
      inout int unsigned symbol_idx,
      input int unsigned reset_after_symbol_idx,
      inout bit did_reset
    );
      if (did_reset)
        return;
      drive_symbol(is_k, byte_value, error, channel);
      if (symbol_idx == reset_after_symbol_idx) begin
        pulse_reset(2);
        did_reset = 1'b1;
      end
      symbol_idx++;
    endtask

    task automatic send_doc_frame_with_reset(
      int unsigned frame_number,
      int unsigned frame_len,
      bit          short_mode,
      bit [7:0]    channel,
      int unsigned reset_after_symbol_idx,
      bit          inject_bad_crc        = 1'b0,
      bit          inject_loss_sync      = 1'b0,
      bit          inject_parity_error   = 1'b0,
      bit          inject_decode_error   = 1'b0
    );
      int unsigned payload_bytes;
      int unsigned symbol_idx;
      bit [2:0]    err_bits;
      byte unsigned payload_byte;
      bit          did_reset;
      byte unsigned frame_bytes[$];
      bit [15:0]    crc_trailer;

      payload_bytes = payload_byte_count(short_mode, frame_len);
      symbol_idx = 0;
      did_reset = 1'b0;
      frame_bytes.push_back(byte'(frame_number[15:8]));
      frame_bytes.push_back(byte'(frame_number[7:0]));
      frame_bytes.push_back(frame_len_msb_byte(short_mode, frame_len));
      frame_bytes.push_back(byte'(frame_len[7:0]));
      for (int unsigned idx = 0; idx < payload_bytes; idx++)
        frame_bytes.push_back(payload_byte_value(frame_number, idx, short_mode));
      crc_trailer = frcv_crc16_trailer(frame_bytes);
      if (inject_bad_crc)
        crc_trailer ^= 16'h0001;

      drive_symbol_and_maybe_reset(1'b1, HEADER_BYTE, '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      drive_symbol_and_maybe_reset(1'b0, byte'(frame_number[15:8]), '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      drive_symbol_and_maybe_reset(1'b0, byte'(frame_number[7:0]), '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      drive_symbol_and_maybe_reset(1'b0, frame_len_msb_byte(short_mode, frame_len), '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      drive_symbol_and_maybe_reset(1'b0, byte'(frame_len[7:0]), '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      if (did_reset)
        return;

      for (int unsigned idx = 0; idx < payload_bytes; idx++) begin
        err_bits = '0;
        if (inject_loss_sync && idx >= (payload_bytes / 2))
          err_bits[2] = 1'b1;
        if (inject_parity_error && idx == (payload_bytes / 3))
          err_bits[1] = 1'b1;
        if (inject_decode_error && idx == ((payload_bytes * 2) / 3))
          err_bits[0] = 1'b1;
        payload_byte = payload_byte_value(frame_number, idx, short_mode);
        drive_symbol_and_maybe_reset(1'b0, payload_byte, err_bits, channel, symbol_idx, reset_after_symbol_idx, did_reset);
        if (did_reset)
          return;
      end

      drive_symbol_and_maybe_reset(1'b0, byte'(crc_trailer[15:8]),
        '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
      if (did_reset)
        return;
      drive_symbol_and_maybe_reset(1'b0, byte'(crc_trailer[7:0]),
        '0, channel, symbol_idx, reset_after_symbol_idx, did_reset);
    endtask

    task automatic drive_ctrl_and_header_same_cycle(
      logic [8:0] cmd,
      string      state_name,
      bit [7:0]   channel
    );
      fork
        begin
          pulse_ctrl(cmd, state_name);
        end
      join_none
      drive_rx_symbol(1'b1, 1'b1, HEADER_BYTE, '0, channel);
    endtask

    task automatic run_edge_repeat_ctrl_case(logic [8:0] cmd, string state_name, int unsigned repeat_count);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      repeat (repeat_count) begin
        send_ctrl(cmd, state_name);
        wait_cycles(1);
      end
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 6, case_id);
    endtask

    task automatic run_edge_illegal_ctrl_case(logic [8:0] cmd, string state_name, bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;

      wait_for_reset_release();
      pulse_ctrl(cmd, state_name);
      wait_cycles(2);
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      drive_symbol(1'b1, HEADER_BYTE, '0, channel);
      drive_symbol(1'b0, 8'h00, '0, channel);
      drive_symbol(1'b0, byte'(bucket_case_index() & 8'hff), '0, channel);
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 8, case_id);
    endtask

    task automatic run_edge_truncated_case(bit [7:0] channel, bit short_mode);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      int unsigned frame_number;
      int unsigned truncate_after_bytes;

      wait_for_reset_release();
      run_start_for_mode();
      base_hits      = m_env.m_scb.hit_count;
      base_real_eops = m_env.m_scb.real_eop_count;
      base_headers   = m_env.m_scb.header_count;
      base_endofruns = m_env.m_scb.endofrun_count;
      frame_number = 16'he100 | bucket_case_index();
      truncate_after_bytes = short_mode ? 2 : 5;
      scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
        random_doc_case(), short_mode, 2, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "truncated", 6);
      send_doc_frame(
        frame_number,
        2,
        short_mode,
        channel,
        1'b0,
        1'b0,
        1'b0,
        1'b0,
        1'b0,
        1'b0,
        truncate_after_bytes,
        32'hffff_ffff
      );
      expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 8, case_id);
    endtask

    task automatic run_edge_explicit_doc_case(bit [7:0] channel);
      case (case_id)
        "E013_ctrl_valid_low_keeps_previous_state": begin
          do_e013_ctrl_valid_low_keeps_previous_state(channel);
        end
        "E009_ctrl_valid_held_high_repeated_word_stable": begin
          run_edge_repeat_ctrl_case(CTRL_RUNNING, "RUNNING", 8);
        end
        "E010_ctrl_illegal_zero_word_maps_error": begin
          run_edge_illegal_ctrl_case(9'b000000000, "ILLEGAL_ZERO", channel);
        end
        "E011_ctrl_twohot_word_maps_error": begin
          run_edge_illegal_ctrl_case(CTRL_IDLE | CTRL_RUNNING, "ILLEGAL_TWOHOT", channel);
        end
        "E012_ctrl_allones_word_maps_error": begin
          run_edge_illegal_ctrl_case(9'h1ff, "ILLEGAL_ALLONES", channel);
        end
        "E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low": begin
          do_e019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low();
        end
        "E020_midframe_valid_low_does_not_pause_parser": begin
          do_e020_midframe_valid_low_does_not_pause_parser();
        end
        "E029_header_following_write_zero_then_one_requires_clocked_enable": begin
          do_e029_header_following_write_zero_then_one_requires_clocked_enable(channel);
        end
        "E121_repeated_terminating_words_do_not_duplicate_eop": begin
          do_e121_repeated_terminating_words_do_not_duplicate_eop(channel);
        end
        "E122_repeated_idle_words_do_not_force_extra_output": begin
          run_edge_repeat_ctrl_case(CTRL_IDLE, "IDLE", 8);
        end
        "E123_header_after_error_state_needs_new_running_or_idle_monitoring": begin
          do_e123_header_after_error_state_needs_new_running_or_idle_monitoring(channel);
        end
        "E124_truncated_long_frame_never_asserts_eop": begin
          run_edge_truncated_case(channel, 1'b0);
        end
        "E125_truncated_short_frame_never_asserts_eop": begin
          run_edge_truncated_case(channel, 1'b1);
        end
        default: begin
          `uvm_fatal("FRCV_CASE_UNIMPLEMENTED",
            $sformatf("Missing explicit EDGE case handler for %s", case_id))
        end
      endcase
    endtask

    task automatic run_reset_doc_case(bit [7:0] channel);
      int unsigned base_hits;
      int unsigned base_real_eops;
      int unsigned base_headers;
      int unsigned base_endofruns;
      int unsigned frame_number;
      int unsigned frame_len;
      int unsigned payload_bytes;
      int unsigned reset_after_symbol_idx;
      bit          short_mode;
      bit          inject_bad_crc;
      bit          inject_loss_sync;
      bit          inject_parity_error;
      bit [31:0]   base_crc_err_counter;
      bit [31:0]   after_crc_err_counter;
      bit [31:0]   ignored_counter;
      bit [31:0]   ignored_read;
      frcv_obs_item hit_obs;

      wait_for_reset_release();
      base_hits       = m_env.m_scb.hit_count;
      base_real_eops  = m_env.m_scb.real_eop_count;
      base_headers    = m_env.m_scb.header_count;
      base_endofruns  = m_env.m_scb.endofrun_count;
      m_env.m_scb.snapshot_counters(base_crc_err_counter, ignored_counter, ignored_counter, ignored_counter);

      if (case_id == "B001_reset_idle_outputs_quiet" || case_id == "X001_reset_in_fs_idle") begin
        pulse_reset(2);
        expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 8, case_id);
        return;
      end

      if (case_id == "X012_reset_while_csr_read_active") begin
        fork
          csr_read(8'h00, ignored_read);
          begin
            wait_cycles(1);
            pulse_reset(2);
          end
        join
        csr_read(8'h00, ignored_read);
        expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 8, case_id);
        return;
      end

      if (case_id == "X013_reset_while_ctrl_valid_high") begin
        fork
          send_ctrl(CTRL_RUNNING, "RUNNING", 2);
          begin
            wait_cycles(1);
            pulse_reset(2);
          end
        join
        base_hits      = m_env.m_scb.hit_count;
        base_real_eops = m_env.m_scb.real_eop_count;
        base_headers   = m_env.m_scb.header_count;
        base_endofruns = m_env.m_scb.endofrun_count;
        send_long_frame(16'h8d13, 48'h102030405060);
        expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 12, case_id);
        return;
      end

      run_start_for_mode();
      frame_number = 16'h8000 | bucket_case_index();
      frame_len = 2;
      short_mode = 1'b0;
      inject_bad_crc = 1'b0;
      inject_loss_sync = 1'b0;
      inject_parity_error = 1'b0;
      reset_after_symbol_idx = 1;

      unique case (case_id)
        "X002_reset_in_frame_counter": reset_after_symbol_idx = 1;
        "X003_reset_in_event_counter": reset_after_symbol_idx = 3;
        "X004_reset_in_long_unpack": reset_after_symbol_idx = 5;
        "X005_reset_in_short_unpack": begin
          short_mode = 1'b1;
          reset_after_symbol_idx = 5;
        end
        "X006_reset_in_unpack_extra": begin
          short_mode = 1'b1;
          frame_len = 3;
          reset_after_symbol_idx = 7;
        end
        "X007_reset_in_crc_calc": begin
          frame_len = 1;
        end
        "X008_reset_in_crc_check": begin
          frame_len = 1;
        end
        "X009_reset_during_bad_crc_frame": begin
          frame_len = 1;
          inject_bad_crc = 1'b1;
        end
        "X010_reset_during_hit_error_frame": begin
          frame_len = 2;
          inject_parity_error = 1'b1;
        end
        "X011_reset_during_losssync_frame": begin
          frame_len = 2;
          inject_loss_sync = 1'b1;
        end
        default: begin
        end
      endcase

      payload_bytes = payload_byte_count(short_mode, frame_len);
      if (case_id == "X007_reset_in_crc_calc")
        reset_after_symbol_idx = 5 + payload_bytes;
      else if (case_id == "X008_reset_in_crc_check" || case_id == "X009_reset_during_bad_crc_frame")
        reset_after_symbol_idx = 5 + payload_bytes;
      else if (case_id == "X010_reset_during_hit_error_frame" || case_id == "X011_reset_during_losssync_frame")
        reset_after_symbol_idx = 6;

      if (case_id == "X004_reset_in_long_unpack" ||
          case_id == "X005_reset_in_short_unpack" ||
          case_id == "X006_reset_in_unpack_extra") begin
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), short_mode, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "reset_drop", 6);
        send_doc_frame_with_reset(
          frame_number,
          frame_len,
          short_mode,
          channel,
          reset_after_symbol_idx,
          inject_bad_crc,
          inject_loss_sync,
          inject_parity_error,
          1'b0
        );
        expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 8, case_id);
        return;
      end

      if (case_id == "X007_reset_in_crc_calc" ||
          case_id == "X008_reset_in_crc_check" ||
          case_id == "X009_reset_during_bad_crc_frame") begin
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), short_mode, frame_len, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "reset_crc", 6);
        send_doc_frame_with_reset(
          frame_number,
          frame_len,
          short_mode,
          channel,
          reset_after_symbol_idx,
          inject_bad_crc,
          inject_loss_sync,
          inject_parity_error,
          1'b0
        );
        expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 1, 1, 1, 0, 8, case_id);
        if (case_id == "X009_reset_during_bad_crc_frame") begin
          m_env.m_scb.snapshot_counters(after_crc_err_counter, ignored_counter, ignored_counter, ignored_counter);
          if (after_crc_err_counter != base_crc_err_counter)
            `uvm_fatal("FRCV_CASE",
              $sformatf("%s expected crc_err_counter to remain %0d got %0d",
                case_id, base_crc_err_counter, after_crc_err_counter))
        end
        return;
      end

      if (case_id == "X010_reset_during_hit_error_frame" ||
          case_id == "X011_reset_during_losssync_frame") begin
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), short_mode, frame_len, 0, 1, 0, 0, 1'b0, 1'b0, 1'b0, 1'b0, "reset_error_preheader", 6);
        send_doc_frame_with_reset(
          frame_number,
          frame_len,
          short_mode,
          channel,
          reset_after_symbol_idx,
          1'b0,
          inject_loss_sync,
          inject_parity_error,
          1'b0
        );
        expect_activity_delta(base_hits, base_real_eops, base_headers, base_endofruns, 0, 0, 1, 0, 8, case_id);
        base_hits      = m_env.m_scb.hit_count;
        base_real_eops = m_env.m_scb.real_eop_count;
        base_headers   = m_env.m_scb.header_count;
        base_endofruns = m_env.m_scb.endofrun_count;
        run_start_for_mode();
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b1,
          random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "post_reset_clean", 6);
        send_long_frame(16'h8d40 | bucket_case_index(), 48'h0f1e2d3c4b5a);
        wait_for_counts(base_hits + 1, base_real_eops + 1, base_headers + 1, 32, case_id);
        hit_obs = find_last_obs(FRCV_OBS_HIT);
        if (hit_obs == null)
          `uvm_fatal("FRCV_CASE", $sformatf("%s expected a clean post-reset hit", case_id))
        if (execution_mode == "isolated") begin
          if (case_id == "X010_reset_during_hit_error_frame" && hit_obs.hit_error[0] !== 1'b0)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected error0 to clear on the post-reset frame", case_id))
          if (case_id == "X011_reset_during_losssync_frame" && hit_obs.hit_error[2] !== 1'b0)
            `uvm_fatal("FRCV_CASE", $sformatf("%s expected error2 to clear on the post-reset frame", case_id))
        end
        return;
      end

      send_doc_frame_with_reset(
        frame_number,
        frame_len,
        short_mode,
        channel,
        reset_after_symbol_idx,
        inject_bad_crc,
        inject_loss_sync,
        inject_parity_error,
        1'b0
      );
      expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 8, case_id);
    endtask

    task automatic run_edge_same_cycle_doc_case(bit [7:0] channel);
      int unsigned frame_number;

      wait_for_reset_release();
      frame_number = 16'he000 | bucket_case_index();

      if (case_id == "E002_running_command_same_cycle_header_not_accepted") begin
        int unsigned base_hits;
        int unsigned base_real_eops;
        int unsigned base_headers;
        int unsigned base_endofruns;

        base_hits      = m_env.m_scb.hit_count;
        base_real_eops = m_env.m_scb.real_eop_count;
        base_headers   = m_env.m_scb.header_count;
        base_endofruns = m_env.m_scb.endofrun_count;
        drive_ctrl_and_header_same_cycle(CTRL_RUNNING, "RUNNING", channel);
        expect_no_new_activity(base_hits, base_real_eops, base_headers, base_endofruns, 4, case_id);
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);
        send_doc_frame(frame_number, 1, 1'b0, channel);
        wait_cycles(8);
        return;
      end

      if (case_id == "E003_idle_command_same_cycle_header_still_sees_previous_running") begin
        run_start_for_mode();
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);
        drive_ctrl_and_header_same_cycle(CTRL_IDLE, "IDLE", channel);
        send_doc_frame_body(frame_number, 1, 1'b0, channel);
        wait_cycles(8);
        return;
      end

      if (case_id == "E004_terminating_command_same_cycle_open_frame_continues") begin
        bit [47:0] hit_word;
        bit [15:0] crc_trailer;
        byte unsigned frame_bytes[$];

        run_start_for_mode();
        scb_expect_txn(case_id, execution_mode, bucket_name_for_case(case_id), "", 1'b0,
          random_doc_case(), 1'b0, 1, 1, 1, 1, 0, 1'b0, 1'b0, 1'b0, 1'b0, "none", 6);
        hit_word = 48'h112233445566;
        frame_bytes.push_back(byte'(frame_number[15:8]));
        frame_bytes.push_back(byte'(frame_number[7:0]));
        frame_bytes.push_back(8'h00);
        frame_bytes.push_back(8'h01);
        for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
          frame_bytes.push_back(byte'(hit_word[byte_idx*8 +: 8]));
        crc_trailer = frcv_crc16_trailer(frame_bytes);
        drive_symbol(1'b1, HEADER_BYTE, '0, channel);
        drive_symbol(1'b0, byte'(frame_number[15:8]), '0, channel);
        force_ctrl_lines(CTRL_TERMINATING, 1'b1, case_id);
        drive_symbol(1'b0, byte'(frame_number[7:0]), '0, channel);
        release_ctrl_lines_nowait(case_id);
        drive_symbol(1'b0, frame_len_msb_byte(1'b0, 1), '0, channel);
        drive_symbol(1'b0, 8'h01, '0, channel);
        for (int byte_idx = 5; byte_idx >= 0; byte_idx--)
          drive_symbol(1'b0, byte'(hit_word[byte_idx*8 +: 8]), '0, channel);
        drive_symbol(1'b0, byte'(crc_trailer[15:8]), '0, channel);
        drive_symbol(1'b0, byte'(crc_trailer[7:0]), '0, channel);
        wait_cycles(8);
      end
    endtask

    function automatic int unsigned expected_hits_before_loss_sync(
      bit          short_mode,
      int unsigned frame_len,
      int unsigned first_bad_payload_idx
    );
      int unsigned completed_hits;
      int unsigned payload_idx;

      if (frame_len == 0)
        return 0;
      if (first_bad_payload_idx == 32'hffff_ffff)
        return frame_len;

      completed_hits = 0;
      payload_idx    = 0;
      while (completed_hits < frame_len) begin
        if (!short_mode) begin
          payload_idx += 6;
        end else if (completed_hits[0] == 1'b0) begin
          payload_idx += 4;
        end else begin
          payload_idx += 3;
        end

        if ((payload_idx - 1) >= first_bad_payload_idx)
          return completed_hits;
        completed_hits++;
      end

      return completed_hits;
    endfunction

    function automatic int unsigned expected_hits_for_frame(
      bit          short_mode,
      int unsigned frame_len,
      int unsigned truncate_after_bytes,
      bit          inject_loss_sync,
      bit inject_comma,
      bit mode_halt_abort
    );
      if (frame_len == 0)
        return 0;
      if (truncate_after_bytes != 32'hffff_ffff)
        return 0;
      if (inject_comma && mode_halt_abort)
        return expected_hits_before_loss_sync(
          short_mode,
          frame_len,
          payload_byte_count(short_mode, frame_len) / 2
        );
      if (inject_loss_sync)
        return expected_hits_before_loss_sync(short_mode, frame_len, payload_byte_count(short_mode, frame_len) / 2);
      return frame_len;
    endfunction

    function automatic int unsigned expected_headers_for_frame(
      int unsigned frame_len,
      int unsigned truncate_after_bytes,
      bit inject_comma,
      bit mode_halt_abort
    );
      if (truncate_after_bytes != 32'hffff_ffff)
        return 0;
      if (inject_comma && mode_halt_abort)
        return 1;
      return 1;
    endfunction

    function automatic int unsigned expected_real_eops_for_frame(
      int unsigned expected_hits,
      bit          inject_loss_sync,
      bit          inject_comma,
      bit          mode_halt_abort
    );
      if (expected_hits == 0)
        return 0;
      if (inject_loss_sync)
        return 0;
      if (inject_comma && mode_halt_abort)
        return 0;
      return 1;
    endfunction

    function automatic string derived_stop_boundary(int unsigned terminate_after_bytes);
      if (terminate_after_bytes == 32'hffff_ffff)
        return "none";
      if (terminate_after_bytes == 0)
        return "before_payload";
      return "mid_payload";
    endfunction

    task automatic run_documented_case_engine();
      int unsigned iterations;
      int unsigned frame_len;
      int unsigned frame_number;
      int unsigned exp_hits;
      int unsigned exp_headers;
      int unsigned exp_real_eops;
      int unsigned truncate_after_bytes;
      int unsigned terminate_after_bytes;
      int unsigned bytes_per_iteration;
      string       stop_boundary;
      bit          short_mode;
      bit [7:0]    channel;
      bit          inject_bad_crc;
      bit          inject_valid_gap;
      bit          inject_loss_sync;
      bit          inject_parity_error;
      bit          inject_decode_error;
      bit          inject_comma;
      bit          mode_halt_abort;

      if (!recognized_doc_case())
        fail_unimplemented_case();

      `uvm_info(
        "FRCV_CASE",
        $sformatf(
          "DOC_CASE_ENGINE_V2 case=%s build=%s effort=%s global_case_idx=%0d bucket_case_idx=%0d",
          case_id, build_tag, effort_mode, global_case_index(), bucket_case_index()
        ),
        UVM_NONE
      )

      scb_case_begin(case_id, execution_mode, random_doc_case());
      wait_for_reset_release();
      iterations            = derive_iterations();
      frame_len             = derive_frame_len();
      short_mode            = short_mode_case();
      channel               = derived_channel_value();
      bytes_per_iteration   = payload_byte_count(short_mode, (frame_len == 0) ? 1 : frame_len) + 7;
      if (bytes_per_iteration > 0 &&
          (iterations * bytes_per_iteration) > doc_payload_cap())
        iterations = (doc_payload_cap() / bytes_per_iteration) > 0
                     ? (doc_payload_cap() / bytes_per_iteration)
                     : 1;
      inject_bad_crc        = contains_ci("bad_crc") || contains_ci("crc_bad") || contains_ci("all_bad");
      inject_valid_gap      = contains_ci("valid_low");
      inject_loss_sync      = contains_ci("loss_sync") || contains_ci("losssync");
      inject_parity_error   = contains_ci("parity");
      inject_decode_error   = contains_ci("decode");
      inject_comma          = contains_ci("comma");
      mode_halt_abort       = inject_comma && (build_tag != "CFG_B");
      truncate_after_bytes  = (contains_ci("header_only") || contains_ci("missing") || contains_ci("cut_mid")) ? 1 : 32'hffff_ffff;
      terminate_after_bytes = (contains_ci("terminating") || contains_ci("stop_midframe") || contains_ci("stop_during"))
                              ? (payload_byte_count(short_mode, frame_len) / 2)
                              : 32'hffff_ffff;

      if (contains_ci("reset")) begin
        run_reset_doc_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "B001_reset_idle_outputs_quiet" ||
          case_id == "B002_idle_force_go_monitor_path_open" ||
          case_id == "B003_idle_force_go_ignores_csr_mask" ||
          case_id == "B004_running_needs_csr_enable_high" ||
          case_id == "B005_running_mask_low_blocks_header_start" ||
          case_id == "B006_run_prepare_clears_parser_state" ||
          case_id == "B007_sync_keeps_parser_closed" ||
          case_id == "B008_running_opens_header_detection" ||
          case_id == "B009_terminating_blocks_new_header_from_idle" ||
          case_id == "B010_terminating_allows_open_frame_to_finish" ||
          case_id == "B011_ctrl_unknown_word_enters_error" ||
          case_id == "B012_ctrl_ready_high_with_valid" ||
          case_id == "B013_ctrl_ready_high_without_valid" ||
          case_id == "B014_ctrl_decode_idle_force_go" ||
          case_id == "B015_ctrl_decode_run_prepare_disable" ||
          case_id == "B016_ctrl_decode_sync_disable" ||
          case_id == "B017_ctrl_decode_running_go" ||
          case_id == "B018_ctrl_decode_terminating_stop_new_headers" ||
          case_id == "B019_ctrl_decode_link_test_disable" ||
          case_id == "B020_ctrl_decode_sync_test_disable" ||
          case_id == "B021_ctrl_decode_reset_disable" ||
          case_id == "B022_ctrl_decode_out_of_daq_disable" ||
          case_id == "B023_csr_control_default_enable" ||
          case_id == "B024_csr_control_write_zero_masks_running" ||
          case_id == "B025_csr_control_write_one_unmasks_running" ||
          case_id == "B026_csr_read_addr0_control_status" ||
          case_id == "B027_csr_read_addr1_crc_counter" ||
          case_id == "B029_csr_read_unused_word_zero" ||
          case_id == "B031_csr_waitrequest_low_on_read" ||
          case_id == "B032_csr_waitrequest_low_on_write" ||
          case_id == "B033_csr_waitrequest_high_when_idle" ||
          case_id == "B034_csr_reserved_control_bits_roundtrip" ||
          case_id == "B036_data_byte_without_kchar_no_start" ||
          case_id == "B039_long_zero_hit_frame_headerinfo_pulse" ||
          case_id == "B041_long_one_hit_sop_and_eop_same_transfer" ||
          case_id == "B059_long_crc_bad_raises_error1_on_eop" ||
          case_id == "B060_long_crc_bad_increments_crc_counter" ||
          case_id == "B079_short_crc_bad_raises_error1_on_last_hit" ||
          case_id == "B080_short_crc_counter_accumulates" ||
          case_id == "B112_long_good_frames_leave_crc_counter_stable" ||
          case_id == "B113_bad_frames_accumulate_crc_counter" ||
          case_id == "B114_mixed_good_bad_crc_count_matches_bad_frames" ||
          case_id == "B098_idle_monitoring_accepts_header_without_running" ||
          case_id == "B099_idle_monitoring_still_ignores_csr_zero") begin
        run_basic_explicit_doc_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "E002_running_command_same_cycle_header_not_accepted" ||
          case_id == "E003_idle_command_same_cycle_header_still_sees_previous_running" ||
          case_id == "E004_terminating_command_same_cycle_open_frame_continues") begin
        run_edge_same_cycle_doc_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "E009_ctrl_valid_held_high_repeated_word_stable" ||
          case_id == "E013_ctrl_valid_low_keeps_previous_state" ||
          case_id == "E010_ctrl_illegal_zero_word_maps_error" ||
          case_id == "E011_ctrl_twohot_word_maps_error" ||
          case_id == "E012_ctrl_allones_word_maps_error" ||
          case_id == "E019_fs_idle_accepts_header_even_if_asi_rx8b1k_valid_low" ||
          case_id == "E020_midframe_valid_low_does_not_pause_parser" ||
          case_id == "E029_header_following_write_zero_then_one_requires_clocked_enable" ||
          case_id == "E121_repeated_terminating_words_do_not_duplicate_eop" ||
          case_id == "E122_repeated_idle_words_do_not_force_extra_output" ||
          case_id == "E123_header_after_error_state_needs_new_running_or_idle_monitoring" ||
          case_id == "E124_truncated_long_frame_never_asserts_eop" ||
          case_id == "E125_truncated_short_frame_never_asserts_eop") begin
        run_edge_explicit_doc_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "X099_csr_mask_low_then_illegal_ctrl") begin
        do_x099_csr_mask_low_then_illegal_ctrl(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "P034_cadence_idle_monitoring_only") begin
        do_p034_cadence_idle_monitoring_only(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "P045_mask_low_long_idle_soak") begin
        do_p045_mask_low_long_idle_soak(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "X014_ctrl_allzero_word" ||
          case_id == "X015_ctrl_twohot_idle_running" ||
          case_id == "X016_ctrl_twohot_running_terminating" ||
          case_id == "X017_ctrl_allones_word" ||
          case_id == "X018_ctrl_random_illegal_word_seed1" ||
          case_id == "X019_ctrl_random_illegal_word_seed2" ||
          case_id == "X020_ctrl_valid_glitch_no_word_change" ||
          case_id == "X021_ctrl_payload_changes_while_valid_high" ||
          case_id == "X022_ctrl_illegal_then_legal_recovery_idle" ||
          case_id == "X023_ctrl_illegal_then_legal_recovery_running" ||
          case_id == "X024_ctrl_ready_not_indicating_recovery" ||
          case_id == "X025_ctrl_valid_low_payload_noise" ||
          case_id == "X026_ctrl_fast_oscillation_legal_illegal_mix") begin
        run_error_control_negative_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (case_id == "X027_long_header_only_no_counters" ||
          case_id == "X028_long_missing_frame_number_byte1" ||
          case_id == "X029_long_missing_event_counter_msb" ||
          case_id == "X030_long_missing_event_counter_lsb" ||
          case_id == "X031_long_declared_len1_missing_payload" ||
          case_id == "X032_long_declared_len2_only_one_hit_present" ||
          case_id == "X033_long_declared_len_large_cut_midpayload" ||
          case_id == "X034_long_bad_trailer_symbol_replaced_with_data" ||
          case_id == "X035_long_extra_payload_after_declared_len" ||
          case_id == "X036_long_new_header_inside_payload" ||
          case_id == "X037_long_comma_inside_payload_mode0" ||
          case_id == "X039_long_valid_low_entire_malformed_frame" ||
          case_id == "X040_short_header_only_no_counters" ||
          case_id == "X041_short_missing_frame_number_byte1" ||
          case_id == "X042_short_missing_event_counter_msb" ||
          case_id == "X043_short_missing_event_counter_lsb" ||
          case_id == "X044_short_declared_len1_missing_payload" ||
          case_id == "X047_short_declared_len_large_cut_midpayload" ||
          case_id == "X048_short_new_header_inside_payload" ||
          case_id == "X049_short_extra_payload_after_declared_len" ||
          (case_id == "X050_short_comma_inside_payload_mode0" && build_tag != "CFG_B") ||
          (case_id == "X051_short_comma_inside_payload_mode1" && build_tag != "CFG_B") ||
          case_id == "X052_short_valid_low_entire_malformed_frame") begin
        run_error_malformed_case(channel);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (contains_ci("allzero_word") || contains_ci("unknown_word"))
        pulse_ctrl(9'b000000000, "ILLEGAL_ZERO");
      else if (contains_ci("twohot"))
        pulse_ctrl(CTRL_IDLE | CTRL_RUNNING, "ILLEGAL_TWOHOT");
      else if (contains_ci("allones_word"))
        pulse_ctrl(9'h1ff, "ILLEGAL_ALLONES");

      apply_case_control_state();
      apply_case_csr_ops();

      if (contains_ci("blocks_header") || contains_ci("no_start") || contains_ci("keeps_outputs_quiet") ||
          contains_ci("no_counters") || contains_ci("stays_quiescent")) begin
        drive_symbol(1'b1, HEADER_BYTE, '0, channel);
        drive_symbol(1'b0, 8'h00, '0, channel);
        drive_symbol(1'b0, byte'(bucket_case_index() & 8'hff), '0, channel);
        wait_cycles(6);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      if (contains_ci("header_only")) begin
        drive_symbol(1'b1, HEADER_BYTE, '0, channel);
        wait_cycles(6);
        scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
        scb_case_end(case_id);
        return;
      end

      for (int unsigned iter = 0; iter < iterations; iter++) begin
        frame_number = ((bucket_case_index() & 16'h00ff) << 8) | (iter & 16'h00ff);
        if (contains_ci("mixed") || contains_ci("alternate") || contains_ci("seed")) begin
          short_mode = (iter[0] ^ short_mode_case());
          frame_len  = (derive_frame_len() == 0) ? iter[1:0] : ((derive_frame_len() + iter) % 8);
          if (contains_ci("seed") && frame_len == 0)
            frame_len = 1;
        end
        if (contains_ci("zero_hit"))
          frame_len = 0;
        if (contains_ci("one_hit"))
          frame_len = 1;
        if (contains_ci("two_hit"))
          frame_len = 2;

        exp_hits = expected_hits_for_frame(short_mode, frame_len, truncate_after_bytes,
                                           inject_loss_sync, inject_comma, mode_halt_abort);
        exp_headers = expected_headers_for_frame(frame_len, truncate_after_bytes, inject_comma, mode_halt_abort);
        exp_real_eops = expected_real_eops_for_frame(
          exp_hits,
          inject_loss_sync,
          inject_comma,
          mode_halt_abort
        );
        stop_boundary = (inject_comma && mode_halt_abort)
                        ? "mode0_abort"
                        : derived_stop_boundary(terminate_after_bytes);
        if (exp_headers != 0 || exp_hits != 0 || exp_real_eops != 0) begin
          scb_expect_txn(
            case_id,
            execution_mode,
            bucket_name_for_case(case_id),
            "",
            1'b0,
            random_doc_case(),
            short_mode,
            frame_len,
            exp_hits,
            exp_headers,
            exp_real_eops,
            (inject_bad_crc || (contains_ci("bad_every") && iter != 0 && iter[0])) ? 1 : 0,
            inject_parity_error || inject_decode_error,
            1'b0,
            1'b0,
            1'b0,
            stop_boundary,
            6
          );
        end

        send_doc_frame(
          frame_number,
          frame_len,
          short_mode,
          channel,
          inject_bad_crc || (contains_ci("bad_every") && iter != 0 && iter[0]),
          inject_valid_gap,
          inject_loss_sync,
          inject_parity_error,
          inject_decode_error,
          inject_comma,
          truncate_after_bytes,
          terminate_after_bytes
        );

        if (contains_ci("stop_after_every_frame") || contains_ci("stop_every_other_frame"))
          pulse_ctrl(CTRL_TERMINATING, "TERMINATING");
        if (contains_ci("idle_after_terminating"))
          issue_ctrl_for_mode(CTRL_IDLE, "IDLE");
        if (contains_ci("running_reopen_window"))
          issue_ctrl_for_mode(CTRL_RUNNING, "RUNNING");
        if (contains_ci("runprepare"))
          issue_ctrl_for_mode(CTRL_RUN_PREPARE, "RUN_PREPARE");
        if (contains_ci("poll") || contains_ci("read") || contains_ci("word"))
          apply_case_csr_ops();
        // Practical iterative runs need one idle cycle between frames so the parser
        // can retire the prior frame through the CRC/idle boundary before the next
        // header arrives. Without this, QuestaOne bucket-frame runs can miss every
        // other fresh header in zero-hit / soak style PROF cases.
        if (effort_mode != "extensive" &&
            iter + 1 < iterations)
          wait_cycles(1);
      end

      scb_wait_case_txn_drain(case_id, DOC_CASE_DRAIN_MAX_CYCLES, case_id);
      scb_case_end(case_id);
    endtask

    task automatic run_case_by_id();
      run_documented_case_engine();
    endtask

    task automatic run_named_case(string next_case_id);
      case_id = next_case_id;
      doc_case_cg.set_inst_name($sformatf("doc_case_cg_%s", case_id));
      run_documented_case_engine();
      doc_case_cg.sample(global_case_index(), bucket_case_index(), random_doc_case());
    endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      run_case_by_id();
      doc_case_cg.sample(global_case_index(), bucket_case_index(), random_doc_case());
      phase.drop_objection(this);
    endtask
  endclass

  class frcv_doc_case_list_test extends frcv_doc_case_test;
    `uvm_component_utils(frcv_doc_case_list_test)

    string case_list_raw;
    string sequence_name;
    string case_queue[$];

    function new(string name, uvm_component parent);
      super.new(name, parent);
      require_case_id = 1'b0;
      execution_mode = "bucket_frame";
      case_list_raw = "";
      sequence_name = "";
    endfunction

    function automatic void split_case_list();
      string token;
      byte unsigned ch;

      token = "";
      case_queue.delete();
      for (int idx = 0; idx < case_list_raw.len(); idx++) begin
        ch = case_list_raw.getc(idx);
        if (ch == 8'd44) begin
          if (token.len() != 0)
            case_queue.push_back(token);
          token = "";
        end else begin
          token = {token, ch};
        end
      end
      if (token.len() != 0)
        case_queue.push_back(token);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!$value$plusargs("FRCV_CASE_LIST=%s", case_list_raw))
        `uvm_fatal("FRCV_CASE_LIST", "Missing +FRCV_CASE_LIST=<comma-separated cases>")
      void'($value$plusargs("FRCV_SEQUENCE_NAME=%s", sequence_name));
      split_case_list();
    endfunction

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      wait_for_reset_release();
      $display(
        "DOC_CASE_LIST_ENGINE_V1 mode=%s build=%s effort=%s seq=%s case_count=%0d iter_cap=%0d payload_cap=%0d",
        execution_mode, build_tag, effort_mode, sequence_name, case_queue.size(), doc_iteration_cap(), doc_payload_cap()
      );
      foreach (case_queue[idx]) begin
        run_named_case(case_queue[idx]);
        wait_cycles(4);
      end
      phase.drop_objection(this);
    endtask
  endclass

  class frcv_cross_bucket_test extends frcv_doc_case_test;
    `uvm_component_utils(frcv_cross_bucket_test)

    string cross_plan;
    int unsigned super_long_txns_override;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      require_case_id = 1'b0;
      execution_mode = "cross";
      cross_plan = "GOOD_ERROR_GOOD";
      super_long_txns_override = 0;
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      void'($value$plusargs("FRCV_CROSS_PLAN=%s", cross_plan));
      void'($value$plusargs("FRCV_SUPER_LONG_TXNS=%d", super_long_txns_override));
    endfunction

    task automatic run_cross_frame(
      string case_name,
      string seq_name,
      int unsigned frame_number,
      bit short_mode,
      int unsigned frame_len,
      bit inject_bad_crc,
      bit inject_loss_sync,
      bit inject_parity_error,
      bit inject_decode_error,
      int unsigned start_delay_cycles,
      bit queued_overlap,
      string stop_boundary = "none",
      int unsigned terminate_after_bytes = 32'hffff_ffff
    );
      int unsigned expected_hits;
      int unsigned expected_real_eops;
      bit          expect_error0;

      wait_cycles(start_delay_cycles);
      if (inject_loss_sync)
        expected_hits = expected_hits_before_loss_sync(short_mode, frame_len, payload_byte_count(short_mode, frame_len) / 2);
      else
        expected_hits = (frame_len == 0) ? 0 : 1;
      expected_real_eops = ((expected_hits != 0) && !inject_loss_sync) ? 1 : 0;
      expect_error0 = inject_parity_error || inject_decode_error;
      scb_expect_txn(
        case_name,
        execution_mode,
        bucket_name_for_case(case_name),
        seq_name,
        1'b1,
        1'b0,
        short_mode,
        frame_len,
        expected_hits,
        1,
        expected_real_eops,
        inject_bad_crc ? 1 : 0,
        expect_error0,
        1'b0,
        1'b0,
        queued_overlap,
        stop_boundary,
        6
      );
      send_doc_frame(
        frame_number,
        frame_len,
        short_mode,
        derived_channel_value(),
        inject_bad_crc,
        1'b0,
        inject_loss_sync,
        inject_parity_error,
        inject_decode_error,
        1'b0,
        32'hffff_ffff,
        terminate_after_bytes
      );
    endtask

    task automatic run_good_error_good();
      for (int iter = 0; iter < 6; iter++) begin
        run_cross_frame("B041_long_one_hit_sop_and_eop_same_transfer", "GOOD_ERROR_GOOD",
                        16'h4100 + iter, 1'b0, 1, 1'b0, 1'b0, 1'b0, 1'b0,
                        $urandom_range(0, 1), (iter != 0));
        run_cross_frame("X066_long_parity_error_first_payload_byte", "GOOD_ERROR_GOOD",
                        16'h6600 + iter, 1'b0, 1, 1'b0, 1'b0, 1'b1, 1'b0,
                        $urandom_range(0, 1), 1'b1);
        run_cross_frame("B127_good_long_then_good_short_sequence", "GOOD_ERROR_GOOD",
                        16'h7f00 + iter, iter[0], 1, 1'b0, 1'b0, 1'b0, 1'b0,
                        $urandom_range(0, 1), 1'b1);
        wait_cycles($urandom_range(1, 4));
      end
    endtask

    task automatic run_interleave_mix();
      for (int iter = 0; iter < 4; iter++) begin
        run_cross_frame("P097_long_clean_short_bad_repeat", "INTERLEAVE_MIX",
                        16'h9700 + iter, 1'b0, 2, 1'b0, 1'b0, 1'b1, 1'b0,
                        $urandom_range(0, 1), 1'b1);
        run_cross_frame("P116_stop_under_losssync_mix", "INTERLEAVE_MIX",
                        16'h1600 + iter, iter[0], 2, 1'b0, 1'b1, 1'b0, 1'b0,
                        $urandom_range(0, 1), 1'b1, "mid_payload",
                        payload_byte_count(iter[0], 2) / 2);
        run_cross_frame("P098_short_clean_long_bad_repeat", "INTERLEAVE_MIX",
                        16'h9800 + iter, 1'b1, 2, 1'b0, 1'b0, 1'b0, 1'b1,
                        $urandom_range(0, 1), 1'b1);
        run_cross_frame("X077_hiterror_plus_losssync_same_frame", "INTERLEAVE_MIX",
                        16'h7700 + iter, iter[0], 2, 1'b0, 1'b1, 1'b1, 1'b0,
                        $urandom_range(0, 1), 1'b1, "mid_payload",
                        payload_byte_count(iter[0], 2) / 2);
        wait_cycles($urandom_range(2, 5));
      end
    endtask

    task automatic run_super_long_counter_soak();
      int unsigned total_txns;
      bit          short_mode;
      int unsigned frame_len;
      int unsigned terminate_after_bytes;
      bit          queued_overlap;
      int unsigned start_delay;

      total_txns = (effort_mode == "extensive") ? 32768 : 4096;
      if (super_long_txns_override != 0)
        total_txns = super_long_txns_override;
      for (int unsigned iter = 0; iter < total_txns; iter++) begin
        short_mode = iter[0];
        frame_len = 1;
        queued_overlap = (iter != 0) && (($urandom_range(0, 3)) != 0);
        start_delay = $urandom_range(0, 1);
        terminate_after_bytes = 32'hffff_ffff;

        case (iter[1:0])
          2'd0: begin
            run_cross_frame(
              "P118_seed01_clean_mix_100k_cycles",
              "SUPER_LONG_COUNTER_SOAK",
              16'hb000 + iter[15:0],
              short_mode,
              frame_len,
              1'b0,
              1'b0,
              1'b0,
              1'b0,
              start_delay,
              queued_overlap
            );
          end
          2'd1: begin
            run_cross_frame(
              "P120_seed03_error_mix_100k_cycles",
              "SUPER_LONG_COUNTER_SOAK",
              16'hc000 + iter[15:0],
              short_mode,
              frame_len,
              1'b1,
              1'b0,
              1'b0,
              1'b0,
              start_delay,
              queued_overlap
            );
          end
          2'd2: begin
            terminate_after_bytes = payload_byte_count(short_mode, frame_len) / 2;
            run_cross_frame(
              "P121_seed04_stop_mix_100k_cycles",
              "SUPER_LONG_COUNTER_SOAK",
              16'hd000 + iter[15:0],
              short_mode,
              frame_len,
              1'b0,
              1'b0,
              1'b0,
              1'b0,
              start_delay,
              queued_overlap,
              "mid_payload",
              terminate_after_bytes
            );
          end
          default: begin
            terminate_after_bytes = payload_byte_count(short_mode, frame_len) / 2;
            run_cross_frame(
              "P125_seed08_badcrc_mix_100k_cycles",
              "SUPER_LONG_COUNTER_SOAK",
              16'he000 + iter[15:0],
              short_mode,
              frame_len,
              1'b1,
              1'b0,
              1'b0,
              1'b0,
              start_delay,
              queued_overlap,
              "mid_payload",
              terminate_after_bytes
            );
          end
        endcase

        if ((iter != 0) && ((iter % 2048) == 0))
          wait_cycles($urandom_range(1, 4));
      end
    endtask

    task run_phase(uvm_phase phase);
      phase.raise_objection(this);
      wait_for_reset_release();
      run_start_for_mode();
      $display("FRCV_CROSS_ENGINE_V1 plan=%s build=%s effort=%s", cross_plan, build_tag, effort_mode);
      if (cross_plan == "INTERLEAVE_MIX")
        run_interleave_mix();
      else if (cross_plan == "SUPER_LONG_COUNTER_SOAK")
        run_super_long_counter_soak();
      else
        run_good_error_good();
      wait_cycles(12);
      phase.drop_objection(this);
    endtask
  endclass
