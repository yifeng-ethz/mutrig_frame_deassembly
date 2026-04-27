include ../../../scripts/questa_one.mk

QVERIFY ?= $(firstword $(wildcard $(QUESTA_HOME)/bin/qverify $(QUESTA_HOME)/linux_x86_64/qverify))

FORMAL_DIR := $(abspath .)
TB_DIR     := $(abspath ..)
IP_SRC     := $(abspath ../..)
WORK_DIR   := $(FORMAL_DIR)/work_formal
RTL_DIR    := $(IP_SRC)/rtl
SV_DUT_DIR := $(RTL_DIR)/sv_ver/frame_rcv_ip
SVA_DIR    := $(TB_DIR)/uvm/sva
OSS_DIR    := $(FORMAL_DIR)/oss

FORMAL_SRCS = \
	$(SV_DUT_DIR)/frame_rcv_ip_dut_sv.sv \
	$(SVA_DIR)/frcv_parser_boundary_sva.sv \
	$(SVA_DIR)/frcv_output_contract_sva.sv \
	$(SVA_DIR)/frcv_counter_contract_sva.sv \
	$(FORMAL_DIR)/frcv_formal_pkt_assumptions.sv \
	$(FORMAL_DIR)/frcv_formal_top.sv

.PHONY: compile znformal_ready oss_crc_prove oss_dut_crc_counter_prove clean oss_clean

znformal_ready:
	@if [ -z "$(QVERIFY)" ]; then \
	  echo "ERROR: qverify/ZnFormal is required now but was not found under $(QUESTA_HOME)."; \
	  echo "Set QVERIFY_BIN or install the Questa ZnFormal executable into the active QuestaOne tree."; \
	  exit 2; \
	fi

compile: $(WORK_DIR)/.compiled

$(WORK_DIR)/.compiled: $(RTL_DIR)/crc16_calc.vhd $(RTL_DIR)/frame_rcv_ip.vhd $(FORMAL_SRCS)
	@rm -rf $(WORK_DIR)
	@mkdir -p $(WORK_DIR)
	@cp $(QSIM_INI) $(WORK_DIR)/modelsim.ini
	@chmod u+w $(WORK_DIR)/modelsim.ini
	@cd $(WORK_DIR) && $(VLIB) work
	@cd $(WORK_DIR) && $(VMAP) -modelsimini modelsim.ini work work > /dev/null
	@cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work work $(RTL_DIR)/crc16_calc.vhd
	@cd $(WORK_DIR) && $(VCOM) -2008 -modelsimini modelsim.ini -work work $(RTL_DIR)/frame_rcv_ip.vhd
	@cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -sv -mfcu \
	  $(SV_DUT_DIR)/frame_rcv_ip_dut_sv.sv \
	  $(SVA_DIR)/frcv_parser_boundary_sva.sv \
	  $(SVA_DIR)/frcv_output_contract_sva.sv \
	  $(SVA_DIR)/frcv_counter_contract_sva.sv \
	  $(FORMAL_DIR)/frcv_formal_pkt_assumptions.sv \
	  $(FORMAL_DIR)/frcv_formal_top.sv
	@touch $@

oss_crc_prove:
	@echo "ERROR: OSS Yosys/SBY proofs are deprecated for this IP." >&2
	@echo "Use Questa ZnFormal (`qverify`) with the active QuestaOne runtime instead." >&2
	@exit 2

oss_dut_crc_counter_prove:
	@echo "ERROR: OSS Yosys/SBY proofs are deprecated for this IP." >&2
	@echo "Use Questa ZnFormal (`qverify`) with the active QuestaOne runtime instead." >&2
	@exit 2

clean:
	rm -rf $(WORK_DIR) transcript vsim.wlf

oss_clean:
	rm -rf $(OSS_DIR)/crc16_calc_residue $(OSS_DIR)/frame_rcv_ip_crc_counter
