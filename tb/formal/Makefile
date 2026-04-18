QUESTA_HOME ?= /data1/intelFPGA_pro/23.1/questa_fse
VLIB        = $(QUESTA_HOME)/bin/vlib
VMAP        = $(QUESTA_HOME)/bin/vmap
VCOM        = $(QUESTA_HOME)/bin/vcom
VLOG        = $(QUESTA_HOME)/bin/vlog
OSS_FORMAL_HOME ?= /data1/oss_formal
SBY         = $(OSS_FORMAL_HOME)/bin/sby

QUESTA_LICENSE := $(QUESTA_HOME)/LR-287689_License.dat
ETH_LIC_SERVER := 8161@lic-mentor.ethz.ch

export LM_LICENSE_FILE   := $(ETH_LIC_SERVER):$(QUESTA_LICENSE)
export MGLS_LICENSE_FILE := $(LM_LICENSE_FILE)

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

.PHONY: compile oss_crc_prove oss_dut_crc_counter_prove clean oss_clean

compile: $(WORK_DIR)/.compiled

$(WORK_DIR)/.compiled: $(RTL_DIR)/crc16_calc.vhd $(RTL_DIR)/frame_rcv_ip.vhd $(FORMAL_SRCS)
	@rm -rf $(WORK_DIR)
	@mkdir -p $(WORK_DIR)
	@cd $(WORK_DIR) && $(VLIB) work
	@cd $(WORK_DIR) && $(VMAP) -c > /dev/null
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
	@. $(OSS_FORMAL_HOME)/activate.sh && \
	  cd $(OSS_DIR) && \
	  $(SBY) -f crc16_calc_residue.sby

oss_dut_crc_counter_prove:
	@. $(OSS_FORMAL_HOME)/activate.sh && \
	  cd $(OSS_DIR) && \
	  $(SBY) -f frame_rcv_ip_crc_counter.sby

clean:
	rm -rf $(WORK_DIR) transcript vsim.wlf

oss_clean:
	rm -rf $(OSS_DIR)/crc16_calc_residue $(OSS_DIR)/frame_rcv_ip_crc_counter
