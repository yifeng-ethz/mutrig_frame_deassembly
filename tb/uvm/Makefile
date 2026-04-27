include ../../../scripts/questa_one.mk

UVM_DIR       := $(abspath .)
IP_SRC        := $(abspath ../..)
LOG_DIR       := $(UVM_DIR)/logs
COV_DIR       = $(UVM_DIR)/cov_$(RTL_VARIANT)
UVM_HOME      ?= $(QUESTA_UVM_HOME)
UVM_SRC       := $(UVM_HOME)/src
UVM_DPI_SO    := $(QUESTA_UVM_DPI_SO)
UVM_DPI_LIB   := $(basename $(UVM_DPI_SO))
DPI_CPP       ?= /usr/bin/gcc

RTL_VARIANT ?= after
TEST        ?= COMBO_FRCV_001_run_contract_test
CASE_ID     ?=
BUCKET_FRAME_TEST ?= COMBO_FRCV_001_run_contract_test
ALL_BUCKETS_FRAME_TEST ?= COMBO_FRCV_001_run_contract_test
BUILD_TAG ?= CFG_A
BUILD_CHANNEL_WIDTH := 4
BUILD_CSR_ADDR_WIDTH := 2
BUILD_MODE_HALT := 0
BUILD_DEBUG_LV := 0
ifeq ($(BUILD_TAG),CFG_B)
BUILD_MODE_HALT := 1
endif
ifeq ($(BUILD_TAG),CFG_C)
BUILD_CHANNEL_WIDTH := 1
endif
ifeq ($(BUILD_TAG),CFG_D)
BUILD_CHANNEL_WIDTH := 8
endif
ifeq ($(BUILD_TAG),CFG_E)
BUILD_CSR_ADDR_WIDTH := 1
endif
ifeq ($(BUILD_TAG),CFG_F)
BUILD_CSR_ADDR_WIDTH := 8
endif
ifeq ($(BUILD_TAG),CFG_G)
BUILD_DEBUG_LV := 2
endif
CHANNEL_WIDTH ?= $(BUILD_CHANNEL_WIDTH)
CSR_ADDR_WIDTH ?= $(BUILD_CSR_ADDR_WIDTH)
MODE_HALT ?= $(BUILD_MODE_HALT)
DEBUG_LV ?= $(BUILD_DEBUG_LV)
EXTRA_PLUSARGS ?=
RUN_ID_OVERRIDE ?=
WORK_SUFFIX ?=
SEED        ?= 1
VERBOSITY   ?= UVM_MEDIUM
TB_TOP      := tb_top
VCOM_COV    ?= -cover bcesft
VLOG_COV    ?= +cover=bcesft

RTL_DIR = $(IP_SRC)/rtl
SV_DUT_DIR = $(RTL_DIR)/sv_ver/frame_rcv_ip
SVA_DIR = $(UVM_DIR)/sva
RTL_HEAD_PATH ?= rtl/frame_rcv_ip.vhd
RTL_SRC = $(if $(filter before,$(RTL_VARIANT)),$(BUILD_DIR)/frame_rcv_ip.vhd,$(RTL_DIR)/frame_rcv_ip.vhd)
RUN_ID  = $(if $(RUN_ID_OVERRIDE),$(RUN_ID_OVERRIDE),$(if $(CASE_ID),$(CASE_ID),$(TEST)))
TB_CFG_TAG = cw$(CHANNEL_WIDTH)_aw$(CSR_ADDR_WIDTH)_mh$(MODE_HALT)_dl$(DEBUG_LV)
WORK_DIR = $(UVM_DIR)/work_$(RTL_VARIANT)_$(TB_CFG_TAG)$(if $(WORK_SUFFIX),_$(WORK_SUFFIX),)
BUILD_DIR = $(WORK_DIR)/build

UVM_SRCS = \
	$(UVM_DIR)/frcv_if.sv \
	$(UVM_DIR)/frcv_cases.svh \
	$(UVM_DIR)/frcv_env_pkg.sv \
	$(SV_DUT_DIR)/frame_rcv_ip_dut_sv.sv \
	$(SVA_DIR)/frcv_parser_boundary_sva.sv \
	$(SVA_DIR)/frcv_output_contract_sva.sv \
	$(SVA_DIR)/frcv_counter_contract_sva.sv \
	$(UVM_DIR)/tb_top.sv

.PHONY: license_check compile run run_sim run_case run_case_list run_cross run_isolated run_bucket_frame run_all_buckets_frame cov_report_case cov_merge cov_report_total run_before_expected_fail run_after prove_delta clean

license_check:
	@echo "═══ Checking Mentor floating license at $(QUESTA_LICENSE_SERVER) ═══"
	@$(LMUTIL) lmstat -f mtiverification -c $(QUESTA_LICENSE_SERVER) >/dev/null 2>&1 || \
	  { echo "ERROR: mtiverification feature not visible via $(QUESTA_LICENSE_SERVER)"; exit 1; }

compile: license_check $(WORK_DIR)/.compiled

$(WORK_DIR)/.compiled: $(RTL_DIR)/crc16_calc.vhd $(RTL_DIR)/frame_rcv_ip.vhd $(UVM_SRCS)
	@echo "═══ Compiling frame_rcv_ip UVM bench ($(RTL_VARIANT)) cfg=$(TB_CFG_TAG) build=$(BUILD_TAG) ═══"
	@rm -rf $(WORK_DIR)
	@mkdir -p $(WORK_DIR) $(BUILD_DIR) $(LOG_DIR) $(COV_DIR)
	@cp $(QSIM_INI) $(WORK_DIR)/modelsim.ini
	@chmod u+w $(WORK_DIR)/modelsim.ini
	@if [ "$(RTL_VARIANT)" = "before" ]; then \
	  git -C $(IP_SRC) show HEAD:$(RTL_HEAD_PATH) > $(BUILD_DIR)/frame_rcv_ip.vhd; \
	fi
	@cd $(WORK_DIR) && $(VLIB) work
	@cd $(WORK_DIR) && $(VMAP) -modelsimini modelsim.ini work work > /dev/null
	@cd $(WORK_DIR) && $(VCOM) -2008 $(VCOM_COV) -modelsimini modelsim.ini -work work $(RTL_DIR)/crc16_calc.vhd
	@cd $(WORK_DIR) && $(VCOM) -2008 $(VCOM_COV) -modelsimini modelsim.ini -work work $(RTL_SRC)
	@cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work $(VLOG_COV) -sv $(UVM_DIR)/frcv_if.sv
	@cd $(WORK_DIR) && $(VLOG) -modelsimini modelsim.ini -work work -mfcu \
	  +define+TB_CHANNEL_WIDTH=$(CHANNEL_WIDTH) \
	  +define+TB_CSR_ADDR_WIDTH=$(CSR_ADDR_WIDTH) \
	  +define+TB_MODE_HALT=$(MODE_HALT) \
	  +define+TB_DEBUG_LV=$(DEBUG_LV) \
	  $(VLOG_COV) +incdir+$(UVM_SRC) +incdir+$(UVM_DIR) -sv \
	  $(UVM_SRC)/uvm_pkg.sv \
	  $(UVM_DIR)/frcv_env_pkg.sv \
	  $(SV_DUT_DIR)/frame_rcv_ip_dut_sv.sv \
	  $(SVA_DIR)/frcv_parser_boundary_sva.sv \
	  $(SVA_DIR)/frcv_output_contract_sva.sv \
	  $(SVA_DIR)/frcv_counter_contract_sva.sv \
	  $(UVM_DIR)/tb_top.sv
	@touch $@

run: compile run_sim

run_case:
	@$(MAKE) -s run TEST=frcv_doc_case_test CASE_ID=$(CASE_ID) SEED=$(SEED)

run_case_list:
	@$(MAKE) -s run TEST=frcv_doc_case_list_test SEED=$(SEED)

run_cross:
	@$(MAKE) -s run TEST=frcv_cross_bucket_test SEED=$(SEED)

run_sim:
	@echo "=== Running $(RUN_ID) test=$(TEST) variant=$(RTL_VARIANT) build=$(BUILD_TAG) cfg=$(TB_CFG_TAG) seed=$(SEED) ==="
	@mkdir -p $(COV_DIR)
	@rm -f $(COV_DIR)/$(RUN_ID)_s$(SEED).ucdb
	@cd $(WORK_DIR) && $(VSIM) -modelsimini modelsim.ini -c -coverage \
	  -dpicpppath $(DPI_CPP) -sv_lib $(UVM_DPI_LIB) \
	  -sv_seed $(SEED) \
	  +UVM_TESTNAME=$(TEST) +UVM_VERBOSITY=$(VERBOSITY) \
	  $(if $(CASE_ID),+FRCV_CASE_ID=$(CASE_ID),) +FRCV_BUILD_TAG=$(BUILD_TAG) $(EXTRA_PLUSARGS) \
	  -suppress 3009 -suppress 3473 -voptargs="+acc" \
	  work.$(TB_TOP) \
	  -do "quietly set NumericStdNoWarnings 1; quietly set StdArithNoWarnings 1; coverage save -onexit $(COV_DIR)/$(RUN_ID)_s$(SEED).ucdb; run -all; quit -f" \
	  | tee $(LOG_DIR)/$(RUN_ID)_$(RTL_VARIANT)_s$(SEED).log
	@if grep -qE '^\# UVM_(FATAL|ERROR) /|^\# \*\* Fatal:|^\# \*\* Error:' $(LOG_DIR)/$(RUN_ID)_$(RTL_VARIANT)_s$(SEED).log; then \
	  echo "FAIL: $(RUN_ID) variant=$(RTL_VARIANT)"; exit 1; \
	elif ! grep -q '\*\*\* TEST PASSED \*\*\*' $(LOG_DIR)/$(RUN_ID)_$(RTL_VARIANT)_s$(SEED).log; then \
	  echo "FAIL: $(RUN_ID) variant=$(RTL_VARIANT)"; exit 1; \
	else \
	  echo "PASS: $(RUN_ID) variant=$(RTL_VARIANT)"; \
	fi

run_isolated:
	@$(MAKE) -s run RTL_VARIANT=$(RTL_VARIANT) TEST=$(TEST) SEED=$(SEED)

run_bucket_frame:
	@$(MAKE) -s run RTL_VARIANT=$(RTL_VARIANT) TEST=$(BUCKET_FRAME_TEST) SEED=$(SEED)

run_all_buckets_frame:
	@$(MAKE) -s run RTL_VARIANT=$(RTL_VARIANT) TEST=$(ALL_BUCKETS_FRAME_TEST) SEED=$(SEED)

cov_report_case:
	@$(VCOVER) report -details -code bcesft $(COV_DIR)/$(RUN_ID)_s$(SEED).ucdb | tee $(COV_DIR)/$(RUN_ID)_s$(SEED).txt

cov_merge:
	@rm -f $(COV_DIR)/merged.ucdb
	@$(VCOVER) merge $(COV_DIR)/merged.ucdb $(COV_DIR)/*.ucdb

cov_report_total: cov_merge
	@$(VCOVER) report -details -code bcesft $(COV_DIR)/merged.ucdb | tee $(COV_DIR)/merged.txt

run_before_expected_fail:
	@if $(MAKE) -s run RTL_VARIANT=before TEST=$(TEST) SEED=$(SEED); then \
	  echo "ERROR: before variant unexpectedly passed"; exit 1; \
	else \
	  echo "Observed expected failure for before variant"; \
	fi

run_after:
	@$(MAKE) -s run RTL_VARIANT=after TEST=$(TEST) SEED=$(SEED)

prove_delta: run_before_expected_fail run_after

clean:
	rm -rf $(UVM_DIR)/work_* $(LOG_DIR) $(UVM_DIR)/cov_* transcript vsim.wlf
