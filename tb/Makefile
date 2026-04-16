QUESTA_HOME ?= /data1/intelFPGA_pro/23.1/questa_fse
VLIB        = $(QUESTA_HOME)/bin/vlib
VMAP        = $(QUESTA_HOME)/bin/vmap
VCOM        = $(QUESTA_HOME)/bin/vcom
VSIM        = $(QUESTA_HOME)/bin/vsim
LMUTIL      = $(QUESTA_HOME)/linux_x86_64/lmutil

QUESTA_LICENSE := $(QUESTA_HOME)/LR-287689_License.dat
ETH_LIC_SERVER := 8161@lic-mentor.ethz.ch

export LM_LICENSE_FILE   := $(ETH_LIC_SERVER):$(QUESTA_LICENSE)
export MGLS_LICENSE_FILE := $(LM_LICENSE_FILE)

IP_SRC   = $(realpath ..)
WORK     = work_frame_rcv
TB_TOP   = tb_frame_rcv_ip

.PHONY: license_check compile run run_all clean

license_check:
	@echo "═══ Checking Mentor floating license at $(ETH_LIC_SERVER) ═══"
	@$(LMUTIL) lmstat -f mtiverification -c $(ETH_LIC_SERVER) >/dev/null 2>&1 || \
	  { echo "ERROR: mtiverification feature not visible via $(ETH_LIC_SERVER)"; exit 1; }

compile: license_check $(WORK)/_vmake

$(WORK)/_vmake: $(IP_SRC)/crc16_calc.vhd \
                $(IP_SRC)/frame_rcv_ip.vhd \
                tb_frame_rcv_ip.vhd
	rm -rf $(WORK)
	$(VLIB) $(WORK)
	$(VMAP) $(WORK) $(WORK)
	$(VCOM) -2008 -work $(WORK) $(IP_SRC)/crc16_calc.vhd
	$(VCOM) -2008 -work $(WORK) $(IP_SRC)/frame_rcv_ip.vhd
	$(VCOM) -2008 -work $(WORK) tb_frame_rcv_ip.vhd
	@touch $@

run: compile
	$(VSIM) -c -do "run -all; quit -f" -work $(WORK) $(TB_TOP)

run_all: run

clean:
	rm -rf $(WORK) transcript vsim.wlf
