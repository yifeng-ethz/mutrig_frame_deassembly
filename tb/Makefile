include ../../scripts/questa_one.mk

IP_SRC   = $(realpath ..)
WORK     = work_frame_rcv
TB_TOP   = tb_frame_rcv_ip

.PHONY: license_check compile run run_all clean

license_check:
	@echo "═══ Checking Mentor floating license at $(QUESTA_LICENSE_SERVER) ═══"
	@$(LMUTIL) lmstat -f mtiverification -c $(QUESTA_LICENSE_SERVER) >/dev/null 2>&1 || \
	  { echo "ERROR: mtiverification feature not visible via $(QUESTA_LICENSE_SERVER)"; exit 1; }

compile: license_check $(WORK)/_vmake

$(WORK)/_vmake: $(IP_SRC)/rtl/crc16_calc.vhd \
                $(IP_SRC)/rtl/frame_rcv_ip.vhd \
                tb_frame_rcv_ip.vhd
	rm -rf $(WORK)
	$(VLIB) $(WORK)
	$(VMAP) $(WORK) $(WORK)
	$(VCOM) -2008 -work $(WORK) $(IP_SRC)/rtl/crc16_calc.vhd
	$(VCOM) -2008 -work $(WORK) $(IP_SRC)/rtl/frame_rcv_ip.vhd
	$(VCOM) -2008 -work $(WORK) tb_frame_rcv_ip.vhd
	@touch $@

run: compile
	$(VSIM) -c -do "run -all; quit -f" -work $(WORK) $(TB_TOP)

run_all: run

clean:
	rm -rf $(WORK) transcript vsim.wlf
