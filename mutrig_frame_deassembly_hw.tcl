

################################################
# mutrig_frame_assembly "MuTRiG Frame Deassembly " v24.0.1021
# Yifeng Wang 2024.10.21
# Deassemble the MuTRiG frame from lvds receiver byte stream into header, data and trailer (with crc checks). 
################################################

################################################
# request TCL package from ACDS 16.1
################################################
package require -exact qsys 16.1


################################################
# module mutrig_frame_deassembly
################################################ 
set_module_property DESCRIPTION "Deassemble the MuTRiG frame from lvds receiver byte stream into header, data and trailer (with crc checks)"
set_module_property NAME mutrig_frame_deassembly
set_module_property VERSION 24.0.1021
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Mu3e Data Plane/Modules"
set_module_property AUTHOR "Yifeng Wang"
set_module_property ICON_PATH ../figures/mu3e_logo.png
set_module_property DISPLAY_NAME "MuTRiG Frame Deassembly "
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE false
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false
set_module_property ELABORATION_CALLBACK myelaborate


################################################
# file sets
################################################ 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL frame_rcv_ip
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file frame_rcv_ip.vhd VHDL PATH frame_rcv_ip.vhd TOP_LEVEL_FILE
add_fileset_file crc16_calc.vhd VHDL PATH crc16_calc.vhd


################################################
# parameters
################################################
add_parameter MODE_HALT NATURAL 0
set_parameter_property MODE_HALT DISPLAY_NAME "Error handling mode"
set_parameter_property MODE_HALT ALLOWED_RANGES {"0:stop" "1:resume"}
set_parameter_property MODE_HALT HDL_PARAMETER true
set dscpt \
"<html>
Select the error handling strategy of the <b><i>frame_rcv</i></b> in the lane corrupt indidence.<br>
Lane corrupt is a critical error, but with the help of new feature of adaptive byte boundary of LVDS controller, fast recovery of accidental bit error is possible. <br>
The <b>resume</b> option can be benefitial with the mentioned feature, if you are operating on a lane with larger phase noise and willing to get most event out of it. <br>
<ul>
	<li><b>stop</b> : stop parsing the current frame, (\"idle mode\") go back to idle state <br> </li>
	<li><b>resume</b> : resume parsing the current frame, after criticl error flag is removed by upstream <br> </li>
	
</ul>
<b>Note</b>: \"parity error\" and \"decode error\" are handled independently of this parameter, as they are translated into \"hit error\" and \"crc error\" accordingly. <br> 
</html>"
set_parameter_property MODE_HALT DESCRIPTION $dscpt
set_parameter_property MODE_HALT LONG_DESCRIPTION $dscpt

add_parameter DEBUG_LV NATURAL 
set_parameter_property DEBUG_LV DEFAULT_VALUE 0
set_parameter_property DEBUG_LV DISPLAY_NAME "Debug level"
set_parameter_property DEBUG_LV TYPE NATURAL
set_parameter_property DEBUG_LV UNITS None
set_parameter_property DEBUG_LV ALLOWED_RANGES {0 1 2}
set_parameter_property DEBUG_LV HDL_PARAMETER true
set dscpt \
"<html>
Select the debug level of the IP (affects generation).<br>
<ul>
	<li><b>0</b> : off <br> </li>
	<li><b>1</b> : on, synthesizble <br> </li>
	<li><b>2</b> : on, non-synthesizble, simulation-only <br> </li>
</ul>
</html>"
set_parameter_property DEBUG_LV DESCRIPTION $dscpt
set_parameter_property DEBUG_LV LONG_DESCRIPTION $dscpt

add_parameter CHANNEL_WIDTH NATURAL 4
set_parameter_property CHANNEL_WIDTH DISPLAY_NAME "Channel width of the avst <rx8b1k>"
set_parameter_property CHANNEL_WIDTH ALLOWED_RANGES {1:4}
set_parameter_property CHANNEL_WIDTH HDL_PARAMETER true
set_parameter_property CHANNEL_WIDTH UNITS bits
set dscpt \
"<html>
Enter the channel width of avst <b>rx8b1k</b> interface.<br>
Must be larger than 0. <br>
</html>"
set_parameter_property CHANNEL_WIDTH DESCRIPTION $dscpt
set_parameter_property CHANNEL_WIDTH LONG_DESCRIPTION $dscpt

add_parameter CSR_ADDR_WIDTH NATURAL 2
set_parameter_property CSR_ADDR_WIDTH DISPLAY_NAME "Width of the address of the avmm <csr>"
set_parameter_property CSR_ADDR_WIDTH ALLOWED_RANGES {1:8}
set_parameter_property CSR_ADDR_WIDTH HDL_PARAMETER true
set_parameter_property CSR_ADDR_WIDTH UNITS bits
set dscpt \
"<html>
Enter the address signal width of avmm <b>csr</b> interface.<br>
Must be larger than 0. <br>
</html>"
set_parameter_property CSR_ADDR_WIDTH DESCRIPTION $dscpt
set_parameter_property CSR_ADDR_WIDTH LONG_DESCRIPTION $dscpt



################################################
# display items
################################################
add_display_item "" "IP Setting" GROUP ""
add_display_item "IP Setting" CHANNEL_WIDTH PARAMETER 
add_display_item "IP Setting" CSR_ADDR_WIDTH PARAMETER 

add_display_item "" "Error Handling" GROUP ""
add_display_item "Error Handling" MODE_HALT PARAMETER 

add_display_item "" "Generation setting" GROUP ""
add_display_item "Generation setting" DEBUG_LV PARAMETER 

add_display_item "" "Description" GROUP ""
set dscpt \
"<html>
Data flow: 
<ul>
    <li><b>ingress</b> : upstream to avst <b>rx8b1k</b> interface. <br> </li>
    <li><b>egress</b> : avst <b>hit_type0</b> interface to downstream. <br> </li>
</ul>
<br>

Data (sideband) flow: 
<ul>
    <li><b>egress</b> : avst <b>headerinfo</b> interface to debug module. <br> </li>
</ul>
<br>

Control flow:
<ul>
    <li><b>timing critical</b> : run management host to avst <b>ctrl</b> interface. <br> </li>
    <li><b>non-timing critical</b> : avalon-mm host to avmm <b>csr</b> interface. <br> </li>
</ul>
<br>

Clock Domain: <br>
<ul>
    <li> Connect <b>clock_sink</b> to the outclock of the <b>LVDS RX IP</b> </li>
</ul>
<br> 

</html>"
add_display_item "Description" "dscpt" TEXT $dscpt

################################################
# connection point rx8b1k
################################################
add_interface rx8b1k avalon_streaming end
set_interface_property rx8b1k associatedClock clock_sink
set_interface_property rx8b1k associatedReset reset_sink
set_interface_property rx8b1k dataBitsPerSymbol 9
set_interface_property rx8b1k errorDescriptor "loss_sync_pattern parity_error decode_error"
set_interface_property rx8b1k firstSymbolInHighOrderBits true

set_interface_property rx8b1k readyLatency 0
set_interface_property rx8b1k ENABLED true

add_interface_port rx8b1k asi_rx8b1k_data data Input 9
add_interface_port rx8b1k asi_rx8b1k_valid valid Input 1

add_interface_port rx8b1k asi_rx8b1k_error error Input 3


# 
# connection point hit_type0
# 
add_interface hit_type0 avalon_streaming start
set_interface_property hit_type0 associatedClock clock_sink
set_interface_property hit_type0 associatedReset reset_sink
set_interface_property hit_type0 dataBitsPerSymbol 45
set_interface_property hit_type0 errorDescriptor ""
set_interface_property hit_type0 firstSymbolInHighOrderBits true
set_interface_property hit_type0 maxChannel 15
set_interface_property hit_type0 readyLatency 0
set_interface_property hit_type0 ENABLED true
set_interface_property hit_type0 EXPORT_OF ""
set_interface_property hit_type0 PORT_NAME_MAP ""
set_interface_property hit_type0 CMSIS_SVD_VARIABLES ""
set_interface_property hit_type0 SVD_ADDRESS_GROUP ""


add_interface_port hit_type0 aso_hit_type0_startofpacket startofpacket Output 1
add_interface_port hit_type0 aso_hit_type0_endofpacket endofpacket Output 1
add_interface_port hit_type0 aso_hit_type0_error error Output 3
add_interface_port hit_type0 aso_hit_type0_data data Output 45
add_interface_port hit_type0 aso_hit_type0_valid valid Output 1


# 
# connection point headerinfo
# 
add_interface headerinfo avalon_streaming start
set_interface_property headerinfo associatedClock clock_sink
set_interface_property headerinfo associatedReset reset_sink
set_interface_property headerinfo dataBitsPerSymbol 42
set_interface_property headerinfo errorDescriptor ""
set_interface_property headerinfo firstSymbolInHighOrderBits true
set_interface_property headerinfo maxChannel 0
set_interface_property headerinfo readyLatency 0
set_interface_property headerinfo ENABLED true
set_interface_property headerinfo EXPORT_OF ""
set_interface_property headerinfo PORT_NAME_MAP ""
set_interface_property headerinfo CMSIS_SVD_VARIABLES ""
set_interface_property headerinfo SVD_ADDRESS_GROUP ""

add_interface_port headerinfo aso_headerinfo_data data Output 42
add_interface_port headerinfo aso_headerinfo_valid valid Output 1
add_interface_port headerinfo aso_headerinfo_channel channel Output 4


# 
# connection point csr
# 
add_interface csr avalon end
set_interface_property csr addressUnits WORDS
set_interface_property csr associatedClock clock_sink
set_interface_property csr associatedReset reset_sink
set_interface_property csr bitsPerSymbol 8
set_interface_property csr burstOnBurstBoundariesOnly false
set_interface_property csr burstcountUnits WORDS
set_interface_property csr explicitAddressSpan 0
set_interface_property csr holdTime 0
set_interface_property csr linewrapBursts false
set_interface_property csr maximumPendingReadTransactions 0
set_interface_property csr maximumPendingWriteTransactions 0
set_interface_property csr readLatency 0
set_interface_property csr readWaitTime 1
set_interface_property csr setupTime 0
set_interface_property csr timingUnits Cycles
set_interface_property csr writeWaitTime 0
set_interface_property csr ENABLED true
set_interface_property csr EXPORT_OF ""
set_interface_property csr PORT_NAME_MAP ""
set_interface_property csr CMSIS_SVD_VARIABLES ""
set_interface_property csr SVD_ADDRESS_GROUP ""

add_interface_port csr avs_csr_read read Input 1
add_interface_port csr avs_csr_readdata readdata Output 32
add_interface_port csr avs_csr_waitrequest waitrequest Output 1
add_interface_port csr avs_csr_write write Input 1
add_interface_port csr avs_csr_writedata writedata Input 32
set_interface_assignment csr embeddedsw.configuration.isFlash 0
set_interface_assignment csr embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment csr embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment csr embeddedsw.configuration.isPrintableDevice 0


# 
# connection point ctrl
# 
add_interface ctrl avalon_streaming end
set_interface_property ctrl associatedClock clock_sink
set_interface_property ctrl associatedReset reset_sink
set_interface_property ctrl dataBitsPerSymbol 9
set_interface_property ctrl errorDescriptor ""
set_interface_property ctrl firstSymbolInHighOrderBits true
set_interface_property ctrl maxChannel 0
set_interface_property ctrl readyLatency 0
set_interface_property ctrl ENABLED true
set_interface_property ctrl EXPORT_OF ""
set_interface_property ctrl PORT_NAME_MAP ""
set_interface_property ctrl CMSIS_SVD_VARIABLES ""
set_interface_property ctrl SVD_ADDRESS_GROUP ""

add_interface_port ctrl asi_ctrl_data data Input 9
add_interface_port ctrl asi_ctrl_valid valid Input 1
add_interface_port ctrl asi_ctrl_ready ready Output 1


# 
# connection point clock_sink
# 
add_interface clock_sink clock end
set_interface_property clock_sink clockRate 0
set_interface_property clock_sink ENABLED true
set_interface_property clock_sink EXPORT_OF ""
set_interface_property clock_sink PORT_NAME_MAP ""
set_interface_property clock_sink CMSIS_SVD_VARIABLES ""
set_interface_property clock_sink SVD_ADDRESS_GROUP ""

add_interface_port clock_sink i_clk clk Input 1


# 
# connection point reset_sink
# 
add_interface reset_sink reset end
set_interface_property reset_sink associatedClock clock_sink
set_interface_property reset_sink synchronousEdges BOTH
set_interface_property reset_sink ENABLED true
set_interface_property reset_sink EXPORT_OF ""
set_interface_property reset_sink PORT_NAME_MAP ""
set_interface_property reset_sink CMSIS_SVD_VARIABLES ""
set_interface_property reset_sink SVD_ADDRESS_GROUP ""

add_interface_port reset_sink i_rst reset Input 1





proc myelaborate {} {
    # -- 
    # some elaboration of the port width 
    add_interface_port csr avs_csr_address address Input [get_parameter_value CSR_ADDR_WIDTH]
    add_interface_port rx8b1k asi_rx8b1k_channel channel Input [get_parameter_value CHANNEL_WIDTH]
    add_interface_port hit_type0 aso_hit_type0_channel channel Output [get_parameter_value CHANNEL_WIDTH]
    # and their attribute
    set_interface_property rx8b1k maxChannel [expr 2**[get_parameter_value CHANNEL_WIDTH]-1]
    set_interface_property hit_type0 maxChannel [expr 2**[get_parameter_value CHANNEL_WIDTH]-1]
    set_interface_property headerinfo maxChannel [expr 2**[get_parameter_value CHANNEL_WIDTH]-1]
    
    return -code ok
}

