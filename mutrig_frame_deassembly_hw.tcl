# mutrig_frame_deassembly_hw.tcl
# Platform Designer component for the MuTRiG frame receiver / deassembly IP

package require -exact qsys 16.1

# ========================================================================
# Packaging constants
# ========================================================================
set SCRIPT_DIR [file dirname [info script]]
if {[string length $SCRIPT_DIR] == 0} {
    set SCRIPT_DIR [pwd]
}

set DEFAULT_CHANNEL_WIDTH_CONST    4
set DEFAULT_CSR_ADDR_WIDTH_CONST   2

set IP_UID_DEFAULT_CONST           1179804502 ;# ASCII "FRCV" = 0x46524356
set VERSION_MAJOR_DEFAULT_CONST    26
set VERSION_MINOR_DEFAULT_CONST    0
set VERSION_PATCH_DEFAULT_CONST    4
set BUILD_DEFAULT_CONST            416
set VERSION_DATE_DEFAULT_CONST     20260416
set VERSION_GIT_DEFAULT_CONST      0
set VERSION_GIT_SHORT_DEFAULT_CONST "unknown"
set VERSION_GIT_DESCRIBE_DEFAULT_CONST "unknown"
if {![catch {set VERSION_GIT_SHORT_DEFAULT_CONST [string trim [exec git -C $SCRIPT_DIR rev-parse --short HEAD]]}]} {
    if {[regexp {^[0-9a-fA-F]+$} $VERSION_GIT_SHORT_DEFAULT_CONST]} {
        scan $VERSION_GIT_SHORT_DEFAULT_CONST %x VERSION_GIT_DEFAULT_CONST
    }
}
catch {
    set VERSION_GIT_DESCRIBE_DEFAULT_CONST [string trim [exec git -C $SCRIPT_DIR describe --always --dirty --tags]]
}
set VERSION_GIT_HEX_DEFAULT_CONST [format "0x%08X" $VERSION_GIT_DEFAULT_CONST]
set VERSION_STRING_DEFAULT_CONST  [format "%d.%d.%d.%04d" \
    $VERSION_MAJOR_DEFAULT_CONST \
    $VERSION_MINOR_DEFAULT_CONST \
    $VERSION_PATCH_DEFAULT_CONST \
    $BUILD_DEFAULT_CONST]
set INSTANCE_ID_DEFAULT_CONST     0

# ========================================================================
# Module properties
# ========================================================================
set_module_property NAME                         mutrig_frame_deassembly
set_module_property DISPLAY_NAME                 "MuTRiG Frame Deassembly"
set_module_property VERSION                      $VERSION_STRING_DEFAULT_CONST
set_module_property DESCRIPTION                  "MuTRiG Frame Deassembly Mu3e IP Core"
set_module_property GROUP                        "Mu3e Data Plane/Modules"
set_module_property AUTHOR                       "Yifeng Wang"
set_module_property ICON_PATH                    ../quartus_system/logo/mu3e_logo.png
set_module_property INTERNAL                     false
set_module_property OPAQUE_ADDRESS_MAP           true
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE                     true
set_module_property REPORT_TO_TALKBACK           false
set_module_property ALLOW_GREYBOX_GENERATION     false
set_module_property REPORT_HIERARCHY             false
set_module_property ELABORATION_CALLBACK         elaborate
set_module_property VALIDATION_CALLBACK          validate

# ========================================================================
# Helper
# ========================================================================
proc add_html_text {group_name item_name html_text} {
    add_display_item $group_name $item_name TEXT ""
    set_display_item_property $item_name DISPLAY_HINT html
    set_display_item_property $item_name TEXT $html_text
}

proc compute_derived_values {} {
    set channel_width [get_parameter_value CHANNEL_WIDTH]
    set csr_addr_w    [get_parameter_value CSR_ADDR_WIDTH]
    set version_string [format "%d.%d.%d.%04d" \
        [get_parameter_value VERSION_MAJOR] \
        [get_parameter_value VERSION_MINOR] \
        [get_parameter_value VERSION_PATCH] \
        [get_parameter_value BUILD]]
    set version_git_hex [format "0x%08X" [get_parameter_value VERSION_GIT]]

    catch {
        set_display_item_property overview_html TEXT [format {<html>\
<b>Function</b><br/>\
Receives the MuTRiG 9-bit decoded byte stream and reconstructs <b>hit_type0</b> words plus\
frame metadata for downstream timestamp processing.<br/><br/>\
<b>Data path</b><br/>\
<b>rx8b1k</b> &rarr; frame parser / CRC checker &rarr; <b>hit_type0</b> + <b>headerinfo</b><br/><br/>\
<b>Run-state contract</b><br/>\
<b>RUNNING</b> opens new header detection, subject to <b>csr.control[0]</b>.<br/>\
<b>TERMINATING</b> blocks fresh header pickup, keeps the active parser state draining, and\
only acknowledges after the parser is idle. If terminate arrives while already idle, the IP\
emits one dedicated <b>hit_type0_endofrun</b> pulse after local drain completion so downstream\
logic still sees a terminal run boundary without fabricating a payload beat.<br/>\
<b>IDLE</b> is quiescent in this packaged revision; the legacy \"monitor while idle\" behavior is\
not part of the delivered contract.<br/><br/>\
<b>Parameterized widths</b><br/>\
Lane channel width: <b>%d</b> bits. CSR address width: <b>%d</b> bits.</html>} \
            $channel_width \
            $csr_addr_w]
    }
    catch {
        set_display_item_property error_mode_html TEXT {<html>\
<b>Lane-corrupt handling</b><br/>\
<b>MODE_HALT=0</b>: resume parsing once the upstream lane-corrupt condition clears.<br/>\
<b>MODE_HALT=1</b>: abandon the current frame and return the parser to idle.<br/><br/>\
Parity and decode errors are still translated into hit / CRC sideband error reporting independently of this mode.</html>}
    }
    catch {
        set_display_item_property debug_html TEXT {<html>\
<b>DEBUG_LV</b><br/>\
<b>0</b> off, <b>1</b> synthesizable debug visibility, <b>2</b> simulation-only instrumentation.</html>}
    }
    catch {
        set_display_item_property profile_html TEXT [format {<html>\
<b>Catalog revision</b><br/>\
Packaged as <b>%s</b>.<br/><br/>\
<b>Delivered behavior</b><br/>\
This image hardens the run-sequence boundary: <b>asi_ctrl_ready</b> is stateful, <b>IDLE</b> is\
quiescent, and <b>TERMINATING</b> closes only after the local parser has finished or emitted the\
dedicated idle-close <b>endofrun</b> pulse.<br/><br/>\
<b>Packaging provenance</b><br/>\
Default git stamp <b>%s</b> (%s). Git describe: <b>%s</b>.</html>} \
            $version_string \
            $version_git_hex \
            $::VERSION_GIT_SHORT_DEFAULT_CONST \
            $::VERSION_GIT_DESCRIBE_DEFAULT_CONST]
    }
    catch {
        set_display_item_property versioning_html TEXT [format {<html>\
<b>VERSION encoding</b><br/>\
VERSION[31:24] = MAJOR, VERSION[23:16] = MINOR, VERSION[15:12] = PATCH, VERSION[11:0] = BUILD.<br/><br/>\
<b>Catalog identity</b><br/>\
UID default is <b>FRCV</b> (0x46524356).<br/>\
Default <b>VERSION_GIT</b> = <b>%s</b> (%s). Git describe = <b>%s</b>.<br/>\
Enable <b>Override Git Stamp</b> to enter a custom value.<br/><br/>\
<b>Editability</b><br/>\
<b>IP_UID</b> and <b>INSTANCE_ID</b> remain integration-editable; version/build/date fields are locked to the packaged image.</html>} \
            $version_git_hex \
            $::VERSION_GIT_SHORT_DEFAULT_CONST \
            $::VERSION_GIT_DESCRIBE_DEFAULT_CONST]
    }
    catch {
        set_display_item_property interfaces_html TEXT [format {<html>\
<b>Clocks and resets</b><br/>\
<b>clock_sink</b> is the MuTRiG datapath byte clock. <b>reset_sink</b> is associated with <b>clock_sink</b> and is packaged as synchronous on both edges, matching the legacy wrapper contract.<br/><br/>\
<b>Ingress stream: rx8b1k</b><br/>\
9-bit symbols: <b>data[7:0]</b> plus <b>byteisk</b> in bit 8. Error descriptor bits are\
<b>{loss_sync_pattern, parity_error, decode_error}</b>. The current RTL keys parsing from the 9-bit symbol and\
the lane-corrupt error bit; <b>asi_rx8b1k_valid</b> is present on the interface but is not consumed internally in this revision.<br/><br/>\
<b>Egress stream: hit_type0</b><br/>\
45-bit payload = ASIC[44:41], channel[40:36], TCC[35:21], TFine[20:16], ECC[15:1], EFlag[0].\
Sideband error = <b>{frame_corrupt, crc_error, hit_error}</b>. <b>startofpacket</b> and <b>endofpacket</b> are aligned to parsed hit beats;\
a dedicated <b>endofrun</b> pulse is emitted once per run after the final local drain, including the idle-terminate case.<br/><br/>\
<b>Egress stream: headerinfo</b><br/>\
42-bit payload = frame_flags[5:0], frame_len[15:6], word_count[25:16], frame_number[41:26]. Channel mirrors the ingress lane selector.<br/><br/>\
<b>Control stream: ctrl</b><br/>\
9-bit one-hot run-state command. <b>asi_ctrl_ready</b> is low while RUN_PREPARE / SYNC / TERMINATING work is still outstanding, and rises only once the local parser obligations are complete.</html>} \
            $channel_width]
    }
    catch {
        set_display_item_property regmap_html TEXT {<html><table border="1" cellpadding="3" width="100%">\
<tr><th>Word</th><th>Byte</th><th>Name</th><th>Access</th><th>Description</th></tr>\
<tr><td>0x00</td><td>0x000</td><td>CONTROL_STATUS</td><td>RW/RO</td><td>[7:0] control write image; [31:24] status mirror of the most recently decoded frame flags.</td></tr>\
<tr><td>0x01</td><td>0x004</td><td>CRC_ERR_COUNT</td><td>RO</td><td>Frames whose CRC checker did not end on the expected magic residue.</td></tr>\
<tr><td>0x02</td><td>0x008</td><td>FRAME_COUNT_DELTA</td><td>RO</td><td>Snapshot of <b>frame_counter_head - frame_counter_tail</b>, updated on hit_type0 EOP.</td></tr>\
</table><br/>\
<table border="1" cellpadding="3" width="100%">\
<tr><th>Word</th><th>Bits</th><th>Field</th><th>Access</th><th>Description</th></tr>\
<tr><td>0x00</td><td>[0]</td><td>enable</td><td>RW</td><td>Allow header pickup in RUNNING when set.</td></tr>\
<tr><td>0x00</td><td>[7:1]</td><td>reserved_control</td><td>RW</td><td>Legacy software-visible control bits; current RTL consumes only bit 0.</td></tr>\
<tr><td>0x00</td><td>[29:24]</td><td>frame_flags</td><td>RO</td><td>Mirror of the latest parsed frame flags when <b>headerinfo</b> is updated.</td></tr>\
<tr><td>0x00</td><td>[31:30]</td><td>reserved_status</td><td>RO</td><td>Reserved.</td></tr>\
</table></html>}
    }
}

proc validate {} {
    compute_derived_values

    set channel_width [get_parameter_value CHANNEL_WIDTH]
    set csr_addr_w    [get_parameter_value CSR_ADDR_WIDTH]
    set ip_uid        [get_parameter_value IP_UID]
    set build_value   [get_parameter_value BUILD]
    set ver_major     [get_parameter_value VERSION_MAJOR]
    set ver_minor     [get_parameter_value VERSION_MINOR]
    set ver_patch     [get_parameter_value VERSION_PATCH]
    set ver_date      [get_parameter_value VERSION_DATE]
    set ver_git       [get_parameter_value VERSION_GIT]
    set instance_id   [get_parameter_value INSTANCE_ID]
    set debug_level   [get_parameter_value DEBUG_LV]
    set mode_halt     [get_parameter_value MODE_HALT]

    if {$channel_width < 1 || $channel_width > 4} {
        send_message error "CHANNEL_WIDTH must stay in the range 1..4."
    }
    if {$csr_addr_w < 1 || $csr_addr_w > 8} {
        send_message error "CSR_ADDR_WIDTH must stay in the range 1..8."
    }
    if {$mode_halt < 0 || $mode_halt > 1} {
        send_message error "MODE_HALT must be 0 or 1."
    }
    if {$debug_level < 0 || $debug_level > 2} {
        send_message error "DEBUG_LV must stay in the range 0..2."
    }
    if {$ip_uid < 0 || $ip_uid > 2147483647} {
        send_message error "IP_UID must stay in the signed 31-bit range."
    }
    if {$build_value < 0 || $build_value > 4095} {
        send_message error "BUILD must stay in the range 0..4095."
    }
    if {$ver_major < 0 || $ver_major > 255} {
        send_message error "VERSION_MAJOR must stay in the range 0..255."
    }
    if {$ver_minor < 0 || $ver_minor > 255} {
        send_message error "VERSION_MINOR must stay in the range 0..255."
    }
    if {$ver_patch < 0 || $ver_patch > 15} {
        send_message error "VERSION_PATCH must stay in the range 0..15."
    }
    if {$ver_date < 0 || $ver_date > 2147483647} {
        send_message error "VERSION_DATE must stay in the signed 31-bit range."
    }
    if {$ver_git < 0 || $ver_git > 2147483647} {
        send_message error "VERSION_GIT must stay in the signed 31-bit range."
    }
    if {$instance_id < 0 || $instance_id > 2147483647} {
        send_message error "INSTANCE_ID must stay in the signed 31-bit range."
    }
}

proc elaborate {} {
    compute_derived_values

    set_parameter_property VERSION_MAJOR ENABLED false
    set_parameter_property VERSION_MINOR ENABLED false
    set_parameter_property VERSION_PATCH ENABLED false
    set_parameter_property BUILD ENABLED false
    set_parameter_property VERSION_DATE ENABLED false
    catch {set_parameter_property VERSION_GIT ENABLED [get_parameter_value GIT_STAMP_OVERRIDE]}

    add_interface_port csr avs_csr_address address Input [get_parameter_value CSR_ADDR_WIDTH]
    add_interface_port rx8b1k asi_rx8b1k_channel channel Input [get_parameter_value CHANNEL_WIDTH]
    add_interface_port hit_type0 aso_hit_type0_channel channel Output [get_parameter_value CHANNEL_WIDTH]
    add_interface_port headerinfo aso_headerinfo_channel channel Output [get_parameter_value CHANNEL_WIDTH]

    set_interface_property rx8b1k maxChannel [expr {2**[get_parameter_value CHANNEL_WIDTH] - 1}]
    set_interface_property hit_type0 maxChannel [expr {2**[get_parameter_value CHANNEL_WIDTH] - 1}]
    set_interface_property headerinfo maxChannel [expr {2**[get_parameter_value CHANNEL_WIDTH] - 1}]
}

# ========================================================================
# File sets
# ========================================================================
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL frame_rcv_ip
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file frame_rcv_ip.vhd VHDL PATH frame_rcv_ip.vhd TOP_LEVEL_FILE
add_fileset_file crc16_calc.vhd  VHDL PATH crc16_calc.vhd

add_fileset SIM_VHDL SIM_VHDL "" ""
set_fileset_property SIM_VHDL TOP_LEVEL frame_rcv_ip
set_fileset_property SIM_VHDL ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VHDL ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file frame_rcv_ip.vhd VHDL PATH frame_rcv_ip.vhd TOP_LEVEL_FILE
add_fileset_file crc16_calc.vhd  VHDL PATH crc16_calc.vhd

# ========================================================================
# Parameters
# ========================================================================
add_parameter CHANNEL_WIDTH NATURAL $DEFAULT_CHANNEL_WIDTH_CONST
set_parameter_property CHANNEL_WIDTH DISPLAY_NAME "Lane Channel Width"
set_parameter_property CHANNEL_WIDTH ALLOWED_RANGES {1 2 3 4}
set_parameter_property CHANNEL_WIDTH UNITS bits
set_parameter_property CHANNEL_WIDTH HDL_PARAMETER true
set_parameter_property CHANNEL_WIDTH DESCRIPTION "Width of the channel sideband on rx8b1k, hit_type0, and headerinfo."

add_parameter CSR_ADDR_WIDTH NATURAL $DEFAULT_CSR_ADDR_WIDTH_CONST
set_parameter_property CSR_ADDR_WIDTH DISPLAY_NAME "CSR Address Width"
set_parameter_property CSR_ADDR_WIDTH ALLOWED_RANGES {1 2 3 4 5 6 7 8}
set_parameter_property CSR_ADDR_WIDTH UNITS bits
set_parameter_property CSR_ADDR_WIDTH HDL_PARAMETER true
set_parameter_property CSR_ADDR_WIDTH VISIBLE false

add_parameter MODE_HALT NATURAL 0
set_parameter_property MODE_HALT DISPLAY_NAME "Lane Corrupt Policy"
set_parameter_property MODE_HALT ALLOWED_RANGES {0 1}
set_parameter_property MODE_HALT HDL_PARAMETER true
set_parameter_property MODE_HALT DESCRIPTION "0 resumes parsing after the lane-corrupt condition clears; 1 abandons the current frame and returns idle."

add_parameter DEBUG_LV NATURAL 0
set_parameter_property DEBUG_LV DISPLAY_NAME "Debug Level"
set_parameter_property DEBUG_LV ALLOWED_RANGES {0 1 2}
set_parameter_property DEBUG_LV UNITS None
set_parameter_property DEBUG_LV HDL_PARAMETER true

add_parameter IP_UID INTEGER $IP_UID_DEFAULT_CONST
set_parameter_property IP_UID DISPLAY_NAME "IP UID"
set_parameter_property IP_UID DESCRIPTION "Catalog identity tag for software-visible provenance."

add_parameter VERSION_MAJOR INTEGER $VERSION_MAJOR_DEFAULT_CONST
set_parameter_property VERSION_MAJOR DISPLAY_NAME "Version Major"

add_parameter VERSION_MINOR INTEGER $VERSION_MINOR_DEFAULT_CONST
set_parameter_property VERSION_MINOR DISPLAY_NAME "Version Minor"

add_parameter VERSION_PATCH INTEGER $VERSION_PATCH_DEFAULT_CONST
set_parameter_property VERSION_PATCH DISPLAY_NAME "Version Patch"

add_parameter BUILD INTEGER $BUILD_DEFAULT_CONST
set_parameter_property BUILD DISPLAY_NAME "Build"

add_parameter VERSION_DATE INTEGER $VERSION_DATE_DEFAULT_CONST
set_parameter_property VERSION_DATE DISPLAY_NAME "Version Date"

add_parameter GIT_STAMP_OVERRIDE BOOLEAN false
set_parameter_property GIT_STAMP_OVERRIDE DISPLAY_NAME "Override Git Stamp"
set_parameter_property GIT_STAMP_OVERRIDE DESCRIPTION "Allow manual editing of VERSION_GIT for non-default packaged images."

add_parameter VERSION_GIT INTEGER $VERSION_GIT_DEFAULT_CONST
set_parameter_property VERSION_GIT DISPLAY_NAME "Version Git"

add_parameter INSTANCE_ID INTEGER $INSTANCE_ID_DEFAULT_CONST
set_parameter_property INSTANCE_ID DISPLAY_NAME "Instance ID"
set_parameter_property INSTANCE_ID DESCRIPTION "Integrator-selected instance tag."

# ========================================================================
# Display items
# ========================================================================
add_display_item "" Configuration GROUP ""
add_html_text "Configuration" overview_html ""
add_display_item "Configuration" parser_group GROUP "Parser and Recovery"
add_display_item "parser_group" CHANNEL_WIDTH PARAMETER
add_display_item "parser_group" MODE_HALT PARAMETER
add_html_text "Configuration" error_mode_html ""
add_display_item "Configuration" debug_group GROUP "Debug"
add_display_item "debug_group" DEBUG_LV PARAMETER
add_html_text "Configuration" debug_html ""

add_display_item "" Identity GROUP ""
add_html_text "Identity" profile_html ""
add_display_item "Identity" identity_group GROUP "Identity Fields"
add_display_item "identity_group" IP_UID PARAMETER
add_display_item "identity_group" INSTANCE_ID PARAMETER
add_display_item "Identity" version_group GROUP "Version Fields"
add_display_item "version_group" VERSION_MAJOR PARAMETER
add_display_item "version_group" VERSION_MINOR PARAMETER
add_display_item "version_group" VERSION_PATCH PARAMETER
add_display_item "version_group" BUILD PARAMETER
add_display_item "version_group" VERSION_DATE PARAMETER
add_display_item "version_group" GIT_STAMP_OVERRIDE PARAMETER
add_display_item "version_group" VERSION_GIT PARAMETER
add_html_text "Identity" versioning_html ""

add_display_item "" Interfaces GROUP ""
add_html_text "Interfaces" interfaces_html ""

add_display_item "" "Register Map" GROUP ""
add_html_text "Register Map" regmap_html ""

# ========================================================================
# Interfaces
# ========================================================================
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

add_interface hit_type0 avalon_streaming start
set_interface_property hit_type0 associatedClock clock_sink
set_interface_property hit_type0 associatedReset reset_sink
set_interface_property hit_type0 dataBitsPerSymbol 45
set_interface_property hit_type0 errorDescriptor ""
set_interface_property hit_type0 firstSymbolInHighOrderBits true
set_interface_property hit_type0 readyLatency 0
set_interface_property hit_type0 ENABLED true
add_interface_port hit_type0 aso_hit_type0_startofpacket startofpacket Output 1
add_interface_port hit_type0 aso_hit_type0_endofpacket   endofpacket   Output 1
add_interface_port hit_type0 aso_hit_type0_endofrun      endofrun      Output 1
add_interface_port hit_type0 aso_hit_type0_error         error         Output 3
add_interface_port hit_type0 aso_hit_type0_data          data          Output 45
add_interface_port hit_type0 aso_hit_type0_valid         valid         Output 1

add_interface headerinfo avalon_streaming start
set_interface_property headerinfo associatedClock clock_sink
set_interface_property headerinfo associatedReset reset_sink
set_interface_property headerinfo dataBitsPerSymbol 42
set_interface_property headerinfo errorDescriptor ""
set_interface_property headerinfo firstSymbolInHighOrderBits true
set_interface_property headerinfo readyLatency 0
set_interface_property headerinfo ENABLED true
add_interface_port headerinfo aso_headerinfo_data  data  Output 42
add_interface_port headerinfo aso_headerinfo_valid valid Output 1

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
set_interface_property csr readLatency 1
set_interface_property csr readWaitTime 1
set_interface_property csr setupTime 0
set_interface_property csr timingUnits Cycles
set_interface_property csr writeWaitTime 0
set_interface_property csr ENABLED true
add_interface_port csr avs_csr_read        read        Input 1
add_interface_port csr avs_csr_readdata    readdata    Output 32
add_interface_port csr avs_csr_waitrequest waitrequest Output 1
add_interface_port csr avs_csr_write       write       Input 1
add_interface_port csr avs_csr_writedata   writedata   Input 32
set_interface_assignment csr embeddedsw.configuration.isFlash 0
set_interface_assignment csr embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment csr embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment csr embeddedsw.configuration.isPrintableDevice 0

add_interface ctrl avalon_streaming end
set_interface_property ctrl associatedClock clock_sink
set_interface_property ctrl associatedReset reset_sink
set_interface_property ctrl dataBitsPerSymbol 9
set_interface_property ctrl errorDescriptor ""
set_interface_property ctrl firstSymbolInHighOrderBits true
set_interface_property ctrl maxChannel 0
set_interface_property ctrl readyLatency 0
set_interface_property ctrl ENABLED true
add_interface_port ctrl asi_ctrl_data  data  Input 9
add_interface_port ctrl asi_ctrl_valid valid Input 1
add_interface_port ctrl asi_ctrl_ready ready Output 1

add_interface clock_sink clock end
set_interface_property clock_sink clockRate 0
set_interface_property clock_sink ENABLED true
add_interface_port clock_sink i_clk clk Input 1

add_interface reset_sink reset end
set_interface_property reset_sink associatedClock clock_sink
set_interface_property reset_sink synchronousEdges BOTH
set_interface_property reset_sink ENABLED true
add_interface_port reset_sink i_rst reset Input 1
