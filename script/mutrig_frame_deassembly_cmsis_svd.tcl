package require Tcl 8.5

set script_dir [file dirname [info script]]
set helper_file [file normalize [file join $script_dir .. .. dashboard_infra cmsis_svd lib mu3e_cmsis_svd.tcl]]
source $helper_file

namespace eval ::mu3e::cmsis::spec {}

proc ::mu3e::cmsis::spec::build_device {} {
    return [::mu3e::cmsis::svd::device MU3E_MUTRIG_FRAME_DEASSEMBLY \
        -version 26.0.6.0417 \
        -description "CMSIS-SVD description of the mutrig_frame_deassembly CSR window. This first-pass schema exposes the 4-word relative aperture as read-only WORD registers until the IP author publishes field-accurate semantics." \
        -peripherals [list \
            [::mu3e::cmsis::svd::peripheral MUTRIG_FRAME_DEASSEMBLY_CSR 0x0 \
                -description "Relative 4-word CSR aperture for MuTRiG frame deassembly." \
                -groupName MU3E_DATA_PATH \
                -addressBlockSize 0x10 \
                -registers [::mu3e::cmsis::svd::word_window_registers 4 \
                    -descriptionPrefix "MuTRiG frame deassembly CSR word" \
                    -fieldDescriptionPrefix "Raw MuTRiG frame deassembly CSR word" \
                    -access read-only]]]]
}

if {[info exists ::argv0] &&
    [file normalize $::argv0] eq [file normalize [info script]]} {
    set out_path [file join $script_dir mutrig_frame_deassembly.svd]
    if {[llength $::argv] >= 1} {
        set out_path [lindex $::argv 0]
    }
    ::mu3e::cmsis::svd::write_device_file \
        [::mu3e::cmsis::spec::build_device] $out_path
}
