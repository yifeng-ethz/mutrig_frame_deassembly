#!/usr/bin/env python3
"""Emit build-aware continuous-frame signoff case sets for mutrig_frame_deassembly."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


TB = Path(__file__).resolve().parents[1]

SEQ_TO_PLAN = {
    "BASIC_CFG_A": "DV_BASIC.md",
    "EDGE_CFG_A": "DV_EDGE.md",
    "PROF_CFG_A": "DV_PROF.md",
    "ERROR_CFG_A": "DV_ERROR.md",
}

CFG_A_EXCLUSIONS = {
    "BASIC_CFG_A": {
        "B095_mode_halt1_comma_inside_frame_holds_state",
    },
    "EDGE_CFG_A": {
        "E023_header_after_comma_recovery_mode1_resumes_old_frame_not_new_header",
        "E084_mode_halt1_comma_on_frame_counter_byte_holds_state",
        "E085_mode_halt1_comma_on_event_counter_byte_holds_state",
        "E086_mode_halt1_comma_on_unpack_byte_holds_state",
        "E088_mode1_resume_with_error_bits_preserved",
        "E091_csr_addr_width1_aliases_words0_1_only",
        "E093_csr_addr_width8_high_unused_reads_zero",
        "E094_channel_width1_headerinfo_maxchannel1",
        "E095_channel_width1_output_channel_singlebit",
        "E096_channel_width4_default_maxchannel15",
        "E097_channel_width8_large_channel_passthrough",
    },
    "PROF_CFG_A": {
        "P071_mode1_comma_every_64th_frame_counter",
        "P072_mode1_comma_every_64th_event_counter",
        "P073_mode1_comma_every_64th_unpack",
        "P074_mode1_comma_random_positions",
        "P075_mode1_hold_with_loss_sync_mix",
        "P077_mode0_to_mode1_build_comparison",
        "P078_mode1_to_mode0_build_comparison",
        "P081_cfgB_modehalt1_clean_soak",
        "P082_cfgB_modehalt1_comma_soak",
        "P083_cfgC_channel1_alternating_soak",
        "P084_cfgD_channel8_wide_sweep",
        "P085_cfgE_addr1_poll_soak",
        "P086_cfgF_addr8_unused_space_sweep",
        "P087_cfgG_debuglv2_functional_soak",
        "P088_cfgC_termination_soak",
        "P089_cfgD_zerohit_onehit_mix",
        "P090_cfgE_control_churn_soak",
        "P127_seed10_comma_mix_mode1_100k_cycles",
        "P128_seed11_cfgD_wide_channel_100k_cycles",
        "P129_seed12_cfgE_narrow_addr_100k_cycles",
    },
    "ERROR_CFG_A": {
        "X038_long_comma_inside_payload_mode1",
        "X051_short_comma_inside_payload_mode1",
        "X085_mode1_comma_in_frame_counter",
        "X086_mode1_comma_in_event_counter",
        "X087_mode1_comma_in_long_unpack",
        "X088_mode1_comma_in_short_unpack",
        "X089_mode1_hold_then_bad_crc",
        "X090_mode1_hold_then_runprepare",
        "X118_cfgC_channel_width1_invalid_channel_drive",
        "X119_cfgD_channel_width8_random_wide_values",
        "X120_cfgE_addrwidth1_highaddr_alias_misuse",
        "X121_cfgF_addrwidth8_highaddr_abuse",
        "X122_cfgB_modehalt1_abort_assumption_negative",
        "X124_cfgG_debuglv2_functional_delta_negative",
    },
}


def parse_plan_case_ids(plan_name: str) -> list[str]:
    case_ids: list[str] = []
    plan = TB / plan_name
    for line in plan.read_text(encoding="utf-8").splitlines():
        match = re.match(r"^\|\s*([BEPX]\d{3}_[^|]+?)\s*\|", line)
        if match:
            case_ids.append(match.group(1).strip())
    return case_ids


def sequence_case_ids(sequence_name: str) -> list[str]:
    if sequence_name == "ALL_CFG_A":
        merged: list[str] = []
        for part in ("BASIC_CFG_A", "EDGE_CFG_A", "PROF_CFG_A", "ERROR_CFG_A"):
            merged.extend(sequence_case_ids(part))
        return merged

    plan_name = SEQ_TO_PLAN[sequence_name]
    full = parse_plan_case_ids(plan_name)
    excluded = CFG_A_EXCLUSIONS.get(sequence_name, set())
    return [case_id for case_id in full if case_id not in excluded]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "sequence_name",
        choices=["BASIC_CFG_A", "EDGE_CFG_A", "PROF_CFG_A", "ERROR_CFG_A", "ALL_CFG_A"],
    )
    parser.add_argument("--format", choices=["csv", "lines", "count"], default="csv")
    args = parser.parse_args()

    case_ids = sequence_case_ids(args.sequence_name)
    if args.format == "count":
        print(len(case_ids))
    elif args.format == "lines":
        print("\n".join(case_ids))
    else:
        print(",".join(case_ids))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
