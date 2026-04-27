#!/usr/bin/env python3
"""Regenerate tb/REPORT/ plus local signoff annotations from DV_REPORT.json."""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path


TB = Path(__file__).resolve().parents[1]
GEN = Path.home() / ".codex" / "skills" / "dv-workflow" / "scripts" / "dv_report_gen.py"
LINTER = Path.home() / ".codex" / "skills" / "rtl-doc-style" / "scripts" / "rtl_doc_style_check.py"
REPORT = TB / "REPORT"

PASS_EMOJI = "✅"
WARN_EMOJI = "⚠️"
FAIL_EMOJI = "❌"
INFO_EMOJI = "ℹ️"


def slug(text: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", text).strip("_")


def load_data() -> dict:
    return json.loads((TB / "DV_REPORT.json").read_text(encoding="utf-8"))


def signoff_status(run: dict) -> str:
    cross = run.get("cross_summary") or {}
    pct = cross.get("pct")
    failed = cross.get("counter_checks_failed", 0) or 0
    unexpected = cross.get("unexpected_outputs", 0) or 0
    if failed > 0 or unexpected > 0:
        return FAIL_EMOJI
    if pct is not None and pct < 50.0:
        return WARN_EMOJI
    return PASS_EMOJI


def replace_section(text: str, heading: str, next_heading: str, body: str) -> str:
    marker = f"{heading}\n\n"
    start = text.find(marker)
    if start < 0:
        raise RuntimeError(f"missing section heading: {heading}")
    body_start = start + len(marker)
    end = text.find(f"\n{next_heading}\n", body_start)
    if end < 0:
        raise RuntimeError(f"missing section boundary after: {heading}")
    return text[:body_start] + body.rstrip() + text[end:]


def replace_section_to_marker(text: str, heading: str, end_marker: str, body: str) -> str:
    marker = f"{heading}\n\n"
    start = text.find(marker)
    if start < 0:
        raise RuntimeError(f"missing section heading: {heading}")
    body_start = start + len(marker)
    end = text.find(end_marker, body_start)
    if end < 0:
        raise RuntimeError(f"missing section marker after: {heading}")
    return text[:body_start] + body.rstrip() + text[end:]


def bug_link(run: dict, relpath: str) -> str:
    bug_ref = run.get("bug_ref")
    if not bug_ref:
        return "-"
    return f"[{bug_ref}]({relpath})"


def render_dv_report_signoff(runs: list[dict]) -> str:
    lines = [
        "<!-- one row per run; follow the run_id link for the full transaction-growth curve. -->",
        "",
        "| status | run_id | kind | build | seq | txns | cross_pct | ref |",
        "|:---:|---|---|---|---|---:|---:|---|",
    ]
    for run in runs:
        cross = run.get("cross_summary") or {}
        pct = cross.get("pct")
        lines.append(
            f"| {signoff_status(run)} | [`{run.get('run_id','?')}`](REPORT/cross/{slug(run.get('run_id','run'))}.md) | "
            f"{run.get('kind','?')} | {run.get('build_tag','?')} | {run.get('sequence_name') or '-'} | "
            f"{cross.get('txns',0)} | {pct if pct is not None else 'n/a'} | {bug_link(run, 'BUG_HISTORY.md')} |"
        )
    return "\n".join(lines)


def render_report_readme_signoff(runs: list[dict]) -> str:
    lines = [
        "| status | run_id | kind | build | bucket | seq | txns | cross_pct | ref |",
        "|:---:|---|---|---|---|---|---:|---:|---|",
    ]
    for run in runs:
        cross = run.get("cross_summary") or {}
        pct = cross.get("pct")
        lines.append(
            f"| {signoff_status(run)} | [`{run.get('run_id','?')}`](cross/{slug(run.get('run_id','run'))}.md) | "
            f"{run.get('kind','?')} | {run.get('build_tag','?')} | {run.get('bucket') or '-'} | "
            f"{run.get('sequence_name') or '-'} | {cross.get('txns',0)} | {pct if pct is not None else 'n/a'} | "
            f"{bug_link(run, '../BUG_HISTORY.md')} |"
        )
    return "\n".join(lines)


def render_dv_cov_signoff(runs: list[dict]) -> str:
    lines = [
        "<!-- one row per bucket_frame / all_buckets_frame signoff run (see REPORT/cross/ for curves). -->",
        "",
        "| status | run_id | kind | build | bucket | case_count | stmt | branch | toggle | functional_cross_pct | txns | ref |",
        "|:---:|---|---|---|---|---:|---|---|---|---:|---:|---|",
    ]
    for run in runs:
        cov = run.get("code_coverage") or {}
        cross = run.get("cross_summary") or {}
        pct = cross.get("pct")

        def p(key: str) -> str:
            value = cov.get(key)
            if isinstance(value, dict) and "pct" in value:
                return f"{value['pct']:.2f}"
            return "n/a"

        lines.append(
            f"| {signoff_status(run)} | [`{run.get('run_id','?')}`](REPORT/cross/{slug(run.get('run_id','run'))}.md) | "
            f"{run.get('kind','?')} | {run.get('build_tag','?')} | {run.get('bucket') or '-'} | "
            f"{run.get('case_count',0)} | {p('stmt')} | {p('branch')} | {p('toggle')} | "
            f"{pct if pct is not None else 'n/a'} | {cross.get('txns',0)} | {bug_link(run, 'BUG_HISTORY.md')} |"
        )
    return "\n".join(lines)


def render_issue_section(run: dict) -> str:
    bug_ref = run.get("bug_ref")
    if not bug_ref:
        return ""
    rerun_date = run.get("rerun_date", "?")
    rerun_log = run.get("rerun_log")
    rerun_note = run.get("rerun_note", "")
    cross = run.get("cross_summary") or {}
    unexpected = cross.get("unexpected_outputs", 0) or 0
    counter_failed = cross.get("counter_checks_failed", 0) or 0
    issue_status = FAIL_EMOJI if unexpected > 0 or counter_failed > 0 else PASS_EMOJI
    rerun_log_md = f"[{rerun_log}](../../{rerun_log})" if rerun_log else "-"
    lines = [
        "## Issue reference",
        "",
        "| status | field | value |",
        "|:---:|---|---|",
        f"| {INFO_EMOJI} | bug_ref | [{bug_ref}](../../BUG_HISTORY.md) |",
        f"| {issue_status} | rerun_date | `{rerun_date}` |",
        f"| {issue_status} | rerun_verdict | {rerun_note} |",
        f"| {INFO_EMOJI} | rerun_log | {rerun_log_md} |",
    ]
    return "\n".join(lines)


def annotate_cross_pages(runs: list[dict]) -> None:
    footer = "\n---\n_Back to [dashboard](../../DV_REPORT.md)_\n"
    for run in runs:
        page = REPORT / "cross" / f"{slug(run.get('run_id', 'run'))}.md"
        text = page.read_text(encoding="utf-8")
        issue = render_issue_section(run)
        if not issue:
            continue
        issue_pattern = re.compile(
            r"\n## Issue reference\n.*?(?=\n---\n_Back to \[dashboard\]\(\.\./\.\./DV_REPORT\.md\)_\n)",
            re.S,
        )
        if issue_pattern.search(text):
            text = issue_pattern.sub("\n" + issue + "\n", text)
        elif footer in text:
            text = text.replace(footer, "\n" + issue + "\n" + footer)
        else:
            raise RuntimeError(f"missing cross-page footer in {page}")
        page.write_text(text, encoding="utf-8")


def annotate_reports(data: dict) -> None:
    runs = data.get("signoff_runs") or []

    dv_report = TB / "DV_REPORT.md"
    text = dv_report.read_text(encoding="utf-8")
    text = replace_section(
        text,
        "## Cross / continuous-frame signoff",
        "## Index",
        render_dv_report_signoff(runs),
    )
    dv_report.write_text(text, encoding="utf-8")

    report_readme = REPORT / "README.md"
    text = report_readme.read_text(encoding="utf-8")
    text = replace_section(
        text,
        "## Cross / continuous-frame runs",
        "## Random long-run cases",
        render_report_readme_signoff(runs),
    )
    report_readme.write_text(text, encoding="utf-8")

    dv_cov = TB / "DV_COV.md"
    text = dv_cov.read_text(encoding="utf-8")
    text = replace_section_to_marker(
        text,
        "## Continuous-frame baselines by build",
        "\n_Regenerate with `",
        render_dv_cov_signoff(runs),
    )
    dv_cov.write_text(text, encoding="utf-8")

    annotate_cross_pages(runs)


def main() -> int:
    cmd = [sys.executable, str(GEN), "--tb", str(TB)]
    subprocess.run(cmd, check=True)
    annotate_reports(load_data())
    subprocess.run([sys.executable, str(LINTER), "--strict", str(TB)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
