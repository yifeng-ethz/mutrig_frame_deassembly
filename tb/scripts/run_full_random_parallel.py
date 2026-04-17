#!/usr/bin/env python3
"""Run randomized all-bucket continuous-frame mutrig sequences in parallel."""

from __future__ import annotations

import argparse
import concurrent.futures
import random
import re
import subprocess
import sys
import time
from collections import OrderedDict
from pathlib import Path


TB = Path(__file__).resolve().parents[1]
UVM_DIR = TB / "uvm"
BUCKET_FILES = OrderedDict(
    [
        ("BASIC", TB / "DV_BASIC.md"),
        ("EDGE", TB / "DV_EDGE.md"),
        ("PROF", TB / "DV_PROF.md"),
        ("ERROR", TB / "DV_ERROR.md"),
    ]
)
RUNNER_LOG_DIR = UVM_DIR / "logs" / "full_random_runner"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--parallel", type=int, default=4)
    parser.add_argument("--duration-min", type=float, default=20.0)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--rtl-variant", default="after")
    parser.add_argument("--build-tag", default="CFG_A")
    parser.add_argument("--verbosity", default="UVM_LOW")
    parser.add_argument("--effort", default="extensive")
    parser.add_argument("--channel-width", type=int, default=4)
    parser.add_argument("--csr-addr-width", type=int, default=2)
    parser.add_argument("--mode-halt", type=int, default=0)
    parser.add_argument("--debug-lv", type=int, default=0)
    parser.add_argument("--max-batches", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def parse_bucket_cases() -> OrderedDict[str, list[str]]:
    bucket_cases: OrderedDict[str, list[str]] = OrderedDict()
    for bucket, path in BUCKET_FILES.items():
        cases: list[str] = []
        for raw in path.read_text(encoding="utf-8").splitlines():
            match = re.match(r"^\|\s*([BEPX]\d{3}_[A-Za-z0-9_]+)\s*\|", raw)
            if match:
                cases.append(match.group(1))
        bucket_cases[bucket] = cases
    return bucket_cases


def build_case_pool() -> OrderedDict[str, list[str]]:
    raw = parse_bucket_cases()
    out: OrderedDict[str, list[str]] = OrderedDict()
    for bucket, cases in raw.items():
        selected = [case_id for case_id in cases if re.fullmatch(r"[BEPX]\d{3}_[A-Za-z0-9_]+", case_id)]
        if not selected:
            raise RuntimeError(f"bucket {bucket} has no implemented documented cases")
        out[bucket] = selected
    return out


def sample_case_list(rng: random.Random, bucket_cases: OrderedDict[str, list[str]], batch_size: int) -> list[str]:
    buckets = list(bucket_cases.keys())
    out: list[str] = []
    for _ in range(batch_size):
        bucket = rng.choice(buckets)
        out.append(rng.choice(bucket_cases[bucket]))
    return out


def write_case_manifest(run_id: str, case_list: list[str]) -> Path:
    RUNNER_LOG_DIR.mkdir(parents=True, exist_ok=True)
    manifest = RUNNER_LOG_DIR / f"{run_id}.cases.txt"
    manifest.write_text("\n".join(case_list) + "\n", encoding="utf-8")
    return manifest


def invoke_batch(worker_idx: int, batch_idx: int, args: argparse.Namespace, case_list: list[str]) -> tuple[int, str, Path]:
    run_id = f"full_random_all_buckets_w{worker_idx:02d}_b{batch_idx:04d}_s{args.seed + worker_idx * 1000 + batch_idx}"
    manifest = write_case_manifest(run_id, case_list)
    extra_plusargs = (
        f"+FRCV_CASE_LIST={','.join(case_list)} "
        "+FRCV_SEQUENCE_NAME=FULL_RANDOM_ALL_BUCKETS "
        "+FRCV_EXEC_MODE=all_buckets_frame "
        f"+FRCV_EFFORT={args.effort}"
    )
    cmd = [
        "make",
        "-C",
        str(UVM_DIR),
        "run_case_list",
        f"RTL_VARIANT={args.rtl_variant}",
        f"BUILD_TAG={args.build_tag}",
        f"VERBOSITY={args.verbosity}",
        f"SEED={args.seed + worker_idx * 1000 + batch_idx}",
        f"CHANNEL_WIDTH={args.channel_width}",
        f"CSR_ADDR_WIDTH={args.csr_addr_width}",
        f"MODE_HALT={args.mode_halt}",
        f"DEBUG_LV={args.debug_lv}",
        f"WORK_SUFFIX=fullrand_w{worker_idx:02d}",
        f"RUN_ID_OVERRIDE={run_id}",
        f"EXTRA_PLUSARGS={extra_plusargs}",
    ]
    if args.dry_run:
        return 0, " ".join(cmd), manifest
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    log_path = RUNNER_LOG_DIR / f"{run_id}.runner.log"
    log_path.write_text(result.stdout, encoding="utf-8")
    return result.returncode, log_path.read_text(encoding="utf-8"), manifest


def worker_main(worker_idx: int, args: argparse.Namespace, bucket_cases: OrderedDict[str, list[str]], deadline: float) -> tuple[int, int]:
    rng = random.Random(args.seed + worker_idx)
    batches = 0
    failures = 0

    while time.time() < deadline:
        if args.max_batches and batches >= args.max_batches:
            break
        case_list = sample_case_list(rng, bucket_cases, args.batch_size)
        rc, output, manifest = invoke_batch(worker_idx, batches, args, case_list)
        if args.dry_run:
            print(f"[full-random] worker={worker_idx} batch={batches} manifest={manifest}", flush=True)
            print(output, flush=True)
            batches += 1
            continue
        if rc != 0:
            failures += 1
            tail = "\n".join(output.splitlines()[-40:])
            print(
                f"[full-random] worker={worker_idx} batch={batches} FAILED manifest={manifest}\n{tail}",
                flush=True,
            )
            break
        print(
            f"[full-random] worker={worker_idx} batch={batches} pass cases={len(case_list)} manifest={manifest.name}",
            flush=True,
        )
        batches += 1

    return batches, failures


def main() -> int:
    args = parse_args()
    bucket_cases = build_case_pool()
    deadline = time.time() + args.duration_min * 60.0

    bucket_summary = ", ".join(f"{name}={len(cases)}" for name, cases in bucket_cases.items())
    print(
        f"[full-random] parallel={args.parallel} duration_min={args.duration_min} batch_size={args.batch_size} "
        f"rtl_variant={args.rtl_variant} build={args.build_tag} buckets={bucket_summary}",
        flush=True,
    )

    total_batches = 0
    total_failures = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.parallel) as executor:
        futures = [
            executor.submit(worker_main, worker_idx, args, bucket_cases, deadline)
            for worker_idx in range(args.parallel)
        ]
        for future in concurrent.futures.as_completed(futures):
            batches, failures = future.result()
            total_batches += batches
            total_failures += failures

    print(
        f"[full-random] done batches={total_batches} failures={total_failures} log_dir={RUNNER_LOG_DIR}",
        flush=True,
    )
    return 1 if total_failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
