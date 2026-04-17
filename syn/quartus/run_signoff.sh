#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_name="mutrig_frame_deassembly_syn"
revisions=(
  "mutrig_frame_deassembly_syn_cfg_a"
)

cd "${script_dir}"

for revision in "${revisions[@]}"; do
  echo "==> compiling ${revision}"
  quartus_sh --flow compile "${project_name}" -c "${revision}"
  echo "==> reports: ${script_dir}/output_files/${revision}"
done
