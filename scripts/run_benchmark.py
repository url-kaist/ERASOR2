#!/usr/bin/env python3
"""Run the SemanticKITTI ERASOR2 benchmark across multiple sequences.

For each `--config` (defaults to the five paper benchmark sequences),
this script invokes `scripts/run_pipeline.py` end-to-end (mapgen ->
run_erasor2 -> evaluate), captures the Preservation / Rejection / F1
that evaluate.py prints, and emits a single consolidated markdown
table at the end.

Typical invocation:

    python scripts/run_benchmark.py \\
        --conda-env ~/.miniconda3/envs/erasor2 \\
        --build-dir ./build

The defaults match the sequences highlighted in the project README;
override with `--config config/erasor2/seq_XX.yaml [...]` to
benchmark a custom set.
"""
import argparse
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path


DEFAULT_CONFIGS = [
    "config/erasor2/seq_00.yaml",
    "config/erasor2/seq_01.yaml",
    "config/erasor2/seq_02.yaml",
    "config/erasor2/seq_05.yaml",
    "config/erasor2/seq_07.yaml",
]


def run_one(pipeline_py, cfg_path, build_dir, conda_env, skip_existing):
    """Run run_pipeline.py for a single config. Returns (pr, rr, f1)."""
    cmd = [sys.executable, str(pipeline_py), "--config", str(cfg_path)]
    if build_dir:
        cmd += ["--build-dir", str(build_dir)]
    if conda_env:
        cmd += ["--conda-env", conda_env]
    if skip_existing:
        # Reuse cached mapgen + erasor2 PCDs if present so reruns are cheap.
        cmd += ["--skip-mapgen", "--skip-erasor2"]
    print("\033[1;36m$ {}\033[0m".format(" ".join(shlex.quote(c) for c in cmd)))
    completed = subprocess.run(cmd, capture_output=True, text=True)
    sys.stdout.write(completed.stdout)
    sys.stderr.write(completed.stderr)
    if completed.returncode != 0:
        raise RuntimeError(
            "{} exited with {}; aborting benchmark.".format(cfg_path, completed.returncode)
        )

    # evaluate.py prints a single orgtbl data row that ends with the
    # three numbers we want: "| ... | 97.582 | 98.992 | 0.9830 |".
    # Grab the last pipe-delimited line that ends in a float.
    pr = rr = f1 = float("nan")
    for line in reversed(completed.stdout.splitlines()):
        if "|" not in line:
            continue
        cells = [c.strip() for c in line.split("|") if c.strip()]
        if len(cells) < 3:
            continue
        try:
            f1 = float(cells[-1])
            rr = float(cells[-2])
            pr = float(cells[-3])
            break
        except ValueError:
            continue
    return pr, rr, f1


def main():
    p = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--config",
        nargs="+",
        default=DEFAULT_CONFIGS,
        help="One or more YAML configs to benchmark. Defaults: {}".format(
            " ".join(DEFAULT_CONFIGS)
        ),
    )
    p.add_argument(
        "--build-dir",
        default=os.environ.get(
            "ERASOR2_BUILD_DIR",
            str(Path(__file__).resolve().parent.parent / "build"),
        ),
    )
    p.add_argument(
        "--conda-env",
        default=os.environ.get("ERASOR2_CONDA_ENV", ""),
        help="path to the erasor2 conda env (e.g. ~/.miniconda3/envs/erasor2)",
    )
    p.add_argument(
        "--no-skip-cached",
        action="store_true",
        help="Re-run mapgen + run_erasor2 even if the output PCDs already exist",
    )
    args = p.parse_args()

    repo_root = Path(__file__).resolve().parent.parent
    pipeline_py = repo_root / "scripts" / "run_pipeline.py"

    # Resolve relative configs against repo_root so the script works from
    # any cwd.
    configs = [
        c if Path(c).is_absolute() else str(repo_root / c) for c in args.config
    ]
    for c in configs:
        if not Path(c).is_file():
            sys.exit("Config not found: {}".format(c))

    results = []  # list of (seq_label, pr, rr, f1)
    for cfg_path in configs:
        # Pull the SemanticKITTI sequence id out of the yaml for the
        # row label without taking a yaml dep on this script.
        seq = Path(cfg_path).stem.replace("seq_", "")
        with open(cfg_path) as f:
            for line in f:
                m = re.match(r'\s*sequence:\s*"?(\w+)"?', line)
                if m:
                    seq = m.group(1)
                    break
        pr, rr, f1 = run_one(
            pipeline_py,
            cfg_path,
            args.build_dir,
            args.conda_env,
            skip_existing=not args.no_skip_cached,
        )
        results.append((seq, pr, rr, f1))

    # Final summary table.
    print()
    print("\033[1;36m=== Benchmark summary ===\033[0m")
    print("| Seq | PR [%]   | RR [%]   | F1     |")
    print("|----:|---------:|---------:|-------:|")
    for seq, pr, rr, f1 in results:
        print(
            "| {:>3} | {:>7.3f}  | {:>7.3f}  | {:>5.4f} |".format(seq, pr, rr, f1)
        )


if __name__ == "__main__":
    main()
