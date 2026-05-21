#!/usr/bin/env python3
"""End-to-end ERASOR2 driver from Python.

There are no native Python bindings — the C++ binaries (`mapgen`,
`run_erasor2`) own the algorithm. This script just orchestrates them via
subprocess plus the existing preprocessing / evaluation Python helpers,
so a single command takes you from raw SemanticKITTI to PR/RR/F1.

Typical invocation:

    python3 scripts/run_pipeline.py \\
        --config       config/seq_05.yaml \\
        --build-dir    ./build \\
        --preprocess                       # only on first run for a seq
"""
import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("Missing PyYAML. `pip install pyyaml`", file=sys.stderr)
    sys.exit(2)


def run(cmd, cwd=None, env=None):
    """Run a command, echoing the line, stream output, and raise on failure."""
    if isinstance(cmd, list):
        printable = " ".join(shlex.quote(c) for c in cmd)
    else:
        printable = cmd
    print("\033[1;36m$ {}\033[0m".format(printable))
    subprocess.run(cmd, cwd=cwd, env=env, check=True, shell=isinstance(cmd, str))


def load_cfg(path):
    with open(path) as f:
        return yaml.safe_load(f)


def reap_orphan_rerun_viewers():
    """Kill any stale rerun viewer holding port 9876 from a previous run.

    The C++ binaries call `g_rec->spawn()` which checks whether the rerun
    grpc port is already listening; if it is, the binary just connects to
    the existing viewer rather than launching a new GUI. When a previous
    run's viewer window was closed but the process lingered, the user sees
    no window at all even though logging "works". Reaping here keeps the
    common case (one viewer per pipeline run) frictionless.
    """
    try:
        subprocess.run(
            ["pkill", "-f", "rerun_sdk/rerun_cli/rerun"],
            check=False,
        )
    except FileNotFoundError:
        # pkill missing (e.g. on a stripped container); user can clean up
        # manually if needed.
        pass


def needs_preprocess(data_dir, seq, start, end):
    seq_dir = Path(data_dir) / seq
    n_needed = end - start + 1
    for sub in ("hdbscan", "patchwork"):
        if not (seq_dir / sub).is_dir():
            return True
        if len(list((seq_dir / sub).glob("*.label"))) < n_needed:
            return True
    return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True, help="path to seq_*.yaml")
    p.add_argument(
        "--build-dir",
        default=os.environ.get(
            "ERASOR2_BUILD_DIR",
            str(Path(__file__).resolve().parent.parent / "build"),
        ),
        help="cmake build dir containing the mapgen / run_erasor2 binaries",
    )
    p.add_argument(
        "--preprocess",
        action="store_true",
        help="run scripts/kitti_clustering.py first (skips if already done)",
    )
    p.add_argument(
        "--skip-mapgen",
        action="store_true",
        help="reuse existing GT pcd in abs_save_dir",
    )
    p.add_argument(
        "--skip-erasor2",
        action="store_true",
        help="reuse existing estimated pcd in abs_save_dir",
    )
    p.add_argument(
        "--no-eval",
        action="store_true",
        help="skip the evaluate.py call at the end",
    )
    p.add_argument(
        "--conda-env",
        default=os.environ.get("ERASOR2_CONDA_ENV", ""),
        help="path to conda env with open3d/sklearn/hdbscan (e.g. ~/.miniconda3/envs/erasor2-3.10)",
    )
    args = p.parse_args()

    reap_orphan_rerun_viewers()

    cfg = load_cfg(args.config)
    dl = cfg["dataloader"]
    seq = dl["sequence"]
    data_dir = dl["abs_data_dir"]
    save_dir = dl["abs_save_dir"]
    start = cfg.get("start_frame", dl.get("start_frame"))
    end = cfg.get("end_frame", dl.get("end_frame"))
    interval = dl["accum_interval"]
    voxel = dl["voxel_size"]

    repo_root = Path(__file__).resolve().parent.parent
    Path(save_dir).mkdir(parents=True, exist_ok=True)

    # 1. Preprocessing -------------------------------------------------------
    if args.preprocess and needs_preprocess(data_dir, seq, start, end):
        py = (
            "{}/bin/python".format(args.conda_env) if args.conda_env else sys.executable
        )
        env = os.environ.copy()
        ld_preload = "{}/lib/libstdc++.so.6".format(args.conda_env)
        if args.conda_env and Path(ld_preload).exists():
            env["LD_PRELOAD"] = ld_preload
        # abs_data_dir points at <kitti_dir>/dataset/sequences; the script
        # wants <kitti_dir>.
        kitti_dir = str(Path(data_dir).parent.parent)
        run(
            [
                py,
                str(repo_root / "scripts" / "kitti_clustering.py"),
                "--kitti_dir",
                kitti_dir,
                "--seq",
                str(seq),
                "--init_stamp",
                str(start),
                "--end_stamp",
                str(end),
                "--save-instance-labels",
                "--save-ground-labels",
            ],
            env=env,
        )

    # 2. Resolve binaries from the cmake build dir --------------------------
    bdir = Path(args.build_dir)
    mapgen_bin = bdir / "mapgen"
    erasor_bin = bdir / "run_erasor2"
    for b in (mapgen_bin, erasor_bin):
        if not b.exists():
            sys.exit(
                "Binary not found: {}.\nDid you run "
                "`cmake -B {} -S . && cmake --build {} -j` first?".format(b, bdir, bdir)
            )

    # The C++ binaries call rerun::RecordingStream::spawn() which `exec`s
    # `rerun` from $PATH. We need the spawned viewer to match the SDK the
    # binaries link against (rerun-sdk 0.32.x); prepend the conda env's
    # bin so the env's rerun wins over any older system one. Try
    # --conda-env, then the env hosting this interpreter.
    cpp_env = os.environ.copy()
    env_root = args.conda_env or os.environ.get("CONDA_PREFIX") or sys.prefix
    if env_root and (Path(env_root) / "bin" / "rerun").exists():
        cpp_env["PATH"] = "{}/bin:{}".format(env_root, cpp_env.get("PATH", ""))

    # 3. mapgen --------------------------------------------------------------
    gt_pcd = Path(save_dir) / "{}_{}_to_{}_w_interval_{}_voxel_{}.pcd".format(
        seq,
        start,
        end,
        interval,
        str(voxel).replace(".", "_"),
    )
    if args.skip_mapgen and gt_pcd.exists():
        print("[pipeline] Reusing existing {}".format(gt_pcd))
    else:
        run([str(mapgen_bin), args.config], env=cpp_env)

    # 4. run_erasor2 ---------------------------------------------------------
    est_pcd = Path(save_dir) / "{}_0_frame_{}_to_{}_estimated.pcd".format(
        seq,
        start,
        end,
    )
    if args.skip_erasor2 and est_pcd.exists():
        print("[pipeline] Reusing existing {}".format(est_pcd))
    else:
        run([str(erasor_bin), args.config], env=cpp_env)

    # 5. Evaluate ------------------------------------------------------------
    if not args.no_eval:
        py = (
            "{}/bin/python".format(args.conda_env) if args.conda_env else sys.executable
        )
        env = os.environ.copy()
        ld_preload = "{}/lib/libstdc++.so.6".format(args.conda_env)
        if args.conda_env and Path(ld_preload).exists():
            env["LD_PRELOAD"] = ld_preload
        run(
            [
                py,
                str(repo_root / "scripts" / "evaluate.py"),
                "--gt",
                str(gt_pcd),
                "--est",
                str(est_pcd),
            ],
            env=env,
        )

    print("\n\033[1;32m[pipeline] Done. Outputs in {}\033[0m".format(save_dir))


if __name__ == "__main__":
    main()
