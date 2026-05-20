# Algorithm parity CI

`.github/workflows/parity.yml` re-runs the seq-05 `[2350..2670]` end-to-end
pipeline on every PR that touches `src/`, `include/`, `CMakeLists.txt`, or
`config/seq_05*.yaml`. It asserts the output PCDs are **byte-identical** to
a frozen golden set, plus a softer PR / RR / F1 tolerance check.

This is the same comparison that was used to validate the ROS-free rerun
migration (`cmp` reported zero byte differences across mapgen output, the
estimated static map, and all 161 per-frame MOS labels).

## One-time setup on the self-hosted runner

The workflow targets `runs-on: [self-hosted, erasor2-rig]`. To register the
runner host:

1. Repository → Settings → Actions → Runners → New self-hosted runner.
   Pick Linux x64. Follow the displayed `./config.sh` / `./run.sh` steps.
1. When `config.sh` asks for labels, add `erasor2-rig` alongside the
   default labels.
1. Install as a systemd service so it survives reboots:
   ```
   sudo ./svc.sh install
   sudo ./svc.sh start
   ```
1. Ensure the runner user is in the `docker` group:
   ```
   sudo usermod -aG docker $USER
   ```

Then populate the fixture dataset and the golden PCDs:

```bash
tests/scripts/prepare_fixtures.sh
```

This will:

| step | output | size |
|---|---|---|
| copy seq-05 \[2350..2670\] velodyne + labels | `~/catkin_ws_for_erasor2/data/.../05/{velodyne,labels}/` | ~764 MB |
| run `scripts/kitti_clustering.py` to make `hdbscan/` + `patchwork/` | same dir, two new subdirs | ~50 MB |
| build current `HEAD` in docker and run `mapgen` + `run_erasor2` | `~/erasor2_golden/*.pcd` | ~57 MB |

Override any path via env var before running, e.g.:

```bash
ERASOR2_DATA_ROOT=/data/kitti/sequences \
ERASOR2_GOLDEN_DIR=/data/erasor2_golden \
tests/scripts/prepare_fixtures.sh
```

## When CI fails

The strict `cmp` fires if even one byte of either output PCD differs.
That's normally what you want — sub-voxel rounding differences during
refactoring are still real differences and worth a look.

If the change is **intentional** (algorithm tweak, dependency bump that
shifts PCL's voxel filter behavior, etc.), refresh the golden after
landing the change:

```bash
rm -rf ~/erasor2_golden
tests/scripts/prepare_fixtures.sh   # rebuilds and saves new golden
```

Commit the change in a single PR; future PRs are then gated against the
new baseline.

The numeric tolerance check (PR ≥ 97.5 %, RR ≥ 98.0 %, F1 ≥ 0.97) is the
backstop: if the bits drift but the metrics stay strong, the workflow log
will say so. Adjust the thresholds in `.github/workflows/parity.yml`.

## Running it locally

The same script the runner uses works for ad-hoc local verification:

```bash
GITHUB_WORKSPACE=$PWD tests/scripts/run_parity.sh
```

Expect ~3 min on first run (cold rerun_sdk fetch), ~1 min on subsequent
runs (the `erasor2_ci_build` docker volume caches the CMake build dir).

## Cost model

Per PR run:

| phase | time |
|---|---|
| docker pull (cached) | 0 s |
| cmake build (cached) | 5–10 s for small diffs, 70 s for cold |
| mapgen + run_erasor2 | ~3–5 min |
| cmp + Python numeric | \<5 s |

Total ~5 min steady-state. Self-hosted runner means $0 cost.
