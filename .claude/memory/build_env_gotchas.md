---
name: build-env-gotchas
description: Ubuntu 20.04 + conda + flake8 quirks that bite if you forget them
metadata:
  type: feedback
---

Workarounds discovered while bringing up v1.0. None of these are visible
from reading the code, so they're easy to re-trip without notes.

**conda env + system libstdc++ mismatch.** open3d 0.19 needs
`CXXABI_1.3.15`; system libstdc++ on Ubuntu 20.04 only ships up to
`CXXABI_1.3.14`. The conda env's libstdc++ has it. Set
`LD_PRELOAD=$CONDA_ENV/lib/libstdc++.so.6` before invoking any python
script that imports open3d (`kitti_clustering.py`, `evaluate.py`,
`assert_metrics.py`). `scripts/run_pipeline.py --conda-env <path>`
handles this automatically.

- **Why:** Ubuntu 20.04 LTS frozen toolchain vs modern PyPI wheels.
- **How to apply:** any time open3d ≥ 0.18 is involved.

**flake8 misreads f-string format specs as E231.** `f"x={v:.3f}"` gives
"E231 missing whitespace after ':'" because flake8 sees the colon as a
dict-style separator. Either use `"x={:.3f}".format(v)` or add
`# noqa: E231`.

- **Why:** flake8 older than 5.0.4 doesn't parse f-string format minilang.
- **How to apply:** when writing Python that touches `tests/` or
  `scripts/` (pre-commit runs flake8 there).

**cpplint demands `explicit` on single-arg constructors.** Will block
commits silently — `Single-parameter constructors should be marked
explicit. [runtime/explicit] [5]`. Mark new ctors `explicit`.

- **Why:** pre-commit hook enforces Google C++ style.
- **How to apply:** any new C++ class with a single-parameter ctor.

**`source /opt/ros/noetic/setup.bash` under `set -u`** trips on
unbound `ROS_DISTRO`. Wrap with `set +u; source ...; set -u` or run the
ROS sourcing before flipping `set -u` on.

**suma_pose.txt vs poses_suma.txt.** The C++ dataloader reads
`<seq>/suma_pose.txt`. The SemanticKITTI distribution ships
`poses_suma.txt`. Copy or symlink to the dataloader-expected name when
populating fixtures.

**Catkin build dir cached across CI runs** via the
`erasor2_ci_build` docker named volume. To force a fresh build, run
`docker volume rm erasor2_ci_build`. Useful when a build appears to use
stale headers after large CMakeLists edits.

**mdformat destroys YAML frontmatter** in `.claude/memory/*.md`. The
pre-commit-config has `exclude: '^(github/|\.claude/)'` to prevent this
— if you add new memory paths, extend the exclude pattern.
