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

**Pose file: `poses_suma_optim.txt`.** The C++ dataloader reads
`<seq>/poses_suma_optim.txt` (the optimized SuMa poses) for every
SemanticKITTI sequence except `19`, which reads `kiss_icp_poses.txt`.
Earlier versions used `suma_pose.txt` and prepare_fixtures.sh did a
rename; that's gone now. Make sure the optimized file is present
under each sequence directory.

**CMake build dir cached across CI runs** via the
`erasor2_ci_build` docker named volume. To force a fresh build, run
`docker volume rm erasor2_ci_build`. Useful when a build appears to use
stale headers after large CMakeLists edits.

**Ubuntu 24.04 host PCL has stale `VTK::mpi` import.** `find_package(PCL
REQUIRED)` errors with `target "VTK::mpi" was not found` unless
`MPI::MPI_C` is already defined. Does not affect the docker (Ubuntu
20.04 + PCL 1.10) — only matters if someone tries to build natively on
24.04. Workaround: add `find_package(MPI QUIET)` before
`find_package(PCL)` if it becomes a recurring problem.

**PCL ≥ 1.11 changed `pcl::PointCloud<T>::Ptr` from `boost::shared_ptr`
to `std::shared_ptr`.** Ubuntu 22.04 ships PCL 1.12, Ubuntu 20.04 ships
PCL 1.10. Any function template signature written as
`boost::shared_ptr<pcl::PointCloud<T>>` will fail template-argument
deduction on 22.04 because callers pass `Ptr` (now a `std::shared_ptr`).
Fix in `include/tools/erasor_utils.hpp` is two overloads side-by-side
(`boost::shared_ptr<...>` + `std::shared_ptr<...>`), since the two
types are unrelated and exactly one matches per distro.

Same release also stopped pulling in `<chrono>` transitively — the
`std::chrono::system_clock` calls in `voxelize_preserving_labels_by_nanoflann`
need an explicit `#include <chrono>`.

- **Why:** PCL 1.11 modernized to C++14 smart pointers. Hit during
  the 22.04 port on 2026-05-20.
- **How to apply:** any new template that takes a PCL cloud pointer
  must accept either `typename pcl::PointCloud<T>::Ptr` (with `T`
  spelled out at the call site) or provide both boost+std overloads.
  Don't hard-code `boost::shared_ptr`.

**mdformat destroys YAML frontmatter** in `.claude/memory/*.md`. The
pre-commit-config has `exclude: '^(github/|\.claude/)'` to prevent this
— if you add new memory paths, extend the exclude pattern.
