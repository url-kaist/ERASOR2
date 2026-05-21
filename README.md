<div align="center">
    <h1>ERASOR2</h1>
    <a href="https://github.com/LimHyungTae/ERASOR2"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/LimHyungTae/ERASOR2"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/LimHyungTae/ERASOR2"><img src="https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-E95420?logo=ubuntu&logoColor=white" /></a>
    <a href="https://github.com/LimHyungTae/ERASOR2"><img src="https://img.shields.io/badge/CMake-064F8C?logo=cmake&logoColor=white" /></a>
    <a href="https://github.com/LimHyungTae/ERASOR2"><img src="https://img.shields.io/badge/license-GPLv3-green" /></a>
    <br />
    <br />
  <p><strong><em>Static-map distillation from accumulated LiDAR maps. ROS-free, catkin-free, just CMake.</em></strong></p>
  <p align="center">
    <strong>(v1.1, 2026-05-20)</strong> Pure-C++ build &mdash; <code>cmake -B build &amp;&amp; cmake --build build -j</code> is all it takes.
    <br />
    Full installation, HeLiPR / HeLiMOS, and YAML reference live in <a href="USAGE.md"><strong>USAGE.md</strong></a>.
  </p>
</div>

______________________________________________________________________

## :bar_chart: Headline numbers

Instance-aware dynamic-point rejection on the SemanticKITTI static-map
benchmark proposed by Lim *et al.* [ERASOR, RA-L 2021]. Each row shows
paper Table II numbers next to a fresh end-to-end measurement on this
repository (`paper / ours`) for both **ERASOR** (v1) and **ERASOR2**
(v2) -- PCL 1.12, GCC 11.4, `poses_suma_optim.txt` poses, and
`pypatchworkpp 1.3.1` ground labels.

### ERASOR2 (this paper)

| Seq | Frames | PR [%] paper / ours | RR [%] paper / ours | F1 paper / ours |
|----:|--------|--------------------:|--------------------:|----------------:|
| 00  | 4390 &ndash; 4530 | 98.788 / **98.654** | 98.582 / **98.454** | 0.987 / **0.9855** |
| 01  |  150 &ndash;  250 | 96.879 / **95.743** | 94.629 / **94.027** | 0.957 / **0.9488** |
| 02  |  860 &ndash;  950 | 98.523 / **99.196** | 99.709 / **99.902** | 0.991 / **0.9955** |
| 05  | 2350 &ndash; 2670 | 97.582 / **97.670** | 98.992 / **98.412** | 0.983 / **0.9804** |
| 07  |  630 &ndash;  820 | 98.977 / **96.135** | 98.459 / **98.989** | 0.987 / **0.9754** |

ERASOR2 reproduces within run-to-run noise (mean |&Delta;F1| = 0.006).

### ERASOR (v1, ported from [LimHyungTae/ERASOR](https://github.com/LimHyungTae/ERASOR))

| Seq | F1 paper / upstream / ours | Notes |
|----:|---------------------------:|-------|
| 00  | 0.955 / -- / 0.8601 | upstream bag interval_2 |
| 01  | 0.934 / **0.9321** / 0.7757 | upstream reproduces paper here &mdash; our port is the gap |
| 02  | 0.921 / -- / 0.5638 | upstream bag interval_2 |
| 05  | 0.985 / **0.9204** / 0.8322 | upstream sits 0.06 below paper because the bag is `interval_2` while seq_05.yaml expects `interval_1` |
| 07  | 0.947 / -- / 0.8079 | upstream bag interval_2 |

The "upstream" column is measured by running the unmodified
[LimHyungTae/ERASOR](https://github.com/LimHyungTae/ERASOR) C++/ROS1 binary
inside a ROS Melodic docker container against the KAIST-distributed
`*_node.bag` files (see [`docker/`](docker/) for the reproducer
harness). Two rows are filled in because those are the two bags the
maintainer distributes publicly.

The v1 port still trails upstream by 0.09&ndash;0.16 F1 on the two
sequences we can ground-truth. The algorithm + every knob upstream
exposes are wired up identically (`compare_vois_and_revert_ground_w_block`,
`removal_interval` per seq, `query_voxel_size`, `tf_lidar2body=[0,0,1.73]`,
per-bin voxelize on ground revert, final voxelize at 0.2m on save),
so the remaining gap is most likely the pose source: upstream reads
the bag's `/node/combined/optimized` topic (KAIST's post-processed
odometry baked into the bag), while we feed `poses_suma_optim.txt`
from the SemanticKITTI tree. Aligning those is the highest-leverage
follow-up work and a candidate for a separate session.

> PR = Preservation Rate (static recall), RR = Rejection Rate
> (dynamic removal), F1 = harmonic mean. Higher is better on all three.

## :rocket: Reproducing the table

```bash
# 1. Build (one cmake call, no ROS/catkin).
cmake -B build -S . && cmake --build build -j

# 2. Conda env for the Python preprocessors + evaluator.
conda env create -f scripts/environment.yml   # creates env "erasor2"
conda activate erasor2

# 3. Generate per-frame ground + instance labels for each sequence.
#    --kitti_dir is the directory ABOVE 'dataset/' for SemanticKITTI.
for seq in 00 01 02 05 07; do
  case $seq in
    00) i=4390; e=4530;;
    01)  i=150; e= 250;;
    02)  i=860; e= 950;;
    05) i=2350; e=2670;;
    07)  i=630; e= 820;;
  esac
  python scripts/kitti_clustering.py \
      --kitti_dir /path/to/kitti \
      --seq $seq --init_stamp $i --end_stamp $e \
      --save-instance-labels --save-ground-labels
done

# 4. Edit `config/erasor2/seq_{00,01,02,05,07}.yaml` to point at your kitti
#    and output directories, then run the full benchmark in one shot.
python scripts/run_benchmark.py
```

`scripts/run_benchmark.py` invokes `run_pipeline.py` for each yaml
(mapgen &rarr; run_erasor2 &rarr; evaluate.py), then prints a single
consolidated PR / RR / F1 table. See
[USAGE.md](USAGE.md#rocket-how-to-run) for the per-step breakdown and
the path-editing conventions.

______________________________________________________________________

## :books: Citation

If you use this code in academic work, please cite the ERASOR / ERASOR2
papers.

<details>
  <summary><strong>See bibtex lists</strong></summary>

```bibtex
@article{lim2025erasor2,
  title   = {{ERASOR2}: Instance-Aware Robust 3D Mapping of the Static World in Dynamic Scenes},
  author  = {Lim, Hyungtae and others},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2025}
}
```

```bibtex
@article{lim2021erasor,
  title   = {{ERASOR}: Egocentric Ratio of Pseudo Occupancy-based Dynamic Object Removal for Static 3D Point Cloud Map Building},
  author  = {Lim, Hyungtae and Hwang, Sungwon and Myung, Hyun},
  journal = {IEEE Robotics and Automation Letters},
  volume  = {6},
  number  = {2},
  pages   = {2272--2279},
  year    = {2021}
}
```

</details>

______________________________________________________________________

## :scroll: License

GPLv3 &mdash; see the `LICENSE` file.
