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
  <p align="center">
    <img width="500" height="379" alt="ERASOR2 demo" src="https://github.com/user-attachments/assets/5dc13005-c22a-4428-a55e-1f3d6ed97339" />
    <br />
    <img width="400" height="184" alt="ERASOR2 comparison" src="https://github.com/user-attachments/assets/05377816-4e71-4ebe-afb6-9a16260d0248" />
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

### ERASOR (v1, ported from [LimHyungTae/ERASOR](https://github.com/LimHyungTae/ERASOR)) &mdash; **TODO**

| Seq | F1 paper / upstream / ours | Notes |
|----:|---------------------------:|-------|
| 00  | 0.955 / -- / 0.8601 | upstream bag not measured |
| 01  | 0.934 / **0.9321** / 0.7757 | upstream reproduces paper &mdash; our port has a real 0.16 F1 gap |
| 02  | 0.921 / -- / 0.5638 | upstream bag not measured |
| 05  | 0.985 / **0.9204** / 0.8322 | upstream sits 0.06 below paper because the bag is `interval_2` while `seq_05.yaml`'s `initial_map_path` expects `interval_1` |
| 07  | 0.947 / -- / 0.8079 | upstream bag not measured |

The "upstream" column is measured by running the unmodified
[LimHyungTae/ERASOR](https://github.com/LimHyungTae/ERASOR) C++/ROS1
binary in a ROS Melodic docker container against the KAIST-distributed
`*_node.bag` files (see [`docker/`](docker/) for the reproducer
harness). Only the two bags the maintainer ships publicly were
measured.

**Status: WIP, not yet at parity.** The algorithm + every upstream
knob is wired identically
(`compare_vois_and_revert_ground_w_block`, `removal_interval` per
seq, `query_voxel_size`, `tf_lidar2body=[0, 0, 1.73]`, per-bin
voxelize on ground revert, final voxelize at 0.2m on save). The
0.09&ndash;0.16 F1 gap traces to the **pose source**: upstream
reads `/node/combined/optimized` (KAIST's post-processed odometry
baked into the bag, in a bag-local world frame), our port feeds
`poses_suma_optim.txt` from the SemanticKITTI tree (a different
world frame entirely). A quick experiment that piped the bag-extracted
poses into our dataloader produced PR&asymp;99 but RR=0 -- the polar
grid then sees a coordinate system the v3 algorithm was tuned
against but the running-map maintenance is still wrong (the bag
poses are *partial*: only for the bag-covered frames, not the full
KITTI sequence). Closing this is a focused follow-up session; the
docker harness in [`docker/`](docker/) keeps the ground-truth
reproducible.

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

```bibtex
@inproceedings{lim2024helimos,
  title        = {{HeLiMOS}: A Dataset for Moving Object Segmentation in 3D Point Clouds from Heterogeneous LiDAR Sensors},
  author       = {Lim, Hyungtae and Jang, Seoyeon and Mersch, Benedikt and Behley, Jens and Myung, Hyun and Stachniss, Cyrill},
  booktitle    = {2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages        = {14087--14094},
  year         = {2024},
  organization = {IEEE}
}
```
