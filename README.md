<div align="center">
    <h1>ERASOR2</h1>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/Ubuntu-20.04%20%7C%2022.04-E95420?logo=ubuntu&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/CMake-064F8C?logo=cmake&logoColor=white" /></a>
    <a href="https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lim2023rss.pdf"><img src="https://img.shields.io/badge/Paper-b33737?logo=arXiv" /></a>
    <a href="https://www.youtube.com/watch?v=cELvWYxfrpY"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
    <a href="https://github.com/url-kaist/ERASOR2"><img src="https://img.shields.io/badge/license-GPLv3-green" /></a>
    <br />
    <br />
    <a href="https://www.youtube.com/watch?v=cELvWYxfrpY">Video</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="#package-installation">Install</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="#rocket-how-to-run">How to Run</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/lim2023rss.pdf">Paper</a>
    <span>&nbsp;&nbsp;•&nbsp;&nbsp;</span>
    <a href="https://github.com/url-kaist/ERASOR2/issues">Contact Us</a>
    <br />
    <br />
  <p align="center">
    <img width="640" height="485" alt="ERASOR2 demo" src="https://github.com/user-attachments/assets/5dc13005-c22a-4428-a55e-1f3d6ed97339" />
    <br />
    <img width="640" height="294" alt="ERASOR2 comparison" src="https://github.com/user-attachments/assets/3eb35e63-7b71-4bfd-84cd-74605ed34a90" />
  </p>
  <p><strong><em>ROS-free, instance-aware static map building</em></strong></p>
</div>

______________________________________________________________________

## :package: Installation

```bash
# 1. Build (one cmake call, no ROS/catkin).
cmake -B build -S . && cmake --build build -j

# 2. Conda env for the Python preprocessors + evaluator.
conda env create -f scripts/environment.yml   # creates env "erasor2"
conda activate erasor2
```

See [**USAGE.md**](USAGE.md) for the full dependency list and per-distro
notes.

______________________________________________________________________

## SemanticKITTI Setup

Download SemanticKITTI so the sequence folders live under
`<kitti_dir>/dataset/sequences`. For example, the benchmark tree should
look like this:

```text
<kitti_dir>/                         # e.g., /home/<user id>/datasets/kitti
└── dataset/
    ├── poses/
    └── sequences/
        ├── 00/
        │   ├── velodyne/
        │   ├── labels/
        │   ├── **poses_suma_optim.txt** (important)
        │   └── times.txt
        ├── 01/
        ├── 02/
        ├── ...
        └── 10/
```

ERASOR2 uses SuMa poses for evaluation. Download the pose archive and
place each `poses_suma_optim.txt` inside its matching sequence directory:

```bash
wget -O suma_poses_for_erasor_eval.zip "https://www.dropbox.com/scl/fi/9q3b1b9npsst1zjawgou3/suma_poses_for_erasor_eval.zip?rlkey=vx4igm68iuo3eobpolgq4tblg&st=yt1ola9b&dl=0"
# unzip, then copy each file to:
# <kitti_dir>/dataset/sequences/<seq>/poses_suma_optim.txt
```

For each benchmark config in `config/erasor2/seq_{00,01,02,05,07}.yaml`,
set `dataloader.abs_data_dir` to `<kitti_dir>/dataset/sequences` and
`dataloader.abs_save_dir` to your ERASOR2 output directory.

______________________________________________________________________

## :rocket: How to Run

```bash
# 3. Generate per-frame Patchwork ground + HDBSCAN instance labels
#    for seqs 00, 01, 02, 05, 07 in one shot.
scripts/generate_labels.sh /path/to/kitti

# 4. Edit config/erasor2/seq_{00,01,02,05,07}.yaml to point at your
#    kitti and output directories, then run the full benchmark.
python scripts/run_benchmark.py
```

`scripts/run_benchmark.py` invokes `run_pipeline.py` for each yaml
(mapgen &rarr; run_erasor2 &rarr; evaluate.py), then prints a single
consolidated PR / RR / F1 table. See [**USAGE.md**](USAGE.md) for further
explanation &mdash; per-step breakdown, path-editing conventions,
visualizer, YAML reference, and HeLiPR / HeLiMOS setup.

______________________________________________________________________

## :bar_chart: Headline numbers

Some reproduced numbers may differ slightly from the paper after the
ROS-free refactor, but the overall performance remains consistent with
the reported HDBSCAN-based results. Because this implementation uses
HDBSCAN for instance segmentation, compare against the HDBSCAN rows in
Table III of the paper.

<div align="center">

| Seq | Frames | PR [%] ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) | RR [%] ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) | F1 ( $\color{#c026d3}\textsf{paper}$ / $\color{#0969da}\textsf{ours}$ ) |
|:---:|:---:|:---:|:---:|:---:|
| 00 | 4390 – 4530 | $\color{#c026d3}98.649$ / $\color{#0969da}\mathbf{98.654}$ | $\color{#c026d3}\mathbf{98.582}$ / $\color{#0969da}98.454$ | $\color{#c026d3}\mathbf{0.986}$ / $\color{#0969da}0.9855$ |
| 01 |  150 –  250 | $\color{#c026d3}93.554$ / $\color{#0969da}\mathbf{95.743}$ | $\color{#c026d3}\mathbf{94.951}$ / $\color{#0969da}94.027$ | $\color{#c026d3}0.943$ / $\color{#0969da}\mathbf{0.9488}$ |
| 02 |  860 –  950 | $\color{#c026d3}98.339$ / $\color{#0969da}\mathbf{99.196}$ | $\color{#c026d3}99.709$ / $\color{#0969da}\mathbf{99.902}$ | $\color{#c026d3}0.990$ / $\color{#0969da}\mathbf{0.9955}$ |
| 05 | 2350 – 2670 | $\color{#c026d3}97.473$ / $\color{#0969da}\mathbf{97.670}$ | $\color{#c026d3}\mathbf{99.113}$ / $\color{#0969da}98.412$ | $\color{#c026d3}\mathbf{0.983}$ / $\color{#0969da}0.9804$ |
| 07 |  630 –  820 | $\color{#c026d3}\mathbf{98.767}$ / $\color{#0969da}96.135$ | $\color{#c026d3}98.800$ / $\color{#0969da}\mathbf{98.989}$ | $\color{#c026d3}\mathbf{0.988}$ / $\color{#0969da}0.9754$ |

</div>

<sub>$\color{#c026d3}\textsf{Magenta}$ = paper (Table III, HDBSCAN row), $\color{#0969da}\textsf{blue}$ = our re-run. **Bold** marks the higher value per cell.</sub>

ERASOR2 reproduces within run-to-run noise (mean |&Delta;F1| = 0.006).
Higher is better on all three metrics:

- **PR (Preservation Rate)** measures how much true
  static structure remains after dynamic-object removal.
- **RR (Rejection Rate)** measures how much dynamic
  structure is correctly rejected from the static map.
- **F1** is the harmonic mean of PR and RR, giving one balanced score
  when preservation and rejection both matter.

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
  title     = {{HeLiMOS: A dataset for moving object segmentation in 3D point clouds from heterogeneous LiDAR sensors}},
  author    = {Lim, Hyungtae and Jang, Seoyeon and Mersch, Benedikt and Behley, Jens and Myung, Hyun and Stachniss, Cyrill},
  booktitle = {2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages     = {14087--14094},
  year      = {2024}
}
```
