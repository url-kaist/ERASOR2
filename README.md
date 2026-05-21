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
  <p><strong><em>Static-map distillation from accumulated LiDAR maps. ROS-free, catkin-free, just CMake.</em></strong></p>
  <p align="center">
    <strong>(v1.1, 2026-05-20)</strong> Pure-C++ build &mdash; <code>cmake -B build &amp;&amp; cmake --build build -j</code> is all it takes.
  </p>
</div>

______________________________________________________________________

## :bar_chart: Headline numbers

| Seq | Frames | PR [%] paper / ours | RR [%] paper / ours | F1 paper / ours |
|----:|--------|--------------------:|--------------------:|----------------:|
| 00  | 4390 &ndash; 4530 | 98.788 / **98.654** | 98.582 / **98.454** | 0.987 / **0.9855** |
| 01  |  150 &ndash;  250 | 96.879 / **95.743** | 94.629 / **94.027** | 0.957 / **0.9488** |
| 02  |  860 &ndash;  950 | 98.523 / **99.196** | 99.709 / **99.902** | 0.991 / **0.9955** |
| 05  | 2350 &ndash; 2670 | 97.582 / **97.670** | 98.992 / **98.412** | 0.983 / **0.9804** |
| 07  |  630 &ndash;  820 | 98.977 / **96.135** | 98.459 / **98.989** | 0.987 / **0.9754** |

ERASOR2 reproduces within run-to-run noise (mean |&Delta;F1| = 0.006).

> PR = Preservation Rate (static recall), RR = Rejection Rate
> (dynamic removal), F1 = harmonic mean. Higher is better on all three.

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
