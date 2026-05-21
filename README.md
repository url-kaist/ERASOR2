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
benchmark proposed by Lim *et al.* [ERASOR, RA-L 2021]. Numbers below
are from Table II of the [ERASOR2 paper](https://arxiv.org/abs/2306.05705):

| Seq | Frames | PR [%]   | RR [%]   | F1     |
|----:|--------|---------:|---------:|-------:|
| 00  | 4390 &ndash; 4530 | **98.788** | **98.582** | **0.987** |
| 01  |  150 &ndash;  250 | **96.879** | **94.629** | **0.957** |
| 02  |  860 &ndash;  950 | **98.523** | **99.709** | **0.991** |
| 05  | 2350 &ndash; 2670 | **97.582** | **98.992** | **0.983** |
| 07  |  630 &ndash;  820 | **98.977** | **98.459** | **0.987** |

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
