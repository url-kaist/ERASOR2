# ERASOR v1 upstream reference runner

Runs github.com/LimHyungTae/ERASOR (the C++/ROS1 reference
implementation) on the KAIST-distributed bag dataset, so we have a
ground-truth number to compare our pure-CMake port (`build/run_erasor`)
against.

## Setup (one-time)

```bash
# Build the ROS Melodic + jsk image
docker build -t erasor1:melodic -f docker/erasor1.Dockerfile docker/

# Stage the upstream sources for catkin build into a persistent ws
mkdir -p ~/erasor1_ws/src
cp -r ~/git/ERASOR ~/erasor1_ws/src/erasor

# First catkin build (one-time)
docker run --rm \
    -v ~/erasor1_ws:/catkin_ws \
    erasor1:melodic bash -c \
    "source /opt/ros/melodic/setup.bash && catkin init && catkin build erasor"
```

## Running on a sequence

Bags are expected at `~/datasets/erasor1/{seq}_{start}_to_{end}_w_interval_{N}_node.bag`.

```bash
SEQ=05
BAG=05_2350_to_2670_w_interval_2

# 1. Build the accumulated map (~30s)
docker run --rm \
    -v ~/erasor1_ws:/catkin_ws \
    -v ~/datasets/erasor1:/data/bags \
    -v ~/erasor1_ws/outputs:/data/outputs \
    erasor1:melodic bash /catkin_ws/run_mapgen.sh $BAG

# 2. Run upstream ERASOR + trigger save flag (~1m)
docker run --rm \
    -v ~/erasor1_ws:/catkin_ws \
    -v ~/datasets/erasor1:/data/bags \
    -v ~/erasor1_ws/outputs:/data/outputs \
    erasor1:melodic bash /catkin_ws/run_erasor.sh $SEQ $BAG

# 3. Evaluate using our evaluate.py (PR / RR / F1 against the
#    mapgen-produced GT map)
conda activate erasor2
python scripts/evaluate.py \
    --gt  ~/erasor1_ws/outputs/${BAG}_voxel_0.200000.pcd \
    --est ~/erasor1_ws/outputs/${SEQ}_result.pcd
```

## Per-seq YAML quirks (matter for the save filename)

Upstream's `config/seq_01.yaml` ships with `data_type: "01"` -- a typo;
the binary reads `/MapUpdater/data_name`, so the result PCD lands as
`00_result.pcd` (the default) unless the YAML is corrected. The
`~/erasor1_ws/src/erasor/config/seq_01.yaml` in this workflow has
`data_name: "01"` so the output name is correct.

The `seq_05.yaml`'s `initial_map_path` references `interval1` but the
distributed bag is `interval_2`. Edit the YAML to point at the
`interval2` PCD that `kitti_mapgen` produces, or copy in the
`interval_1` PCD you generated separately if you have one.

## Numbers measured this way

| Seq | Paper F1 | Upstream (this harness) |
|----:|---------:|------------------------:|
| 01  | 0.934    | 0.9321                  |
| 05  | 0.985    | 0.9204                  |

seq 01 matches paper within run-to-run noise. seq 05 sits 0.06 below
paper -- the paper used a denser `interval_1` initial map; with the
`interval_2` bag the user gave me, even upstream can't reach 0.985.
