# ERASOR2

## Prerequisites

### System Dependencies
- ROS (Melodic/Noetic)
- PCL (Point Cloud Library)
- OpenCV
- OpenMP
- Catkin workspace

### ROS Package Dependencies
```bash
# Install required ROS packages
sudo apt-get install ros-$ROS_DISTRO-grid-map-core \
                     ros-$ROS_DISTRO-grid-map-ros \
                     ros-$ROS_DISTRO-grid-map-cv \
                     ros-$ROS_DISTRO-grid-map-msgs \
                     ros-$ROS_DISTRO-grid-map-rviz-plugin \
                     ros-$ROS_DISTRO-jsk-recognition-msgs \
                     ros-$ROS_DISTRO-costmap-2d \
                     ros-$ROS_DISTRO-pcl-conversions \
                     ros-$ROS_DISTRO-pcl-ros \
                     ros-$ROS_DISTRO-cv-bridge \
                     ros-$ROS_DISTRO-laser-geometry \
                     ros-$ROS_DISTRO-visualization-msgs
```

## Build

```bash
cd /path/to/your/catkin_ws
catkin_make
source devel/setup.bash
```

## Configuration

Edit configuration files in `config/` directory:
- `erasor2.yaml` - Main ERASOR2 parameters
- `HeLiPR.yaml` - HeLiPR dataset configuration
- `kitti_mapgen.yaml` - KITTI map generation settings
- `your_own_env.yaml` - Custom environment settings

## Running ERASOR2

### For Demo in Harmonic Space

1. After setup your own conda env, extract `*.label` files:

```
conda create -n erasor2 python=3.10 
conda activate erasor2
cd scripts
pip3 install -r requirements.txt
python3 kitti_clustering.py --seq "05" --init_stamp 2350 --end_stamp 2670 --save-instance-labels --save-ground-labels 
```

2. Generate ground truth map cloud

```
roslaunch erasor2 mapgen.launch seq:="05" start_frame:=2350 end_frame:=2670
```

3. Then, run ERASOR2 taking `*.label` files as inputs


```
roslaunch erasor2 run_erasor2.launch target_seq:="seq_05_for_keti"
```

4. Then, evaluate the results as follows:

```
python3 scripts/evaluate.py --gt /media/shapelim/UX9803/erasor2_test_benchmark/outputs/05_2350_to_2670_w_interval_2_voxel_0_2.pcd --est "/media/shapelim/UX9803/erasor2_test_benchmark/outputs/05_0_frame_2350_to_2670_estimated.pcd
```

5. Visualization

```
roslaunch erasor2 compare_map.launch
```

### Basic Usage

```bash
roslaunch erasor2 run_erasor2.launch target_seq:=<CONFIG_NAME>
```

Available configurations:
- `HeLiPR_kitti` (default)
- `seq_00`, `seq_01`, `seq_02`, `seq_05`, `seq_07`, `seq_19`
- `your_own_env`, `your_own_env_ouster`, `your_own_env_vel16`

### Map Generation

```bash
roslaunch erasor2 mapgen.launch seq:=<SEQUENCE_NAME> start_frame:=<START> end_frame:=<END> accum_interval:=<INTERVAL>
```

Parameters:
- `seq`: Sequence name (default: "Merged")
- `start_frame`: Starting frame number (default: 8600)
- `end_frame`: Ending frame number (default: 9000)
- `accum_interval`: Accumulation interval (default: 2)

### Data Processing

#### Convert HeLiPR to KITTI format
```bash
roslaunch erasor2 helipr_to_kitti.launch
```

#### Transform INS to LiDAR frame
```bash
roslaunch erasor2 transformINStoLiDAR.launch
```

#### Merge HeLiPR clouds to KITTI
```bash
roslaunch erasor2 merge_helipr_to_kitti.launch
```

### Utilities

#### Compare maps
```bash
roslaunch erasor2 compare_map.launch
rosrun erasor2 compare_map
```

#### Accumulate 4D-MOS labels
```bash
rosrun erasor2 accum_4dmos
```

#### Fill REMOVERT labels
```bash
rosrun erasor2 fill_removert_labels
```

### Visualization

#### Visualize KITTI map
```bash
roslaunch erasor2 viz_kitti_map.launch
```

#### Large scale visualization
```bash
roslaunch erasor2 run_erasor_in_large_scale.launch
```

## Custom Environment Setup

1. Copy and modify configuration files:
   ```bash
   cp config/your_own_env.yaml config/my_config.yaml
   ```

2. Update data paths and sensor parameters in the config file

3. Run with custom config:
   ```bash
   roslaunch erasor2 run_erasor2.launch target_seq:=my_config
   ```

## Key Parameters

In your configuration file, adjust:
- `grid_resolution`: Grid map resolution
- `range_of_interest`: Maximum detection range
- `min_z_voi`, `max_z_voi`: Vertical region of interest
- `scan_ratio_threshold`: Detection sensitivity (higher = more aggressive)

## Data Format

Ensure your data follows the expected format:
- Point clouds in PCD or bag format
- Trajectory/poses in appropriate coordinate frame
- Proper timestamps for synchronization
