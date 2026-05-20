# Python Scripts for ERASOR2

This folder contains Python utilities for point cloud analysis, clustering, and evaluation.

## Environment Setup

### Option 1: Using Conda (Recommended)

1. Create conda environment from the provided file:

```bash
conda env create -f ../environment.yml
```

2. Activate the environment:

```bash
conda activate erasor2
```

### Option 2: Using pip

1. Install dependencies using pip:

```bash
pip install -r ../requirements.txt
```

## Available Scripts

### 1. KITTI Clustering (`kitti_clustering.py`)

Performs clustering on KITTI dataset point clouds using HDBSCAN.

```bash
python kitti_clustering.py -s 00 -i 0 -e 100
```

**Arguments:**

- `-s, --seq`: Sequence name (default: "Merged")
- `-i, --init_stamp`: Initial frame number (default: 0)
- `-e, --end_stamp`: End frame number (default: 12477)

### 2. Analysis Scripts

Point cloud analysis and evaluation tools.

**Python 3 version (`analysis_py3.py`):**

```bash
python analysis_py3.py --gt path/to/ground_truth.pcd --est path/to/estimate.pcd
```

**Python 2.7 version (`analysis.py`):**

```bash
python analysis.py --gt path/to/ground_truth.pcd --est path/to/estimate.pcd --seq sequence_name
```

**Arguments:**

- `--gt`: Path to ground truth PCD file
- `--est`: Path to estimated PCD file
- `--seq`: Sequence name (for analysis.py only)

### 3. Velodyne-16 Clustering (`vel16_clustering.py`)

Clustering specifically for Velodyne-16 LiDAR data.

```bash
python vel16_clustering.py
```

### 4. NumPy to Label Converter (`npy2label.py`)

Converts numpy arrays to SemanticKITTI label format.

```bash
python npy2label.py
```

### 5. Point Cloud Preprocessing (`pcd_preprocess.py`)

Contains clustering functions used by other scripts:

- `clusters_hdbscan()`: HDBSCAN clustering implementation
- `clusters_from_pcd()`: DBSCAN clustering from Open3D
- `clusterize_pcd()`: Complete clustering pipeline

## Dependencies

The scripts require the following Python libraries:

### Core Libraries

- `numpy>=1.18.0`: Numerical computing
- `matplotlib>=3.3.0`: Plotting and visualization
- `open3d>=0.15.0`: Point cloud processing
- `tqdm>=4.62.0`: Progress bars

### Machine Learning

- `scikit-learn>=0.24.0`: Machine learning utilities
- `hdbscan>=0.8.28`: Hierarchical clustering

### Utilities

- `tabulate>=0.8.9`: Table formatting
- `pathlib2>=2.3.7`: Path utilities
- `argparse`: Command-line argument parsing

## Configuration

### Data Paths

Update the absolute paths in the scripts to match your data location:

**For KITTI dataset (`kitti_clustering.py`):**

```python
ABS_DATA_DIR = "/path/to/your/SemanticKITTI/dataset/sequences"
ABS_SAVE_DIR = "/path/to/your/output/directory"
```

**For other datasets:**
Update the corresponding paths in each script as needed.

### Clustering Parameters

HDBSCAN parameters can be adjusted in `pcd_preprocess.py`:

```python
clusterer = hdbscan.HDBSCAN(
    algorithm='best',
    alpha=1.0,
    min_cluster_size=15,
    # ... other parameters
)
```

## Output Formats

### Label Files

Scripts generate label files compatible with SemanticKITTI format:

- Ground points and sub-clustered points: label = 0
- Instance clusters: label > 0
- Format: 32-bit unsigned integers

### Analysis Results

Analysis scripts output evaluation metrics including:

- Preservation rate
- Rejection rate
- F1 score
- Class-wise statistics

## Legacy Support

A Python 2.7 environment configuration is provided in `py2.7_environment.yml` for legacy compatibility with older analysis scripts.
