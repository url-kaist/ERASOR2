# ERASOR2 (RSS'23) + DynaKAIST 

- v1.1 Update 23.01.10
- v1. Update 23.01.08


## Test Env.
The code is tested successfully at
* Linux 20.04 LTS
* ROS Noetic

### 수정사항
3rdparty 폴더는 무시해주세요...


### HeLiPR 데이터 폴더 형식
```
${abs_data_dir} ex) /media/se0yeon00/SY_Other/HeliPR/KAIST05
│   ├── Calibration
│   ├── Inertial_data
│   ├── LiDAR
│       ├── Aeva
│       ├── Avia
│       ├── Ouster
│       └── Velodyne
│   └── LiDAR_GT
│       ├── Aeva_gt.txt 
│       ├── Avia_gt.txt 
│       ├── Ouster_gt.txt 
│       ├── Velodyne_gt.txt 
        ...    

```

### 1. inspva.csv 파일로 부터 INS trajectory 구하기 (for Deskewing)
`inspva.csv` 파일로 부터 INS trajectory 를 계산해, txt 파일로 만들어줍니다.
이는 `timestamp.bin` 으로 명명되어 있는 각각의 라이다 스캔 데이터를 deskewing 할 때 사용됩니다.
erasor2 의 `config/HeLiPR_kitti.yaml` 을 본인의 폴더 경로와 같이 수정해주세요.

```
dataprocessor:
    dataset_root: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/"
    process_lidar_list: ["Aeva", "Avia", "Ouster", "Velodyne"]
    save_ins_to_LiDAR_root: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/ins_to_lidar"
dataloader:
    abs_data_dir: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR"
```

여기서 `/dataprocessor/dataset_root` 는 HELiPR KAIST05 가 들어있는 path 를,
`/dataprocessor/save_ins_to_LiDAR_root` 는 INS trajectory 에 대한 txt 파일이 저장되는 곳입니다.
그리고 `/dataloader/abs_data_dir` 은, 추후에 디스큐잉된 라이다가 semanticKitti 포맷으로 저장될 경로입니다.

그 다음, 아래와 같이 시행해주세요.
```
catkin build erasor2
roslaunch erasor2 transformINStoLiDAR.launch
```

이렇게 하면, `/dataprocessor/save_ins_to_LiDAR_root`에 `/<sensorType>_trajectory.txt` 와 같은 파일이 생성됩니다.



### 2. HeLiPR 데이터 포맷을 SemanticKitti 포맷으로 변환하기 

```
roslaunch erasor2 helipr_to_kitti.launch
```

실행시키고나면 Semantickitti 포맷으로, `/dataloader/abs_data_dir` 위치에 velodyne 폴더 및 poses.txt 파일이 생성됩니다.

![](img/labeler_test.png)

포인트 레이블러 파에서도 잘 불러 와집니다 (위에 데이터는 velodyne!)

### 3. Merge SemanticKitti Dataset with HeLiPR Dataset

그 후, `data processor`에서 사용한 `PointCloudProcessor`를 재활용해서 각 point cloud의 timestamp 중 가장 가까운 것들끼리 묶어서 하나의 bin 파일로 다시 저장함

현재 2번을 시행하고 나서 config 파일이 ROS에 남아있으므로, 그걸 기반으로 
```commandline
rosrun erasor2 merge_heliclouds
```
를 실행함.

**주의사항** 현재 Ouster frame을 기준으로 맞춤. 왜냐하면 **KAIST05** Ouster가 가장 frame 수가 많고 (12477) LiDAR 센서 중 가장 처음 들어오는 timestamp이기 때문이다.
따라서, 만약에 IROS 이후 Heli dataset을 extension할 때는 이 부분을 주의해서 수정해줘야 할 것으로 보인다 (의도한 대로 timestamp가 잘 묶이는지 manual로 & 코드 레벨로 확인이 필요함). 


4. 그 후, ERASOR2를 돌리기 위해서는 ground segmentation label과 instance segmentation label이 필요함. ground segmentation label은 Patchwork를 통해 추출했고, 추출한 후에
```python
python3 kitti_clustering.py
```
실행하면 hdbscan이라는 폴더 내에 해당하는 instance label을 저장해준다.

이 때 필요한 환경은 아래와 같다

```python
conda install -c conda-forge hdbscan
conda install -c conda-forge joblib==1.1.0
```

**그런데** 저 joblib 버전 이슈로 인해 `__init__() got an unexpected keyword argument 'cachedir'`라는 **괴상한 에러가 발생한다.**
독일에 있을 때도 이 에러로 인해서 골치 아팠는데, 이 에러는 joblib 버전을 1.1.0으로 downgrade 해주면 해결된다.

Downgrade 해도 해결이 안되면, hdbscan와 관련된 `site_packages`에 직접 가서 `cachedir`이라고 입력으로 주는 부분을 `location`으로 수동으로 고쳐주면 돌아간다.
(참고: [여기 이슈의 greenmna 참조](https://github.com/scikit-learn-contrib/hdbscan/issues/565))

혹은 hdbscan 0.8.**29** 버전을 깔면 해결된다고 함

최종적으로 아래와 같이 데이터가 준비된다. (ERASOR2의 입력이 되는 데이터는 `Merged` 폴더 내에 있어야 함)

```python
.
├── Aeva
│   ├── poses.txt
│   └── velodyne
├── Avia
│   ├── poses.txt
│   └── velodyne
├── Merged
│   ├── hdbscan
│   ├── patchwork
│   ├── poses.txt
│   └── velodyne
├── Ouster
│   ├── poses.txt
│   └── velodyne
└── Velodyne
    ├── poses.txt
    └── velodyne

```

### 4. ERASOR2 HeLiPR 에서 돌리기 (Updated 24.01.23)


- `dataloader.cpp` 에 HeLiPRLoader 를 만들었습니다. 사용 방법은 먼저 configure 파일에서 아래를 설정해줍니다. (`HeLiPR_kitti.yaml` 참조)
```yaml
start_frame: 8450 # 시작 프레임 지정
end_frame:  8459 # 끝 프레임 지정
viz_interval: 10

dataloader:
    dataset_name: "HeLiPR"
    abs_data_dir: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR3/"
    cloud_format: ".bin"
    pose_path: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR3/"
    sequence: "Merged"
    abs_save_dir: "/media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR3/test_erasor" 
    instance_seg_method: "hdbscan"

    accum_interval: 1
    voxel_size: 0.2
    map_voxel_size: 0.2
```

여기서 `/data_loader/abs_data_dir` 은 Merged 폴더가 들어있는 경로에 해당하고, \
`/data_loader/abs_save_dir` 은 mos label 이 추후 저장되는 경로에 해당합니다. \
`accum_interval` 은 반드시 1 로 설정해줘야 bin 파일 마다 MOS label 이 생성됩니다. (현재는...)

경로를 설정해줬으면, 아래와 같이 실행해주세요.

```
roslaunch erasor2 run_erasor2.launch
```

그럼 `main.cpp` 에서

```cpp
erasor2->setSubmap();
erasor2->updateSteppableRegion();
erasor2->detectMovingObjects();
erasor2->filterDynamicObjects(); 

erasor2->saveDynamicLabels(dynamic_label_root, start_frame);
erasor2->saveStaticMap(map_path);
erasor2->publishStaticMapResults();
```

저 `saveDynamicLabels` 함수를 통해 label 이 저장이 됩니다. 이 때 저장되는 label 은 동적 포인트는 251, 정적 포인트는 0을 가집니다.
위까지 실행하고 나면, 최종 폴더 형태는 아래와 같습니다.
```
${abs_data_dir}
└── Merged
   ├── hdbscan
   ├── patchwork
   ├── poses.txt
   └── velodyne

${abs_save_dir}
    ├── mos
    │   ├── 000000.label
    │   ├── 000001.label
    │    ...
    ├── static_map.pcd
```

실제 MOS label 저장 함수는 `erasor_utils.hpp` 의 `save_dyn_label` 를 참조해주세요.

수정 사항

- bin 파일을 읽어서 데이터 로드할 때 `std::vector<float> buffer(1000000);` 이렇게 로드해오는데, 현재 merged cloud 는 가지고 있는 포인트 수가 매우많아 포인트 오버플로가 빈번하게 일어나는 것 같습니다. 지금 설정된 값으로 하면 1000000 / 4 = 250000 개 까지 밖에 저장되지 않아서... 값을 바꿔주었습니다.
마찬가지로, erasor2 에서 reserve 로 크기를 지정해주는 부분 또한 기존에 정의 된 것보다 큰 사이즈로 설정해주었습니다.

- 두번째로, `erasor2.cpp` 에서 `viz_set_scan_and_pose_` 플래그로 visualize 를 진행할 때 parseCurrCloud 함수에서 (아마 241번째 줄) `for (int i = 0; i < max_idx + 1; ++i) {` 라고 되어 있는데, max_idx + 1 까지 colors 에 push_back 해주면 뒤에서 인덱스 에러 발생, max_idx + 2 로 고쳐주었습니다.
