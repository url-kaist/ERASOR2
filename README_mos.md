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