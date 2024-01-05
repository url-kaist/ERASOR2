# ERASOR2 (RSS'22) + DynaKAIST

Official page of *"ERASOR2"*

[Video] [Preprint Paper] 

## Test Env.
The code is tested successfully at
* Linux 20.04 LTS
* ROS Noetic

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

### Option1. HeLiPR 포맷을 ERASOR2 의 데이터 로더에서 바로 돌리기 
```
catkin build
roslaunch erasor2 run_erasor.launch
```
하면, 
- 현재 `config/HeLiPR.yaml` 에서 불러온 세팅대로 scan 및 pose 가 불러와지는 것을 확인할 수 있습니다. (label 은 없어서 불러오지 않도록 함) 
- 기존 SemanticKitti Loader 에서는 불러오고 싶은 seq를 숫자로 (예를 들어, `00`, `01`, `02`...) 지정해줬다면, HeLiPR 에서는 seq 에 센서 이름을 넣어주면 됩니다. (예를 들어, `Avia`, `Velodyne`, `Ouster`...)

- 라이다마다의 pose 를 불러올 때 라이다 간의 extrinsic 을 곱해서 모두 Ouster 의 좌표계로 align 되도록 했는데, 맞게한 건지는 확인해보지 않았습니다... 아직 수정전입니다.

### Option2. HeLiPR 포맷을 SemanticKitti 포맷으로 변환하기 (아직 코드 정리 안함)
기존에 데이터셋 툴박스에 있는 코드를 기준으로 작성했습니다.
아직 코드 정리를 하지 않아서 이해가 어렵습니다...ㅜㅜ https://github.com/minwoo0611/HeLiPR-Pointcloud-Toolbox.git <- 여기를 같이 참조하시는게 좋을 것 같습니다.
```
cd 3rdParty
mkdir build && cd build
cmake ..
make
```


```
cd 3rdparty/python
python3 transformINStoLiDAR.py
```
```
cd 3rdparty/bin
./helipr_tool
```
옵션 선택은 아래와 같이 해주면 됩니다.
```bash
Enter the path to the directory containing the .bin files (end with folder/): /media/se0yeon00/SY_Other/HeliPR/KAIST05

Enter the sensor type: Aeva

Total 12475 poses are loaded

Enter the path to the trajectory file: /media/se0yeon00/SY_Other/HeliPR/KAIST05/Trajectory/Aeva/trajectory.txt

Enter the path to the directory to save the processed point clouds (end with '/'): /media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR/Aeva/

poses.txt is created in /media/se0yeon00/SY_Other/HeliPR/KAIST05/deskewed_LiDAR/Aeva/poses.txt
The path does not exist. Creating a new directory.
Enter the LiDAR type (0: Ouster, 1: Velodyne, 2: Livox, 3: Aeva): 3
Enter the distance threshold (m) (default: 10, >= 0): 0.01
Enter the number of intervals for interpolation (default: 1000, >= 1): 1000
Enter the number of point clouds to accumulate before processing (default: 20, >= 1): 1
Enter the downsample flag (0: false, 1: true) (default: 1): 0
Enter the undistort flag (0: false, 1: true) (default: 1): 1
```

실행시키고나면 Semantickitti 포맷으로 velodyne 폴더 및 poses.txt 파일이 생성됩니다.

![](img/labeler_test.png)

Point labeler 에서도 불러와지기는 하는 것 같습니다... (근데 잘못 불러와집니다. 문제점 수정하기 필요)