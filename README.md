# ROS2-Automato-Robot-Software
自動収穫ロボットのROS2版

## 1. Introduction
R03のポット栽培に適応したトマト自動収穫ロボットの動作に係るソフトウェアレポジトリです．
ハードウェアについては外装フレームとポットの把持・回転機構は[Automato-Robot-Hardware](https://github.com/KazuyaOguma18/Automato-Robot-Hardware)に，
エンドエフェクタは[Automato-EndEffector](https://github.com/KazuyaOguma18/Automato-EndEffector)にアップロードされています．


## 2. install
- ### 2.1 Install [ROS Foxy](https://docs.ros.org/en/foxy/Installation.html) 

- ### 2.2 Install [Moveit2](https://moveit.ros.org/install-moveit2/source/)  

- ### 2.3 Install [ros2_control, ros2_controllers](https://ros-controls.github.io/control.ros.org/getting_started.html)  

- ### 2.4 Install [gazebo_ros_pkgs](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)  
  
- ### 2.5 Install [Tensorflow](https://www.tensorflow.org/install/pip?hl=ja)
- ### 2.6 Install [Intel Realsense](https://github.com/IntelRealSense/realsense-ros/tree/ros2)
- ### 2.7 Install [Azure Kinect DK](https://github.com/microsoft/Azure_Kinect_ROS_Driver/tree/foxy-devel)
- ### 2.8 Install [Dynamixel](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/foxy-devel)

## 3. build
- ### 3.1 create a work space
    ```bash
    # skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p ros2_ws/src
    ```
- ### 3.2 Install depencies
    ```bash
    # Remember to source ros foxy environment settings first
    $ cd ~/ros2_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
    ```

- ### 3.3 Build ROS2-Automato-Robot-Software
    ```bash
    # Remember to source ros foxy and moveit2 environment settings first
    $ cd ~/ros2_ws/
    $ colcon build
    $ colcon build --symlink-install
    ```

## 4. Package introduction 
### 4.1 harvest_studio
launchファイルが格納されているパッケージ

### 4.2 harvest_studio_description
自動収穫ロボットのフレーム情報が格納されているパッケージ

### 4.3 cpp_harvest_studio
C++で書かれたプログラムが格納されているパッケージ
・収穫動作生成
・把持・回転機構制御
・カメラ角制御
・Rvizに果実の検出状態表示

### 4.4 py_harvest_studio
Pythonで書かれたプログラムが格納されているパッケージ
・物体検出
・果実データ処理
・エンドエフェクタ制御（シリアル通信）

### 4.5 xarm_ros2
ロボットアームUFactory xArm 5 Liteの制御

### 4.6 STM32
把持・回転機構に取り付けられているマイコンのSTM32に書き込まれているプログラム

### 4.7 calculation dummy_test_pkg
テスト用のパッケージ

## 5. Question
何か不明点等あれば，小熊(kzy.basect@gmail.com)まで質問してください．