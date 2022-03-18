# harvest_studio
[ROS2-Automato-Robot-Hardware](https://github.com/KazuyaOguma18/ROS2-Automato-Robot-Software)のlaunchファイルだけが集められたパッケージです．
基本的にはautomatic_harvesting_launch.pyを使用します．

それぞれの限定的な機能を使用したい場合は，以下のlaunchファイルを使用してみてください．

## 1. automatic_harvesting_launch.py
自動収穫を実行するlaunchファイル

## 2. camera_launch.py
realsenseやazure kinectのカメラ起動，TF定義，画像の圧縮を行うlaunchファイル

## 3. demo_launch.py
RVizのみで自動収穫ロボットの起動を実行するlaunchファイル

## 4. real_demo_launch
RVizと実機の自動収穫ロボットを実行するlaunchファイル

## 5. demo_random_launch.py
RVizのみでロボットアームのランダム動作生成を実行し，動作させるlaunchファイル

## 6. real_random_launch.py
Rvizと実機でロボットアームのランダム動作生成を実行し，動作させるlaunchファイル

## 7. get_xarm_state_launch.py
現在のロボットアームの情報を1秒ごとに表示するlaunchファイル

## 8. tomato_detector_launch.py
トマトの検出をrealsenseとkinect両方の画像データを基に行うlaunchファイル

## 9. tomato_detector_with_studio_launch.py
トマト検出と収穫ロボット実機の起動を行うlaunchファイル
