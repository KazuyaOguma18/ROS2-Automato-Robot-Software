# cpp_harvest_studio
C++で書かれたソースコード

## 1. add_rviz_marker.cpp
RVizに検出された果実情報，データ処理ノードに保持している果実情報を可視化するノード

## 2. camera2dynamixel.cpp
カメラ角の制御に用いるDynamixel AX-12Aの制御用ノード，USBを介して目標値，現在角の送受信を行う．

## 3. change_joint_states.cpp
自動収穫ロボットとxArmの間をエラーが起こらないように上手くつなぐノード．
/joint_stateを書き換えている．

## 4. generate_motion_point.cpp
ロボットアームとエンドエフェクタの制御指令を生成するノード．実質自動収穫システムの核．データ処理ノードからサービス通信で，果実情報を取得し，収穫動作生成を実行する，

## 5. harvest_studio_control.cpp
カメラ角，把持回転機構を制御するノード．検出状態等から収穫完了判定を行い，各機構の制御信号を生成する．

## 6. point_cloud_updater.cpp
MoveItの仕様上，点群が常に送信されるとマップを更新し続けて，肝心なアームの動作生成が止まってしまうので，動作生成する直前に点群を更新するようにするノード．

## 7. generate_motion_point_random.cpp
ロボットアームを動作可能な範囲でランダム生成し，動作させるノード．宴会芸

## 8. get_xarm_state.cpp
現在のロボットアームの状態を取得するノード．同じ名前のlaunchファイルを起動することで動作可能

## 9. dyna_angle_publisher.cpp
カメラ角を周期的に変更するノード．動作確認用

## 10. dyna_angle_control.cpp
カメラ角の制御を行うノード．harvest_studio_control.cppの劣化版