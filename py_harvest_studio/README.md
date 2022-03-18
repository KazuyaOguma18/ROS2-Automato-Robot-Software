# py_harvest_studio
Pythonのスクリプトのみのパッケージ

## 1. tomato_detector.py
トマトの物体検出並びに三次元位置推定を行うノード．物体検出の都合上，ズーム処理を実装してある．詳しくは[修論](https://drive.google.com/file/d/1Y749WhPhdFxS_Jme-FnlU8SoXcdUXA_n/view?usp=sharing)を参照してください．

## 2. tomato_detector_zero.py
ズーム処理だけ削除した，トマトの三次元位置を推定するノード．

## 3. fruit_data_processor.py
果実のデータ処理を行うノード．物体検出と収穫動作を柔軟に接続するデータベース的な役割を果たしている．詳細は[修論](https://drive.google.com/file/d/1Y749WhPhdFxS_Jme-FnlU8SoXcdUXA_n/view?usp=sharing)

## 4. hand_ros2serial.py
エンドエフェクタとＰＣ間のシリアル通信を行うノード．エンドエフェクタの動作完了判定も併せて行う．

## 5. sample_tomato_data_publisher.py
果実のダミーデータの配信を行うノード．手元にRGB-Dカメラがない場合に使う．

## 6. sample_depth_center.py
画像中央部のDepth値を取得するノード．RGB-Dカメラから配信されてるDepth画像が正しくエンコードされているかチェックする場合に使う．