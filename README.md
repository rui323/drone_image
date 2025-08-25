# Drone_image
## 内容
## 実行前に
 - どのデバイスに接続されているか確認する
~~~bash!
# 現在使用されているデバイス一覧
python3 -m cv2_enumerate_cameras

# 出力例
# Enumerate using V4L2 backend:
# +-------+--------------------------------+------+------+-------------+
# | index |              name              | vid  | pid  |     path    |
# +-------+--------------------------------+------+------+-------------+
# |     7 |          UVC-01: Roland UVC-01 | 0582 | 02C8 | /dev/video7 |
# |     5 |          UVC-01: Roland UVC-01 | 0582 | 02C8 | /dev/video5 |
# |     1 | Chicony USB2.0 Camera: Chicony | 04F2 | B59E | /dev/video1 |
# |     0 | Chicony USB2.0 Camera: Chicony | 04F2 | B59E | /dev/video0 |
# +-------+--------------------------------+------+------+-------------+
~~~
 - USBで接続してから少し表示までに時間がかかる
 - 上記の例の場合、プロポから流れてくる映像は/dev/video5または7である
 - この２つのうち１つはダミーであるため映像が流れてくるデバイスを手動(以下のコード)で確認する
~~~bash!
ffplay /dev/video<番号> # 上記の場合　<番号>は5または7

# 映像が流れていない場合 以下のメッセージがコマンドラインに表示される
# [video4linux2,v4l2 @ 0x7dcce0000cc0] ioctl(VIDIOC_G_INPUT): Inappropriate ioctl for device 
# /dev/video5: Inappropriate ioctl for device
# 流れている場合　新しくウィンドウが開き、映像が流れているのを確認できる
~~~
 - 表示までに時間がかかるため　両方エラーが出た場合、根気強く実行を繰り返す
 - これから使うデバイスがどれくらいの速度(fps)で送信できるか確認する
~~~bash!
v4l2-ctl --device=/dev/video2 --list-formats-ext
# [0]: 'YUYV' (YUYV 4:2:2)
#                Size: Discrete 800x600
#                        Interval: Discrete 0.033s (30.000 fps)
#                        Interval: Discrete 0.033s (29.970 fps)
#                        Interval: Discrete 0.040s (25.000 fps)
#                        Interval: Discrete 0.042s (24.000 fps)
#                        Interval: Discrete 0.042s (23.970 fps)
#                        Interval: Discrete 0.050s (20.000 fps)
#                        Interval: Discrete 0.067s (15.000 fps)
#                        Interval: Discrete 0.083s (12.000 fps)
#                        Interval: Discrete 0.100s (10.000 fps)
#                        Interval: Discrete 0.125s (8.000 fps)
#                        Interval: Discrete 0.200s (5.000 fps)
#                Size: Discrete 1024x768
#                        Interval: Discrete 0.040s (25.000 fps)
#                        Interval: Discrete 0.042s (24.000 fps)
#                        Interval: Discrete 0.042s (23.970 fps)
#                        Interval: Discrete 0.050s (20.000 fps)
#                        Interval: Discrete 0.067s (15.000 fps)
#                        Interval: Discrete 0.083s (12.000 fps)
#                        Interval: Discrete 0.100s (10.000 fps)
#                        Interval: Discrete 0.125s (8.000 fps)
#                        Interval: Discrete 0.200s (5.000 fps)
~~~
## 実行コード 
 - 映像が流れているデバイスの確認ができた場合、以下のコードを実行しros2のメッセージとして送信を行う
~~~bash!
cd ワークスペース
colcon build --packages-select drone_image
source install/setup.bash
ros2 run drone_image camera --ros-args -p device:=/dev/video<認識できたデバイス番号> # ex. ros2 run drone_image camera --ros-args -p device:=/dev/video3
~~~
