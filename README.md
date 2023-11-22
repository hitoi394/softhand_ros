


# 必要なPythonモジュールのインストール
```
$ pip3 install numpy
$ pip3 install numpy-quaternion
$ pip3 install scipy
$ sudo pip3 install ds4drv
```
# 環境構築
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive git@github.com:yamachaso/softhand_ros.git -b left-only
$ rosdep install --from-paths . --ignore-src -r -y
$ catkin build
```

# 実行
### PS4コントローラーとの接続
```
$ sudo ds4drv
```
を打ち込み、コントローラーのPlayStationマークのボタンとSHAREボタンを本体が白く点滅するまで同時に長押し。

### ROSの起動
別のターミナルで
```
$ roslaunch ps4_robot_control bringup.launch
```
仕様によりコントローラーとロボットのnodeを時間差を設けて起動しているので、両方起動するまで5秒ほど待つ。

### rosbag
```
$ rosbag record -O ~/rosbag/<file_name: 20231025_depth_points_side2.bag> /camera/depth/color/points /tf /joy /hand_angle
```
realsense setup
```
$ rosrun realsense2_camera rs_camera.launch
```

### sensor position
           0 4
        black dome
2 1                   5 3
          
          no sensor