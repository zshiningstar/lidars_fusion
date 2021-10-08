##### 0 dependences
设备:ubuntu16.04
```
sudo apt-get install ros-kinetic-pcl*
```
##### 1 说明
* 1.话题消息：
> /cloud1: left_lidar
> /cloud2: right_lidar
> /fusion_topic: 点云融合后

* 2.两个雷达点云消息同步订阅
> message_filters

* 3.两个雷达点云拼接
> pcl点云格式直接相加

* 4.将融合后的坐标转换到车辆坐标系
> waitForTransform 与 lookupTransform

