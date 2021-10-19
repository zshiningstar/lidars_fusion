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

* 5.避障策略
> 因左右两边各安装一个雷达,因此不仅需要考虑前方的障碍物,侧边的障碍物也需要考虑.考虑环形安全距离,当有障碍物进入距离车身环形安全距离时,便停车等待.
   - 需要手动测量此环形的安全距离范围的半径Rsafe及圆心
   - 创建避障坐标系(此坐标系仅用来避障,若两雷达融合后的坐标系与之重合最好)
   - 计算障碍物距离坐标的距离,若小于Rsafe,便停车等待
