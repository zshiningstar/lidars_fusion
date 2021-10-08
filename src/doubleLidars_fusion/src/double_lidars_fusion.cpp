#include <ros/ros.h>
#include <string>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <boost/thread/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
typedef pcl::PointXYZI PointT;

class DoubleLidarsFusion
{
public:
    DoubleLidarsFusion();
    ~DoubleLidarsFusion();
    
    void Callback(const sensor_msgs::PointCloud2::ConstPtr &left_msg,
                  const sensor_msgs::PointCloud2::ConstPtr &right_msg);
                  
private:
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_left_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_right_cloud_;
    message_filters::Synchronizer<MySyncPolicy> *m_sync_;
    
    tf::TransformListener tf_listener_;
    ros::NodeHandle nh_;
    ros::Publisher pub_fusion_cloud_;
    
	string base_link_frame_;
	
//  //判断tf变换是否为单位矩阵
//  bool isTfTransformIdentity(const tf::StampedTransform& transform)
//  {
//    auto R = transform.getBasis();
//    auto T = transform.getOrigin();

//    if(R == R.getIdentity() && T.getX() ==0 && T.getY() ==0 && T.getZ() ==0)
//      return true;
//    return false;
//  }
};

DoubleLidarsFusion::DoubleLidarsFusion()
{
	//时间同步
	sub_left_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud1", 1, ros::TransportHints().tcpNoDelay());
	sub_right_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "/cloud2", 1, ros::TransportHints().tcpNoDelay());
	m_sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *sub_left_cloud_, *sub_right_cloud_);
	//只有两个话题消息同时到达时才会进入回调函数
	m_sync_->registerCallback(boost::bind(&DoubleLidarsFusion::Callback, this, _1, _2));
	
	//发布融合后的点云
	pub_fusion_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/fusion_topic", 10);
	
    base_link_frame_ = nh_.param<std::string>("base_link_frame", "");
}

DoubleLidarsFusion::~DoubleLidarsFusion()
{
	delete sub_left_cloud_;
	delete sub_right_cloud_;
	delete m_sync_;
}

void DoubleLidarsFusion::Callback(const sensor_msgs::PointCloud2::ConstPtr &left_msg, const sensor_msgs::PointCloud2::ConstPtr &right_msg)
{
	//终端打印先看时间戳
    printf("left_msg stamp:%f\n", left_msg->header.stamp.toSec());
    printf("right_msg stamp:%f\n", right_msg->header.stamp.toSec());
    
    //将ROS下的点云类型转换为pcl类型
	pcl::PointCloud<PointT>::Ptr left_local_laser(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*left_msg, *left_local_laser);
	
	pcl::PointCloud<PointT>::Ptr right_local_laser(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*right_msg, *right_local_laser);
	
	//点云为空直接退出
    if(left_local_laser->empty() || right_local_laser->empty())
		return;
		
	//点云数据相加
    pcl::PointCloud<PointT>::Ptr fusion_cloud(new pcl::PointCloud<PointT>());
    *fusion_cloud = *left_local_laser + *right_local_laser;
    
    fusion_cloud->header.frame_id = "fusion_link_frame";
    //将所有的点云坐标系赋值为fusion_link_frame?

    // 如果定义了基坐标，将点云转换到基坐标
    if(!base_link_frame_.empty()) 
    {
		if(!tf_listener_.canTransform(base_link_frame_, fusion_cloud->header.frame_id, ros::Time(0))) 
		{
		    std::cerr << "failed to find transform between " << base_link_frame_ << " and " << fusion_cloud->header.frame_id << std::endl;
		    return;
  		}

		//是否已经获得坐标转换(获取融合后的点云坐标到车辆坐标转换)
		static bool isGetTransform = false; 
		static tf::StampedTransform transform;
		if(!isGetTransform)
		{
			tf_listener_.waitForTransform(base_link_frame_, fusion_cloud->header.frame_id, ros::Time(0), ros::Duration(2.0));
		    tf_listener_.lookupTransform(base_link_frame_, fusion_cloud->header.frame_id, ros::Time(0), transform);
		    isGetTransform = true;
	  	}
	  	
	  	//是否需要点云坐标转换,是单位矩阵就不需要在转换了
		static bool isNeedTransform = true; 
//		if(isNeedTransform) //&& isTfTransformIdentity(transform)
//			isNeedTransform = false;

		pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());

		if(isNeedTransform)
		{
		    pcl_ros::transformPointCloud(*fusion_cloud, *transformed, transform);
			transformed->header.frame_id = base_link_frame_;
		    transformed->header.stamp = fusion_cloud->header.stamp;
		    fusion_cloud = transformed;
		}
		pub_fusion_cloud_.publish(fusion_cloud);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DoubleLidarsFusion_node");
    DoubleLidarsFusion DoubleLidarsFusion;
    ros::spin();
    return 0;
}
