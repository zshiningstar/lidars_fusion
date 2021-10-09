#include <ros/ros.h>
#include <ros/time.h>

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
	
  //判断tf变换是否为单位矩阵
  bool isTfTransformIdentity(const tf::StampedTransform& transform)
  {
    auto R = transform.getBasis();
    auto T = transform.getOrigin();

    if(R == R.getIdentity() && T.getX() ==0 && T.getY() ==0 && T.getZ() ==0)
      return true;
    return false;
  }
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
	static tf::StampedTransform trans_left_lidar_in_base, trans_right_lidar_in_base;
	static bool left_transform_get=false, right_transform_get=false;
	//如果没有得到左雷达相对于车体坐标系的转换
	if(!left_transform_get)
	{
		if(!tf_listener_.canTransform(base_link_frame_, left_msg->header.frame_id, ros::Time(0))) 
		{
		    std::cerr << "failed to find transform between " << base_link_frame_ << " and " << left_msg->header.frame_id << std::endl;
		    return;
		}

		bool ok = tf_listener_.waitForTransform(base_link_frame_, left_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
	    tf_listener_.lookupTransform(base_link_frame_, left_msg->header.frame_id, ros::Time(0), trans_left_lidar_in_base);
	    if(ok)
	    	left_transform_get = true;
	}
	//如果没有得到右雷达相对于车体坐标系的转换
	if(!right_transform_get)
	{
		if(!tf_listener_.canTransform(base_link_frame_, right_msg->header.frame_id, ros::Time(0))) 
		{
		    std::cerr << "failed to find transform between " << base_link_frame_ << " and " << right_msg->header.frame_id << std::endl;
		    return;
  		}

		bool ok = tf_listener_.waitForTransform(base_link_frame_, right_msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
	    tf_listener_.lookupTransform(base_link_frame_, left_msg->header.frame_id, ros::Time(0), trans_left_lidar_in_base);
	    if(ok)
	    	right_transform_get = true;
	}
	
	if(!left_transform_get || !right_transform_get) return;
	
	//将ROS下的点云类型转换为pcl类型
	pcl::PointCloud<PointT>::Ptr left_local_laser(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*left_msg, *left_local_laser);
	
	pcl::PointCloud<PointT>::Ptr right_local_laser(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(*right_msg, *right_local_laser);
	
	pcl::PointCloud<PointT>::Ptr left_transformed(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr right_transformed(new pcl::PointCloud<PointT>());

	pcl_ros::transformPointCloud(*left_local_laser, *left_transformed, trans_left_lidar_in_base);
	pcl_ros::transformPointCloud(*right_local_laser, *right_transformed, trans_right_lidar_in_base);
     
	pcl::PointCloud<PointT>::Ptr left_right_cloud(new pcl::PointCloud<PointT>());
	*left_right_cloud =  *left_transformed + *right_transformed;
	
	//将pcl类型点云转换为ROS下类型点云
	sensor_msgs::PointCloud2::Ptr fusion_cloud(new sensor_msgs::PointCloud2);
	
	pcl::toROSMsg (*left_right_cloud, *fusion_cloud);
	fusion_cloud->header.frame_id = base_link_frame_;
	
	//ros::Time::now()只对ros下的点云有效
	fusion_cloud->header.stamp = ros::Time::now();
	pub_fusion_cloud_.publish(fusion_cloud);
	return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DoubleLidarsFusion_node");
    DoubleLidarsFusion DoubleLidarsFusion;
    ros::spin();
    return 0;
}
