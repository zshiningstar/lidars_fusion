#include "double_lidars_fusion.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DoubleLidarsFusion_node");
    DoubleLidarsFusion DoubleLidarsFusion;
    ros::spin();
    return 0;
}
