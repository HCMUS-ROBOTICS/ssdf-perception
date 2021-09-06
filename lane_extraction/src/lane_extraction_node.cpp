#include <ros/ros.h>
#include "lane_extraction/lane_extraction.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_extraction_node");
    LaneExtractor laneExtractor;

    ROS_INFO("lane_extraction_node starts");
    laneExtractor.run();

    return 0;
}
