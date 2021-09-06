#include "lane_extraction/lane_extraction.h"
#include "lane_extraction/window_extractor.h"
#include "ssdf_msgs/Lane.h"
#include <opencv2/highgui.hpp>

LaneExtractor::LaneExtractor()
    : _subscriberCount{0}
{
    std::string lanePubTopic;
    ROS_ASSERT(ros::param::get("~pub_lane_topic", lanePubTopic));

    this->_lanePub = _nh.advertise<ssdf_msgs::Lane>(
        lanePubTopic,
        QUEUE_SIZE,
        [this](const ros::SingleSubscriberPublisher &sub)
        {
            ROS_INFO_STREAM("New subscriber from " << sub.getSubscriberName());
            this->_subscriberCount++;
        },
        [this](const ros::SingleSubscriberPublisher &sub)
        {
            ROS_INFO_STREAM("Disconnect from subscriber " << sub.getSubscriberName());
            this->_subscriberCount--;
        });

    // TODO: switch between detector
    detectImpl = std::make_shared<WindowExtractImpl>();
}

LaneExtractor::~LaneExtractor()
{
    cv::destroyAllWindows();
}

void LaneExtractor::run()
{
    ssdf_msgs::Lane msg;
    std::vector<double> leftParams;
    std::vector<double> rightParams;
    ros::Rate publishRate{static_cast<double>(PUBLISH_RATE)};

    while (_nh.ok())
    {
        ros::spinOnce();
        detectImpl->update();

        if (this->_subscriberCount > 0)
        {
            const LaneParams &leftParams = detectImpl->getLeftParams();
            const LaneParams &rightParams = detectImpl->getRightParams();
            msg.left = leftParams;
            msg.right = rightParams;

            this->_lanePub.publish(msg);
        }
        publishRate.sleep();
    }
}
