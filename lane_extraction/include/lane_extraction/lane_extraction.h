#pragma once

#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>

#include "lane_extraction/interface.h"

class IExtractorImpl;

/**
 * @class LaneExtractor
 * @brief Class to run Lane detection using variety of implementation.
 */
class LaneExtractor {
 public:
  LaneExtractor();
  ~LaneExtractor();

  void run();

 private:
  const size_t QUEUE_SIZE = 10;
  const size_t PUBLISH_RATE = 15;

  size_t _subscriberCount;

  std::shared_ptr<IExtractorImpl> detectImpl;
  ros::NodeHandle _nh;
  ros::Publisher _lanePub;
};
