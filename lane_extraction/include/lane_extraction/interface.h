#pragma once
#include <opencv2/core.hpp>
#include <vector>

typedef std::vector<cv::Point2i> LanePoints;
typedef std::vector<double> LaneParams;

/**
 * @brief an interface for extractor classes
 */
class IExtractorImpl {
 public:
  /**
   * @brief Update method. This will be called every loop by LaneExtractor class
   *
   * The main purpose of this method is to update its internal state, i.e. update lane parameters
   */
  virtual void update() = 0;

  /**
   * @brief Get the left lane's parameters
   * It's up to you to define the number of parameters
   * @return line parameters
   */
  virtual LaneParams getLeftParams() const = 0;

  /**
   * @brief Get the right lane's parameters
   * It's up to you to define the number of parameters
   * @return line parameters
   */
  virtual LaneParams getRightParams() const = 0;
};
