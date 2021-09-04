#pragma once
#include <vector>
#include <opencv2/core.hpp>

typedef std::vector<cv::Point2i> LanePoints;
typedef std::vector<double> LaneParams;

class IExtractorImpl
{
public:
    virtual void update() = 0;
    virtual LaneParams getLeftParams() const = 0;
    virtual LaneParams getRightParams() const = 0;
};
