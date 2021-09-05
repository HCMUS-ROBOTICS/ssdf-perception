#pragma once
#include <vector>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include "lane_extraction/interface.h"

class LaneLine
{
public:
    void update(const cv::Mat &lineImage);
    void detect();
    void track();

    inline LaneParams getParams() const
    {
        return this->params;
    }

    inline cv::Mat getLineImage() const
    {
        return this->lineImage;
    }

    inline cv::Rect getBirdviewRect() const
    {
        return cv::Rect2i(0, 0, this->lineImage.cols - 1, this->lineImage.rows - 1);
    }

    inline bool isFound() const
    {
        return !this->points.empty();
    }

    LanePoints getPoints() const
    {
        return this->points;
    }

    virtual std::string getName() const = 0;
    virtual cv::Scalar getLaneColor() const = 0;
    virtual cv::Rect getDetectBeginPointRegion() const = 0;

protected:
    virtual cv::Point2f calcPerpendicular(const cv::Point2i &point) const = 0;
    bool findPointInWindow(const cv::Mat &roi, cv::Point &outPoint, int &outVal) const;
    bool updateNewCenter(const cv::Mat &region, cv::Point &center, int *const nonZeroValue) const;
    bool findBeginPoint(cv::Point &returnPoint) const;
    std::vector<cv::Point> findPoints(const cv::Point &beginPoint, int direct) const;

private:
    cv::Mat lineImage;
    LanePoints points;
    LaneParams params;

    const bool IS_REFIT = true;
    const size_t POLY_FIT_DEGREE = 3;
    const size_t MAX_NON_ZERO_THRESHOLD = 10;
    const size_t MIN_POINT_TRACK = 5;
    const size_t MIN_POINT_DETECT = 5;
    const u_short H_TRACKING = 16;
    const u_short W_TRACKING = 25;
    const size_t N_BINS = 10;

    int confident_score = -1;
    int offsetX = 0;
    int width = 160;
};

class LeftLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override
    {
        return cv::Scalar{0, 255, 0}; // Green
    }

    cv::Rect getDetectBeginPointRegion() const
    {
        const auto &&lineImage = this->getLineImage();
        return cv::Rect{0, 0, lineImage.cols / 2, lineImage.rows};
    }

    virtual cv::Point2f calcPerpendicular(const cv::Point2i &point) const override
    {
        cv::Point2f norm = point / cv::norm(point);
        cv::Point2f perpendicular{norm.y, -norm.x};
        return perpendicular;
    }

    virtual std::string getName() const override
    {
        return "Left";
    }
};

class RightLane : public LaneLine
{
public:
    virtual cv::Scalar getLaneColor() const override
    {
        return cv::Scalar{255, 0, 0}; // Blue
    }

    virtual cv::Rect getDetectBeginPointRegion() const override
    {
        const auto &&lineImage = this->getLineImage();
        return cv::Rect{lineImage.cols / 2, 0, lineImage.cols / 2, lineImage.rows};
    }

    virtual cv::Point2f calcPerpendicular(const cv::Point2i &point) const override
    {
        cv::Point2f norm = point / cv::norm(point);
        cv::Point2f perpendicular{-norm.y, norm.x};
        return perpendicular;
    }

    virtual std::string getName() const override
    {
        return "Right";
    }
};

class WindowExtractImpl
    : public IExtractorImpl
{
public:
    WindowExtractImpl();

    virtual void update() override;

    virtual LaneParams getLeftParams() const override
    {
        return this->left.getParams();
    }
    virtual LaneParams getRightParams() const override
    {
        return this->right.getParams();
    }

private:
    LeftLane left;
    RightLane right;
    cv::Mat _laneSegImage;

    // Bird view configuration
    int dropTop = 0;
    int offsetLeft = 85;
    int offsetRight = 85;
    int birdwidth = 224;
    int birdheight = 224;
    int skyline = 135;

private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _imageTransport;
    image_transport::Subscriber _laneSegSub;
};
