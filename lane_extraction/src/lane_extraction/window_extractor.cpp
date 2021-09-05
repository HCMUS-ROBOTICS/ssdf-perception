#include <cv_bridge/cv_bridge.h>
#include <array>
#include <memory>
#include <algorithm>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "lane_extraction/utils.h"
#include "lane_extraction/window_extractor.h"

WindowExtractImpl::WindowExtractImpl()
    : _imageTransport{_nh}
{
    std::string lane_seg_topic;
    ROS_ASSERT(ros::param::get("~sub_laneseg", lane_seg_topic));

    image_transport::TransportHints transport_hint{"compressed"};

    _laneSegSub = _imageTransport.subscribe(
        lane_seg_topic,
        10,
        [this](const sensor_msgs::ImageConstPtr &msg)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
                if (!cv_ptr->image.empty())
                {
                    this->_laneSegImage = cv_ptr->image.clone();
                }
            }
            catch (cv_bridge::Exception &e)
            {
                ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
            }
        },
        ros::VoidPtr(), transport_hint);
}

void WindowExtractImpl::update()
{
    if (this->_laneSegImage.empty())
    {
        return;
    }

    cv::Mat lineImage = this->_laneSegImage;
    if (IS_USE_BIRDVIEW)
    {
        cv::Mat birdTransform;
        std::tie(lineImage, birdTransform) = birdviewTransformation(this->_laneSegImage, birdwidth, birdheight,
                                                                    skyline, offsetLeft, offsetRight);
        lineImage(cv::Rect(0, 0, lineImage.cols, dropTop)) = cv::Scalar{0};
    }
    left.update(lineImage);
    right.update(lineImage);

    cv::Mat debugImage;
    cv::cvtColor(lineImage, debugImage, cv::COLOR_GRAY2BGR);

    for (const auto &point : left.getPoints())
    {
        cv::circle(debugImage, point, 3, left.getLaneColor(), -1);
    }

    for (const auto &point : right.getPoints())
    {
        cv::circle(debugImage, point, 3, right.getLaneColor(), -1);
    }

    cv::imshow("Frame", debugImage);
    cv::waitKey(1);
}

//////////////////////////////////////////////////////////////////////////////

void LaneLine::update(const cv::Mat &lineImage)
{
    this->lineImage = lineImage.clone();

    if (this->isFound())
    {
        this->track();
    }
    else
    {
        this->detect();
    }
}

void LaneLine::detect()
{
    points.clear();

    cv::Point beginPoint;
    if (findBeginPoint(beginPoint))
    {
        std::vector<cv::Point> &&pointsUp = findPoints(beginPoint, 1);
        std::vector<cv::Point> &&pointsDown = findPoints(beginPoint, -1);
        std::move(pointsUp.begin(), pointsUp.end(), std::back_inserter(points));
        std::move(pointsDown.begin(), pointsDown.end(), std::back_inserter(points));
    }

    if (points.size() < MIN_POINT_DETECT)
    {
        // line is lost
        points.clear();
        params.clear();
        return;
    }

    params = polyfit(this->points, this->POLY_FIT_DEGREE, &(this->points));
}

void LaneLine::track()
{
    LanePoints trackingPoints;

    for (cv::Point2i &center : points)
    {
        if (updateNewCenter(lineImage, center, nullptr))
        {
            trackingPoints.push_back(center);
        }
    }

    if (trackingPoints.size() < MIN_POINT_DETECT)
    {
        // line is lost
        points.clear();
        params.clear();
        return;
    }

    params = polyfit(trackingPoints, this->POLY_FIT_DEGREE);
    trackingPoints.clear();
    for (int y = lineImage.rows - 1; y > 0; y -= H_TRACKING)
    {
        trackingPoints.push_back({0, y});
    }

    evalPoly(params, trackingPoints);

    points = trackingPoints;
}

bool LaneLine::findPointInWindow(const cv::Mat &roi, cv::Point &outPoint, int &outVal) const
{
    size_t maxBinValue = 0;
    cv::Point maxPoint;

    const int BIN_WIDTH = roi.cols / N_BINS;
    cv::Mat binImage;
    for (int bin = 0; bin < N_BINS; bin++)
    {
        cv::Rect binROI{bin * BIN_WIDTH, 0, BIN_WIDTH, roi.rows};
        binImage = roi(binROI);

        size_t binValue = static_cast<size_t>(cv::sum(binImage).val[0]);
        float binScore = binValue / 255.f / binROI.area();

        if (binScore > 0.1f && binValue > maxBinValue)
        {
            maxBinValue = binValue;
            maxPoint = cv::Point2i{int((bin + 0.5) * BIN_WIDTH), int(binImage.rows / 2)};
        }
    }

    if (maxBinValue == 0)
    {
        return false;
    }

    outPoint = maxPoint;
    outVal = maxBinValue;
    return true;
}

bool LaneLine::updateNewCenter(const cv::Mat &region, cv::Point &center, int *const nonZeroValue) const
{
    cv::Rect roiTracking{center.x - W_TRACKING / 2, center.y - H_TRACKING / 2, W_TRACKING, H_TRACKING};

    roiTracking &= this->getBirdviewRect();

    if (roiTracking.empty())
    {
        return false;
    }

    const cv::Mat trackingImage = region(roiTracking);

    cv::Point maxPointResult;
    int value = 0;
    bool found = findPointInWindow(trackingImage, maxPointResult, value);
    if (found)
    {
        center = roiTracking.tl() + maxPointResult;
        if (nonZeroValue)
        {
            *nonZeroValue = value;
        }
    }
    return found;
}

bool LaneLine::findBeginPoint(cv::Point &returnPoint) const
{
    cv::Rect &&regionRect = getDetectBeginPointRegion();
    const cv::Mat &&region = lineImage(regionRect);

    for (int center_y = region.rows - 1 - H_TRACKING / 2; center_y > H_TRACKING / 2 + 1; center_y -= H_TRACKING)
    {
        int maxNonZero = 0;
        cv::Point maxPoint;

        for (int center_x = W_TRACKING / 2 + 1; center_x < region.cols - W_TRACKING / 2 - 1; center_x += W_TRACKING)
        {
            cv::Point center{center_x, center_y};
            int nonZeroValue = 0;
            updateNewCenter(region, center, &nonZeroValue);
            if (nonZeroValue > maxNonZero)
            {
                maxNonZero = nonZeroValue;
                maxPoint = center;
            }
        }

        if (maxNonZero > MAX_NON_ZERO_THRESHOLD)
        {
            returnPoint = regionRect.tl() + maxPoint;
            return true;
        }
    }

    return false;
}

std::vector<cv::Point> LaneLine::findPoints(const cv::Point &beginPoint, int direct) const
{
    std::vector<cv::Point> result;
    cv::Point prevPoint = beginPoint;
    cv::Point nextPoint{beginPoint.x, beginPoint.y + direct * H_TRACKING / 2};
    cv::Point2f delta;

    while (true)
    {
        if (!updateNewCenter(lineImage, nextPoint, nullptr))
        {
            break;
        }
        result.push_back(nextPoint);
        delta = nextPoint - prevPoint;
        delta = delta / cv::norm(delta);

        prevPoint = nextPoint;
        nextPoint.x += static_cast<int>(W_TRACKING * delta.x);
        nextPoint.y += static_cast<int>(H_TRACKING * delta.y / 2);
    }
    return result;
}