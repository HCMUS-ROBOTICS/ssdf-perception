#include "lane_extraction/utils.h"

#include "opencv2/imgproc.hpp"

LaneParams polyfit(const LanePoints &points, size_t degree, LanePoints *refittedPoints) {
  // solve z = M p (or A x = b) using SVD
  cv::Mat y(static_cast<int>(points.size()), 1, CV_64F);
  cv::Mat b(static_cast<int>(points.size()), 1, CV_64F);

  for (size_t i = 0; i < points.size(); i++) {
    b.at<double>(i) = static_cast<double>(points[i].x);
    y.at<double>(i) = static_cast<double>(points[i].y);
  }

  // solve Ax=b
  cv::Mat A = cv::Mat::ones(points.size(), degree + 1, CV_64F);
  for (size_t i = 0; i < degree + 1; i++) {
    cv::pow(y, i, A.colRange(i, i + 1));
  }

  cv::SVD svd(A);
  cv::Mat solution;
  svd.backSubst(b, solution);

  if (refittedPoints) {
    refittedPoints->clear();
    cv::Mat newX = A * solution;
    int x, y;
    for (int i = 0; i < newX.rows; i++) {
      x = static_cast<int>(newX.at<double>(i, 0));
      y = points[i].y;
      refittedPoints->push_back({x, y});
    }
  }

  std::vector<double> params;
  for (size_t i = 0; i < solution.rows; i++) {
    params.push_back(solution.at<double>(i, 0));
  }
  return params;
}

void evalPoly(const LaneParams &params, LanePoints &points) {
  for (auto &point : points) {
    point.x = 0;
    for (size_t i = 0; i < params.size(); i++) {
      point.x += params[i] * std::pow(point.y, i);
    }
  }
}

std::pair<cv::Mat, cv::Mat> birdviewTransformation(const cv::Mat &src, int birdwidth,
                                                   int birdheight, int skyline, int offsetLeft,
                                                   int offsetRight) {
  int W = src.cols;
  int H = src.rows;

  cv::Point2f inQuad[4] = {cv::Point(0, skyline), cv::Point(W - 1, skyline), cv::Point(0, H - 1),
                           cv::Point(W - 1, H - 1)};

  cv::Point2f outQuad[4] = {cv::Point(0, skyline), cv::Point(W - 1, skyline),
                            cv::Point(offsetLeft, H - 1), cv::Point(W - offsetRight, H - 1)};

  cv::Mat M = cv::getPerspectiveTransform(inQuad, outQuad);
  cv::Mat resultBirdview(birdheight, birdwidth, src.type());
  cv::warpPerspective(src, resultBirdview, M, resultBirdview.size(), cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT);
  return std::make_pair(resultBirdview, M);
}
