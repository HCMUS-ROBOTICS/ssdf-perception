#include "lane_extraction/interface.h"

/** 
 * @brief Polynomial fitting a line through the given points
 *
 * The line formula would be
 * x = a_0 + a_1 y + a_2 y^2 + ... + a_n y^n
 * 
 * For example, with degree = 2, the fitting formulas should be a parabol:
 * x = a_0 + a_1 y + a_2 y^2
 *
 * @param points the points to fit
 * @param degree the degree of the formula
 * @param refittedPoints points will be refitted using the computed parameters. Pass nullptr to avoid refitting.
 * @return the parameters [a_0, a_1, ..., a_n]
 */
LaneParams polyfit(const LanePoints &points, size_t degree, LanePoints *refittedPoints = nullptr);
void evalPoly(const LaneParams &params, LanePoints &points);

cv::Mat birdviewTransformation(const cv::Mat &src, int birdwidth, int birdheight, int skyline, int offsetLeft, int offsetRight, cv::Mat &returnM);
