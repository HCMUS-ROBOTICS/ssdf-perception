#include <opencv2/core.hpp>
#include <utility>

#include "lane_extraction/interface.h"

/**
 * @brief Polynomial fitting a line through the given points
 *
 * The line formula would be
 * \f[x = a_0 + a_1 y + a_2 y^2 + ... + a_n y^n\f]
 *
 * For example, with degree = 2, the fitting formulas should be a parabol:
 * \f[x = a_0 + a_1 y + a_2 y^2\f]
 *
 * @param points the points to fit
 * @param degree the degree of the formula
 * @param refittedPoints points will be refitted using the computed parameters. Pass nullptr to
 * avoid refitting.
 * @return the parameters \f$[a_0, a_1, ..., a_n]\f$
 */
LaneParams polyfit(const LanePoints &points, size_t degree, LanePoints *refittedPoints = nullptr);

/**
 * @brief Polynomial computing through the given points
 *
 * The line formula would be
 * \f[ x = a_0 + a_1 y + a_2 y^2 + ... + a_n y^n \f]
 *
 * For example, with degree = 2, the fitting formulas should be a parabol:
 * \f[ x = a_0 + a_1 y + a_2 y^2 \f]
 *
 * @param params the line parameters \f$[a_0, a_1, ..., a_n]\f$
 * @param points the points to eval. This function will compute and modify inplace
 * the \f$x\f$ attribute of the point given its \f$y\f$
 */
void evalPoly(const LaneParams &params, LanePoints &points);

/**
 * @brief Birdview Transformation or Top-down transformation using warp persective transform
 * @param src The image to convert
 * @param birdwidth the width of the birdview image
 * @param birdheight the height of the birdview image
 * @param skyline the \f$y\f$ value indicating how far it can look
 * @param offsetLeft the offset of the bottom left
 * @param offsetRight the offset of the bottom right
 * @return a pair of birdview image and birdview transformation matrix
 */
std::pair<cv::Mat, cv::Mat> birdviewTransformation(const cv::Mat &src, int birdwidth,
                                                   int birdheight, int skyline, int offsetLeft,
                                                   int offsetRight);
