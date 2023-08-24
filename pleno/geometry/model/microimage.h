#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "types.h"

LineCoefficients line_parameters_from_slope_and_one_point(double s, const P2D& p);
LineCoefficients line_paramete