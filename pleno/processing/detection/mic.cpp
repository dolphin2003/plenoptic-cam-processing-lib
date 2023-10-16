#include "detection.h"

#include <opencv2/imgproc.hpp>

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "processing/imgproc/improcess.h"

static void contrast_strech(const Image& input, Image& output, int threshold)
{
    for (int row = 0; row < output.rows; ++row)
    {
        for (int col = 0; col < output.cols; ++col)
        {
            auto intensity = input.at<uchar>(row, col);
            if (intensity < threshold) intensity = 0;

            output.at<uchar>(row, col) = intensity;
        }
    }
}

static void impreprocess(const Image& raw, Image& preprocessed, std: