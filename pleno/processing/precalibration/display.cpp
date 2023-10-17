#include "display.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/fast_math.hpp>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

#include "processing/tools/stats.h"

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - Histograms
////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, double min, double max, int binSize, int height, int nbbin)
{		
	const double step = (max - min) / nbbin;
	std::vector<int> hist(nbbin);
	for(const auto&d : data)
	{
		if(d<min or d>max) continue;
		int i =  (d - min) / step;
		hist[i] +=1;	
	}

    int max_value = *std::max_element(hist.begin(), hist.end());
    int rows = 0;
    int cols = 0;
    double scale = 1;
    if (height == 0) {
        rows = max_value + 10;
    }
    else {
        rows = height; 
        scale = double(height) / (max_value + 10);
    }
    cols = hist.size() * binSize;
    dst = Image::zeros(rows, cols, CV_8UC3);
    for (std::size_t i = 0; i < hist.size(); ++i)
    {
        const int h = rows - int(scale * hist[i]);
        cv::rectangle(dst, cv::Point(i*binSize, h), cv::Point((i + 1)*binSize - 1, rows), (i % 2) ? cv::Scalar(0, 100, 255) : cv::Scalar(0, 0, 255), cv::FILLED);
    }

	const double rmean = mean(data);
	const double rmedian = median(data);
	
	const double col_mean = (rmean - min) / step * binSize;
	const double col_median = (rmedian - min) / step * binSize; 
	
	cv::line(dst, cv::Point(col_mean, 20), cv::Point(col_mean, rows), cv::Scalar(255,0,0));
	cv::line(dst, cv::Point(col_median, 10), cv::Point(col_median, rows), cv::Scalar(0,255,0));
	
	cv::putText(dst, 
            "Mean= " + std::to_string(rmean),
            cv::Point(col_mean+5, 20), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.5, // Scale. 2.0 = 2x bigger
