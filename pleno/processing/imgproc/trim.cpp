#include "trim.h"

////////////////////////////////////////////////////////////////////////////////
/*
	Image must be in 8-bits format
*/
void trim(Image& img, double r, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2.;
	const double centery = height / 2.;
	
	uchar * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<uchar>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) < r+tolerance) continue;
			
			pixel[x] = 0;
		}
	}
}
/*
	Image must be in 8-bits format
*/
void trim_binarize(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2.;
	const double centery = height / 2.;
	
	uchar * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr