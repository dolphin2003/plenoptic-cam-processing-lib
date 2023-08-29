#include "colormap.h"

//Define colormaps
const Image PLENO_COLORMAP_VIRIDIS = [](void) -> Image {
	Image colormap = Image(1, 256, CV_8UC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = 
			cv::Vec3b{
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][2]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][1]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][0])
			};	
	return colormap;
}();