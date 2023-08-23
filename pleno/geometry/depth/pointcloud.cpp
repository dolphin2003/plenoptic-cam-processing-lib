#include "pointcloud.h"

#include <omp.h>
#include <algorithm> // std::swap_iter, std::minmax_element

#include "io/printer.h" //PRINT_ERR, PRINT_DEBUG
#include "processing/algorithms/neighbour_search.h" //NNS

//******************************************************************************
//******************************************************************************
PointCloud::PointCloud(std::size_t n) 
{ 
	if (n != reserve(n)) 
	{
		PRINT_ERR("Cannot reserve enought space for " << n << " points."); 
	} 
}

PointCloud::PointCloud(const PointCloud& o) 
: features_{o.features_}, pixels_{o.pixels_}, colors_{o.colors_} 
{ 
	PRINT_DEBUG("Copied pointcloud."); 
}

PointCloud::PointCloud(PointCloud&& o) 
: features_{std::move(o.features_)}, pixels_{std::move(o.pixels_)}, colors_{std::move(o.colors_)} 
{ 
	PRINT_DEBUG("Moved pointcloud."); 
}

//******************************************************************************
//******************************************************************************
PointCloud::PointCloud(const DepthMap& dm, const PlenopticCamera& model, const Image& image)
{
	DEBUG_ASSERT((image.type() == CV_8UC3 or image.type() == CV_8UC1), "Conversion need image format U8C3/1");
	
	const DepthMap mdm = dm.to_metric(model);
	
	const auto& sensor 	= model.sensor();
	const auto& mia 	= model.mia();
	
	const int W 	= static_cast<int>(sensor.width());
	const int H 	= static_cast<int>(sensor.height());
	const double R 	= mia.radius();// + 1;
	
	reserve(static_cast<std::size_t>(W * H));

	//for each micro-image
	for (std::size_t k= 0; k < mia.width(); ++k)
	{
		for (std::size_t l = 0; l < mia.height(); ++l)
		{	
			if (mdm.is_coarse_map() and mdm.depth(k