#include "depthmap.h"

//******************************************************************************
// constructors
//******************************************************************************
DepthMap::DepthMap(
	std::size_t width, std::size_t height,
	double mind, double maxd,
	DepthType dtype, MapType mtype
) : //map
	map{height/* rows */, width /* cols */},
	//distances
	min_depth_{mind}, max_depth_{maxd},
	//depth map configuration
	depth_type{dtype}, map_type{mtype}
{
}

DepthMap::DepthMap(const DepthMap& o)
:	//map
	map{o.height(), o.width()},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
	copy_from(o); //copy depth map data
}

DepthMap::DepthMap(DepthMap&& o)
:	//map
	map{std::move(o.map)},
	//distances
	min_depth_{o.min_depth()}, max_depth_{o.max_depth()},
	//depth map configuration
	depth_type{o.depth_type}, map_type{o.map_type}
{
}


DepthMap::DepthMap(const PointCloud& pc, const PlenopticCamera& mfpc)
:	//map
	map{mfpc.sensor().height(), mfpc.sensor().width()},
	//distances
	min_depth_{pc.min()}, max_depth_{pc.max()},
	//depth map configuration
	depth_type{METRIC}, map_type{REFINED}
{
	using ZBufferContainer = Eigen::Matrix<std::vector<double>, Eigen::Dynamic /* row */, Eigen::Dynamic /* col */>;
	
	ZBufferContainer mzbuffer{height(), width()};
	auto zbuffer = [&](int k, int l) -> std::vector<double>& { return mzbuffer(l, k); }; 
	
	#pragma omp parallel for
