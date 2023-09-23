#pragma once

#include "types.h"
#include "unused.h"

#include <unordered_map>
#include <vector>

#include "processing/algorithms/neighbour_search.h"
#include "processing/algorithms/p3p.h"

#include "geometry/object/checkerboard.h"
#include "geometry/object/constellation.h" //PointsConstellation
#include "geometry/camera/camera.h"

#include "geometry/pose.h"
#include "geometry/mia.h"
#include "geometry/observation.h"

#include "processing/tools/matrix.h"


//******************************************************************************
template<typename Observations> Observations compute_barycenters(const Observations& observations);
template<typename Observations> void get_4_corners(const Observations& observations, Observations& corners); /* tl - tr - br - bl */

//******************************************************************************
template<typename Observations, typename CameraModel>
void link_cluster_to_node_index(
    Observations& observations, /* in/out */
    const Observations& barycenters,
	const CameraModel& monocular, 
	const CheckerBoard& grid,
    const Pose& pose
);
void link_center_to_node_index(MICObservations& centers, const MIA& grid);

//******************************************************************************
template<typename Observations>
void link_cluster_to_point_constellation_index(
	Observations& observations,
	const PlenopticCamera& pcm, 
	const PointsConstellation& constellation,	
	const Image& gray
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations>
Observations compute_barycenters(const Observations& observations) 
{
	using Observation = typename Observations::value_type;
	
	//Split observations according to cluster
	std::unordered_map<int /* cluster index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.cluster].push_back(ob);
		
	Observations barycenters;
	barycenters.reserve(obs.size());
	
	struct Accumulator {
		double u, v;
		double n = 0;
	};
		
	for(const auto & [c, ob] : obs)
	{		
		Accumulator acc = std::accumulate(
			ob.begin(),
			ob.end(), 
			Accumulator{0.,0.},
			[](Accumulator acc, const Observation& current) {
				return Accumulator{acc.u + current[0], acc.v + current[1], acc.n+1};
			}
		);
		
		Observation temp; 
			temp[0] = acc.u / acc.n; 
			temp[1] = acc.v / acc.n;
			temp.cluster = c;
			temp.frame = ob[0].frame;
		
		barycenters.emplace_back(
			std::move(temp) //FIXME: check
		);
	}
	
	return barycenters;
}

template<typename Observations>
void get_4_corners(const Observations& observations, Observations& corners) /* tl - tr - br - bl */
{
    corners.clear();
    corners.resize(4); /* tl - tr - br - bl */
    
    Observations obs{observations.begin(), observations.end()};
	auto accessor = [](const auto& c) { return P2D{c[0], c[1]}; };
	
	const auto& [maxx, maxy] = [&](const Observations& obs) -> std::pair<double, double> {
		double maxx=-1e12, maxy=-1e12;
		for(const auto&ob : obs) {
			const double x = ob[0];
			const double y = ob[1];
			if(x > maxx) maxx = x;
			if(y > maxy) maxy = y;
		}
		return {std::ceil(maxx), std::ceil(maxy)};
	}(obs); 

	//---bottom-right
	P2D br = FNS::find(obs, P2D{0.0, 0.0}, accessor);
	corners[Corner::BR][0] = br[0]; corners[Corner::BR][1] = br[1];
	
	//---top-left
	//P2D tl = FNS::find(obs, P2D{maxx, maxy}, accessor);
	P2D tl = FNS::find(obs, br, accessor); UNUSED(maxx); UNUSED(maxy);
	corners[Corner::TL][0] = tl[0]; corners[Corner::TL][1] = tl[1];
	
	//---top-right    
    P2D tr = FNS::find(obs, P2D{tl[0], br[1]}, accessor);
	corners[Corner::TR][0] = tr[0]; corners[Corner::TR][1] = tr[1];
	
	//---bottom-left    
    P2D bl = FNS::find(obs, P2D{br[0], tl[1]}, accessor);
	corners[Corner::BL][0] = bl[0]; corners[Corner::BL][1] = bl[1];
				
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations, typename CameraModel>
void link_cluster_to_node_index(
    Observations& observations, /* in/out */
    const Observations& barycenters,
	const CameraModel& monocular, 
	const CheckerBoard& grid,
    const Pose& pose,
    bool verbose = true
)
{
	std::unordered_map<int /* old id */, int /* new id */> id_mapping;
		
	//Find mean dist inter-reprojected node
	P2D p00, p01, p10;
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(0,0)), p00);
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(grid.width()-1, 0)), p01);
	monocular.project(to_coordinate_s