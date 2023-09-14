#pragma once

#include <vector>
#include <stack>

#include "types.h"
#include "neighbour_search.h"


//DBSCAN clustering algorithm
template<typename Point_t>
struct DBSCAN 
{
//--types
	using DP = std::vector<Point_t,  Eigen::aligned_allocator<Point_t>>;
	using Index = typename NNS::Index;
	using Label = int;
	
private:
	struct PointWithIndex {
		Point_t point;
		Index 	id;
	};
			
private:
	static constexpr Label NOISE = -2;
	static constexpr Label UNCLASSED = -1;
	
public:	
//--attributes
	double epsilon;
	std::size_t minPts;
	
	std::vector<std::vector<Index>> clusters;
	
private:
	const DP *