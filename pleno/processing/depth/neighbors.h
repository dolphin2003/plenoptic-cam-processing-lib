#pragma once

#include <vector>
#include <map>

#include "geometry/mia.h"
#include "geometry/sensor.h"

#include "types.h"

NeighborsIndexes
inner_ring(const MIA& mia, std::size_t k, std::size_t l);

NeighborsIndexes neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double mi