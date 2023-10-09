#pragma once

#include "types.h"
#include "geometry/camera/plenoptic.h"

#include "strategy.h"

template <typename Functors>
void make_functors(
	Functors& functors, const NeighborsIndexes& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image