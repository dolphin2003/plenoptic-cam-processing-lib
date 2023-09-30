#pragma once

#include <random>

#include "types.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

constexpr double AUTOMATIC_FILTER_SIZE = -1.;

//******************************************************************************
//******************************************************************************
DepthMap median_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE, bool permicroimage = false
);
void inplace_median_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE, bool permicroimage = false
);

DepthMap mean_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE
);
void inplace_mean_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE
);

DepthMap minmax_filter_depth(const DepthMap& dm, double min, double max); 
void inplace_minmax_filter_depth(DepthMap& dm, double min, double max); 

//******************************************************************************
//******************************************************************************
DepthMap erosion_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc,
	double size = AUTOMATIC_FILTER_SIZE
);
void inplace_erosion_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc,
	double size = AUTOMATIC_FILTER_SIZE
); 

//******************************************************************************
//******************************************************************************
DepthMap bilateral_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double sigmar = AUTOMATIC_FILTER_SIZE, double sigmad = AUTOMATIC_FILTER_SIZE, 
	bool permicroimage = false
);
void inplace_bilateral_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double sigmar = AUTOMATIC_FILTER_SIZE, double sigmad = AUTOMATIC_FILTER_SIZE, 
	bool permicroimage = false
); 

//******************************************************************************
//******************************************************************************
DepthMap consistency_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double threshold = AUTOMATIC_FILTER_SIZE);
void inplace_consistency_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double threshold = AUTOMATIC_F