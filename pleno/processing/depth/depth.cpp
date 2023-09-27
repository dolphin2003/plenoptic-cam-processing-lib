#include "depth.h"

#include <thread> //std::thread

#include "io/printer.h"
#include "io/choice.h"

#include "graphic/display.h"

#include "processing/tools/stats.h" //median

#include "geometry/depth/depthmap.h"
#include "processing/tools/chrono.h"

#include "strategy.h"
#include "filter.h"
#include "initialization.h"
#include "compute.h"

#define	DISPLAY_FRAME					1

//******************************************************************************
//******************************************************************************
//******************************************************************************
void estimate_depth(
	DepthMap& depthmap,
	const PlenopticCamera& mfpc,
	const Image& img,
	const DepthEstimationStrategy& strategies,
	const Image& color,
	bool gui
)
{	
	std::string ss = "";
	if (strategies.mtype == DepthMap::MapType::REFINED) ss = "refined ";
	else if (strategies.probabilistic) ss = "probabilistic ";
	
	PRINT_INFO("=== Start " << ss <<"depth estimation" << (mfpc.multifocus() ? " (BLADE)": " (DISP)"));	
#if DISPLAY_FRAME
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Frame"),
		img
  	);
#endif	
//------------------------------------------------------------------------------
	DepthMap dm{depthmap};
	
	const unsigned int nbthreads = strategies.multithread ?
			(strategies.nbthread == -1 ?  std::thread::hardware_concurrency()-1 : strategies.nbthread)
		: 	1;
		
//------------------------------------------------------------------------------
	Chrono::tic();
	// Run depth estimation
	std::vector<std::thread> threads;
	for(unsigned int i=0; i< nbthreads; ++i)
	{
		PRINT_DEBUG("Running estimation on thread (" << i <<")...");

		const auto [k,l] = initialize_kl(i, nbthreads, mfp