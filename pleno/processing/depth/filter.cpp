#include "filter.h"

#include "processing/tools/stats.h"

#include "neighbors.h"
//******************************************************************************
//******************************************************************************
DepthMap median_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size, bool permicroimage)
{
	const auto& mia = mfpc.mia();
	
	DepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	if (dm.is_coarse_map())
	{
		const std::size_t kmax = dm.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = dm.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				double sz = size;
				if (sz == AUTOMATIC_FILTER_SIZE) 
				{
					sz = dm.depth(k,l);
					if (sz != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
					{
						const P2D idx =  mfpc.mi2ml(k,l);
						sz = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
					}
				}
				//get neighbors
			 	NeighborsIndexes neighs = neighbors(mia, k, l, sz, sz); 
				
				std::vector<double> depths; depths.reserve(neighs.size()+1);
				depths.emplace_back(dm.depth(k,l));
				
				for(auto&n : neighs) depths.emplace_back(dm.depth(n.k, n.l));
			
				filtereddm.depth(k,l) = median(depths);
			}
		}
	}
	else if (dm.is_refined_map())
	{
		if (permicroimage)
		{	
			const std::size_t kmax = dm.width()-margin; 
			const std::size_t kmin = 0+margin;
			const std::size_t lmax = dm.height()-margin; 
			const std::size_t lmin = 0+margin;
			
			for(std::size_t k = kmin; k < kmax; ++k)
			{
				for(std::size_t l = lmin; l < lmax; ++l)
				{					
					const std::size_t sz = static_cast<std::size_t>(size == AUTOMATIC_FILTER_SIZE ? 2. : size);
					
					//get neighbors				
					std::vector<double> depths; depths.reserve((sz * 2 + 1) * (sz * 2 + 1));
					
					for (std::size_t u = std::max(kmin, k - sz); u < std::min(kmax, k + sz); ++u)
					{
						for (std::size_t v = std::max(lmin, l - sz); v < std::min(lmax, l + sz); ++v)
						{
							if (double depth = dm.depth(u,v); depth != DepthInfo::NO_DEPTH)
								depths.emplace_back(dm.depth(u,v));
						}
					}
								
					filtereddm.depth(k,l) = median(depths);
				}
			}
		}
		else
		{
			constexpr std::size_t margin = 0;
		
			const std::size_t kmax = mia.width()-margin; 
			const std::size_t kmin = 0+margin;
			const std::size_t lmax = mia.height()-margin; 
			const std::size_t lmin = 0+margin;
			
			for(std::size_t k = kmin; k < kmax; ++k)
			{
				for(std::size_t l = lmin; l < lmax; ++l)
				{
					//get pixels
				 	const NeighborsIndexes pixels = pixels_neighbors(mfpc.mia(), dm.width(), dm.height(), k, l); 
					
					for (const auto& pixel : pixels)
					{		
						double d = dm.depth(pixel.k, pixel.l);
						if (d == DepthInfo::NO_DEPTH) continue;	
						
						if (dm.is_metric_depth()) 
						{
							const P2D idx =  mfpc.mi2ml(k, l);
							d = mfpc.obj2v(dm.depth(pixel.k, pixel.l), idx(0), idx(1));
						}
						
						//get neighbors
					 	NeighborsIndexes neighs = neighbors(mfpc.mia(), k, l, d, d); 	
					 	std::vector<double> depths; depths.reserve(neighs.size() + 1);
					 	depths.emplace_back(d);
					 	
						for(auto&n : neighs) 
						{
							const P2D disparity = mfpc.disparity(k, l, n.k, n.l, d);
							const std::size_t nk = static_cast<std::size_t>(pixel.k - disparity[0]); 
							const std::size_t nl = static_cast<std::size_t>(pixel.l - disparity[1]); 
							
							const double nd = dm.depth(nk, nl); 
							if (nd == DepthInfo::NO_DEPTH) continue;
							depths.emplace_back(nd);						
						}
						
						filtereddm.depth(pixel.k,pixel.l) = (depths.size() > 3 ? median(depths) : DepthInfo::NO_DEPTH);
					}
				}
			}
		}
	}
	
	return filtereddm;	
}

void inplace_median_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size, bool permicroimage)
{	
	const DepthMap temp = median_filter_depth(dm, mfpc, size, permicroimage);
	temp.copy_to(dm);
} 

//******************************************************************************
DepthMap mean_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const auto& mia = mfpc.mia();
	
	DepthMap filtereddm{dm};
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			double sz = size;
			if (sz == AUTOMATIC_FILTER_SIZE) 
			{
				sz = dm.depth(k,l);
				if (sz != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
				{
					const P2D idx =  mfpc.mi2ml(k,l);
					sz = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
				}
			}
			
			//get neighbors
		 	NeighborsIndexes neighs = neighbors(mia, k, l, sz, sz); 
			
			std::vector<double> depths; depths.reserve(neighs.size()+1);
			depths.emplace_back(dm.depth(k,l));
			
			for(auto&n : neighs) 
				if (double nd = dm.depth(n.k, n.l); nd != DepthInfo::NO_DEPTH) 
					depths.emplace_back(nd);
		
			filtereddm.depth(k,l) = mean(depths);
		}
	}
	
	return filtereddm;	
}
void inplace_mean_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = mean_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
DepthMap minmax_filter_depth(const DepthMap& dm, double min, double max)
{	
	DepthMap temp{dm};
	inplace_minmax_filter_depth(temp, min, max);
	
	return temp;
}

void inplace_minmax_filter_depth(DepthMap& dm, double min, double max)
{		
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			const double d = dm.depth(k,l);
			if(d < min or d > max) dm.depth(k,l) = DepthInfo::NO_DEPTH;
		}
	}
}

//******************************************************************************
//******************************************************************************
DepthMap erosion_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	const auto& mia = mfpc.mia();
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	DepthMap filtereddm{dm}; //depths are copied
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = dm.width()-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = dm.height()-margin; 
	const std::size_t lmin = 0+margin;
	
	for(std::size_t k = kmin; k < kmax; ++k)
	{
		for(std::size_t l = lmin; l < lmax; ++l)
		{
			if(dm.depth(k,l) == DepthInfo::NO_DEPTH)
			{
				//get neighbors
		 		NeighborsIndexes neighs;
		 		
		 		if (size == AUTOMATIC_FILTER_SIZE) neighs = inner_ring(mia, k, l);
		 		else neighs = neighbors(mia, k, l, size, size); 
		 		
		 		for(auto& n: neighs) filtereddm.depth(n.k, n.l) = DepthInfo::NO_DEPTH;
			}
		}
	}
	
	return filtereddm;	
}
void inplace_erosion_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size)
{
	DEBUG_ASSERT((dm.is_coarse_map()), "No filter implemented for dense map.");
	
	const DepthMap temp = erosion_filter_depth(dm, mfpc, size);
	temp.copy_to(dm);
} 

//******************************************************************************
//******************************************************************************
DepthMap bilateral_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, double sigmar, double sigmad, bool permicroimage
)
{
	const auto& mia = mfpc.mia();
	DepthMap filtereddm{dm};
	
	double sr = sigmar;
	if (sr == AUTOMATIC_FILTER_SIZE)
		sr = dm.is_virtual_depth() ? 1. : 20. /* mm */; 
	
	if (dm.is_coarse_map())
	{
		constexpr std::size_t margin = 2;
	
		const std::size_t kmax = dm.width()-margin; 
		const std::size_t kmin = 0+margin;
		const std::size_t lmax = dm.height()-margin; 
		const std::size_t lmin = 0+margin;
		
		auto kernel = [&](std::size_t k, std::size_t l, std::size_t nk, std::size_t nl, double sd, double sr) -> double {
			return std::exp(
				- (mia.nodeInWorld(k,l) - mia.nodeInWorld(nk, nl)).squaredNorm() / (2. * sd * sd)
				- (dm.depth(k, l) - dm.depth(nk, nl)) * (dm.depth(k, l) - dm.depth(nk, nl))	 / (2. * sr * sr)
			);
		};
	
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				double sd = sigmad;
				if (sd == AUTOMATIC_FILTER_SIZE) 
				{
					sd = dm.depth(k,l);
					if (sd != DepthInfo::NO_DEPTH and dm.is_metric_depth()) 
					{
						const P2D idx =  mfpc.mi2ml(k,l);
						sd = mfpc.obj2v(dm.depth(k,l), idx(0), idx(1));
					}
				}				
				/