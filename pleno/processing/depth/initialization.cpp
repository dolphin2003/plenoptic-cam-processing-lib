#include "initialization.h"

#include <random>

#include "io/printer.h" //DEBUG_ASSERT, PRINT_DEBUG

#include "search.h" //gss, bruteforce

#define USE_SAME_SEED 1

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::pair<double,double> initialize_min_max_distance(const PlenopticCamera& mfpc)
{
	const double F =  mfpc.focal();
	const double nearfocusd = 20. * F; 
	const double farfocusd = 100. * F; 
	const double h = mfpc.distance_focus();
	
	double mind, maxd;
	
	if(h < nearfocusd) //short distances
	{
		maxd = mfpc.distance_focus() * 1.2;
		mind = 4. * std::ceil(F); //8. * std::ceil(F); //
	}
	else if (h <= farfocusd) //middle distances
	{
		maxd = mfpc.distance_focus() * 2.;
		mind = 6. * std::ceil(F); //8. * std::ceil(F); //
	}
	else //far distances
	{
		maxd = farfocusd;
		mind = 8. * std::ceil(F); 
	}
	
	return {mind, maxd};
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
IndexPair initialize_kl(std::size_t ith, std::size_t nbthread, const MIA& mia, InitStrategy mode)
{
#if USE_SAME_SEED 
    static std::mt19937 mt;
#else
    static std::random_device rd;
    static std::mt19937 mt(rd());
#endif
	
	constexpr std::size_t margin = 2;
	
	const std::size_t kmax = mia.width()-1-margin; 
	const std::size_t kmin = 0+margin;
	const std::size_t lmax = mia.height()-1-margin; 
	const std::size_t lmin = 0+margin;
		
	if (mode == InitStrategy::RANDOM)
	{
		std::uniform_int_distribution<std::size_t> distk(kmin, kmax);
		std::uniform_int_distribution<std::size_t> distl(lmin, lmax);
		
		return {distk(mt), distk(mt)};
	}
	else if (mode == InitStrategy::FROM_LEFT_BORDER)
	{		
		const std::size_t stepl = (lmax - lmin) / (nbthread+1);
		const std::size_t stepk = (kmax - kmin) / 2;
		return {kmin + (ith%2)*stepk , lmin + ith*stepl};	
	}
	else if (mode == InitStrategy::REGULAR_GRID) //FIXME: use kmeans?
	{
		NeighborsIndexes v; v.reserve(nbthread);
		
		switch (nbthread)
		{
			case (1): {
				v.emplace_back(kmin+(kmax-kmin)/2, lmin+(lmax-lmin)/2);
				break;
			}
			case (2): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + (lmax-lmin)/2);
				break;
			}
			case (3): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (4): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/3 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (5): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				break;
			}
			case (6): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (7): {
				v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 5 * (kmax-kmin)/6 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 2 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/6 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (8): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 1 * (lmax-lmin)/3);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/3);
				break;
			}
			case (9): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 3 * (lmax-lmin)/4);
				break;
			}
			case (10): {
				v.emplace_back(kmin + 1 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				v.emplace_back(kmin + 3 * (kmax-kmin)/4 , lmin + 1 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin + 1 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 2 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4); 
				v.emplace_back(kmin + 3 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				v.emplace_back(kmin + 4 * (kmax-kmin)/5 , lmin + 2 * (lmax-lmin)/4);
				//
				v.emplace_back(kmin 