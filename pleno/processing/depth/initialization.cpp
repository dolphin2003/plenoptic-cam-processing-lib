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