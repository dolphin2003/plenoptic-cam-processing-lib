#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

struct RelativeBlurCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //SAD
	
	//The ref image is more out-of-focus than edi image
	const Image ref; 
	const Image edi; //equally defocused image
		
    const double rho_r; //std::sqrt(std::abs(bap_i.rho * bap_i.rho - bap_j.rho * bap_j.rho));
    
    RelativeBlurCostError(const Image& ref_, const Image& edi_, double r) : ref{ref_.clone()}, edi{edi_.clone()}, rho_r{r} {