#include "disparity.h"

#include "unused.h"

#include "io/printer.h"
#include "io/choice.h"

#include "processing/imgproc/improcess.h"

#define ENABLE_DEBUG_DISPLAY 0
#define ENABLE_DEBUG_SAVE_IMG 0
#define USE_CORRECTED_DISPARITY 1

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const MicroImage& mii_, const MicroImage& mij_, 
	const PlenopticCamera& mfpc_, const P2D& at_, BlurMethod method_
) : mii{mii_}, mij{mij_}, mfpc{mfpc_}, at{at_}, method{method_}
{ }

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	const DisparityCostError_& o
) : mii{o.mii}, mij{o.mij}, mfpc{o.mfpc},
	at{o.at}, method{o.method}
{ } 

template <bool useBlur>
DisparityCostError_<useBlur>::DisparityCostError_(
	DisparityCostError_&& o
) : mii{std::move(o.mii)}, mij{std::move(o.mij)}, mfpc{o.mfpc},
	at{std::move(o.at)}, method{o.method}
{ }

template <bool useBlur>
P2D DisparityCostError_<useBlur>::disparity(double v) const 
{ 
	//mi k,l indexes are in mi space, convert to mla space to access micro-lenses
	const P2D idxi = mfpc.mi2ml(mii.k, mii.l);
	
#if USE_CORRECTED_DISPARITY //Corrected disparity (see §5, Eq.(24))
	const P2D deltac = (mii.center - mij.center);
	
	const double D = mfpc.D(idxi(0), idxi(1)); 
	const double d = mfpc.d(idxi(0), idxi(1)); 
	
	const double lambda = D / (D + d);
	
	const P2D disparity = (delta