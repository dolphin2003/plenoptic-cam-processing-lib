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
	
	const P2D disparity = (deltac) * (
		((1. - lambda) * v + lambda) / (v)
	); 
#else //Commonly used disparity, i.e., orthogonal projection (see §5, Eq.(23)) 
	const P2D idxj = mfpc.mi2ml(mij.k, mij.l); 
	
	const P3D mli = mfpc.mla().node(idxi(0), idxi(1)); 
	const P3D mlj = mfpc.mla().node(idxj(0), idxj(1));

	const P2D deltaC = (mlj - mli).head<2>();
	
	const P2D disparity = (deltaC) / (v * mfpc.sensor().scale());
#endif
	return disparity; //in pixel
}

template <bool useBlur>
double DisparityCostError_<useBlur>::weight(double v) const 
{ 
	P2D disp = disparity(v); 
	const double d = disp.norm();
	
	const double sigma2_disp = 1.;
	const double sigma2_v = sigma2_disp * (v * v) / (d * d); 
	
	return 1. / sigma2_v; 
}

template <bool useBlur>
bool DisparityCostError_<useBlur>::compute_at_pixel() const
{
	return not (at[0] == -1. and at[1] == -1.);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <bool useBlur>
bool DisparityCostError_<useBlur>::operator()(
    const VirtualDepth& depth,
	ErrorType& error
) const
{    
	constexpr int window_size = 5; //FIXME: size ? N=5 ?
    constexpr double threshold_reprojected_pixel = 9.; //double(window_size * window_size) - 1.; //FIXME: 9?
    constexpr double epsilon = 2.2204e-16;

    error.setZero();
    
	const double v = depth.v;
	const double radius = mfpc.mia().radius() - mfpc.mia().border() - 1.5;
    
//0) Check hypotheses:
	//0.1) Discard observation?
	const P2D disp = disparity(v);
	
	if (disp.norm() >= 2. * radius)
	{
		//PRINT_ERR("Discard observation for hypothesis = " << v << ", disparity = " << disparity.norm());
		return false;
	}
	//0.2) Is disparity estimation possible? Is object real?
	if (std::fabs(v) < 2. or mfpc.v2obj(v) < mfpc.focal()) 
	{
		error[0] = 255. / v;
		return true;
	}
    
//1) get ref and edi
	Image fref, ftarget;	
	mii.mi.convertTo(fref, CV_64FC1, 1./255.); //(i)-view is ref
	mij.mi.convertTo(ftarget, CV_64FC1, 1./255.); //(j)-view has to be warped
	
//2) compute mask	
	Image fmask = Image{fref.size(), CV_64FC1, cv::Scalar::all(1.)};
	trim_double(fmask, radius);

#if ENABLE_DEBUG_SAVE_IMG
	Image buff;  
	fref.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("ref-"+std::to_string(getpid())+".png", buff);
	ftarget.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("target-"+std::to_string(getpid())+".png", buff);
	fmask.convertTo(buff, CV_8UC1, 255.);
	cv::imwrite("mask-"+std::to_string(getpid())+".png", buff);
#endif
	
//3) compute blur	
	Image fedi, lpedi;		
	if constexpr (useBlur)
	{
		if (mii.type != mij.type) //not same type
		{
			const P2D idxi = mfpc.mi2ml(mii.k, mii.l);
			const P2D idxj = mfpc.mi2ml(mij.k, mij.l); 
			
			const double A = mfpc.mlaperture(); //mm
			const double d_i = mfpc.d(idxi(0), idxi(1));	//mm, orthogonal distance center-sensor
			const double d_j = mfpc.d(idxj(0), idxj(1));	//mm, orthogonal distance center-sensor
			const double a_i = mfpc.obj2mla(mfpc.focal_plane(mii.type), idxi(0), idxi(1));  //mm, negative signed orthogonal distance to mla
			const double a_j = mfpc.obj2mla(mfpc.focal_plane(mij.type), idxj(0), idxj(1));  //mm, negative signed orthogonal distance to mla

			const double m_ij = ((A * A) / 2.) * ((d_i / a_i) - (d_j / a_j)); //mm², see Eq.(19)
			const double q_ij = ((A * A) / 4.) * (((d_i * d_i) / (a_i * a_i)) - ((d_j * d_j) / (a_j * a_j))); //mm², see Eq.(20)

			const double rel_blur = m_ij * (1. / v) + q_ij; //mm², see Eq.(18)

			const double rho_sqr_r = rel_blur / (mfpc.sensor().scale() * mfpc.sensor().scale()); //pix²
			const double kappa = mfpc.params().kappa; 
			const double sigma_sqr_r = kappa * rho_sqr_r; //pix²
			const double sigma_r = std::sqrt(std::fabs(sigma_sqr_r)); //pix, see Eq.(22)

			const bool isOrdered = (rel_blur >= 0.); //(i)-view is more defocused the the (j)-view
		
			if (isOrdered) //(i)-view is more defocused the the (j)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = target); ref ("<< mii.type+1<<"), target ("<< mij.type+1<<")");
		#endif 
				fedi = ftarget; //(j)-view has to be equally-defocused
			}
			else //(j)-view is more defocused the the (i)-view
			{
		#if ENABLE_DEBUG_DISPLAY
				PRINT_DEBUG("Views: (edi = ref); ref ("<< mii.type+1<<"), target ("<< mij.type+1<<")");
		#endif 
				fedi = fref; //(i)-view has to be equally-defocused
			}
			
			switch (method)
			{	
				case S_TRANSFORM:
				{
					cv::Laplacian(fedi, lpedi, CV_64FC1); //FIXME: Kernel size ? 3x3
					cv::add(fedi, (std::fabs(sigma_sqr_r) / 4.) * lpedi, fedi); //see Eq.(29)
					
					#if ENABLE_DEBUG_SAVE_IMG
						lpedi.convertTo(buff,