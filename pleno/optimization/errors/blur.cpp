
#include "blur.h"

#include "io/printer.h"
#include "io/choice.h"

bool RelativeBlurCostError::operator()(
    const BlurProportionalityCoefficient& kappa,
	ErrorType& error
) const
{    
    error.setZero();
	
	Image blurred, fedi, fref;
	ref.convertTo(fref, CV_64FC1, 1./255.); 
	edi.convertTo(fedi, CV_64FC1, 1./255.);
	
	//compute equally-defocused image
	const double sigma_r = kappa.kappa * rho_r;
	cv::GaussianBlur(fedi, blurred, cv::Size{0,0}, sigma_r, sigma_r);	
	
	const double cost = cv::norm(fref, blurred, cv::NORM_L2); // / (fref.cols * fref.rows);	
	
	error[0] = cost;

#if 0
	DEBUG_VAR(rho_r);
	DEBUG_VAR(sigma_r);
	
	const int ksize = fref.cols;
	Image kernelX = cv::getGaussianKernel(ksize, sigma_r, CV_64F);
	Image kernelY = cv::getGaussianKernel(ksize, sigma_r, CV_64F);
	
	Image kernel = kernelX * kernelY.t();
	cv::normalize(kernel, kernel, 1., 0., cv::NORM_MINMAX);

	cv::namedWindow("ref", cv::WINDOW_NORMAL);
	cv::namedWindow("edi", cv::WINDOW_NORMAL);
	cv::namedWindow("bli", cv::WINDOW_NORMAL);
	cv::namedWindow("kernel", cv::WINDOW_NORMAL);
	
	cv::imshow("ref", fref);
	cv::imshow("edi", fedi);
	cv::imshow("bli", blurred);
	cv::imshow("kernel", kernel);
	
	cv::resizeWindow("ref", 200u, 200u);
	cv::resizeWindow("edi", 200u, 200u);
	cv::resizeWindow("bli", 200u, 200u);
	cv::resizeWindow("kernel", 200u, 200u);
	
	DEBUG_VAR(cost);
	wait();
#endif
	
    return true;
}