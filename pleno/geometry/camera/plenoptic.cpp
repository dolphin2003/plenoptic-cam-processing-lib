#include "plenoptic.h"

#include "sampler.h"

#include "processing/tools/rotation.h"

#include "io/printer.h"
#include "io/cfg/camera.h"


//******************************************************************************
//******************************************************************************
//******************************************************************************
//Accessors	
PlenopticCamera::Mode
PlenopticCamera::mode() const
{
    double f_ = main_lens().focal();
    double d_ = D();
    
    //If we consider micro-lenses focal lengths, use f<d as definition for mode
    if (focused())
    {
    	f_ = mla().f(0);
    	d_ = d();    
    }
    
    if (unfocused()) { return Unfocused; }  
    else if ( f_ < d_ ) { return Keplerian; }
	else /* if (f_ > d_) */ { return Galilean; }
}

const QuadraticFunction& PlenopticCamera::scaling() const { return scaling_; }
QuadraticFunction& PlenopticCamera::scaling() { return scaling_; }

const MicroLensesArray& PlenopticCamera::mla() const { return mla_; }
MicroLensesArray& PlenopticCamera::mla() { return mla_; }

const MicroImagesArray& PlenopticCamera::mia() const { return mia_; }
MicroImagesArray& PlenopticCamera::mia() { return mia_; }     

const InternalParameters& PlenopticCamera::params() const { return params_; }
InternalParameters& PlenopticCamera::params() { return params_; }   

double PlenopticCamera::distance_focus() const { return dist_focus_; } 
double& PlenopticCamera::distance_focus() { return dist_focus_; }   

const ThinLensCamera& PlenopticCamera::main_lens() const { return main_lens_; }
ThinLensCamera& PlenopticCamera::main_lens() { return main_lens_; }

const Distortions& PlenopticCamera::main_lens_distortions() const { return distortions_; }
Distortions& PlenopticCamera::main_lens_distortions() { return distortions_; }

const Distortions& PlenopticCamera::main_lens_invdistortions() const { return invdistortions_; }
Distortions& PlenopticCamera::main_lens_invdistortions() { return invdistortions_; }

double PlenopticCamera::focal() const { return main_lens().focal(); }
double& PlenopticCamera::focal() { return main_lens().focal(); }   

double PlenopticCamera::aperture() const { return main_lens().aperture(); }
double& PlenopticCamera::aperture() { return main_lens().aperture(); }   

double PlenopticCamera::mlaperture() const { return mla().diameter(); }

std::size_t PlenopticCamera::I() const { return mla().I(); }

double PlenopticCamera::d(std::size_t k, std::size_t l) const 
{ 
	return - sensor().pose().translation().z() - D(k,l); 
}
double PlenopticCamera::D(std::size_t k, std::size_t l) const 
{ 
	const P3D pkl = mla().nodeInWorld(k,l);
	return std::fabs(pkl.z()); 
}

double PlenopticCamera::focal_plane(std::size_t i, std::size_t k, std::size_t l) const 
{ 
	if (not focused()) return scaling()(v2obj(std::max(2., (focal() - D(k,l)) / d(k,l) + 1e-6) , k, l)); 
	else return scaling()(mla2obj((mla().f(i) * d(k,l)) / (d(k,l) - mla().f(i) + (focal() > D() ? -1. : 1. ) * 1e-24))); 
}

bool PlenopticCamera::focused() const { return I() > 0; }
bool PlenopticCamera::multifocus() const { return I() > 1; }
bool PlenopticCamera::unfocused() const { return (I() > 0) and (d() > mla().f(0)); }

//******************************************************************************
//******************************************************************************
//******************************************************************************
PlenopticCamera::PrincipalPoint 
PlenopticCamera::pp() const
{
	P3D PP = main_lens().pose().translation(); //CAMERA
	PP = to_coordinate_system_of(sensor().pose(), PP); // SENSOR
	
	P2D pp = sensor().metric2pxl(PP).head<2>(); //IMAGE XY
	xy2uv(pp); //IMAGE UV
		
	return pp;
}
    
PlenopticCamera::PrincipalPoint
PlenopticCamera::mlpp(std::size_t k, std::size_t l) const
{
	const P2D idx = ml2mi(k, l);
	
	const P2D ckl = mia().nodeInWorld(idx[0], idx[1]); //IMAGE
	const double ratio = d() / (D() + d());
	
	const PrincipalPoint mpp = this->pp();
	
	const P2D ppkl = ratio * (mpp - ckl) + ckl;
	
	return ppkl;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void PlenopticCamera::init(
	const Sensor& sensor, 
	const MicroImagesArray& mia, 
	const InternalParameters& params, 
	double F, double aperture, double h,
	PlenopticCamera::Mode mode 
)
{		
	mia_ 	= mia; //already calibrated
	params_ = params; //already computed
	
	//I
	const std::size_t I = params.I;
	
	//DISTANCE FOCUS
	dist_focus_ = h;
	//POSE
	pose_.translation() = Translation::Zero();
	pose_.rotation()	= Rotation::Identity();
	
	//MAIN LENS
	main_lens_.focal() 				= F;
	main_lens_.aperture() 			= aperture;
	main_lens_.pose().translation() = pose_.translation();
	main_lens_.pose().rotation()	= pose_.rotation();
		
	//DISTORTIONS
	distortions_.radial() 		<< 0., 0., 0.;
	distortions_.tangential() 	<< 0., 0.;
	distortions_.depth() 		<< 0., 0., 0.;
	
	invdistortions_.radial() 		<< 0., 0., 0.;
	invdistortions_.tangential() 	<< 0., 0.;
	invdistortions_.depth() 		<< 0., 0., 0.;
	
	//SCALING
	scaling_.a = 0.; //coef z²
	scaling_.b = 1.; //coef z
	scaling_.c = 0.; //coef 1
		
	/* 	
		Init of d and D is kinda tricky: 
			- In Keplerian configuration, F < D whatever the focus distance.
			- In Galilean configuration, normally we should have F > D, 
				- but when the main lens focus distance is at infinity
				- the main lens focuses on the TCP (i.e., v = 2)
				- or when h decrease, D increase,
				- so in most cases, we will still have F < D
	*/
	double d = 0., D = 0.;
	const double m = std::fabs(params_.m);
	const double H = (h / 2.) * (1. - std::sqrt(1. - 4. * (F / h))); DEBUG_VAR(H); // eq.(18)
	
	switch(mode)
	{
		case Unfocused:
			d = m * 2.; D = F;
		break;
		
		case Keplerian: 
		{
			d = (2. * m * H) / (F - 4. * m); // eq.(17)
			D = H + 2. * d; // eq.(17)
		}
		break;
		
		case Galilean:
		{
			d = (2. * m * H) / (F + 4. * m); // eq.(17)
			D = H - 2. * d; // eq.(17)
		}
		break;
	}
	
	//std::atan2( mia_.pose().rotation()(1, 0), mia_.pose().rotation()(0, 0) );
	const double theta_z 	= get_rotation_angle(mia_.pose().rotation()); 
	const double t_x		= sensor.pxl2metric(mia_.pose().translation()[0]);
	const double t_y		= sensor.pxl2metric(mia_.pose().translation()[1]);
	const double lambda		= (D / (D + d)); // eq.(3)
	const double dC 		= params_.dc * lambda ; // eq.(3)
	
	//re-set internals
	params_.lambda 	= lambda;
	params_.dC 		= dC;
	params_.N 		= aperture;
	
	//SENSOR
	sensor_ = sensor;
	sensor_.pose().translation()[0] = sensor_.pxl2metric(-(sensor_.width() / 2.)); //set x coordinate
	sensor_.pose().translation()[1] = sensor_.pxl2metric(-(sensor_.height() / 2.)); //set y coordinate
	sensor_.pose().translation()[2] = -(D + d); //set z coordinate
	
	//MLA
	mla_.geometry() 	= mia_.geometry();
	mla_.width() 		= mia_.width();
	mla_.height() 		= mia_.height();
	mla_.pitch() 		= P2D{dC, dC};
	
	mla_.pose().rotation() << 	std::cos(theta_z),	std::sin(theta_z),		0.,
								-std::sin(theta_z),	std::cos(theta_z),		0.,
								0.,					0.,				