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

std::size_t PlenopticCamera::I() const { 