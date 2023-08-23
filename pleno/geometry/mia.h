#pragma once

#include "geometry/mesh.h"

//io
#include "cfg/mia.h"

//******************************************************************************
//******************************************************************************
struct MicroImage {
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	std::size_t k, l;
	
	P2D center;
	
	double radius;
	int type;
	
	Image mi;
	
	MicroImage(std::size_t k_ = 0ul, std::size_t l_ = 0ul, const P2D& c = {-1., -1.}, double r = -1., int t = -1);
	MicroImage(std::size_t k_, std::size_t l_, const P2D& c, double r, int t, const Image& i);
	
	MicroImage(const MicroImage& o);
	MicroImage(MicroImage&& o);
	
	double contrast() const;
};

//******************************************************************************
//******************************************************************************
struct MicroImagesArray : public GridMesh2D {
	static constexpr double MI_BORDER = 1.5; //1.5 pixel
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

	MicroImagesArray(const MIAConfig& config = {});
	
	static double borde