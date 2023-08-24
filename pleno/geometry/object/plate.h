#pragma once

#include "types.h" //Image, PnD, RGBA

#include "geometry/pose.h" //Pose
#include "geometry/plane.h" //PlaneCoefficients

#include "io/cfg/scene.h" //PlateConfig

struct Plate {
private:
	Pose pose_;
	
	double width_; //width in mm
	double height_; //height in mm
	
	double scale_; //mm per pixel 
	
	Image texture_; //CV_32FC3, getRectSubPix not working with 64F...

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Plate(const PlateConfig& config) : 	
		pose_{config.pose()}, 
		width_{config.width()}, height_{config.height()}, scale_{config.scale()}
	{
		Image txtr = cv::imread(config.texture(), cv::IMREAD_COLOR);
		PRINT_DEBUG("Load texture from: " << config.texture());
		PRINT_DEBUG("Texture info: (" << txtr.cols << ", " << txtr.rows << ", " << txtr.channels() << ")");
		
		//if no dimensions are given, take image as dimension
		if (height() < 0. or width() < 0.)
		{
			height() 	= txtr.rows;
			width() 	= txtr.cols;
		}
		
		Image txture;
		txtr.convertTo(txture, CV_32FC3, 1./255.);
		
		//rescale image with scale
		int wpixel = static_cast<int>(width() / scale());
		int hpixel = static_cast<int>(height() / scale());
	
		cv::resize(txture, texture_, cv::Size{wpixel, hpixel}, 0, 0, cv::INTER_LINEAR);
		
		PRINT_DEBUG("Resize texture info: (" << texture().cols << ", " << texture().rows << ", " << texture().channels() << ")");
	}

//accessors	
	Pose& pose() { return pose_; }
	const Pose& pose() const { return pose_; }
	
	double& width() { return width_; }
	double width() const { return width_; }
	
	double& height() { return height_; }
	double height() const { retur