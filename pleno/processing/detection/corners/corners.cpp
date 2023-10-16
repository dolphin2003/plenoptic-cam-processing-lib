/*
 * corners.cpp
 * This file is part of libpleno
 *
 * Copyright (C) 2019 - Mathieu Labussiere
 *
 * libpleno is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * multifocus_calibration is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with multifocus_calibration. If not, see <http://www.gnu.org/licenses/>.
 */

 
#include "processing/detection/detection.h"

#include <numeric>
#include <array>
#include <omp.h>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

#include "geometry/model/microimage.h"

#include "processing/imgproc/improcess.h"
#include "processing/tools/stats.h"
#include "processing/tools/lens.h"

//detection/corner
#include "caracterize.h"
#include "optimize.h"
#include "clusterize.h"

std::array<P2D, 4> extract_checkerboard_view(
	const Image& gray, const MIA& grid, double radius
)
{
	PRINT_WARN("Extracting coordinates of checkerboard in image...");
	
FORCE_GUI(true);
	
	const double ratio = double(gray.rows) / double(gray.cols);
	const int base_size = 800;
	
	GUI(
		RENDER_DEBUG_2D(
			Viewer::context().size(base_size,base_size*ratio).layer(Viewer::layer()).name("Checkerboard"), 
			gray
		);
    );
    Viewer::update();
		
	//const auto& t = grid.pose().translation();
	//const double offsetx = t[0];
	//const double offsety = t[1];
	
	volatile bool finished = false;
	int nbclick = 0;
	
	std::array<P2D, 4> view;
	std::array<P2D, 4> xy;
	
	PRINT_INFO("=== Click on the top-left micro-image delimiting the checkerboard view");
	Viewer::context().on_click([&](float x, float y)
    {
		const int k = static_cast<int>( x * grid.width() /  gray.cols );
		const int l = static_cast<int>( y * grid.height() / gray.rows );

		auto is_out_of_range = [&](int k, int l) {
			return (k < 0 or k > int(grid.width()) or l < 0  or l > int(grid.height()));
		};
		
		if(false and is_out_of_range(k,l))
		{
			PRINT_ERR("Click out of bound, unvalidated.");
			return;
		}
				
		GUI(
			P2D c = P2D{x,y};
  			Viewer::context().layer(Viewer::layer())
  				.name("Checkerboard bounds")
  				.pen_color(v::purple).pen_width(5)
  				.add_circle(c[0], c[1], radius)
  				.update();
		);
				
        switch(nbclick)
        {
        	case 0:
        		view[0] = P2D{k,l};
        		xy[0] = P2D{x,y};
			
				PRINT_DEBUG("Top-left bound set to ("<<k <<", "<<l<<")");
				PRINT_INFO("=== Click on the top-right micro-image delimiting the checkerboard view");
        	break;
        		
        	case 1:
        		view[1] = P2D{k,l};
        		xy[1] = P2D{x,y};
        		
        		GUI(
					auto& a = xy[0];
					auto& b = xy[1];
					Viewer::context().layer(Viewer::layer())
						.name("Checkerboard bounds")
						.pen_color(v::purple).pen_width(3)
						.add_line(a[0], a[1], b[0], b[1])
						.update();
				);
			
				PRINT_DEBUG("Top-right bound set to ("<<k <<", "<<l<<")");
				PRINT_INFO("=== Click on the bottom-right micro-image delimiting the checkerboard view");
        	break;
        	
        	case 2:
        		view[2] = P2D{k,l};
        		xy[2] = P2D{x,y};
        		
        		GUI(
					auto& a = xy[1];
					auto& b = xy[2];
					Viewer::context().layer(Viewer::layer())
						.name("Checkerboard bounds")
						.pen_color(v::purple).pen_width(3)
						.add_line(a[0], a[1], b[0], b[1])
						.update();
				);
	
				PRINT_DEBUG("Bottom-right bound set to ("<<k <<", "<<l<<")");
				PRINT_INFO("=== Click on the bottom micro-image delimiting the checkerboard view");	
        	break;
        	
        	case 3:
        		view[3] = P2D{k,l};	
        		xy[3] = P2D{x,y};	
        		
        		GUI(
					auto& a = xy[2];
					auto& b = xy[3];
					Viewer::context().layer(Viewer::layer())
						.name("Checkerboard bounds")
						.pen_color(v::purple).pen_width(3)
						.add_line(a[0], a[1], b[0], b[1])
						.update();
				);
	
				PRINT_DEBUG("Bottom bound set to ("<<k <<", "<<l<<")");
				
        		finished = true;
        	break;
        	
        	default: //more than 4
        	break;   
        }
        
        nbclick++;
    });
	
	while(not finished);
	
	GUI(
		auto& a = xy[3];
		auto& b = xy[0];
		Viewer::context().layer(Viewer::layer())
			.name("Checkerboard bounds")
			.pen_color(v::purple).pen_width(3)
			.add_line(a[0], a[1], b[0], b[1])
			.update();
	);
	
	Viewer::context().on_click([](float,float){});	
	Viewer::update();

FORCE_GUI(false);
	return view;
}

CBObservations
detection_corners(const Image& raw, const MIA& mia, const InternalParameters& params)
{
	[[maybe_unused]] constexpr int roipaddingfactor = 1.13;
	const int roiw = std::ceil(mia.diameter());// * roipaddingfactor); //=28;
    const int roih = roiw;
	const int roipadding = 0.; //std::floor(roiw * (1. - 1. / 1.13));
		
#if 0 //USING CANNY EDGES    
	constexpr int kernel_size = 3;
	constexpr double ratio = 3.; //Canny recommended a upper:lower ratio between 2:1 and 3:1.
#endif    
    const double radius = std::fabs(params.I == 0ul ? mia.radius() : params.radius(0));
    
    static const decltype(v::red) colors[] = {v::red, v::green, v::blue, v::purple, v::yellow, v::orange};
        
    PRINT_DEBUG("Grayscale conversion");
    Image img = Image::zeros(raw.rows, raw.cols, CV_8UC1);
    cv::cvtColor(raw, img, cv::COLOR_BGR2GRAY);

    CBObservations obs;
    obs.reserve(mia.size());
    
    //auto [k_min, k_max, l_min, l_max] = extract_checkerboard_coordinates(img, mia, radius);
    std::array<P2D, 4> view = extract_checkerboard_view(img, mia, radius);

	auto is_inside_quad = [](const auto& p, const auto& quad) -> bool	{    
		auto angle = [](const auto& p, const auto& r, const auto& q) -> double {
			return std::acos(((q-r).squaredNorm() + (p-r).squaredNorm() - (p-q).squaredNorm()) / (2.0 * (q-r).norm() * (p-r).norm())) * 180.0/M_PI;
		};
	
		return 	angle(quad[0],quad[1],p) < angle(quad[0],quad[1],quad[2]) and
				angle(quad[1],quad[2],p) < angle(quad[1],quad[2],quad[3]) and
				angle(quad[2],quad[3],p) < angle(quad[2],quad[3],quad[0]) and
				angle(quad[3],quad[0],p) < angle(quad[3],quad[0],quad[1]);
	};
    
    Viewer::stash();
//1) For each micro-image, determine the type of the mi: NONE, FULL, BORDER, CORNER
	PRINT_INFO("=== Caracterizing micro-images' type");
    for (std::size_t k = 0; k < mia.width(); ++k) //iterate through columns //x-axis
    {
    	for (std::size_t l = 0; l < mia.height(); ++l) //iterate through lines //y-axis
		{
	 		if (not is_inside_quad(P2D{k,l}, view)) continue;
	 		 
	 		Viewer::pop();	
	 				
	 		const P2D c = mia.nodeInWorld(k,l); //col,row
			const int t = mia.type(params.I, k,l); //static_cast<int>(std::fmod(std::fmod(l,2)+k, 3)); //k=col, l=row
			const double r = std::fabs(params.I > 1ul ? mia.radius() : params.radius(t)); //radius
			//crop image aroud the center
			double X = c[0], Y = c[1]; 
 	
	//1.1) EXTRACT ROI			
    		Image roi = extract_roi(img, X, Y, roiw, roih).clone(); //crop
 			cv::threshold(roi, roi, 100, 255, cv::THRESH_BINARY); //cv::THRESH_TOZERO); //
 			trim(roi, r, - 1.5 * mia.border());
 			
 	//1.2) COMPUTE MASK (only keeping internal edges)
#if 0 //USING CANNY EDGES
 			Image mask;
 			//https://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
 			const double lower_threshold = std::max(static_cast<double>(cv::mean(roi)[0] * (1. - 0.33)), 0.); //sigma 0.333		
 			const double upper_threshold = std::min(static_cast<double>(cv::mean(roi)[0] * (1. + 0.33)), 255.); //sigma 0.333
	 		cv::Canny(roi, mask, lower_threshold, upper_threshold, kernel_size); //compute edge
    		
 			trim(mask, r, - 0.4 * r ); //set mask (kernel_size + 1.)  		
#else //USING TRUNCATED ROI
 			Image mask = roi.clone();
 			trim(mask, r, - 2.2 * mia.border());//trim(mask, r, - 0.4 * r);
#endif    	
	
 	//1.3) CARACTERIZE MICRO-IMAGE TYPE
			MicroImageType mitype = caracterize(roi, mask);
			
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++)
	  				.name("corners:type(NONE:red,FULL:green,BORDER:blue,CORNER:yellow)")
	  				.pen_color(colors[mitype]).pen_width(5),
	  			Disk{c,r}
			);
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++)
	  				.name("mi:type(0:red,1:green,2:blue)")
	  				.pen_color(colors[t]).pen_width(5),
	  			c
			);		
	
	//1.4) SAVE OBSERVATIONS
			if (mitype == CORNER)
			{
				obs.emplace_back(
					CheckerBoardObservation{
						static_cast<int>(k), static_cast<int>(l), //k,l
						c[0], c[1] //u,v
					}
				);
			}
		}		
	}
	Viewer::update();
	
//2) Optimize position of each corner (using the computed model and a warping process)
	PRINT_INFO("=== Optimizing corner position in micro-images");
	
#pragma omp parallel for
	for (int i = 0; i < int(obs.size()); ++i) //for(auto& cbo : obs)
	{
		auto& cbo = obs[i];
		
		const int k = cb