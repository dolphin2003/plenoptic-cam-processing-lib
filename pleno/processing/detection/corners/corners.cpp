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
	
				PRINT_DEBUG("Bottom bound set to ("<<k <<