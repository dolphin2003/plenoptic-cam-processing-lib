
#include "calibration.h"

#include <omp.h>

#include "processing/algorithms/neighbour_search.h"
#include "processing/tools/matrix.h"

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "optimization/optimization.h"
#include "optimization/errors/grid.h"

#include "link.h"

void optimize(MIA& grid, const MICObservations& observations)
{
    lma::Solver<GridReconstructionError> solver(0.001 /* lambda = -1.0 */, 45 /* iteration */);

	int nbobs=0;
    for (const auto& o : observations)
    {    	
        if (o.isValid)
        {
            solver.add(
            	GridReconstructionError{o}, //error
            	&grid, 			//MIA (pitch + angle)
            	&grid.pose()	//MIA POSE (theta_z + t_x + t_y)
            );
        	++nbobs;
        }
    }
    
    PRINT_DEBUG("#validobs= " << nbobs);

    solver.solve(lma::DENSE, lma::enable_verbose_output());
}

//GRID OPTIMIZATION
void calibration_MIA(MicroImagesArray& grid, const MICObservations& centers)
{ 	
    // attach four corners of all bunch
    MICObservations corners;
    get_4_corners(centers, corners); //tl - tr - br - bl

    //setting the micro image index
    corners[Corner::TL].k = 0; corners[Corner::TL].l = 0; //tl
    corners[Corner::TR].k = grid.width() - 1; corners[Corner::TR].l = 0; //tr
    corners[Corner::BR].k = grid.width() - 1; corners[Corner::BR].l = grid.height() - 1; //br
    
    DEBUG_VAR(corners[Corner::TL]);
    DEBUG_VAR(corners[Corner::TR]);
    DEBUG_VAR(corners[Corner::BR]);
    
    corners.pop_back(); //remove BL
    
//////////////////////////////////////////////Initialization////////////////////////////////////////
    PRINT_INFO("Corners initialization");
    optimize(grid, corners);
	
	RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).pen_color(v::red).pen_width(5).name("Optimization:cornersgrid(red)"), grid);

///////////////////////////////Linking all observations to grid nodes///////////////////////////////
    PRINT_INFO("Linking all observations to grid nodes");
    MICObservations observations{centers.begin(), centers.end()};
	link_center_to_node_index(observations, grid);	
	
	PRINT_DEBUG("Remove not affected centers");
	observations.erase(
		std::remove_if(observations.begin(), observations.end(), 
			[](const auto& c) { return (not c.isValid); }
		),
		observations.end()
	);
    observations.shrink_to_fit();
    PRINT_DEBUG("#centers="<<observations.size());

	GUI(
		Viewer::context().layer(Viewer::layer()).name("Optimization:displacement(red)").pen_color(v::red).pen_width(1);
		for (const auto& oc : observations)
		{
		    const P2D p = grid.nodeInWorld(oc.k, oc.l);
		    Viewer::context().add_line(p[0], p[1], oc[0], oc[1]);
		}
		Viewer::context().layer(Viewer::layer()++).update();//.clear();
	);

//////////////////////////////////////////////Optimization//////////////////////////////////////////
    PRINT_INFO("Optimization");
    optimize(grid, observations);
	
	GUI(
		Viewer::context().layer(Viewer::layer()).name("Optimization:displacement(green)").pen_color(v::green).pen_width(1);
		for (const auto& oc : observations)
		{
		    const P2D p = grid.nodeInWorld(oc.k, oc.l);
		    Viewer::context().add_line(p[0], p[1], oc[0], oc[1]);
		}
		Viewer::context().layer(Viewer::layer()++).update();//.clear();
	);
}